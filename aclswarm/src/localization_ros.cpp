/**
 * @file localization_ros.cpp
 * @brief ROS wrapper for localization components of aclswarm
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 18 Oct 2019
 */

#include <eigen_conversions/eigen_msg.h>

#include "aclswarm/localization_ros.h"
#include "aclswarm/utils.h"

namespace acl {
namespace aclswarm {

LocalizationROS::LocalizationROS(const ros::NodeHandle nh,
                                  const ros::NodeHandle nhp)
: nh_(nh), nhp_(nhp)
{
  if (!utils::loadVehicleInfo(vehname_, vehid_, vehs_)) {
    ros::shutdown();
    return;
  }

  // number of vehicles in swarm
  n_ = vehs_.size();

  init();

  //
  // Load parameters
  //

  nhp_.param<double>("tracking_dt", tracking_dt_, 0.02);

  //
  // Instantiate modules
  //

  tracker_.reset(new VehicleTracker(n_));

  //
  // Timers
  //

  tim_tracking_ = nh_.createTimer(ros::Duration(tracking_dt_),
                                        &LocalizationROS::trackingCb, this);

  //
  // ROS pub/sub communication
  //

  sub_formation_ = nh_.subscribe("/formation", 1,
                                        &LocalizationROS::formationCb, this);
  sub_assignment_ = nh_.subscribe("assignment", 1,
                                        &LocalizationROS::assignmentCb, this);
  sub_state_ = nh_.subscribe("state", 1, &LocalizationROS::stateCb, this);

  pub_tracker_ = nh_.advertise<aclswarm_msgs::VehicleEstimates>(
                                                      "vehicle_estimates", 1);

}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void LocalizationROS::init()
{
  // initialize an identity assignment permutation
  P_.setIdentity(n_);
  Pt_.setIdentity(n_);
}

// ----------------------------------------------------------------------------

void LocalizationROS::formationCb(const aclswarm_msgs::FormationConstPtr& msg)
{
  // keep track of the current formation graph
  adjmat_ = utils::decodeAdjMat(msg->adjmat);
  assert(n_ == adjmat_.rows());

  // update subscribers so that we are connected to our neighboring vehicles  
  connectToNeighbors();
}

// ----------------------------------------------------------------------------

void LocalizationROS::assignmentCb(const std_msgs::UInt8MultiArrayConstPtr& msg)
{
  // update our permutation matrix
  P_ = AssignmentPerm(Eigen::Map<const AssignmentVec>(msg->data.data(), msg->data.size()));
  Pt_ = P_.transpose();

  // update subscribers so that we are connected to our neighboring vehicles  
  connectToNeighbors();
}

// ----------------------------------------------------------------------------

void LocalizationROS::stateCb(const acl_msgs::StateConstPtr& msg)
{
  // copy data to usable form
  uint64_t stamp_ns = msg->header.stamp.toNSec();
  Eigen::Vector3d position;
  tf::vectorMsgToEigen(msg->pos, position);

  // update this vehicle's position information
  tracker_->updateVehicle(vehid_, vehid_, stamp_ns, position);
}

// ----------------------------------------------------------------------------

void LocalizationROS::vehicleTrackerCb(
                const aclswarm_msgs::VehicleEstimatesConstPtr& msg, int vehid)
{
  for (size_t i=0; i<msg->positions.size(); ++i) {
    const auto& pos = msg->positions[i];

    // copy data to usable form
    uint64_t stamp_ns = pos.header.stamp.toNSec();
    Eigen::Vector3d position;
    tf::pointMsgToEigen(pos.point, position);

    // update this vehicle's position information based on what vehid knows 
    tracker_->updateVehicle(vehid, i, stamp_ns, position);
  }
}

// ----------------------------------------------------------------------------

void LocalizationROS::trackingCb(const ros::TimerEvent& event)
{
  aclswarm_msgs::VehicleEstimates msg;
  msg.header.stamp = ros::Time::now();
  msg.positions.reserve(n_);

  for (size_t i=0; i<n_; ++i) {
    Eigen::Vector3d p = tracker_->getVehiclePosition(i);
    msg.positions.push_back(geometry_msgs::PointStamped());
    tf::pointEigenToMsg(p, msg.positions.back().point);

    uint64_t stamp_ns = tracker_->getVehicleStamp(i);
    msg.positions.back().header.stamp.fromNSec(stamp_ns);
  }

  pub_tracker_.publish(msg);
}

// ----------------------------------------------------------------------------

void LocalizationROS::connectToNeighbors()
{
  // which formation point am I currently assigned to?
  const auto i = P_.indices()(vehid_);

  // loop through the other formation points in graph
  for (size_t j=0; j<n_; ++j) {
    // which vehicle is at this formation point?
    const auto j_vehid = Pt_.indices()(j);

    // neighbor check:
    // is there an edge between my formation point and this other one?
    if (adjmat_(i, j)) {
      std::string ns = vehs_[j_vehid];

      // create a closure to pass additional arguments to callback
      boost::function<void(const aclswarm_msgs::VehicleEstimatesConstPtr&)> cb =
        [=](const aclswarm_msgs::VehicleEstimatesConstPtr& msg) {
          vehicleTrackerCb(msg, j_vehid);
        };

      vehsubs_[j_vehid] = nh_.subscribe("/" + ns + "/vehicle_estimates", 1, cb);
    } else {
      // if a subscriber exists, break communication
      if (vehsubs_.find(j_vehid) != vehsubs_.end()) vehsubs_[j_vehid].shutdown();
    }
  }
}

} // ms aclswarm
} // ns acl
