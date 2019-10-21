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

void LocalizationROS::formationCb(const aclswarm_msgs::FormationConstPtr& msg)
{
  // keep track of the current formation graph
  adjmat_ = utils::decodeAdjMat(msg->adjmat);
  assert(n_ == adjmat_.rows());

  // set the default assignment map to identity. Spoof via assignment msg
  std_msgs::UInt8MultiArrayPtr sigma(new std_msgs::UInt8MultiArray());
  sigma->data.resize(n_); std::iota(sigma->data.begin(), sigma->data.end(), 0);
  sigma->layout.dim.push_back(std_msgs::MultiArrayDimension());
  sigma->layout.dim[0].label = "assignment";
  sigma->layout.dim[0].size = sigma->data.size();
  sigma->layout.dim[0].stride = 1;
  assignmentCb(sigma);
}

// ----------------------------------------------------------------------------

void LocalizationROS::assignmentCb(const std_msgs::UInt8MultiArrayConstPtr& msg)
{
  // update our bijective assignment map
  assignment_ = msg->data;
  invassignment_ = utils::invertAssignment(assignment_);

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
                          const aclswarm_msgs::VehicleEstimatesConstPtr& msg,
                          const std::string& vehname, int vehid)
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
  }

  pub_tracker_.publish(msg);
}

// ----------------------------------------------------------------------------

void LocalizationROS::connectToNeighbors()
{
  // which formation point and I currently assigned to? Lookup my row in adjmat
  const auto& myrow = adjmat_.row(assignment_[vehid_]);

  // loop through the other formation points in graph
  for (size_t j=0; j<n_; ++j) {
    // is there an edge between my formation point and this other one?
    auto e = myrow(j);

    // which vehicle is at this formation point?
    auto nbhrid = invassignment_[j];

    // if there is an edge btwn me and this other formation point, connect.
    if (e) {
      std::string ns = vehs_[nbhrid];

      // create a closure to pass additional arguments to callback
      boost::function<void(const aclswarm_msgs::VehicleEstimatesConstPtr&)> cb =
        [=](const aclswarm_msgs::VehicleEstimatesConstPtr& msg) {
          vehicleTrackerCb(msg, ns, nbhrid);
        };

      vehsubs_[nbhrid] = nh_.subscribe("/" + ns + "/vehicle_estimates", 1, cb);
    } else {
      // if a subscriber exists, break communication
      if (vehsubs_.find(nbhrid) != vehsubs_.end()) vehsubs_[nbhrid].shutdown();
    }
  }
}

} // ms aclswarm
} // ns acl
