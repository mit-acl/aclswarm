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
  if (!utils::loadVehicleInfo(vehname_, vehs_)) {
    ros::shutdown();
    return;
  }

  //
  // Load parameters
  //

  nhp_.param<double>("tracking_dt", tracking_dt_, 0.02);

  //
  // Timers
  //

  tim_tracking_ = nh_.createTimer(ros::Duration(tracking_dt_),
                                        &LocalizationROS::trackingCb, this);l

  //
  // ROS pub/sub communication
  //

  sub_formation_ = nh_.subscribe("/formation", 1,
                                        &LocalizationROS::formationCb, this);
  sub_assignment_ = nh_.subscribe("assignment", 1,
                                        &LocalizationROS::assignmentCb, this);
  sub_state_ = nh_.subscribe("state", 1, &LocalizationROS::stateCb, this);

}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void LocalizationROS::formationCb(const aclswarm_msgs::FormationConstPtr& msg)
{
  // recv adjacency matrix encoding formation
  formation_received_ = true;
}

// ----------------------------------------------------------------------------

void LocalizationROS::assignmentCb(const std_msgs::Uint8MultiArray& msg)
{
  // set assignment to something other than identity
}

// ----------------------------------------------------------------------------

void LocalizationROS::stateCb(const acl_msgs::StateConstPtr& msg)
{
  // copy data to usable form
  uint64_t stamp_ns = msg->header.stamp.toNSec();
  Eigen::Vector3d position;
  tf::vectorMsgToEigen(msg->pos, position);

  // add this vehicle's state to the list of vehicle estimates
  tracker_->updateVehicle(vehid_, stamp_ns, position);
}

// ----------------------------------------------------------------------------

void LocalizationROS::connectToNeighbors()
{

  // I can find out which vehicles I need to talk to, given:
  //  1) who am I (I am UAV i <-- vehicle id)
  //  2) what is the current formation (encoded in the adjacency matrix)
  //  3) what is the current assignment (the sigma / permutation mapping)

}

// ----------------------------------------------------------------------------

} // ms aclswarm
} // ns acl
