/**
 * @file localization_ros.h
 * @brief ROS wrapper for localization components of aclswarm
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 18 Oct 2019
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <std_msgs/UInt8MultiArray.h>
#include <acl_msgs/State.h>
#include <aclswarm_msgs/Formation.h>

namespace acl {
namespace aclswarm {

  class LocalizationROS
  {
  public:
    LocalizationROS(const ros::NodeHandle nh, const ros::NodeHandle nhp);
    ~LocalizationROS() = default;

  private:
    ros::NodeHandle nh_, nhp_;
    ros::Timer tim_tracking_;
    ros::Subscriber sub_formation_, sub_assignment_, sub_state_;

    std::string vehname_; ///< name of the vehicle this node is running on
    std::vector<std::string> vehs_; ///< list of all vehicles in swarm

    /// \brief Parameters
    double tracking_dt_; ///< period of mutual localization task

    /// \brief ROS callback handlers
    void formationCb(const aclswarm_msgs::FormationConstPtr& msg);
    void assignmentCb(const std_msgs::Uint8MultiArray& msg);
    void stateCb(const acl_msgs::StateConstPtr& msg);

  };

} // ns aclswarm
} // ns acl
