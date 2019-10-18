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

namespace acl {
namespace aclswarm {

  class LocalizationROS
  {
  public:
    LocalizationROS(const ros::NodeHandle nh, const ros::NodeHandle nhp);
    ~LocalizationROS() = default;

  private:
    ros::NodeHandle nh_, nhp_;

    std::string vehname_; ///< name of the vehicle this node is running on
    std::vector<std::string> vehs_; ///< list of all vehicles in swarm

  };

} // ns aclswarm
} // ns acl
