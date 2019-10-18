/**
 * @file coordination_ros.h
 * @brief ROS wrapper for coordination components of aclswarm
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 2019
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include <aclswarm_msgs/Formation.h>

#include "aclswarm/distcntrl.h"
#include "aclswarm/assignment.h"

namespace acl {
namespace aclswarm {

  class CoordinationROS
  {
  public:
    CoordinationROS(const ros::NodeHandle nh, const ros::NodeHandle nhp);
    ~CoordinationROS() = default;

    void spin();

  private:
    ros::NodeHandle nh_, nhp_;
    ros::CallbackQueue task_queue_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    ros::Timer tim_assignment_, tim_control_;
    ros::Subscriber sub_formation_;

    std::string vehname_; ///< name of the vehicle this node is running on
    std::vector<std::string> vehs_; ///< list of all vehicles in swarm

    /// \brief Modules
    std::unique_ptr<DistCntrl> controller_; ///< module for control task
    std::unique_ptr<Assignment> assignment_; ///< module for assignment task
    
    /// \brief Internal state
    bool formation_received_ = false; ///< should a new gain matrix be used?

    /// \brief Parameters
    double assignment_dt_; ///< period of assignment task
    double control_dt_; ///< period of high-level distributed control task

    /// \brief ROS callback handlers
    void formationCb(const aclswarm_msgs::FormationConstPtr& msg);
    void assignCb(const ros::TimerEvent& event);
    void controlCb(const ros::TimerEvent& event);
  };

} // ns aclswarm
} // ns acl
