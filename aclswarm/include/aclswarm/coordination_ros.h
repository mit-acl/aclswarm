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

#include <Eigen/Dense>

#include <acl_msgs/State.h>
#include <aclswarm_msgs/Formation.h>
#include <aclswarm_msgs/VehicleEstimates.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8MultiArray.h>

#include "aclswarm/distcntrl.h"
#include "aclswarm/assignment.h"
#include "aclswarm/utils.h"

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
    ros::Subscriber sub_formation_, sub_tracker_;
    ros::Publisher pub_distcmd_, pub_assignment_;

    uint8_t n_; ///< number of vehicles in swarm
    uint8_t vehid_; ///< ID of vehicle (index in veh named list)
    std::string vehname_; ///< name of the vehicle this node is running on
    std::vector<std::string> vehs_; ///< list of all vehicles in swarm

    /// \brief Modules
    std::unique_ptr<DistCntrl> controller_; ///< module for control task
    std::unique_ptr<Assignment> assignment_; ///< module for assignment task

    /// \brief Internal state
    bool formation_received_ = false; ///< should a new gain matrix be used?
    std::shared_ptr<DistCntrl::Formation> formation_; ///< current formation
    // AssignmentMap assignment_; ///< assignment map (sigma: vehid --> formpt)
    // AssignmentMap invassignment_; ///< inv map (sigma^-1: formpt --> vehid)
    PtsMat q_; ///< 3D positions of swarm vehicles
    Eigen::Vector3d vel_; ///< my current velocity

    /// \brief Parameters
    double assignment_dt_; ///< period of assignment task
    double control_dt_; ///< period of high-level distributed control task

    /**
     * @brief      Initialize control and assignment modules with rosparams
     */
    void init();

    /// \brief ROS callback handlers
    void formationCb(const aclswarm_msgs::FormationConstPtr& msg);
    void vehicleTrackerCb(const aclswarm_msgs::VehicleEstimatesConstPtr& msg);
    void stateCb(const acl_msgs::StateConstPtr& msg);
    void assignCb(const ros::TimerEvent& event);
    void controlCb(const ros::TimerEvent& event);
  };

} // ns aclswarm
} // ns acl
