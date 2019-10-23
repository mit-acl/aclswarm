/**
 * @file localization_ros.h
 * @brief ROS wrapper for localization components of aclswarm
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 18 Oct 2019
 */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <acl_msgs/State.h>
#include <aclswarm_msgs/Formation.h>
#include <aclswarm_msgs/VehicleEstimates.h>

#include "aclswarm/vehicle_tracker.h"
#include "aclswarm/utils.h"

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
    ros::Publisher pub_tracker_;

    uint8_t n_; ///< number of vehicles in swarm
    uint8_t vehid_; ///< ID of vehicle (index in veh named list)
    std::string vehname_; ///< name of the vehicle this node is running on
    std::vector<std::string> vehs_; ///< list of all vehicles in swarm

    /// \brief Modules
    std::unique_ptr<VehicleTracker> tracker_;

    /// \brief Internal States
    std::map<int, ros::Subscriber> vehsubs_; ///< subscribers keyed by vehid
    AdjMat adjmat_; ///< current adjacency matrix for formation
    AssignmentPerm P_; ///< nxn assignment permutation (P: vehid --> formpt)
    AssignmentPerm Pt_; ///< nxn inv assign. permutation (Pt: formpt --> vehid)

    /// \brief Parameters
    double tracking_dt_; ///< period of mutual localization task

    void init();
    void connectToNeighbors();

    /// \brief ROS callback handlers
    void formationCb(const aclswarm_msgs::FormationConstPtr& msg);
    void assignmentCb(const std_msgs::UInt8MultiArrayConstPtr& msg);
    void stateCb(const acl_msgs::StateConstPtr& msg);
    void vehicleTrackerCb(const aclswarm_msgs::VehicleEstimatesConstPtr& msg,
                          const std::string& vehname, int vehid);
    void trackingCb(const ros::TimerEvent& event);

  };

} // ns aclswarm
} // ns acl
