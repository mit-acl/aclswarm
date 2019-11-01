/**
 * @file coordination_ros.h
 * @brief ROS wrapper for coordination components of aclswarm
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 2019
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include <Eigen/Dense>

#include <acl_msgs/State.h>
#include <aclswarm_msgs/CBAA.h>
#include <aclswarm_msgs/Formation.h>
#include <aclswarm_msgs/VehicleEstimates.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8MultiArray.h>

#include "aclswarm/distcntrl.h"
#include "aclswarm/auctioneer.h"
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
    ros::NodeHandle nh_, nhQ_, nhp_;
    ros::CallbackQueue task_queue_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    ros::Timer tim_auctioneertick_, tim_control_;
    ros::Subscriber sub_formation_, sub_tracker_;
    ros::Publisher pub_distcmd_, pub_assignment_, pub_cbaabid_;

    uint8_t n_; ///< number of vehicles in swarm
    uint8_t vehid_; ///< ID of vehicle (index in veh named list)
    std::string vehname_; ///< name of the vehicle this node is running on
    std::vector<std::string> vehs_; ///< list of all vehicles in swarm

    /// \brief Modules
    std::unique_ptr<DistCntrl> controller_; ///< module for control task
    std::unique_ptr<Auctioneer> auctioneer_; ///< module for assignment task

    /// \brief Internal state
    bool formation_received_ = false; ///< should a new gain matrix be used?
    std::shared_ptr<DistCntrl::Formation> formation_; ///< current formation
    PtsMat q_; ///< 3D positions of swarm vehicles
    Eigen::Vector3d vel_; ///< my current velocity
    std::map<int, ros::Subscriber> vehsubs_; ///< subscribers keyed by vehid

    /// \brief Parameters
    double assignment_dt_; ///< period of continual reassignment
    double auctioneer_tick_dt_; ///< period of auctioneer tick (fsm)
    double control_dt_; ///< period of high-level distributed control task

    /**
     * @brief      Initialize control and assignment modules with rosparams
     */
    void init();

    void startAuction();

    void waitForNewAssignment();

    /**
     * @brief      Update communication graph to neighbors using current adjmat
     */
    void connectToNeighbors();

    /// \brief ROS callback handlers
    void formationCb(const aclswarm_msgs::FormationConstPtr& msg);
    void vehicleTrackerCb(const aclswarm_msgs::VehicleEstimatesConstPtr& msg);
    void stateCb(const acl_msgs::StateConstPtr& msg);
    void cbaabidCb(const aclswarm_msgs::CBAAConstPtr& msg, int vehid);
    void auctioneertickCb(const ros::TimerEvent& event);
    void controlCb(const ros::TimerEvent& event);

    /// \brief Auctioneer callback handlers
    void newAssignmentCb(const AssignmentPerm& P);
    void sendBidCb(uint32_t iter, const Auctioneer::BidConstPtr& bid);
  };

} // ns aclswarm
} // ns acl
