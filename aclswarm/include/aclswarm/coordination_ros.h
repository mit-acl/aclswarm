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

#include "aclswarm/admm.h"
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
    ros::Timer tim_auctioneer_, tim_autoauction_, tim_control_;
    ros::Subscriber sub_formation_, sub_tracker_, sub_central_assignment_;
    ros::Publisher pub_distcmd_, pub_assignment_, pub_cbaabid_;

    uint8_t n_; ///< number of vehicles in swarm
    uint8_t vehid_; ///< ID of vehicle (index in veh named list)
    std::string vehname_; ///< name of the vehicle this node is running on
    std::vector<std::string> vehs_; ///< list of all vehicles in swarm

    /// \brief Modules
    std::unique_ptr<ADMM> admm_; ///< module for 3D gain design
    std::unique_ptr<DistCntrl> controller_; ///< module for control task
    std::unique_ptr<Auctioneer> auctioneer_; ///< module for assignment task

    /// \brief Internal state
    std::shared_ptr<DistCntrl::Formation> formation_; ///< current formation
    std::shared_ptr<DistCntrl::Formation> newformation_; ///< rcv'd formation
    PtsMat q_; ///< 3D positions of swarm vehicles
    Eigen::Vector3d vel_; ///< my current velocity
    std::map<int, ros::Subscriber> vehsubs_; ///< subscribers keyed by vehid
    ros::Time formationsent_; ///< timestamp of when formation was sent
    ros::Time startauction_; ///< timestamp of when to start next auction
    bool first_assignment_; ///< indicates first assignment of a new formation
    bool central_assignment_rcvd_; ///< used with central assignments
    AssignmentPerm Pcentral_; ///< global assign. from centralized coordinator

    /// \brief Parameters
    bool central_assignment_; ///< rcv global assignment (for sim testing)
    bool use_assignment_; ///< use auctioneer or just set identity assignment?
    double form_settle_time_; ///< time to wait after formation was sent
    double auctioneer_dt_; ///< period at which rcvd bids are processed
    double autoauction_dt_; ///< period of auto auctions (btwn form rcvd)
    double control_dt_; ///< period of high-level distributed control task

    /**
     * @brief      Initialize control and assignment modules with rosparams
     */
    void init();

    /**
     * @brief      Update comm. graph. Connects to new nbrs using adjmat.
     *
     * @return     True if a new connection was made
     */
    bool connectToNeighbors();

    void sendZeroControl();

    /// \brief ROS callback handlers
    void formationCb(const aclswarm_msgs::FormationConstPtr& msg);
    void vehicleTrackerCb(const aclswarm_msgs::VehicleEstimatesConstPtr& msg);
    void stateCb(const acl_msgs::StateConstPtr& msg);
    void cbaabidCb(const aclswarm_msgs::CBAAConstPtr& msg, int vehid);
    void auctioneerCb(const ros::TimerEvent& event);
    void autoauctionCb(const ros::TimerEvent& event);
    void controlCb(const ros::TimerEvent& event);
    void centralAssignmentCb(const std_msgs::UInt8MultiArrayConstPtr& msg);

    /// \brief Auctioneer callback handlers
    void newAssignmentCb(const AssignmentPerm& P);
    void sendBidCb(uint32_t auctionid, uint32_t iter,
                    const Auctioneer::BidConstPtr& bid);
  };

} // ns aclswarm
} // ns acl
