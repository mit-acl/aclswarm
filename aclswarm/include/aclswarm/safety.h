/**
 * @file safety.h
 * @brief Ensures autopilot commands are safe
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 2019
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <acl_msgs/QuadGoal.h>
#include <acl_msgs/QuadFlightMode.h>
#include <acl_msgs/State.h>

namespace acl {
namespace aclswarm {

  class Safety
  {
  public:
    Safety(const ros::NodeHandle nh, const ros::NodeHandle nhp);
    ~Safety() = default;
    
  private:
    ros::NodeHandle nh_, nhp_;
    ros::Subscriber sub_fmode_, sub_cmdin_, sub_state_;
    ros::Publisher pub_cmdout_;
    ros::Timer tim_control_;

    uint8_t vehid_; ///< ID of vehicle (index in veh named list)
    std::string vehname_; ///< name of the vehicle this node is running on
    std::vector<std::string> vehs_; ///< list of all vehicles in swarm

    struct Goal {
        acl_msgs::QuadGoal msg; ///< actual goal signal
        bool active = false; ///< should this goal even be considered?
        int priority = 0; ///< highest priority wins

        // to enable sorting of goals
        bool operator<(const Goal& other) const { return priority < other.priority; }
        bool operator>(const Goal& other) const { return priority > other.priority; }
    };

    /// \brief Internal state
    enum class Mode { NOT_FLYING, TAKEOFF, FLYING, LANDING };
    Mode mode_ = Mode::NOT_FLYING; ///< current mode derived from global flight mode
    geometry_msgs::PoseStamped pose_; ///< current pose of the vehicle
    enum class GoalType { DIST, JOY };
    std::map<GoalType, Goal> goals_; ///< goals to consider (by priority)

    /// \brief Parameters
    double bounds_x_min_, bounds_x_max_; ///< safety bounds to
    double bounds_y_min_, bounds_y_max_; ///< keep the vehicle
    double bounds_z_min_, bounds_z_max_; ///< in the room.
    double control_dt_; ///< period at which outer loop commands are sent
    double spinup_time_; ///< how long to wait for motors to spin up
    double takeoff_inc_; ///< altitude increment used for smooth takeoff
    double takeoff_alt_; ///< desired takeoff altitude (maybe relative)
    bool takeoff_rel_; ///< should desired alt be relative to current alt?
    double landing_fast_threshold_; ///< above this alt, land "fast"
    double landing_fast_dec_; ///< use bigger decrements for "fast" landing
    double landing_slow_dec_; ///< use smaller decrements for "slow" landing

    void init();
    void setHoverGoalMsg(acl_msgs::QuadGoal& goal);

    /// \brief ROS callback handlers
    void flightmodeCb(const acl_msgs::QuadFlightModeConstPtr& msg);
    void cmdinCb(const geometry_msgs::Vector3StampedConstPtr& msg);
    void stateCb(const acl_msgs::StateConstPtr& msg);
    void controlCb(const ros::TimerEvent& event);

  };

} // ns aclswarm
} // ns acl