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
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <Eigen/Dense>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <acl_msgs/QuadGoal.h>
#include <acl_msgs/QuadFlightMode.h>
#include <acl_msgs/State.h>
#include <aclswarm_msgs/VehicleEstimates.h>
#include <aclswarm_msgs/SafetyStatus.h>

#include "aclswarm/utils.h"

namespace acl {
namespace aclswarm {

  class Safety
  {
  public:
    Safety(const ros::NodeHandle nh, const ros::NodeHandle nhp);
    ~Safety() = default;
    
  private:
    ros::NodeHandle nh_, nhp_;
    ros::Subscriber sub_fmode_, sub_cmdin_, sub_state_, sub_tracker_;
    ros::Publisher pub_cmdout_, pub_status_;
    ros::Timer tim_control_;

    uint8_t vehid_; ///< ID of vehicle (index in veh named list)
    std::string vehname_; ///< name of the vehicle this node is running on
    std::vector<std::string> vehs_; ///< list of all vehicles in swarm

    struct VelocityGoal {
        double stamp_s = 0; ///< timestamp in seconds
        double vx = 0, vy = 0, vz = 0; ///< linear velocity goal signal
        double r = 0; ///< yawrate goal signal
        bool active = false; ///< should this goal even be considered?
        int priority = 0; ///< highest priority wins
        bool modified = false; ///< indicates active collision avoidance

        // to enable sorting of velocity goals
        bool operator<(const VelocityGoal& other) const { return priority < other.priority; }
        bool operator>(const VelocityGoal& other) const { return priority > other.priority; }
    };

    /// \brief Internal state
    enum class Mode { NOT_FLYING, TAKEOFF, FLYING, LANDING };
    Mode mode_ = Mode::NOT_FLYING; ///< current mode derived from global flight mode
    geometry_msgs::PoseStamped pose_; ///< current pose of the vehicle
    enum class GoalSrc { DIST, JOY };
    std::map<GoalSrc, VelocityGoal> goals_; ///< goals to consider
    PtsMat q_; ///< 3D positions of swarm vehicles

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
    double max_accel_xy_, max_accel_z_; ///< maximum translational accels
    double max_vel_xy_; ///< maximum planar translational velocity
    double max_vel_z_; ///< maximum vertical translational velocity
    double d_avoid_thresh_; ///< do collision avoidance if within this distance
    double r_keep_out_; ///< the radius around obstacles to stay out of

    void init();
    void makeSafeTraj(double dt, const VelocityGoal& g,
                              acl_msgs::QuadGoal& goal);
    void collisionAvoidance(VelocityGoal& goal);

    /// \brief ROS callback handlers
    void flightmodeCb(const acl_msgs::QuadFlightModeConstPtr& msg);
    void cmdinCb(const geometry_msgs::Vector3StampedConstPtr& msg);
    void stateCb(const acl_msgs::StateConstPtr& msg);
    void controlCb(const ros::TimerEvent& event);
    void vehicleTrackerCb(const aclswarm_msgs::VehicleEstimatesConstPtr& msg);

  };

} // ns aclswarm
} // ns acl