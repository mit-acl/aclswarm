/**
 * @file safety.cpp
 * @brief Ensures autopilot commands are safe
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 2019
 */

#include "aclswarm/safety.h"
#include "aclswarm/utils.h"

namespace acl {
namespace aclswarm {

Safety::Safety(const ros::NodeHandle nh,
                const ros::NodeHandle nhp)
: nh_(nh), nhp_(nhp)
{
  if (!utils::loadVehicleInfo(vehname_, vehid_, vehs_)) {
    ros::shutdown();
    return;
  }

  init();

  //
  // Load parameters
  //

  // room bounds
  nh_.param<double>("/room_bounds/x_min", bounds_x_min_, 0.0);
  nh_.param<double>("/room_bounds/x_max", bounds_x_max_, 1.0);
  nh_.param<double>("/room_bounds/y_min", bounds_y_min_, 0.0);
  nh_.param<double>("/room_bounds/y_max", bounds_y_max_, 1.0);
  nh_.param<double>("/room_bounds/z_min", bounds_z_min_, 0.0);
  nh_.param<double>("/room_bounds/z_max", bounds_z_max_, 1.0);

  nh_.param<double>("cntrl/spinup_time", spinup_time_, 2.0);
  nhp_.param<double>("control_dt", control_dt_, 0.01);
  nhp_.param<double>("takeoff_inc", takeoff_inc_, 0.0035);
  nhp_.param<double>("takeoff_alt", takeoff_alt_, 1.0);
  nhp_.param<bool>("takeoff_rel", takeoff_rel_, false);
  nhp_.param<double>("landing_fast_threshold", landing_fast_threshold_, 0.400);
  nhp_.param<double>("landing_fast_dec", landing_fast_dec_, 0.0035);
  nhp_.param<double>("landing_slow_dec", landing_slow_dec_, 0.001);

  //
  // Timers
  //

  tim_control_ = nh_.createTimer(ros::Duration(control_dt_),
                                                &Safety::controlCb, this);

  //
  // ROS pub/sub communication
  //

  sub_fmode_ = nh_.subscribe("/globalflightmode", 1, &Safety::flightmodeCb, this);
  sub_cmdin_ = nh_.subscribe("distcmd", 1, &Safety::cmdinCb, this);
  sub_state_ = nh_.subscribe("state", 1, &Safety::stateCb, this);

  pub_cmdout_ = nh_.advertise<acl_msgs::QuadGoal>("goal", 1);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void Safety::init()
{
  // initialize goal signals to mux
  goals_.insert(std::make_pair(GoalType::DIST, Goal()));
  goals_.insert(std::make_pair(GoalType::JOY, Goal()));

  // setup priorities
  goals_[GoalType::DIST].priority = 0;
  goals_[GoalType::JOY].priority = 1;
}

// ----------------------------------------------------------------------------

void Safety::flightmodeCb(const acl_msgs::QuadFlightModeConstPtr& msg)
{
  // handle safety state machine transitions based on
  // global flight modes dispatched by the operator.

  if (mode_ == Mode::NOT_FLYING &&
        msg->mode == acl_msgs::QuadFlightMode::GO) {
    mode_ = Mode::TAKEOFF;
    ROS_INFO("Spinning up motors for takeoff...");

  } else if ((mode_ == Mode::TAKEOFF || mode_ == Mode::FLYING) &&
        msg->mode == acl_msgs::QuadFlightMode::LAND) {
    mode_ = Mode::LANDING;
    ROS_INFO("Landing...");

  } else if (msg->mode == acl_msgs::QuadFlightMode::KILL) {
    mode_ = Mode::NOT_FLYING;
    ROS_WARN("Killing!");

  }
}

// ----------------------------------------------------------------------------

void Safety::stateCb(const acl_msgs::StateConstPtr& msg)
{
  pose_.header = msg->header;
  pose_.pose.position.x = msg->pos.x;
  pose_.pose.position.y = msg->pos.y;
  pose_.pose.position.z = msg->pos.z;
  pose_.pose.orientation.x = msg->quat.x;
  pose_.pose.orientation.y = msg->quat.y;
  pose_.pose.orientation.z = msg->quat.z;
  pose_.pose.orientation.w = msg->quat.w;
}

// ----------------------------------------------------------------------------

void Safety::cmdinCb(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  // for convenience
  auto& goal = goals_[GoalType::DIST];

  setHoverGoalMsg(goal.msg);
  goal.msg.vel = msg->vector;

  goal.active = true;
}

// ----------------------------------------------------------------------------

void Safety::controlCb(const ros::TimerEvent& event)
{
  static acl_msgs::QuadGoal goalmsg;
  static bool flight_initialized = false;
  static ros::Time takeoff_time;
  static double takeoff_alt;
  static double initial_alt;

  // set the goal to hover at our current position + yaw
  setHoverGoalMsg(goalmsg);

  if (mode_ == Mode::TAKEOFF) {

    if (!flight_initialized) {
      // capture the initial time
      takeoff_time = ros::Time::now();

      // allow the outer loop to send low-level autopilot commands
      goalmsg.cut_power = false;

      // what is our initial altitude before takeoff?
      initial_alt = pose_.pose.position.z;

      // what should our desired takeoff altitude be?
      takeoff_alt = takeoff_alt_ + ((takeoff_rel_) ? initial_alt : 0.0);

      flight_initialized = true;
    }

    // wait for the motors to spin up all the way before sending a command
    if (ros::Time::now() - takeoff_time >= ros::Duration(spinup_time_)) {

      constexpr double TAKEOFF_THRESHOLD = 0.100;
      if ((std::abs(goalmsg.pos.z - pose_.pose.position.z) < TAKEOFF_THRESHOLD) &&
            std::abs(goalmsg.pos.z - takeoff_alt) < TAKEOFF_THRESHOLD) {
        mode_ = Mode::FLYING;
        ROS_INFO("Takeoff complete!");
      } else {
        // Increment the z cmd each timestep for a smooth takeoff.
        // This is essentially saturating tracking error so actuation is low.
        goalmsg.pos.z = utils::clamp(goalmsg.pos.z + takeoff_inc_, 0.0, takeoff_alt);
      }
    }

  } else if (mode_ == Mode::FLYING) {

    // unpack any goal signals
    std::vector<Goal> goals;
    utils::mapToVec(goals_, goals);
    std::sort(goals.begin(), goals.end(), std::greater<Goal>());

    // retrieve the highest priority goal signal
    for (auto&& g : goals) {
      if (g.active) {
        goalmsg = g.msg;

        // mark this goal as used
        g.active = false;
        break;
      }
    }

  } else if (mode_ == Mode::LANDING) {

    constexpr double LANDING_THRESHOLD = 0.050;
    if (std::abs(pose_.pose.position.z - initial_alt) < LANDING_THRESHOLD) {
      mode_ = Mode::NOT_FLYING;
      ROS_INFO("Landing complete!");
    } else {
      // choose between fast landing and slow landing
      const double dec = (pose_.pose.position.z > landing_fast_threshold_) ?
                                      landing_fast_dec_ : landing_slow_dec_;

      goalmsg.pos.z = utils::clamp(goalmsg.pos.z - dec, 0.0, bounds_z_max_);
    }

  } else if (mode_ == Mode::NOT_FLYING) {
    goalmsg.cut_power = true;
    flight_initialized = false;
  }

  goalmsg.header.stamp = ros::Time::now();
  goalmsg.header.frame_id = "body";
  pub_cmdout_.publish(goalmsg);
}

// ----------------------------------------------------------------------------

void Safety::setHoverGoalMsg(acl_msgs::QuadGoal& goal)
{
  goal.pos.x = pose_.pose.position.x;
  goal.pos.y = pose_.pose.position.y;
  goal.pos.z = pose_.pose.position.z;
  goal.vel.x = 0;
  goal.vel.y = 0;
  goal.vel.z = 0;
  goal.yaw = tf2::getYaw(pose_.pose.orientation);
  goal.dyaw = 0;

  // Use position control to hover in place
  goal.xy_mode = acl_msgs::QuadGoal::MODE_POS;
  goal.z_mode = acl_msgs::QuadGoal::MODE_POS;
}

} // ns aclswarm
} // ns acl
