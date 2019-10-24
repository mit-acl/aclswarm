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
  nhp_.param<double>("max_accel_xy", max_accel_xy_, 0.5);
  nhp_.param<double>("max_accel_z", max_accel_z_, 0.8);

  nhp_.param<double>("max_vel_xy", max_vel_xy_, 0.5);
  
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
  goals_.insert(std::make_pair(GoalSrc::DIST, VelocityGoal()));
  goals_.insert(std::make_pair(GoalSrc::JOY, VelocityGoal()));

  // setup priorities
  goals_[GoalSrc::DIST].priority = 0;
  goals_[GoalSrc::JOY].priority = 1;
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
  auto& goal = goals_[GoalSrc::DIST];

  // update the velocity goal
  goal.active = true;
  goal.stamp_s = msg->header.stamp.toSec();
  goal.vx = msg->vector.x;
  goal.vy = msg->vector.y;
  goal.vz = msg->vector.z;
  goal.r = 0;

  // saturate planar velocities (keeping the same direction)
  double velxy = std::sqrt(goal.vx*goal.vx + goal.vy*goal.vy);
  if (velxy > max_vel_xy_) {
    goal.vx  = goal.vx/velxy * max_vel_xy_;
    goal.vy  = goal.vy/velxy * max_vel_xy_;
  }
}

// ----------------------------------------------------------------------------

void Safety::controlCb(const ros::TimerEvent& event)
{
  static acl_msgs::QuadGoal goalmsg;
  static bool flight_initialized = false;
  static ros::Time takeoff_time;
  static double last_active_goal_time;
  static double takeoff_alt;
  static double initial_alt;

  if (mode_ == Mode::TAKEOFF) {

    if (!flight_initialized) {
      // capture the initial time
      takeoff_time = ros::Time::now();

      // set the goal to our current position + yaw
      goalmsg.pos.x = pose_.pose.position.x;
      goalmsg.pos.y = pose_.pose.position.y;
      goalmsg.pos.z = pose_.pose.position.z;
      goalmsg.vel.x = 0;
      goalmsg.vel.y = 0;
      goalmsg.vel.z = 0;
      goalmsg.yaw = tf2::getYaw(pose_.pose.orientation);
      goalmsg.dyaw = 0;

      // There is no real velocity control, since the ACL outer loop tracks
      // trajectories and their derivatives. To achieve velocity control,
      // we will integrate the velocity commands to obtain the trajectory.
      goalmsg.xy_mode = acl_msgs::QuadGoal::MODE_POS;
      goalmsg.z_mode = acl_msgs::QuadGoal::MODE_POS;

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

    // unpack goal signals and sort from highest priority to lowest
    std::vector<VelocityGoal> goals;
    utils::mapToVec(goals_, goals);
    std::sort(goals.begin(), goals.end(), std::greater<VelocityGoal>());

    // Find the highest priority active goal signal
    for (auto&& g : goals) {
      if (g.active) {
        const double dt = g.stamp_s - last_active_goal_time;

        // Use this velocity goal to make a safe pos+vel trajectory goal
        makeSafeTraj(dt, g, goalmsg);

        // mark this goal as used
        g.active = false;
        last_active_goal_time = g.stamp_s;
        break;
      }
    }

    // TODO: Probably add a timeout to then send a hover command
    // If no active goal signal is found, just keep sending the last goal msg

  } else if (mode_ == Mode::LANDING) {

    goalmsg.vel.x = goalmsg.vel.y = goalmsg.vel.z = 0;
    goalmsg.dyaw = 0;

    // TODO: allow velocity tweaks from JOY

    constexpr double LANDING_THRESHOLD = 0.050;
    if (std::abs(initial_alt - pose_.pose.position.z) < LANDING_THRESHOLD) {
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

void Safety::makeSafeTraj(double dt, const VelocityGoal& g,
                          acl_msgs::QuadGoal& goal)
{
  // The following assumes that we are dealing with velocity goals.
  // Using the current goal state, this function generates a position+yaw
  // trajectory by integrating the new desired velocity goal.

  // unpack velocity goals so we can safely integrate them
  auto vx = g.vx;
  auto vy = g.vy;
  auto vz = g.vz;
  auto r = g.r;

  //
  // Initial signal conditioning of velocity goals
  //

  // rate limit the velocities (to indirectly limit accels)
  utils::rateLimit(dt, -max_accel_xy_, max_accel_xy_, goal.vel.x, vx);
  utils::rateLimit(dt, -max_accel_xy_, max_accel_xy_, goal.vel.y, vy);
  utils::rateLimit(dt, -max_accel_z_ , max_accel_z_ , goal.vel.z, vz);

  // TODO: limit yawrate?


  //
  // Generate position goals and ensure room bounds are maintained
  //

  // n.b. this logic does not prevent a discontinuous position waypoint
  // from being generated outside the room, e.g., from a mouse click.
  // This is because we are working with velocity goals, not position goals.

  // with this goal vel, what will my predicted next goal position be?
  // Note: we use predict forward using the goal so that the goal position
  // trajectory is smooth---allowing the outer loop control to deal with
  // actually getting there.
  const double nextx = goal.pos.x + vx * dt;
  const double nexty = goal.pos.y + vy * dt;
  const double nextz = goal.pos.z + vz * dt;

  // If the predicted position is outside the room bounds, only allow
  // movements that move the vehicle back into the room.
  bool xclamped = false, yclamped = false, zclamped = false;
  goal.pos.x = utils::clamp(nextx, std::min(bounds_x_min_, goal.pos.x),
                        std::max(bounds_x_max_, goal.pos.x), xclamped);
  goal.pos.y = utils::clamp(nexty, std::min(bounds_y_min_, goal.pos.y),
                        std::max(bounds_y_max_, goal.pos.y), yclamped);
  goal.pos.z = utils::clamp(nextz, std::min(bounds_z_min_, goal.pos.z),
                        std::max(bounds_z_max_, goal.pos.z), zclamped);

  // if the predicted position is outside the room bounds, zero the velocities.
  if (xclamped) vx = 0;
  if (yclamped) vy = 0;
  if (zclamped) vz = 0;

  // But zero the velocities so that acceleration is bounded
  if (xclamped) utils::rateLimit(dt, -max_accel_xy_, max_accel_xy_, goal.vel.x, vx);
  if (yclamped) utils::rateLimit(dt, -max_accel_xy_, max_accel_xy_, goal.vel.y, vy);
  if (zclamped) utils::rateLimit(dt, -max_accel_z_, max_accel_z_, goal.vel.z, vz);

  // set linear velocity
  goal.vel.x = vx;
  goal.vel.y = vy;
  goal.vel.z = vz;

  //
  // Generate yaw goal
  //

  // with this goal yawrate, what will my predicted next goal yaw be?
  // Note: same as position---we use the goal yaw instead of current yaw.
  const double nextY = goal.yaw + r * dt;
  goal.yaw = utils::wrapToPi(nextY); // doesn't really matter (becomes a quat)

  // set angular rate
  goal.dyaw = r;

}

} // ns aclswarm
} // ns acl
