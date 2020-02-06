/**
 * @file safety.cpp
 * @brief Ensures autopilot commands are safe
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 2019
 */

#include "aclswarm/safety.h"

#include <eigen_conversions/eigen_msg.h>

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
  nhp_.param<double>("max_vel_z", max_vel_z_, 0.3);
  nhp_.param<double>("d_avoid_thresh", d_avoid_thresh_, 1.5);
  nhp_.param<double>("r_keep_out", r_keep_out_, 1.2);

  nhp_.param<bool>("leader", leader_, false);
  nhp_.param<double>("joy/kx", joy_kx_, 2.0);
  nhp_.param<double>("joy/ky", joy_ky_, 2.0);
  nhp_.param<double>("joy/kz", joy_kz_, 0.5);
  nhp_.param<double>("joy/kr", joy_kr_, 2.0);

  //
  // Timers
  //

  tim_control_ = nh_.createTimer(ros::Duration(control_dt_),
                                                &Safety::controlCb, this);

  //
  // ROS pub/sub communication
  //

  if (leader_) sub_joy_ = nh_.subscribe("joy", 1, &Safety::joyCb, this);
  sub_fmode_ = nh_.subscribe("/globalflightmode", 1, &Safety::flightmodeCb, this);
  sub_cmdin_ = nh_.subscribe("distcmd", 1, &Safety::cmdinCb, this);
  sub_state_ = nh_.subscribe("state", 1, &Safety::stateCb, this);
  sub_tracker_ = nh_.subscribe("vehicle_estimates", 1,
                                &Safety::vehicleTrackerCb, this);

  pub_cmdout_ = nh_.advertise<snapstack_msgs::QuadGoal>("goal", 1);
  pub_status_ = nhp_.advertise<aclswarm_msgs::SafetyStatus>("status", 1);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void Safety::init()
{
  q_ = PtsMat::Zero(vehs_.size(), 3);

  // initialize goal signals to mux
  goals_.insert(std::make_pair(GoalSrc::DIST, VelocityGoal()));
  goals_.insert(std::make_pair(GoalSrc::JOY, VelocityGoal()));

  // setup priorities
  goals_[GoalSrc::DIST].priority = 0;
  goals_[GoalSrc::JOY].priority = 1;
}

// ----------------------------------------------------------------------------

void Safety::flightmodeCb(const snapstack_msgs::QuadFlightModeConstPtr& msg)
{
  // handle safety state machine transitions based on
  // global flight modes dispatched by the operator.

  if (mode_ == Mode::NOT_FLYING &&
        msg->mode == snapstack_msgs::QuadFlightMode::GO) {
    mode_ = Mode::TAKEOFF;
    ROS_INFO("Spinning up motors for takeoff...");

  } else if ((mode_ == Mode::TAKEOFF || mode_ == Mode::FLYING) &&
        msg->mode == snapstack_msgs::QuadFlightMode::LAND) {
    mode_ = Mode::LANDING;
    ROS_INFO("Landing...");

  } else if (msg->mode == snapstack_msgs::QuadFlightMode::KILL) {
    mode_ = Mode::NOT_FLYING;
    ROS_WARN("Killing!");

  }
}

// ----------------------------------------------------------------------------

void Safety::stateCb(const snapstack_msgs::StateConstPtr& msg)
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

void Safety::vehicleTrackerCb(const aclswarm_msgs::VehicleEstimatesConstPtr& msg)
{
  assert(vehs_.size() == msg->positions.size());

  for (size_t i=0; i<msg->positions.size(); ++i) {
    Eigen::Vector3d qrow;
    tf::pointMsgToEigen(msg->positions[i].point, qrow);
    q_.row(i) = qrow;
  }
}

// ----------------------------------------------------------------------------

void Safety::joyCb(const sensor_msgs::JoyConstPtr& msg)
{
  static constexpr size_t LEFT_X = 0;
  static constexpr size_t LEFT_Y = 1;
  static constexpr size_t RIGHT_X = 3;
  static constexpr size_t RIGHT_Y = 4;

  // for convenience
  auto& goal = goals_[GoalSrc::JOY];

  goal.active = true;
  goal.stamp_s = msg->header.stamp.toSec();
  goal.vx = msg->axes[RIGHT_Y] * joy_kx_;
  goal.vy = msg->axes[RIGHT_X] * joy_ky_;
  goal.vz = msg->axes[LEFT_Y] * joy_kz_;
  goal.r = msg->axes[LEFT_X] * joy_kr_;
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

  // saturate vertical velocity (keeping the same direction)
  double velz = std::abs(goal.vz);
  if (velz > max_vel_z_) {
    goal.vz = goal.vz/velz * max_vel_z_;
  }
}

// ----------------------------------------------------------------------------

void Safety::controlCb(const ros::TimerEvent& event)
{
  static aclswarm_msgs::SafetyStatus statusmsg;
  static snapstack_msgs::QuadGoal goalmsg;
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
      statusmsg.collision_avoidance_active = false;

      // There is no real velocity control, since the ACL outer loop tracks
      // trajectories and their derivatives. To achieve velocity control,
      // we will integrate the velocity commands to obtain the trajectory.
      goalmsg.xy_mode = snapstack_msgs::QuadGoal::MODE_POS;
      goalmsg.z_mode = snapstack_msgs::QuadGoal::MODE_POS;

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
        // TODO: does this make sense for joy override?
        const double dt = control_dt_;
        // const double dt = g.stamp_s - last_active_goal_time;

        collisionAvoidance(g);

        // notify others if collision avoidance mode is active
        statusmsg.collision_avoidance_active = g.modified;

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

    statusmsg.collision_avoidance_active = false;
    goalmsg.vel.x = goalmsg.vel.y = goalmsg.vel.z = 0;
    goalmsg.dyaw = 0;

    // TODO: allow velocity tweaks from JOY

    constexpr double LANDING_THRESHOLD = 0.005;
    if ((pose_.pose.position.z - initial_alt) < LANDING_THRESHOLD) {
      mode_ = Mode::NOT_FLYING;
      ROS_INFO("Landing complete!");
    } else {
      // choose between fast landing and slow landing
      const double landing_fast_th = landing_fast_threshold_
                                      + ((takeoff_rel_) ? initial_alt : 0.0);
      const double dec = (pose_.pose.position.z > landing_fast_th) ?
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

  statusmsg.header.stamp = goalmsg.header.stamp;
  pub_status_.publish(statusmsg);
}

// ----------------------------------------------------------------------------

void Safety::makeSafeTraj(double dt, const VelocityGoal& g,
                          snapstack_msgs::QuadGoal& goal)
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

// ----------------------------------------------------------------------------

void Safety::collisionAvoidance(VelocityGoal& goal)
{
  bool didWrap = false;
  std::vector<std::pair<double, int>> edges;

  // Identify the edges of the "no-fly zones"---sectors describing
  // velocity obstacles.
  for (size_t j=0; j<q_.rows(); ++j) {
    if (vehid_ == j) continue;

    // calculate the relative translation btwn me and this other vehicle
    // n.b., this is in "vehicle space".
    const Eigen::Vector3d qij = q_.row(j) - q_.row(vehid_);

    // check if we are even close enough to start worrying about collision
    if (qij.head<2>().norm() > d_avoid_thresh_) continue;

    //
    // Calculate the "no-fly zones" (the 'pizza slices' in polar coordinates)
    //

    // calculate angle between world-x and this obstacle
    double theta = std::atan2(qij.y(), qij.x());

    // half-angle of pizza slice / sector / velocity obstacle
    double alpha = std::abs(std::asin(std::min(1.0, r_keep_out_/qij.head<2>().norm())));

    // beginning and ending angles of the tangent lines (on edge of sector)
    const double beg = utils::wrapToPi(theta-alpha);
    const double end = utils::wrapToPi(theta+alpha);

    // start edges are denoted with +1, stop edges with -1.
    // Note that this pairing (beg,+1);(end,-1) is essential for proper merging
    edges.push_back({beg, +1});
    edges.push_back({end, -1});

    // did we cross the pi/-pi boundary? if so, break this sector into
    // two sectors at pi/-pi---these edges will be removed later.
    if (beg > end) {
      didWrap = true;
      edges.push_back({-M_PI, +1}); // sector: [-pi, end] //< first after sort
      edges.push_back({ M_PI, -1}); // sector: [beg,  pi]
    }
  }

  // if there is no risk of collision, any command is safe.
  if (edges.size() == 0) return;

  //
  // Take the union of all half-sectors (i.e., if they are overlapping)
  //

  // sort the edges in ascending order by angle
  std::sort(edges.begin(), edges.end());

  // the final no-fly zones are created by counting edges from above.
  // Similar to how a parser counts parentheses to ensure a valid expression.
  int count = 0;
  double start;
  std::vector<std::pair<double, double>> nfzones;
  for (const auto& e : edges) {
    // if starting a new zone
    if (count == 0) start = e.first;

    count += e.second;

    // if zone has ended
    if (count == 0) nfzones.push_back({start, e.first});
  }

  //
  // Check if the desired velocity goal is safe
  //

  double psi = std::atan2(goal.vy, goal.vx);
  bool safe = true;
  for (const auto& z : nfzones) {
    if (psi > z.first && psi < z.second) {
      safe = false;
      break;
    }
  }

  // if the desired velocity goal is outside of all no-fly zones, we are done
  if (safe) return;

  //
  // Find the closest safe direction
  //

  // notify others that this goal was modified by collision avoidance
  goal.modified = true;

  // The nearest safe direction is an edge. Flatten zones into edges only.
  // If we wrapped, then pi/-pi edges (artificially inserted or not) are unsafe
  std::vector<double> nfzedges;
  for (const auto& z : nfzones) {
    if (!didWrap || std::abs(z.first) != static_cast<double>(M_PI))
      nfzedges.push_back(z.first);
    if (!didWrap || std::abs(z.second) != static_cast<double>(M_PI))
      nfzedges.push_back(z.second);
  }

  // if we are surrounded, there are no safe edges and we must surrender.
  if (nfzedges.size() == 0) {
    goal.vx = goal.vy = 0;
    goal.vz = 0;
    return;
  }

  // sort the edges in ascending order
  std::sort(nfzedges.begin(), nfzedges.end());

  // find the closest edge angle to the desired direction
  const size_t closestIdx = utils::closest(nfzedges, psi);
  const double closestEdge = nfzedges[closestIdx];

  // for formation control, we can guarantee convergence if the actual velocity
  // is in the half-plane of the commanded velocity. We leverage this here.
  if (std::abs(utils::wrapToPi(closestEdge - psi)) <= M_PI/2) {
    const double umag = std::sqrt(goal.vx*goal.vx + goal.vy*goal.vy);
    goal.vx = umag*std::cos(closestEdge);
    goal.vy = umag*std::sin(closestEdge);
    return;
  }

  // otherwise, nothing we do is safe. Just stop.
  goal.vx = goal.vy = 0;
  goal.vz = 0;
}

} // ns aclswarm
} // ns acl
