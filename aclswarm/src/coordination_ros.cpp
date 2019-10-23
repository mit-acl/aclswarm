/**
 * @file coordination_ros.cpp
 * @brief ROS wrapper for coordination components of aclswarm
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 2019
 */

#include <eigen_conversions/eigen_msg.h>

#include "aclswarm/coordination_ros.h"
#include "aclswarm/utils.h"

namespace acl {
namespace aclswarm {

CoordinationROS::CoordinationROS(const ros::NodeHandle nh,
                                  const ros::NodeHandle nhp)
: nh_(nh), nhp_(nhp), formation_(new DistCntrl::Formation)
{
  if (!utils::loadVehicleInfo(vehname_, vehid_, vehs_)) {
    ros::shutdown();
    return;
  }

  // number of vehicles in swarm
  n_ = vehs_.size();

  // initialize vehicle positions and my velocity
  formation_->qdes = PtsMat::Zero(n_, 3);
  q_ = PtsMat::Zero(n_, 3);
  vel_ = Eigen::Vector3d::Zero();

  initializeModules();

  //
  // Load parameters
  //

  nhp_.param<double>("assignment_dt", assignment_dt_, 0.5);
  nhp_.param<double>("control_dt", control_dt_, 0.05);

  //
  // Timers for assignment and distributed control tasks
  //

  ros::NodeHandle nhQ(nh_);
  nhQ.setCallbackQueue(&task_queue_);

  tim_assignment_ = nhQ.createTimer(ros::Duration(assignment_dt_),
                                            &CoordinationROS::assignCb, this);
  tim_control_ = nhQ.createTimer(ros::Duration(control_dt_),
                                            &CoordinationROS::controlCb, this);

  //
  // ROS pub/sub communication
  //

  // subscriber using the default global callback queue
  sub_formation_ = nh_.subscribe("formation", 1,
                                    &CoordinationROS::formationCb, this);

  sub_tracker_ = nhQ.subscribe("vehicle_estimates", 1,
                                    &CoordinationROS::vehicleTrackerCb, this);

  pub_distcmd_ = nhQ.advertise<geometry_msgs::Vector3Stamped>("distcmd", 1);

  // Create a pool of threads to handle the task queue.
  // This prevent timer tasks (and others) from blocking each other
  constexpr int NUM_TASKS = 3; // there are only two timers + normal pub/sub
  spinner_ = std::unique_ptr<ros::AsyncSpinner>(
                            new ros::AsyncSpinner(NUM_TASKS, &task_queue_));

}

// ----------------------------------------------------------------------------

void CoordinationROS::spin()
{
  // We do not expect formations to be sent that frequently, so slow check.
  ros::Rate r(5);
  while (ros::ok()) {

    if (formation_received_) {
      // stop tasks
      tim_control_.stop();
      tim_assignment_.stop();

      // We only need to solve gains if they were not already provided
      if (formation_->gains.size() == 0) {
        // solve for gains
      }

      // let the controller know about the new formation
      controller_->set_formation(formation_);

      // allow downstream tasks to continue
      tim_control_.start();
      tim_assignment_.start();
      formation_received_ = false;
    }

    ros::spinOnce();
    r.sleep();
  }
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void CoordinationROS::initializeModules()
{
  //
  // Instantiate module objects for tasks
  //

  controller_.reset(new DistCntrl(vehid_, n_));
  assignment_.reset(new Assignment());

  //
  // Distributed Control
  //

  double K, kp, kd;
  nhp_.param<double>("cntrl/K", K, 4.0);
  nhp_.param<double>("cntrl/kp", kp, 1.0);
  nhp_.param<double>("cntrl/kd", kd, 0.0);

  controller_->set_gains(K, kp, kd);
}

// ----------------------------------------------------------------------------

void CoordinationROS::formationCb(const aclswarm_msgs::FormationConstPtr& msg)
{
  // keep track of the current formation graph
  formation_->adjmat = utils::decodeAdjMat(msg->adjmat);
  assert(n_ == formation_->adjmat.rows());

  // store the desired formation points
  for (size_t i=0; i<n_; ++i) {
    Eigen::Vector3d qrow;
    tf::pointMsgToEigen(msg->points[i], qrow);
    formation_->qdes.row(i) = qrow;
  }

  // if no gains are sent, this will be empty---causing the solver to run
  formation_->gains = utils::decodeGainMat(msg->gains);

  formation_->name = msg->name;
  formation_received_ = true;
}

// ----------------------------------------------------------------------------

void CoordinationROS::vehicleTrackerCb(
                  const aclswarm_msgs::VehicleEstimatesConstPtr& msg)
{
  assert(n_ == msg->positions.size());

  for (size_t i=0; i<n_; ++i) {
    Eigen::Vector3d qrow;
    tf::pointMsgToEigen(msg->positions[i].point, qrow);
    q_.row(i) = qrow;
  }
}

// ----------------------------------------------------------------------------

void CoordinationROS::stateCb(const acl_msgs::StateConstPtr& msg)
{
  // we just need our velocity for damping in the controller
  tf::vectorMsgToEigen(msg->vel, vel_);
}

// ----------------------------------------------------------------------------

void CoordinationROS::assignCb(const ros::TimerEvent& event)
{
  // assignment_->tick();

  // if (assignment_->hasNewAssignment()) {
  //   // publish
  // }
}

// ----------------------------------------------------------------------------

void CoordinationROS::controlCb(const ros::TimerEvent& event)
{
  const Eigen::Vector3d u = controller_->compute(q_, vel_);

  geometry_msgs::Vector3Stamped msg;
  msg.header.stamp = ros::Time::now();
  tf::vectorEigenToMsg(u, msg.vector);
  pub_distcmd_.publish(msg);
}

} // ms aclswarm
} // ns acl
