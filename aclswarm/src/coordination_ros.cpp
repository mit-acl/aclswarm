/**
 * @file coordination_ros.cpp
 * @brief ROS wrapper for coordination components of aclswarm
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 2019
 */

#include "aclswarm/coordination_ros.h"

#include <eigen_conversions/eigen_msg.h>

namespace acl {
namespace aclswarm {

CoordinationROS::CoordinationROS(const ros::NodeHandle nh,
                                  const ros::NodeHandle nhp)
: nh_(nh), nhp_(nhp), formation_(nullptr), newformation_(nullptr)
{
  if (!utils::loadVehicleInfo(vehname_, vehid_, vehs_)) {
    ros::shutdown();
    return;
  }

  // number of vehicles in swarm
  n_ = vehs_.size();

  // initialize vehicle positions and my velocity
  q_ = PtsMat::Zero(n_, 3);
  vel_ = Eigen::Vector3d::Zero();

  init();

  //
  // Load parameters
  //

  nhp_.param<double>("auctioneer_dt", auctioneer_dt_, 0.001);
  nhp_.param<double>("autoauction_dt", autoauction_dt_, 0.2);
  nhp_.param<double>("control_dt", control_dt_, 0.05);

  //
  // Timers for assignment and distributed control tasks
  //

  nhQ_ = ros::NodeHandle(nh_);
  nhQ_.setCallbackQueue(&task_queue_);

  tim_auctioneer_ = nhQ_.createTimer(ros::Duration(auctioneer_dt_),
                                    &CoordinationROS::auctioneerCb, this);
  tim_autoauction_ = nhQ_.createTimer(ros::Duration(autoauction_dt_),
                                    &CoordinationROS::autoauctionCb, this);
  tim_autoauction_.stop();
  tim_control_ = nhQ_.createTimer(ros::Duration(control_dt_),
                                            &CoordinationROS::controlCb, this);
  tim_control_.stop();

  //
  // ROS pub/sub communication
  //

  // subscriber using the default global callback queue
  sub_formation_ = nh_.subscribe("/formation", 10, // don't miss a msg
                                    &CoordinationROS::formationCb, this);

  sub_tracker_ = nhQ_.subscribe("vehicle_estimates", 1,
                                    &CoordinationROS::vehicleTrackerCb, this);

  pub_distcmd_ = nhQ_.advertise<geometry_msgs::Vector3Stamped>("distcmd", 1);
  pub_assignment_ = nhQ_.advertise<std_msgs::UInt8MultiArray>("assignment", 1);
  pub_cbaabid_ = nhQ_.advertise<aclswarm_msgs::CBAA>("cbaabid", 1);

  // Create a pool of threads to handle the task queue.
  // This prevent timer tasks (and others) from blocking each other
  constexpr int NUM_TASKS = 1; // there are only two timers + normal pub/sub
  spinner_ = std::unique_ptr<ros::AsyncSpinner>(
                            new ros::AsyncSpinner(NUM_TASKS, &task_queue_));
  spinner_->start();
}

// ----------------------------------------------------------------------------

void CoordinationROS::spin()
{
  // We do not expect formations to be sent that frequently, so slow check.
  ros::Rate r(5);
  while (ros::ok()) {

    if (newformation_ != nullptr /*&& auctioneer_->isIdle()*/) {
      // stop tasks
      tim_control_.stop();
      tim_autoauction_.stop();

      // sends zero commands
      sendZeroControl();

      // commit to the new formation
      formation_ = newformation_;

      // We only need to solve gains if they were not already provided
      if (formation_->gains.size() == 0) {
        // solve for gains
        ROS_ERROR("Online gain design not yet implemented.");
      }

      // let the controller know about the new formation
      ROS_INFO_STREAM("Changing formation to: \033[34;1m"
                        << formation_->name << "\033[0m");
      controller_->setFormation(formation_);

      if (!auctioneer_->isIdle()) ROS_ERROR("FYI, hard resetting auctioneer");

      // setup the parameters for this new formation
      auctioneer_->setFormation(formation_->qdes, formation_->adjmat);

      // FYI: We assume that our communication graph is identical to the
      // formation graph. Make sure that we can talk to our neighbors as
      // defined by the adjmat of the formation.
      connectToNeighbors();

      // Assumption: Each vehicle is told to start an auction at the same time.
      // Challenge: Clearly, there will be jitter across all the vehicles. This
      //  jitter may cause some vehicles to start a new auction and send their
      //  initial bid before their neighbors have had a chance to setup their
      //  communications. As a result, if I am a slow vehicle, then I will miss
      //  my fast nbr's first bid and will be stuck waiting for them forever.
      // Hack solution: Wait for a duration longer than the expected jitter.

      // n.b., this sleep is okay since this thread is only handling the
      // formation callback (which should have a queue to make sure no vehicle
      // drops a msg while all the other swarm vehicles move on)
      ros::Duration(0.5).sleep(); // this hack is so annoying

      // Now that we have neighbors to talk to, let the bidding begin.
      auctioneer_->start(q_);

      // wait for CBAA to converge so that we get a good assignment
      waitForNewAssignment();

      // allow downstream tasks to continue
      tim_control_.start();
      tim_autoauction_.start();
      newformation_ = nullptr;
    }

    ros::spinOnce();
    r.sleep();
  }
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void CoordinationROS::init()
{
  //
  // Instantiate module objects for tasks
  //

  controller_.reset(new DistCntrl(vehid_, n_));
  auctioneer_.reset(new Auctioneer(vehid_, n_));

  //
  // Auctioneer Callbacks
  //

  namespace ph = std::placeholders;
  auctioneer_->setNewAssignmentHandler(std::bind(
                            &CoordinationROS::newAssignmentCb, this, ph::_1));
  auctioneer_->setSendBidHandler(std::bind(
                          &CoordinationROS::sendBidCb, this,
                          ph::_1, ph::_2, ph::_3));

  //
  // Distributed Control
  //

  double K, kp, kd;
  nhp_.param<double>("cntrl/K", K, 4.0);
  nhp_.param<double>("cntrl/kp", kp, 1.0);
  nhp_.param<double>("cntrl/kd", kd, 0.0);

  controller_->setGains(K, kp, kd);
}

// ----------------------------------------------------------------------------

void CoordinationROS::formationCb(const aclswarm_msgs::FormationConstPtr& msg)
{
  // create a new formation corresponding to the newly received formation
  newformation_.reset(new DistCntrl::Formation);

  // keep track of the current formation graph
  newformation_->adjmat = utils::decodeAdjMat(msg->adjmat);
  assert(n_ == newformation_->adjmat.rows());

  // store the desired formation points
  newformation_->qdes = PtsMat::Zero(n_, 3);
  for (size_t i=0; i<n_; ++i) {
    Eigen::Vector3d qrow;
    tf::pointMsgToEigen(msg->points[i], qrow);
    newformation_->qdes.row(i) = qrow;
  }

  // if no gains are sent, this will be empty---causing the solver to run
  if (msg->gains.layout.dim.size() == 2) {
    newformation_->gains = utils::decodeGainMat(msg->gains);
  } else {
    newformation_->gains = Eigen::MatrixXd();
  }

  newformation_->name = msg->name;
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

void CoordinationROS::cbaabidCb(const aclswarm_msgs::CBAAConstPtr& msg, int vehid)
{
  Auctioneer::Bid bid;
  bid.price = msg->price;
  bid.who = msg->who;
  auctioneer_->enqueueBid(vehid, msg->auctionId, msg->iter, bid);
}

// ----------------------------------------------------------------------------

void CoordinationROS::newAssignmentCb(const AssignmentPerm& P)
{
  // let distributed controller know
  controller_->setAssignment(P);

  // change the communication graph accordingly
  connectToNeighbors();

  // publish
  std_msgs::UInt8MultiArray msg;
  msg.data = std::vector<uint8_t>(P.indices().data(),
                                  P.indices().data() + P.indices().size());
  pub_assignment_.publish(msg);
}

// ----------------------------------------------------------------------------

void CoordinationROS::sendBidCb(uint32_t auctionid, uint32_t iter,
                                const Auctioneer::BidConstPtr& bid)
{
  aclswarm_msgs::CBAA msg;
  msg.header.stamp = ros::Time::now();
  msg.price = bid->price;
  msg.who = bid->who;
  msg.iter = iter;
  msg.auctionId = auctionid;
  pub_cbaabid_.publish(msg);
}

// ----------------------------------------------------------------------------

void CoordinationROS::autoauctionCb(const ros::TimerEvent& event)
{
  // Assumption: the autoauction period is sufficiently long enough so that
  //  the comms graph has been setup from the last assignment. Otherwise,
  //  some messages may be lost during the communication setup.

  if (auctioneer_->stopTimer_) {
    auctioneer_->stopTimer_ = false;
    tim_autoauction_.stop();
    return;
  }

  // make sure auctioneer is not in the middle of something
  if (!auctioneer_->isIdle()) {
    ROS_ERROR("Auctioneer is busy!");
    tim_autoauction_.stop();
    return;
  }

  auctioneer_->start(q_);
}

// ----------------------------------------------------------------------------

void CoordinationROS::auctioneerCb(const ros::TimerEvent& event)
{
  auctioneer_->tick();
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

// ----------------------------------------------------------------------------

void CoordinationROS::sendZeroControl()
{
  geometry_msgs::Vector3Stamped msg;
  msg.header.stamp = ros::Time::now();
  msg.vector.x = msg.vector.y = msg.vector.z = 0;
  pub_distcmd_.publish(msg);
}

// ----------------------------------------------------------------------------

void CoordinationROS::connectToNeighbors()
{
  // which formation point am I currently assigned to?
  const auto i = auctioneer_->getAssignment().indices()(vehid_);

  // loop through the other formation points in graph
  for (size_t j=0; j<n_; ++j) {
    // which vehicle is at this formation point?
    const auto j_vehid = auctioneer_->getInvAssignment().indices()(j);

    // neighbor check:
    // is there an edge between my formation point and this other one?
    if (formation_->adjmat(i, j)) {
      std::string ns = vehs_[j_vehid];

      // create a closure to pass additional arguments to callback
      boost::function<void(const aclswarm_msgs::CBAAConstPtr&)> cb =
        [=](const aclswarm_msgs::CBAAConstPtr& msg) {
          cbaabidCb(msg, j_vehid);
        };

      // don't subscribe if already subscribed
      if (vehsubs_.find(j_vehid) == vehsubs_.end()) {
        constexpr int Qsize = 10; // we don't want to loose any of these
        vehsubs_[j_vehid] = nhQ_.subscribe("/" + ns + "/cbaabid", Qsize, cb);
      }
    } else {
      // if a subscriber exists, break communication and remove from map
      if (vehsubs_.find(j_vehid) != vehsubs_.end()) {
        vehsubs_[j_vehid].shutdown();
        vehsubs_.erase(j_vehid);
      }
    }
  }
}

// ----------------------------------------------------------------------------

void CoordinationROS::waitForNewAssignment()
{
  constexpr double TIMEOUT_SEC = 1;
  constexpr double POLL_PERIOD = 0.1;
  auto start = ros::Time::now();
  while (ros::ok() && !auctioneer_->isIdle()) {
    ros::Duration(POLL_PERIOD).sleep();

    // if we couldn't come up with an assignment, just bail...
    if ((ros::Time::now() - start).toSec() > TIMEOUT_SEC) {
      ROS_ERROR_STREAM("Assignment auction timed out. Missing: " 
                        << auctioneer_->reportMissing());
      return;
    }
  }
}

} // ms aclswarm
} // ns acl
