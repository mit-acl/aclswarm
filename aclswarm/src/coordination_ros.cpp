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

  init();

  //
  // Load parameters
  //

  nhp_.param<double>("assignment_dt", assignment_dt_, 0.5);
  nhp_.param<double>("auctioneer_tick_dt", auctioneer_tick_dt_, 0.02);
  nhp_.param<double>("control_dt", control_dt_, 0.05);

  //
  // Timers for assignment and distributed control tasks
  //

  ros::NodeHandle nhQ(nh_);
  nhQ.setCallbackQueue(&task_queue_);

  tim_auctioneertick_ = nhQ.createTimer(ros::Duration(auctioneer_tick_dt_),
                                    &CoordinationROS::auctioneertickCb, this);
  tim_auctioneertick_.stop();
  tim_control_ = nhQ.createTimer(ros::Duration(control_dt_),
                                            &CoordinationROS::controlCb, this);
  tim_control_.stop();

  //
  // ROS pub/sub communication
  //

  // subscriber using the default global callback queue
  sub_formation_ = nh_.subscribe("/formation", 10, // don't miss a msg
                                    &CoordinationROS::formationCb, this);

  sub_tracker_ = nhQ.subscribe("vehicle_estimates", 1,
                                    &CoordinationROS::vehicleTrackerCb, this);

  pub_distcmd_ = nhQ.advertise<geometry_msgs::Vector3Stamped>("distcmd", 1);
  pub_assignment_ = nhQ.advertise<std_msgs::UInt8MultiArray>("assignment", 1);
  pub_cbaabid_ = nhQ.advertise<aclswarm_msgs::CBAA>("cbaabid", 1);

  // Create a pool of threads to handle the task queue.
  // This prevent timer tasks (and others) from blocking each other
  constexpr int NUM_TASKS = 3; // there are only two timers + normal pub/sub
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

    if (formation_received_) {
      // stop tasks
      tim_control_.stop();
      tim_auctioneertick_.stop();

      // We only need to solve gains if they were not already provided
      if (formation_->gains.size() == 0) {
        // solve for gains
        ROS_ERROR("Online gain design not yet implemented.");
      }

      // let the controller know about the new formation
      ROS_INFO_STREAM("Changing formation to: \033[34;1m"
                        << formation_->name << "\033[0m");
      controller_->set_formation(formation_);

      // find a reassignment for the new formation
      auctioneer_->setFormation(formation_->qdes, formation_->adjmat);
      startAuction();

      // wait for CBAA to converge so that we get a good assignment
      waitForNewAssignment();

      // allow downstream tasks to continue
      tim_control_.start();
      tim_auctioneertick_.start();
      formation_received_ = false;
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
                            &CoordinationROS::sendBidCb, this, ph::_1));

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
  if (msg->gains.layout.dim.size() == 2) {
    formation_->gains = utils::decodeGainMat(msg->gains);
  } else {
    formation_->gains = Eigen::MatrixXd();
  }

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

void CoordinationROS::cbaabidCb(const aclswarm_msgs::CBAAConstPtr& msg, int vehid)
{
  Auctioneer::Bid bid;
  bid.price = msg->price;
  bid.who = msg->who;
  bid.iter = msg->iter;
  auctioneer_->receiveBid(bid, vehid);
}

// ----------------------------------------------------------------------------

void CoordinationROS::newAssignmentCb(const AssignmentPerm& P)
{
  // let distributed controller know
  controller_->set_assignment(P);

  // change the communication graph accordingly
  connectToNeighbors();

  // publish
  // pub_assignment_.publish();
}

// ----------------------------------------------------------------------------

void CoordinationROS::sendBidCb(const Auctioneer::BidConstPtr& bid)
{
  aclswarm_msgs::CBAA msg;
  msg.header.stamp = ros::Time::now();
  msg.price = bid->price;
  msg.who = bid->who;
  msg.iter = bid->iter;
  pub_cbaabid_.publish(msg);
}

// ----------------------------------------------------------------------------

void CoordinationROS::auctioneertickCb(const ros::TimerEvent& event)
{
  // Assumption: the auction period is sufficiently long so that the comm graph
  //  is correctly setup from the assignment of the last auction. Otherwise,
  //  some messages may be lost during the communication setup.

  // Challenge: what if the operator requests a new formation right after an
  //  auction, during a communication graph setup? What saves us is the delay
  //  in startAuction(), which is *hopefully* long enough to accommodate both
  //  "slow" vehicles and comm graph setup time.
  // Solution: A real solution would probably involve a condvar

  // determine if a new assignment should be sought for.
  // if (time_to_perform_assignment) startAuction();
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

      constexpr int Qsize = 20; // we don't want to loose any of these
      vehsubs_[j_vehid] = nh_.subscribe("/" + ns + "/cbaabid", Qsize, cb);
    } else {
      // if a subscriber exists, break communication
      if (vehsubs_.find(j_vehid) != vehsubs_.end()) vehsubs_[j_vehid].shutdown();
    }
  }
}

// ----------------------------------------------------------------------------

void CoordinationROS::startAuction()
{
  // Assumption: Each vehicle is told to start an auction at the same time.
  // Challenge: Clearly, there will be jitter across all the vehicles. This
  //  jitter may some vehicles to start a new auction and send their initial
  //  before other vehicles (i.e., their nbrs) have had a chance to properly
  //  reset. As a result, if I am a slow vehicle, then I will miss my fast
  //  nbr's initial bid and will be stuck waiting for it forever.
  // Solution: To avoid this, we tell each vehicle to wait for some time
  //  after reseting its receive buffers and before sending its initial bid.

  // TODO: Think about an async solution to this

  auctioneer_->reset();

  // n.b., this sleep is okay since this thread is only handling the
  // formation callback (which should have a queue to make sure no vehicle
  // drops a msg while all the other swarm vehicles move on)
  ros::Duration(0.5).sleep();

  auctioneer_->start(q_);
}

// ----------------------------------------------------------------------------

void CoordinationROS::waitForNewAssignment()
{
  constexpr double POLL_PERIOD = 0.1;
  while (!auctioneer_->auctionComplete()) {
    ros::Duration(POLL_PERIOD).sleep();
  }
}

} // ms aclswarm
} // ns acl
