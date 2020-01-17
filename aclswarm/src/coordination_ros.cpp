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
: nh_(nh), nhp_(nhp), formation_(nullptr), newformation_(nullptr),
  central_assignment_rcvd_(false)
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

  nhp_.param<double>("form_settle_time", form_settle_time_, 1.5);
  nhp_.param<double>("auctioneer_dt", auctioneer_dt_, 0.001);
  nhp_.param<double>("autoauction_dt", autoauction_dt_, 0.2);
  nhp_.param<double>("control_dt", control_dt_, 0.05);
  nhp_.param<bool>("use_assignment", use_assignment_, true);

  if (!use_assignment_) ROS_ERROR("Not using auctioneer");

  nh_.param<bool>("/operator/central_assignment", central_assignment_, false);
  if (central_assignment_) {
    ROS_ERROR("Expecting centralized assignment. Cheater!");
    sub_central_assignment_ = nh_.subscribe("/central_assignment", 1,
                                &CoordinationROS::centralAssignmentCb, this);
  }

  //
  // Timers for assignment and distributed control tasks
  //

  nhQ_ = ros::NodeHandle(nh_);
  nhQ_.setCallbackQueue(&task_queue_);

  tim_auctioneer_ = nhQ_.createTimer(ros::Duration(auctioneer_dt_),
                                    &CoordinationROS::auctioneerCb, this);
  tim_autoauction_ = nhQ_.createTimer(ros::Duration(auctioneer_dt_),
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

    if (newformation_ != nullptr) {
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
        auto timestart = ros::Time::now();
        formation_->gains = admm_->calculateFormationGains(formation_->qdes,
                                                          formation_->adjmat);
        ROS_INFO_STREAM("Generated gains in " <<
                          (ros::Time::now() - timestart).toSec() << " secs.");
      }

      // let the controller know about the new formation
      ROS_INFO_STREAM("Changing formation to: \033[34;1m"
                        << formation_->name << "\033[0m");
      controller_->setFormation(formation_);

      if (!auctioneer_->isIdle()) ROS_WARN("Interrupting current auction");

      // setup the parameters for this new formation
      auctioneer_->setFormation(formation_->qdes, formation_->adjmat);

      // FYI: We assume that our communication graph is identical to the
      // formation graph. Make sure that we can talk to our neighbors as
      // defined by the adjmat of the formation.
      connectToNeighbors();

      // indicate that this is the first assignment of the new formation.
      // When this is true, the control timer is started, but only after
      // a valid assignment.
      first_assignment_ = true;

      // calculate the time remaining for formation setup
      // (i.e., when to start the next assignment)
      startauction_ = (formationsent_ + ros::Duration(form_settle_time_));

      if (use_assignment_) {
        // manage when auctions should be initiated
        tim_autoauction_.start();
      } else {
        // just skip the auctioneer and set an identity assignment
        AssignmentPerm Pident;
        Pident.setIdentity(n_);
        newAssignmentCb(Pident);
      }
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

  bool verbose;
  nhp_.param<bool>("verbose", verbose, false);

  //
  // Instantiate module objects for tasks
  //

  admm_.reset(new ADMM(n_));
  controller_.reset(new DistCntrl(vehid_, n_));
  auctioneer_.reset(new Auctioneer(vehid_, n_, verbose));

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

  formationsent_ = msg->header.stamp;
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

void CoordinationROS::centralAssignmentCb(const std_msgs::UInt8MultiArrayConstPtr& msg)
{
  // unpack permutation vector
  Pcentral_ = AssignmentPerm(Eigen::Map<const AssignmentVec>(msg->data.data(),
                                                          msg->data.size()));

  bool assignment_changed = !(Pcentral_.indices().isApprox(auctioneer_->getAssignment().indices()));
  if (first_assignment_ || assignment_changed) central_assignment_rcvd_ = true;
}

// ----------------------------------------------------------------------------

void CoordinationROS::newAssignmentCb(const AssignmentPerm& P)
{
  // let distributed controller know
  controller_->setAssignment(P);

  // change the communication graph accordingly
  // Assumption: autoauction period is long enough to let comm changes settle
  connectToNeighbors();

  // publish
  std_msgs::UInt8MultiArray msg;
  msg.data = std::vector<uint8_t>(P.indices().data(),
                                  P.indices().data() + P.indices().size());
  pub_assignment_.publish(msg);

  // if this is the first valid assignment of the new formation, start ctrl
  if (first_assignment_) {
    first_assignment_ = false;
    tim_control_.start();
  }
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
  // are we ready to start a new auction?
  if (ros::Time::now() < startauction_) return;

  // set the time of the next autoauction
  startauction_ = ros::Time::now() + ros::Duration(autoauction_dt_);

  if (central_assignment_) {

    // this mode is to receive a global swarm assignment from a centralized
    // coordinator. It is for comparing the distributed assignment method.
    // Use this assignment as if the auctioneer had decided it.
    if (central_assignment_rcvd_) {
      auctioneer_->setAssignment(Pcentral_);
      newAssignmentCb(Pcentral_);
      central_assignment_rcvd_ = false;
    }

    // no need to start the auctioneer since we will not perform CBAA
    return;
  }

  if (auctioneer_->didConvergeOnInvalidAssignment()) {
    // since invalid auctions are reached in consensus, this is synchronized.
    // Therefore, it is okay to  skip this auction, because *all* vehicles
    // will be told to skip this auction.
    // Also, since the old/new data in the queue caused this problem, flush.
    auctioneer_->flush();
    return;
  }

  // indicate if auctioneer is in the middle of an auction. If this happens
  // frequently, then autoauction_dt may need to be increased.
  if (!auctioneer_->isIdle()) ROS_WARN("Auctioneer is busy! Restarting.");

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

bool CoordinationROS::connectToNeighbors()
{
  bool was_changed = false; // indicates if nbr was added

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
        constexpr int Qsize = 1; // we don't want to loose any of these
        vehsubs_[j_vehid] = nhQ_.subscribe("/" + ns + "/cbaabid", Qsize, cb);
        was_changed = true;
      }
    } else {
      // if a subscriber exists, break communication and remove from map
      if (vehsubs_.find(j_vehid) != vehsubs_.end()) {
        vehsubs_[j_vehid].shutdown();
        vehsubs_.erase(j_vehid);
      }
    }
  }

  return was_changed;
}

} // ms aclswarm
} // ns acl
