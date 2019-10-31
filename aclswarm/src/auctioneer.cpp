/**
 * @file auctioneer.cpp
 * @brief Distributed task assignment for aclswarm via CBAA
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 26 Oct 2019
 */

#include "aclswarm/auctioneer.h"

namespace acl {
namespace aclswarm {

Auctioneer::Auctioneer(vehidx_t vehid, uint8_t n)
: n_(n), vehid_(vehid)
{
  P_.setIdentity(n_);
  Pt_.setIdentity(n_);
}

// ----------------------------------------------------------------------------

void Auctioneer::setNewAssignmentHandler(
                                std::function<void(const AssignmentPerm&)> f)
{
  fn_assignment_ = f;
}

// ----------------------------------------------------------------------------

void Auctioneer::setSendBidHandler(
                        std::function<void(const Auctioneer::BidConstPtr&)> f)
{
  fn_sendbid_ = f;
}

// ----------------------------------------------------------------------------

void Auctioneer::setFormation(const PtsMat& p, const AdjMat& adjmat, bool reset)
{
  assert(n_ == p.rows());
  assert(n_ == adjmat.rows());

  p_ = p;
  adjmat_ = adjmat;

  constexpr uint32_t diameter = 2; // hardcoded for now...
  cbaa_max_iter_ = n_ * diameter;
}

// ----------------------------------------------------------------------------

void Auctioneer::start(const PtsMat& q)
{
  // Assumption: my internal state has already been restarted

  // store the current state of vehicles in the swarm to be used throughout
  q_ = q;

  //
  // Alignment
  //

  // align current swarm positions to desired formation (using neighbors only)
  // paligned_ = alignFormation(adjmat_, p_, q_);
  paligned_ = p_;

  //
  // Assignment (kick off with an initial bid)
  //

  // Using only knowledgde of my current state and what I think the aligned
  // formation is, make an initial bid for the formation point I am closest to.
  selectTaskAssignment();

  // send my bid to my neighbors
  notifySendBid();
}

// ----------------------------------------------------------------------------

void Auctioneer::receiveBid(const Bid& bid, vehidx_t vehid)
{
  // keep track of all bids across bid iterations
  bids_[bid.iter].insert({vehid, bid});

  // once my neighbors' bids are in, tally them up and decide who the winner is
  if (bidIterComplete()) {

    //
    // Update CBAA
    //

    // update my local understanding of who deserves which task based on the
    // highest bidder of each task, within my neighborhood.
    bool was_outbid = updateTaskAssignment();

    // If I was outbid, I will need to select a new task.
    if (was_outbid) selectTaskAssignment();

    // start the next iteration
    ++biditer_;

    //
    // Determine convergence or continue bidding
    //

    if (auctionComplete()) {
      // TODO:
      notifyNewAssignment();
    } else {
      // send latest bid to my neighbors
      notifySendBid();
    }
  }
}

// ----------------------------------------------------------------------------

bool Auctioneer::auctionComplete()
{
  return biditer_ >= cbaa_max_iter_;
}

// ----------------------------------------------------------------------------

void Auctioneer::reset()
{
  biditer_ = 0;

  // initialize my bid
  bid_->price.clear();
  std::fill_n(std::back_inserter(bid_->price), n_, 0.0);
  bid_->who.clear();
  std::fill_n(std::back_inserter(bid_->who), n_, -1);
  bid_->iter = 0;

  // Create a table to hold my neighbor's bids for each CBAA iteration.
  bids_.clear();
  bids_.resize(cbaa_max_iter_);
}


// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void Auctioneer::notifySendBid()
{
  // let the caller know
  fn_sendbid_(bid_);
}

// ----------------------------------------------------------------------------

void Auctioneer::notifyNewAssignment()
{
  //
  // Extract new assignment
  //



  // let the caller know
  fn_assignment_(P_);
}

// ----------------------------------------------------------------------------

void Auctioneer::alignFormation()
{

}

// ----------------------------------------------------------------------------

bool Auctioneer::bidIterComplete()
{
  bool complete = true;

  // for convenience: my neighbors' bids from this bid iteration
  const auto& bids = bids_[biditer_];

  // work in "formation space" since we are using the adjmat to check nbhrs
  const vehidx_t i = P_.indices()(vehid_);

  for (size_t j=0; j<n_; ++j) {
    if (adjmat_(i, j)) {
      // map back to "vehicle space" since that's how our bids are keyed
      vehidx_t nbhr = Pt_.indices()(j);

      // CBAA iteration is not complete if I am missing any of my nbhrs' bids
      if (bids.find(nbhr) == bids.end()) complete = false;
    }
  }

  return complete;
}

// ----------------------------------------------------------------------------

bool Auctioneer::updateTaskAssignment()
{
  // Given all of the bids from my neighbors (and me), which agent is most
  // deserving of each of the formation points (tasks)? In other words,
  // At this point in the current auction, who currently has the highest bids?
  // n.b., a nbhr might have *info* about who has the highest bid for a given
  // formpt, but it may not be that nbhr---it's just in their local info

  // for convenience: my neighbors' bids from this bid iteration
  auto& bids = bids_[biditer_];
  bool was_outbid = false;

  // add myself so that my local information is considered
  bids.insert({vehid_, *bid_});

  // loop through each task and decide on the winner
  for (size_t j=0; j<n_; ++j) {

    //
    // Which of my nbhrs (or me) bid the most for task / formpt j?
    //

    // arbitrarily assume that the first nhbr in the list has the highest bid
    auto maxit = bids.cbegin();

    // but then loop through each nbhr (and me) and decide who bid the most
    for (auto it = bids.cbegin(); it!=bids.cend(); it++) {
      if (it->second.price[j] > maxit->second.price[j]) maxit = it;
    }

    //
    // Update my local understanding of who has bid the most for each task
    //

    // check if I was outbid by someone else
    if (bid_->who[j] == vehid_ && maxit->second.who[j] != vehid_) was_outbid = true;

    // who should be assigned task j?
    bid_->who[j] = maxit->second.who[j];

    // how much is this winning agent willing to bid on this task?
    bid_->price[j] = maxit->second.price[j];
  }

  // did someone outbid me for my desired formation point / task?
  return was_outbid;
}

// ----------------------------------------------------------------------------

void Auctioneer::selectTaskAssignment()
{
  // Determine the highest price this agent is willing to pay to be assigned
  // a specific task / formpt.
  float max = 0;
  size_t task = 0;
  bool was_assigned = false;
  for (size_t j=0; j<n_; ++j) {
    // n.b., within the same auction, this list of prices will be the same
    const float price = getPrice(q_.row(vehid_), paligned_.row(j));

    // In addition to finding the task that I am most interested in,
    // only bid on a task if I think I will win (highest bidder of my nbhrs)
    if (price > max && price > bid_->price[j]) {
      max = price;
      task = j;
      was_assigned = true;
    }
  }

  // update my local information to reflect my bid
  if (was_assigned) { // TODO: will this always be true?
    bid_->price[task] = max;
    bid_->who[task] = vehid_;
  }
}

// ----------------------------------------------------------------------------

float Auctioneer::getPrice(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
  return 1.0 / ((p1 - p2).norm() + 1e-8);
}

} // ns aclswarm
} // ns acl
