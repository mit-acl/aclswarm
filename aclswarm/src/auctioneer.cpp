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

void Auctioneer::setFormation(const FormPts& p, const AdjMat& adjmat)
{
  assert(n_ == p.rows());
  assert(n_ == adjmat.rows());

  p_ = p;
  adjmat_ = adjmat;

  constexpr uint32_t diameter = 2; // hardcoded for now...
  cbaa_max_iter_ = n_ * diameter;

  // initialize neighbor bid list

  // formation space or vehicle space?
}

// ----------------------------------------------------------------------------

bool Auctioneer::hasNewAssignment()
{
  return false;
}

// ----------------------------------------------------------------------------

bool Auctioneer::wantsToSendBid()
{
  return (state_ == State::AUCTION);
}

// ----------------------------------------------------------------------------

void Auctioneer::start(const FormPts& q)
{
  //
  // Alignment
  //

  // align current swarm positions to desired formation (using neighbors only)
  paligned_ = alignFormation(adjmat_, p_, q);


  //
  // Initial bid
  //

  // Using only knowledgde of my current state and what I think the aligned
  // formation is, make an initial bid for the formation point I am closest to.
  selectTaskAssignment();
  
  // update the initial bid
  reset();
  bid_.price[argmax] = max;
  bid_.who[argmax] = vehid_;
  bid_.iter = 0;

  // state_ = State::AUCTION;
}

// ----------------------------------------------------------------------------

void Auctioneer::tick()
{
  // State nextstate = state_;

  // if (state_ == State::AUCTION) {

  //   // do I have everyone's bid for this bidding iteration?
  //   // if (bidIterComplete())

  // }

  // // transition to next state
  // state_ = nextstate;
}

// ----------------------------------------------------------------------------

void Auctioneer::receiveBid(const Bid& bid, vehidx_t vehid)
{
  // keep track of all bids across bid iterations
  bids_[bid.iter].insert({vehid, bid});

  // once my neighbors' bids are in, tally them up and decide who the winner is
  if (bidIterComplete()) {

    // update my local understanding of who deserves which task based on the
    // highest bidder of each task, within my neighborhood.
    updateTaskAssignment();

    // If I was outbid, I will need to select a new task.
    if (was_outbid) {
      selectTaskAssignment();
    }

    // start the next iteration
    ++biditer_;
  }

  if (auctionComplete()) ;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void Auctioneer::reset()
{
  biditer_ = 0;

  // initialize my bid
  bid_.price.clear();
  std::fill_n(std::back_inserter(bid_.price), n_, 0.0);
  bid_.who.clear();
  std::fill_n(std::back_inserter(bid_.who), n_, -1);
  bid_.iter = 0;

  // Create a table to hold my neighbor's bids for each CBAA iteration.
  bids_.clear();
  bids_.resize(cbaa_max_iter_);

  state_ = State::IDLE;
}

// ----------------------------------------------------------------------------

void Auctioneer::alignFormation()
{

}

// ----------------------------------------------------------------------------

bool Auctioneer::bidIterComplete()
{
  return false;
}

// ----------------------------------------------------------------------------

double Auctioneer::getPrice(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
  return 1.0 / ((p1 - p2).norm() + 1e-8);
}

// ----------------------------------------------------------------------------

void Auctioneer::updateTaskAssignment()
{
  //
  // Update task assignment
  //

  // Given all of the bids from my neighbors (and me), who had the highest bids
  // this iteration and what where those bid amounts (prices)?

  // Update my local understanding with this information.

  // If I was outbid, then I need to select a new task assignment
}

// ----------------------------------------------------------------------------

void Auctioneer::selectTaskAssignment()
{
  // Determine the highest price this agent is willing to pay to be assigned
  // at a specific formation point. Note: real prices are non-negative.
  // Only bid on a task if I think I will win (highest bidder of my neighbors)
  double max = -1;
  size_t argmax = 0;
  for (size_t j=0; j<n_; ++j) {
    // n.b., for a given auction, this list of prices will be constant
    const double price = getPrice(q.row(vehid_), paligned_.row(j));
    if (price > max) {
      max = price;
      argmax = j;
    }
  }
}

// ----------------------------------------------------------------------------

} // ns aclswarm
} // ns acl
