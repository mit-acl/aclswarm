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

  // determine the highest price this agent is willing to pay to be assigned
  // at a specific formation point. Note: real prices are non-negative.
  double max = -1;
  size_t argmax = 0;
  for (size_t j=0; j<n_; ++j) {
    const double price = getPrice(q.row(vehid_), paligned_.row(j));
    if (price > max) {
      max = price;
      argmax = j;
    }
  }
  
  // update the initial bid
  reset();
  bid_.price[argmax] = max;
  bid_.who[argmax] = vehid_;
  bid_.iter = 0;

  state_ = State::AUCTION;
}

// ----------------------------------------------------------------------------

void Auctioneer::tick()
{
  State nextstate = state_;

  if (state_ == State::AUCTION) {

    // do I have everyone's bid for this bidding iteration?
    // if (bidIterComplete())

  }

  // transition to next state
  state_ = nextstate;
}

// ----------------------------------------------------------------------------

void Auctioneer::receiveBid(const Bid& bid, vehidx_t vehid)
{
  // make sure this bid is from the cbaa iteration that I care about
  if (bid.iter != biditer_) ; 

  // update neighbor bid list
  // bids_.insert({vehid, bid});
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void Auctioneer::reset()
{
  biditer_ = 0;

  bid_.price.clear();
  std::fill_n(std::back_inserter(bid_.price), n_, 0.0);

  bid_.who.clear();
  std::fill_n(std::back_inserter(bid_.who), n_, -1);

  bid_.iter = 0;

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

} // ns aclswarm
} // ns acl
