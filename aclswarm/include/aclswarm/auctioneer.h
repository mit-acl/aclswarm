/**
 * @file auctioneer.h
 * @brief Distributed task assignment for aclswarm via CBAA 
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 26 Oct 2019
 */

#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "aclswarm/utils.h"
#include "aclswarm/distcntrl.h"

namespace acl {
namespace aclswarm {

  class Auctioneer
  {
  public:
    struct Bid {
      std::vector<double> price;
      std::vector<int> who;
      int iter;
    };
    using BidConstPtr = std::shared_ptr<const Bid>;

  public:
    Auctioneer(vehidx_t vehid, uint8_t n);
    ~Auctioneer() = default;

    void start(const FormPts& q);

    void tick();

    void setFormation(const std::shared_ptr<DistCntrl::Formation>& f);
    
    bool hasNewAssignment();

    bool wantsToSendBid();
    
    AssignmentPerm getAssignment() const { return P_; }
    AssignmentPerm getInvAssignment() const { return Pt_; }
    const Bid& getBid() const { return bid_; }

  private:
    enum class State { IDLE, AUCTION };
    using BidMap = std::map<vehidx_t, Bid>;

    /// \brief Internal state
    uint8_t n_; ///< number of vehicles in swarm
    vehidx_t vehid_; ///< ID of vehicle (index in veh named list)
    State state_; ///< current state of auctioneer (state machine)
    AssignmentPerm P_; ///< nxn assignment permutation (P: vehid --> formpt)
    AssignmentPerm Pt_; ///< nxn inv assign. permutation (Pt: formpt --> vehid)
    biditer_; ///< current bidding iteration of the CBAA process
    Bid bid_; ///< my current bid, to be sent to others
    std::map<uint32_t, BidMap> bids_; ///< all bids this round
    FormPts p_; ///< the desired formation points
    AdjMat adjmat_; ///< the required formation graph adjacency matrix
    uint32_t cbaa_max_iter_; ///< number of iterations until convergence

    void reset();
    void AlignFormation();
  };

} // ns aclswarm
} // ns acl
