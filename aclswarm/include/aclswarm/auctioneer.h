/**
 * @file auctioneer.h
 * @brief Distributed task assignment for aclswarm via CBAA 
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 26 Oct 2019
 */

#pragma once

#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
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
      std::vector<float> price;
      std::vector<int> who;
    };
    using BidPtr = std::shared_ptr<Bid>;
    using BidConstPtr = std::shared_ptr<const Bid>;

  public:
    Auctioneer(vehidx_t vehid, uint8_t n);
    ~Auctioneer() = default;

    /**
     * @brief      Registers an event handler for when an auction is complete
     *             and the auctioneer wants to notify the caller.
     *
     * @param[in]  f     The function to call
     */
    void setNewAssignmentHandler(std::function<void(const AssignmentPerm&)> f);

    /**
     * @brief      Registers an event handler for when this agent is ready to
     *             send a bid.
     *
     * @param[in]  f     The function to call
     */
    void setSendBidHandler(std::function<void(uint32_t, const Auctioneer::BidConstPtr&)> f);

    /**
     * @brief      Sets the desired formation points and the current adjacency
     *             matrix describing the formation.
     *             
     *             Note: the default behavior is to use the assignment from the
     *             last auction to understand who the vehicle's neighbors are.
     *             By setting 'reset' to true, the auction will assume an
     *             identity assignment (i.e., use the adjmat as given) for the
     *             next auction iteration.
     *
     * @param[in]  p       The new desired formation points
     * @param[in]  adjmat  The underlying adjacency matrix to use
     * @param[in]  reset   Whether or not to reset the assignment to Id
     */
    void setFormation(const PtsMat& p, const AdjMat& adjmat, bool resetAssignment=false);

    /**
     * @brief      Kicks off the auction. A snapshot of the current states of
     *             vehicles in the swarm is given. Note that only information
     *             about neighbors is used (via adjmat and current assignment).
     *
     * @param[in]  q     A snapshot of the current states.
     */
    void start(const PtsMat& q);

    void receiveBid(uint32_t iter, const Bid& bid, vehidx_t vehid);
    bool auctionComplete() const { return auctionCompleted_; }
    
    AssignmentPerm getAssignment() const { return P_; }
    AssignmentPerm getInvAssignment() const { return Pt_; }

  private:
    enum class State { IDLE, AUCTION };
    using BidMap = std::map<vehidx_t, Bid>;

    /// \brief Internal state
    uint8_t n_; ///< number of vehicles in swarm
    vehidx_t vehid_; ///< ID of vehicle (index in veh named list)
    State state_; ///< current state of auctioneer (state machine)
    AssignmentPerm P_; ///< nxn assignment permutation (P: vehid --> formpt)
    AssignmentPerm Pt_; ///< nxn inv assign. permutation (Pt: formpt --> vehid)
    int biditer_; ///< current bidding iteration of the CBAA process
    BidPtr bid_; ///< my current bid, to be sent to others
    std::vector<BidMap> bids_; ///< all of the bids from the current auction
    PtsMat q_; ///< the current formation points
    PtsMat p_; ///< the desired formation points
    PtsMat paligned_; ///< the desired formation points, aligned
    AdjMat adjmat_; ///< the required formation graph adjacency matrix
    uint32_t cbaa_max_iter_; ///< number of iterations until convergence
    bool auctionCompleted_; ///< auctioneer has done work and is now idle
    std::mutex mtx_; ///< for synchronizing start before bids are received

    /// \brief Function handles for callbacks
    std::function<void(const AssignmentPerm&)> fn_assignment_;
    std::function<void(uint32_t, const Auctioneer::BidConstPtr&)> fn_sendbid_;

    PtsMat alignFormation(const PtsMat& q,
                          const AdjMat& adjmat, const PtsMat& p);
    void logAssignment(const PtsMat& q, const AdjMat& adjmat,
                       const PtsMat& p, const PtsMat& aligned,
                       const AssignmentPerm& lastP, const AssignmentPerm& P);

    bool hasReachedConsensus() const;
    bool bidIterComplete() const;
    void reset();

    void notifySendBid();
    void notifyNewAssignment();

    void selectTaskAssignment();
    bool updateTaskAssignment();
    float getPrice(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
  };

} // ns aclswarm
} // ns acl
