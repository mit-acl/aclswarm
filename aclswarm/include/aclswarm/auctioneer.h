/**
 * @file auctioneer.h
 * @brief Distributed task assignment for aclswarm via CBAA 
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 26 Oct 2019
 */

#pragma once

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#include <queue>

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

      friend std::ostream& operator<<(std::ostream& o, const Bid& b)
      {
        o << "<";
        for (size_t j=0; j<b.who.size(); ++j) {
          o << std::to_string(b.who[j]) << "(";
          o << std::fixed << std::setprecision(2) << b.price[j] << ")";
          if (j != (b.who.size()-1)) o << ", ";
        }
        o << ">";
        return o;
      }
    };
    using BidPtr = std::shared_ptr<Bid>;
    using BidConstPtr = std::shared_ptr<const Bid>;

  public:
    Auctioneer(vehidx_t vehid, uint8_t n, bool verbose = false);
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
    void setSendBidHandler(std::function<void(uint32_t, uint32_t,
                                          const Auctioneer::BidConstPtr&)> f);

    /**
     * @brief      Sets the desired formation points and the current adjacency
     *             matrix describing the formation.
     *
     *             Note: by calling setFormation, the assignment will be reset
     *             back to identity (i.e., use the admat as given). Thus, it is
     *             imperative that vehicle's are connected according to the
     *             adjmat (since assignment is identity).
     *
     *             For the intermediate auto auctions, this method need not be
     *             called since these params only change for new formations.
     *
     * @param[in]  p       The new desired formation points
     * @param[in]  adjmat  The underlying adjacency matrix to use
     */
    void setFormation(const PtsMat& p, const AdjMat& adjmat);

    /**
     * @brief      Kicks off the auction. A snapshot of the current states of
     *             vehicles in the swarm is given. Note that only information
     *             about neighbors is used (via adjmat and current assignment).
     *
     * @param[in]  q     A snapshot of the current states.
     */
    void start(const PtsMat& q);

    void enqueueBid(vehidx_t vehid, uint32_t auctionid, uint32_t iter,
                    const Bid& bid);
    void tick();

    void flush();

    AssignmentPerm getAssignment() const { return P_; }
    AssignmentPerm getInvAssignment() const { return Pt_; }

    // only used as a "backdoor" when we want to override the auctioneer
    void setAssignment(const AssignmentPerm& P) { P_ = P; Pt_ = P.transpose(); }

    bool isIdle() const { return !auction_is_open_; }
    bool didConvergeOnInvalidAssignment() const { return invalid_assignment_; }

  private:
    enum class State { IDLE, AUCTION };
    using BidMap = std::map<vehidx_t, Bid>;
    using BidPkt = std::tuple<vehidx_t, uint32_t, uint32_t, Bid>;

    /// \brief Internal state
    uint8_t n_; ///< number of vehicles in swarm
    vehidx_t vehid_; ///< ID of vehicle (index in veh named list)
    State state_; ///< current state of auctioneer (state machine)
    AssignmentPerm P_; ///< nxn assignment permutation (P: vehid --> formpt)
    AssignmentPerm Pt_; ///< nxn inv assign. permutation (Pt: formpt --> vehid)
    int auctionid_; ///< unique id associated with the current auction
    int biditer_; ///< current bidding iteration of the CBAA process
    BidPtr bid_; ///< my current bid, to be sent to others
    BidMap bids_zero_; ///< save these in case my nbr starts before I do
    BidMap bids_curr_; ///< the bids of the current iteration
    BidMap bids_next_; ///< bids from nbrs who have started the next iter
    std::queue<BidPkt> rxbids_; ///< Queue of received bids to process
    PtsMat q_; ///< the current formation points
    PtsMat p_; ///< the desired formation points
    PtsMat paligned_; ///< the desired formation points, aligned
    AdjMat adjmat_; ///< the required formation graph adjacency matrix
    uint32_t cbaa_max_iter_; ///< number of iterations until convergence
    bool auction_is_open_; ///< auctioneer is ready to receive/send bids
    std::mutex queue_mtx_; ///< for bid queue resource management
    std::mutex auction_mtx_; ///< for synchronization of start and bid proc
    bool invalid_assignment_; ///< this is not CBAA's fault
    bool formation_just_received_; ///< first auction of new formation?
    bool verbose_; ///< should print verbose auction/bid information

    /// \brief Function handles for callbacks
    std::function<void(const AssignmentPerm&)> fn_assignment_;
    std::function<void(uint32_t, uint32_t,
                        const Auctioneer::BidConstPtr&)> fn_sendbid_;

    void processBid(const BidPkt& bidpkt);

    PtsMat alignFormation(const PtsMat& q,
                          const AdjMat& adjmat, const PtsMat& p) const;

    std::string reportMissing();

    void logAssignment(const PtsMat& q, const AdjMat& adjmat,
                       const PtsMat& p, const PtsMat& aligned,
                       const AssignmentPerm& lastP, const AssignmentPerm& P);

    bool isValidAssignment(const std::vector<vehidx_t>& permvec) const;
    bool shouldUseAssignment(const AssignmentPerm& newP) /*const*/;
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
