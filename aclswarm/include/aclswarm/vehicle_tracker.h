/**
 * @file vehicle_tracker.cpp
 * @brief Message passing and logic for estimating swarm vehicle positions
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 20 Oct 2019
 */

#pragma once

#include <cstdint>

#include <Eigen/Dense>

#include "aclswarm/utils.h"

namespace acl {
namespace aclswarm {

  class VehicleTracker
  {
  public:
    VehicleTracker(uint8_t n);
    ~VehicleTracker() = default;

    void setAdjacencyMatrix(const AdjMat& A);
    
    bool updateVehicle(uint8_t srcid, uint8_t vehid,
                      uint64_t time_ns, const Eigen::Vector3d& pos);

    Eigen::Vector3d getVehiclePosition(uint8_t vehid);
    uint64_t getVehicleStamp(uint8_t vehid);

  private:
    const uint8_t n_; ///< number of vehicles in the swarm

    /// \brief Internal state
    AdjMat A_;
    std::vector<uint64_t> stamps_;
    std::vector<Eigen::Vector3d> positions_;
  };

} // ns aclswarm
} // ns acl
