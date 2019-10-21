/**
 * @file vehicle_tracker.cpp
 * @brief Message passing and logic for estimating swarm vehicle positions
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 20 Oct 2019
 */

#include "aclswarm/vehicle_tracker.h"

namespace acl {
namespace aclswarm {

VehicleTracker::VehicleTracker(uint8_t n)
: n_(n)
{
  // initialize data structures
  stamps_.resize(n, 0);
  positions_.resize(n);
}

// ----------------------------------------------------------------------------

void VehicleTracker::setAdjacencyMatrix(const AdjMat& A)
{
  // TODO: We will also need to know the assignment
  A_ = A;
}

// ----------------------------------------------------------------------------

bool VehicleTracker::updateVehicle(uint8_t srcid, uint8_t vehid,
                                  uint64_t time_ns, const Eigen::Vector3d& pos)
{
  bool updated = false;
  // TODO: Make this smarter using Djikstra's

  // Only update my estimate of vehid if this estimate is later
  if (time_ns > stamps_[vehid]) {
    stamps_[vehid] = time_ns;
    positions_[vehid] = pos;
    updated = true;
  }

  return updated;
}

// ----------------------------------------------------------------------------

Eigen::Vector3d VehicleTracker::getVehiclePosition(uint8_t vehid)
{
  return positions_[vehid];
}

// ----------------------------------------------------------------------------

uint64_t VehicleTracker::getVehicleStamp(uint8_t vehid)
{
  return stamps_[vehid];
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

} // ns aclswarm
} // ns acl
