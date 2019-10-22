/**
 * @file distcntrl.cpp
 * @brief Distributed high-level controller for aclswarm 
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 17 Oct 2019
 */

#include "aclswarm/distcntrl.h"

namespace acl {
namespace aclswarm {

DistCntrl::DistCntrl()
{

}

// ----------------------------------------------------------------------------

void DistCntrl::compute()
{
  // 1.

  // which formation point am I currently assigned to?
  const uint8_t i = assignment_[vehid_];

  // Lookup my row in the adjacency matrix to know
  // who my neighboring formation points are
  const auto& myrow = adjmat_.row(i);

  // loop through the other formation points in graph
  for (size_t j=0; j<n_; ++j) {
    // is there an edge between my formation point and this other one?
    auto e = myrow(j);

    if (e) {
      // locate the relevant block in the gain matrix.
      auto Aij = formation_->gains.block<3, 3>(3*i, 3*j);

      // resolve which vehicle is assigned to this formation point
      uint8_t sigmainv_j = invassignment_[j];

      // calculate the relative translation to this vehicle
      auto qij = q_[sigmainv_j] - q_[vehid_];

      //
      // Compute the control
      //

      // This term controls the scale
      auto F = std::atan(qij.norm() - Dd)

    }

  }
}

// ----------------------------------------------------------------------------

void DistCntrl::set_formation(const std::shared_ptr<Formation>& f)
{
  formation_ = f;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------

} // ns aclswarm
} // ns acl
