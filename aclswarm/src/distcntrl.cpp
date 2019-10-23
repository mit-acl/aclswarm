/**
 * @file distcntrl.cpp
 * @brief Distributed high-level controller for aclswarm 
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 17 Oct 2019
 */

#include "aclswarm/distcntrl.h"

namespace acl {
namespace aclswarm {

DistCntrl::DistCntrl(vehidx_t vehid, uint8_t n)
: n_(n), vehid_(vehid)
{
  P_.setIdentity(n_);
}

// ----------------------------------------------------------------------------

void DistCntrl::set_gains(double K, double kp, double kd)
{
  K = K_;
  kp_ = kp;
  kd_ = kd;
}

// ----------------------------------------------------------------------------

void DistCntrl::set_formation(const std::shared_ptr<Formation>& f)
{
  formation_ = f;

  // calculate the distance matrix for desired swarm scale
  formation_->dstar = utils::pdistmat(formation_->qdes);
}

// ----------------------------------------------------------------------------

void DistCntrl::set_assignment(const AssignmentVec& a)
{
  // represent the assignment as a permutation matrix
  //  that maps vehicle id to formation point.
  P_ = AssignmentPerm(a);
}

// ----------------------------------------------------------------------------

Eigen::Vector3d DistCntrl::compute(const PtsMat& q_veh, const Eigen::Vector3d vel)
{
  // initialize control output to [0 0 0]
  Eigen::Vector3d u = Eigen::Vector3d::Zero();

  // q_veh stores positions of vehicles in "vehicle space".
  // Permute to "formation space". Everything below is in "formation space".
  const auto q = P_ * q_veh;

  // which formation point am I currently assigned to?
  const auto i = P_.indices()(vehid_);

  // loop through the other formation points in graph
  for (size_t j=0; j<n_; ++j) {
    // neighbor check:
    // is there an edge between my formation point and this other one?
    if (formation_->adjmat(i, j)) {
      // locate the relevant block in the gain matrix ("formation space").
      auto Aij = formation_->gains.block<3, 3>(3*i, 3*j);

      // calculate the relative translation btwn my and this formation point
      const Eigen::Vector3d qij = q.row(j) - q.row(i);

      //
      // Compute the control
      //

      // This term controls the scale
      const double eps = 1. / (K_ * qij.norm());
      const auto Fij = eps * std::atan(qij.norm() - formation_->dstar(i,j));

      // proportional control
      const auto up = Aij * qij + Fij * qij;

      // derivative control
      const auto ud = -vel;

      // combined control
      u += kp_ * up + kd_ * ud;

    }
  }

  return u;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------

} // ns aclswarm
} // ns acl
