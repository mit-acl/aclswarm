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

void DistCntrl::setGains(const Gains& gains)
{
  gains_ = gains;
}

// ----------------------------------------------------------------------------

void DistCntrl::setFormation(const std::shared_ptr<Formation>& f)
{
  formation_ = f;

  // calculate the distance matrix for desired swarm scale
  formation_->dstar_xy = utils::pdistmat(formation_->qdes.leftCols<2>());
  formation_->dstar_z = utils::pdistmat(formation_->qdes.rightCols<1>());
}

// ----------------------------------------------------------------------------

void DistCntrl::setAssignment(const AssignmentPerm& P)
{
  P_ = P;
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
      const auto Aij = formation_->gains.block<3, 3>(3*i, 3*j);

      // calculate the relative translation btwn my and this formation point
      const Eigen::Vector3d qij = q.row(j) - q.row(i);

      //
      // Compute the scale (nonlinear) control
      //

      // for 2d xy component of the formation
      const double e_xy = qij.head<2>().norm() - formation_->dstar_xy(i,j);
      const double Fij_xy = gains_.K1_xy * std::atan(gains_.K2_xy * e_xy);

      // for 3d z component of the formation
      const double e_z = qij.tail<1>().norm() - formation_->dstar_z(i,j);
      const double Fij_z = gains_.K1_z * std::atan(gains_.K2_z * e_z);

      Eigen::Vector3d Fij = Eigen::Vector3d::Zero();
      if (std::abs(e_xy) > gains_.e_xy_thr) Fij.x() = Fij.y() = Fij_xy;
      if (std::abs(e_z) > gains_.e_z_thr) Fij.z() = Fij_z;

      //
      // Compute the total control
      //

      // proportional control
      const auto up = Aij * qij + Fij.asDiagonal() * qij;

      // derivative control
      const auto ud = -vel;

      // combined control
      u += gains_.kp * up + gains_.kd * ud;

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
