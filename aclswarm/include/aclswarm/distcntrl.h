/**
 * @file distcntrl.h
 * @brief Distributed high-level controller for aclswarm 
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 17 Oct 2019
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "aclswarm/utils.h"

namespace acl {
namespace aclswarm {

  class DistCntrl
  {
  public:
    struct Formation {
      std::string name; ///< name of current formation
      AdjMat adjmat; ///< current adjacency matrix for formation (nxn)
      GainMat gains; ///< gains for the current formation (3xn3x)
      PtsMat qdes; ///< desired 3D positions of swarm (nx3)

      Eigen::MatrixXd dstar_xy; ///< desired 2D scale (derived from qdes)
      Eigen::MatrixXd dstar_z; ///< desired z scale (derived from qdes)
    };

    struct Gains {
      double K1_xy; ///< nonlinear scale control gain
      double K2_xy; ///< nonlinear scale control gain -- multiplies error
      double K1_z; ///< nonlinear scale control gain
      double K2_z; ///< nonlinear scale control gain -- multiplies error
      double e_xy_thr; ///< no scale control when scale error is below this
      double e_z_thr; ///< no scale control when scale error is below this
      double kp; ///< proportional gain on distance error
      double kd; ///< derivative gain on velocity error
    };

  public:
    DistCntrl(vehidx_t vehid, uint8_t n);
    ~DistCntrl() = default;
    
    void setGains(const Gains& gains);
    void setFormation(const std::shared_ptr<Formation>& f);
    void setAssignment(const AssignmentPerm& P);

    Eigen::Vector3d compute(const PtsMat& q_veh, const Eigen::Vector3d vel);

  private:

    /// \brief Internal state
    uint8_t n_; ///< number of vehicles in swarm
    vehidx_t vehid_; ///< ID of vehicle (index in veh named list)
    std::shared_ptr<Formation> formation_; ///< the current formation to achieve
    AssignmentPerm P_; ///< nxn assignment permutation (P: vehid --> formpt)

    /// \brief Control parameters
    Gains gains_;

  };

} // ns aclswarm
} // ns acl
