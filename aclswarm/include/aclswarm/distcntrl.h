/**
 * @file distcntrl.h
 * @brief Distributed high-level controller for aclswarm 
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 17 Oct 2019
 */

#pragma once

#include <cmath>
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
    DistCntrl();
    ~DistCntrl() = default;
    
    void set_formation(const std::shared_ptr<Formation>& f);

    void compute(/*my pos, nbhr pos, my vel*/);

    struct Formation {
      std::string name; ///< name of current formation
      AdjMat adjmat; ///< current adjacency matrix for formation
      GainMat gains; ///< gains for the current formation
      std::vector<Eigen::Vector3d> qdes; ///< desired 3D positions of swarm

      std::vector<double> dstar; ///< desired scale (derived from qdes)
    };

  private:

    /// \brief Internal state
    std::shared_ptr<Formation> formation_; ///< the current formation to achieve
    AssignmentMap assignment_; ///< assignment map (sigma: vehid --> formpt)
    AssignmentMap invassignment_; ///< inv map (sigma^-1: formpt --> vehid)

  };

} // ns aclswarm
} // ns acl
