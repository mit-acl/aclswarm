/**
 * @file distcntrl.h
 * @brief Distributed high-level controller for aclswarm 
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 17 Oct 2019
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

namespace acl {
namespace aclswarm {

  class DistCntrl
  {
  public:
    DistCntrl();
    ~DistCntrl() = default;
    
    void set_formation(/*permuted A, permuted d*, name*/);

    void compute(/*my pos, nbhr pos, my vel*/);
  };

} // ns aclswarm
} // ns acl
