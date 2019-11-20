/**
 * @file comm_layer.h
 * @brief Communication layer for asynchronous auctions. Handles ACK handshake.
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 8 Nov 2019
 */

#pragma once

#include <algorithm>
#include <condition_variable>
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

  class CommLayer
  {
  public:
    CommLayer();
    ~CommLayer() = default;
    
  };

} // ns aclswarm
} // ns acl
