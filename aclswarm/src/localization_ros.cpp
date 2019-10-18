/**
 * @file localization_ros.cpp
 * @brief ROS wrapper for localization components of aclswarm
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 18 Oct 2019
 */

#include "aclswarm/localization_ros.h"
#include "aclswarm/utils.h"

namespace acl {
namespace aclswarm {

LocalizationROS::LocalizationROS(const ros::NodeHandle nh,
                                  const ros::NodeHandle nhp)
: nh_(nh), nhp_(nhp)
{
  if (!utils::loadVehicleInfo(vehname_, vehs_)) {
    ros::shutdown();
    return;
  }

  //
  // Load parameters
  //


  //
  // ROS pub/sub communication
  //

}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

} // ms aclswarm
} // ns acl
