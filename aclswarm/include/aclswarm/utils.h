/**
 * @file utils.h
 * @brief Utilities for swarm
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 2019
 */

#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

namespace acl {
namespace aclswarm {
namespace utils {

/**
 * @brief      Loads information about vehicles from rosparam server.
 *
 * @param      name      The name of this vehicle
 * @param      vehicles  A list of all vehicles in swarm
 *
 * @return     True if successful.
 */
static bool loadVehicleInfo(std::string& name, std::vector<std::string>& vehicles)
{
  //
  // Resolve this vehicle's index using this node's ROS ns as the name
  //

  // get the name of this vehicle from the namespace and make uppercase.
  std::string ns = ros::this_node::getNamespace();
  if (ns == "/") {
    ROS_ERROR("Namespace cannot be empty");
    return false;
  }
  name = ns.substr(ns.find_first_not_of('/'));

  // get list of vehicle names, ordered by vehicle index.
  ros::param::get("/vehs", vehicles);
  if (std::find(vehicles.begin(), vehicles.end(), name) == vehicles.end()) {
    ROS_ERROR_STREAM("Could not find myself ('" << name << "') in vehlist");
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Clamp function to saturate between a low and high value
 *
 * @param[in]  val      The value to clamp
 * @param[in]  lower    The lower bound
 * @param[in]  upper    The upper bound
 * @param      clamped  Wether or not the value was clamped
 *
 * @tparam     T        The template type
 *
 * @return     The clamped value
 */
template<typename T>
static T clamp(const T& val, const T& lower, const T& upper, bool& clamped) {
  if (val < lower) {
    clamped = true;
    return lower;
  }

  if (val > upper) {
    clamped = true;
    return upper;
  }

  clamped = false;
  return val;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Specialization of clamp without clamp indicator
 */
template<typename T>
static T clamp(const T& val, const T& lower, const T& upper) {
  bool clamped = false;
  return clamp(val, lower, upper, clamped);
}

} // ns utils
} // ns aclswarm
} // ns acl