/**
 * @file utils.h
 * @brief Utilities for swarm
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 2019
 */

#pragma once

#include <algorithm>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

namespace acl {
namespace aclswarm {

using vehidx_t = uint8_t;
using GainMat = Eigen::MatrixXd;
using AdjMat = Eigen::Matrix<vehidx_t, Eigen::Dynamic, Eigen::Dynamic>;
using PtsMat = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using AssignmentVec = Eigen::Matrix<vehidx_t, Eigen::Dynamic, 1>;
using AssignmentPerm = Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, vehidx_t>;

namespace utils {

/**
 * @brief      Loads information about vehicles from rosparam server.
 *
 * @param      name      The name of this vehicle
 * @param      id        The id of this vehicle (idx in vehicles list)
 * @param      vehicles  A list of all vehicles in swarm
 *
 * @return     True if successful.
 */
static bool loadVehicleInfo(std::string& name, vehidx_t& vehid,
                            std::vector<std::string>& vehicles)
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
  const auto& it = std::find(vehicles.begin(), vehicles.end(), name);
  if (it == vehicles.end()) {
    ROS_ERROR_STREAM("Could not find myself ('" << name << "') in vehlist");
    return false;
  }

  // deduce vehicle id
  vehid = std::distance(vehicles.begin(), it);

  ROS_WARN_STREAM("Loading '" << name << "' as index "
                  << static_cast<int>(vehid));
  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Converts adjmat msg to Eigen
 *
 * @param[in]  msg   The adjmat multiarray
 *
 * @return     The eigen matrix
 */
static AdjMat decodeAdjMat(const std_msgs::UInt8MultiArray& msg)
{
  assert(msg.layout.dim.size() == 2);

  const int rows = msg.layout.dim[0].size;
  const int cols = msg.layout.dim[1].size;

  AdjMat A = AdjMat::Zero(rows, cols);

  for (size_t i=0; i<rows; ++i) {
    for (size_t j=0; j<cols; ++j) {
      A(i,j) = msg.data[msg.layout.data_offset + msg.layout.dim[1].stride * i + j];
    }
  }

  return A;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Converts gain msg to Eigen
 *
 * @param[in]  msg   The gains multiarray
 *
 * @return     The eigen matrix
 */
static GainMat decodeGainMat(const std_msgs::Float32MultiArray& msg)
{
  assert(msg.layout.dim.size() == 2);

  const int rows = msg.layout.dim[0].size;
  const int cols = msg.layout.dim[1].size;

  GainMat A = GainMat::Zero(rows, cols);

  for (size_t i=0; i<rows; ++i) {
    for (size_t j=0; j<cols; ++j) {
      A(i,j) = msg.data[msg.layout.data_offset + msg.layout.dim[1].stride * i + j];
    }
  }

  return A;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Compute distance matrix
 *
 * @param[in]  M     The nx3 matrix of (rowwise) data
 *
 * @return     A symmetric nxn matrix of pairwise distances
 */
static Eigen::MatrixXd pdistmat(const Eigen::MatrixXd& M)
{
  int n = M.rows();

  // compute a matrix of pair-wise distances---squareform(pdist(M))
  // Uses the fact that |x-y|²=|x|²+|y|²-2x'y
  Eigen::VectorXd N = M.rowwise().squaredNorm();
  Eigen::MatrixXd D = N.replicate(1, n) + N.transpose().replicate(n, 1);
  D.noalias() -= 2. * M * M.transpose();
  return D.array().sqrt();
}

// ----------------------------------------------------------------------------

/**
 * @brief      Given an arbitrary numeric vector, return the indices
 *             that would sort the original vector---the sort indices.
 *             In other words, the sort indices describe where each
 *             element of sort(v) originally was in v.
 *
 *                        (idx  0   1  2  3)
 *             v            = [36, 12, 3, 9]
 *             sort(v)      = [3, 9, 12, 36]
 *             ---
 *             sort indices = [2, 3, 1, 0]
 *
 * @param[in]  v
 *
 * @return     The sort indices that would sort the original vector
 */
template<typename T>
static std::vector<T> sortIndices(const std::vector<T>& v)
{
  // initialize original index locations
  std::vector<T> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  std::sort(idx.begin(), idx.end(),
       [&v](T i1, T i2) {return v[i1] < v[i2];});

  return idx;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Copy std::map second value to a vector
 *
 * @param[in]  m     The map to extract from
 * @param      v     The vector to place into
 */
template<typename M, typename V>
static void mapToVec(const M& m, V& v)
{
  v.reserve(m.size());

  for (typename M::const_iterator it = m.begin(); it != m.end(); ++it) {
    v.push_back(it->second);
  }
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

// ----------------------------------------------------------------------------

/**
 * @brief      Saturate the current value of a signal by limit its
 *             rate of change, given its current (desired) value, 
 *             last value, and the timestep.
 *
 * @param[in]  dt        Timestep between current and last value
 * @param[in]  lRateLim  Lower rate limit
 * @param[in]  uRateLim  Upper rate limit
 * @param[in]  v0        The last value
 * @param      v1        The current (des) value, to be rate limited (output)
 */
template<typename T>
static void rateLimit(double dt, const T lRateLim, const T uRateLim,
                      const T v0, T& v1)
{
  // calculate the highest / lowest the components are allowed to attain
  const T upper = v0 + uRateLim * dt;
  const T lower = v0 + lRateLim * dt;

  // make sure current value does not exceed rate of change constraints
  if (v1 > upper) v1 = upper;
  if (v1 < lower) v1 = lower;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Wrap angle so that it is in [-pi, pi]
 *
 * @param[in]  angle  The angle to wrap
 *
 * @return     The wrapped angle
 */
static double wrapToPi(double angle)
{
  if (angle >  M_PI) return angle - 2*M_PI;
  if (angle < -M_PI) return angle + 2*M_PI;
  return angle;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Wrap angle so that it is in [0, 2*pi]
 *
 * @param[in]  angle  The angle to wrap
 *
 * @return     The wrapped angle
 */
static double wrapTo2Pi(double angle)
{
  if (angle > 2*M_PI) return angle - 2*M_PI;
  if (angle < 0)      return angle + 2*M_PI;
  return angle;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Find the index of the element closest to the input
 *
 * @param[in]  x     The vector to search (presorted)
 * @param[in]  v     The input value
 *
 * @return     The index of the closest value
 */
template<typename T>
static size_t closest(const std::vector<T>& x, T v)
{
  // find the first element of x that is >= v
  auto it = std::lower_bound(x.begin(), x.end(), v);
  
  // there can be no lower, this is the closest.
  if (it == x.begin())
    return std::distance(x.begin(), it);
  
  // make sure to look at the previous value, which might be closer.
  // Especially if there was no it \in x s.t. it >= v
  auto prev = it-1;
  if (it == x.end() || std::abs(*prev-v) < std::abs(*it-v))
    return std::distance(x.begin(), prev);
  
  return std::distance(x.begin(), it);
}

} // ns utils
} // ns aclswarm
} // ns acl
