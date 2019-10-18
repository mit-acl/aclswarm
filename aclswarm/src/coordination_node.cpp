/**
 * @file coordination_node.cpp
 * @brief Entry point for coordination ROS node
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 2019
 */

#include <ros/ros.h>

#include "aclswarm/coordination_ros.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "coordination");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  acl::aclswarm::CoordinationROS node(nhtopics, nhparams);
  node.spin();
  return 0;
}
