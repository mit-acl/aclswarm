/**
 * @file safety_node.cpp
 * @brief Entry point for safety ROS node
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 2019
 */

#include <ros/ros.h>

#include "aclswarm/safety.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "safety");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  acl::aclswarm::Safety node(nhtopics, nhparams);
  ros::spin();
  return 0;
}
