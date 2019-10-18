/**
 * @file localization_node.cpp
 * @brief Entry point for localization ROS node
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 18 Oct 2019
 */

#include <ros/ros.h>

#include "aclswarm/localization_ros.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "localization");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  acl::aclswarm::LocalizationROS node(nhtopics, nhparams);
  ros::spin();
  return 0;
}
