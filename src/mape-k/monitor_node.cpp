// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include "SysEnvMonitor.h"


int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "monitor_node");

  ros::NodeHandle nh;

  SysEnvMonitor monitor = SysEnvMonitor(&nh);
  
  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

