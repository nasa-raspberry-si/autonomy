// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include "TaskPlanner.h"


int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "planner_node");

  ros::NodeHandle nh;

  TaskPlanner task_planner(&nh);

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

