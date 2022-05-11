// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include "AdaptationAnalyzer.h"


int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "analysis_node");

  ros::NodeHandle nh;

  AdaptationAnalyzer adaptation_analyzer(&nh);

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    // determine if some adaptation should be performed
    adaptation_analyzer.adaptation_analysis();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

