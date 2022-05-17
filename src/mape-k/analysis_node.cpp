// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include "AdaptationAnalyzer.h"

char *is_debug_str = getenv("DEBUG_MODE");

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "analysis_node");

  ros::NodeHandle nh;
  bool is_debug = false;
  if ((is_debug_str!=NULL) && (std::string(is_debug_str))=="true")
  {
	is_debug = true;
  }
  AdaptationAnalyzer adaptation_analyzer(&nh, is_debug);

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    // determine if some adaptation should be performed
    adaptation_analyzer.adaptation_analysis();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

