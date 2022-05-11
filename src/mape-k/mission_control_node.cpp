// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include "MissionController.h"

#include <string>



int main(int argc, char* argv[])
{
  std::string mission_spec_fp = "None"; 

  if (argc==2 && std::string(argv[1]).compare("None") != 0)
  {
    mission_spec_fp = std::string(argv[1]);
  }
  else
  {
    ROS_ERROR("[Mission Control Node] The mission specification filepath is not given.");
    return 1;
  }

  // Initialization
  ros::init(argc, argv, "autonomy_node");
  ros::NodeHandle nh;

  MissionController mission_controller(&nh);
  // Load the mission specification and start to run the first task
  mission_controller.load_mission_spec(mission_spec_fp);
  mission_controller.prepare_task_to_run();

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

