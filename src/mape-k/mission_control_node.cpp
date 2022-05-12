// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include "MissionController.h"

#include <string>

char *eval_root_dir = getenv("EVALUATION_ROOT_DIR");

int main(int argc, char* argv[])
{
  std::string mission_spec_filename = "None"; 

  if (argc==2 && std::string(argv[1]).compare("None") != 0)
  {
    mission_spec_filename = std::string(argv[1]);
  }
  else
  {
    ROS_ERROR("[Mission Control Node] The mission specification filepath is not given.");
    return 1;
  }

  // Initialization
  ros::init(argc, argv, "autonomy_node");
  ros::NodeHandle nh;

  char* eval_root_dir = getenv("EVALUATION_ROOT_DIR");
  if(eval_root_dir == NULL) {
    ROS_ERROR("Environment variable $PLEXIL_PLAN_DIR is not set.");
    return 2;
  }

  ROS_INFO("[Mission Control Node] the mission specification filename: %s", mission_spec_filename.c_str());
  ROS_INFO("[Mission Control Node] the evaluation root dir: %s", eval_root_dir);

  MissionController mission_controller(&nh, std::string(eval_root_dir));
  // Load the mission specification and start to run the first task
  std::string mission_spec_fp = std::string(eval_root_dir) + "/"+ std::string(mission_spec_filename);
  mission_controller.load_mission_spec(mission_spec_fp);
  mission_controller.prepare_new_task_to_run();

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

