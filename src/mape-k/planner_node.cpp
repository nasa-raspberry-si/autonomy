// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/PInfo.h>
#include <rs_autonomy/AInfo.h>


bool a = false;
rs_autonomy::PInfo p_msg;

void aInfoSubCallback(const rs_autonomy::AInfo current_task)
{
  ROS_INFO_STREAM("[Planner Node] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  p_msg.task_name = current_task.task_name + "-P";
  p_msg.task_status = current_task.task_status;

  a = true;
}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "planner_node");

  ros::NodeHandle nh;

  ros::Publisher p_pub = nh.advertise<rs_autonomy::PInfo>("/PInfo", 3);
  ros::Subscriber p_sub = nh.subscribe<rs_autonomy::AInfo>("/AInfo",
                                                         3,
							 aInfoSubCallback);

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    if (a) {
      p_pub.publish(p_msg);
      a = false;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

