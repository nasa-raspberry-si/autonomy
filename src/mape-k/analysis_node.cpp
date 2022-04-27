// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/MInfo.h>
#include <rs_autonomy/AInfo.h>

bool m = false;
rs_autonomy::AInfo a_msg;

void mInfoSubCallback(const rs_autonomy::MInfo current_task)
{
  ROS_INFO_STREAM("[Analysis Node] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  a_msg.task_name = current_task.task_name + "-A";
  a_msg.task_status = current_task.task_status;

  m = true;
}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "analysis_node");

  ros::NodeHandle nh;

  ros::Publisher a_pub = nh.advertise<rs_autonomy::AInfo>("/AInfo", 3);
  ros::Subscriber a_sub = nh.subscribe<rs_autonomy::MInfo>("/MInfo",
                                                         3,
							 mInfoSubCallback);

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    if(m) {
      a_pub.publish(a_msg);
      m = false;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

