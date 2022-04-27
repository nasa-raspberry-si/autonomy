// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/EInfo.h>
#include <rs_autonomy/PInfo.h>

bool p = false;
rs_autonomy::EInfo e_msg;

void pInfoSubCallback(const rs_autonomy::PInfo current_task)
{
  ROS_INFO_STREAM("[Execute Node] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  e_msg.task_name = current_task.task_name + "-E";
  e_msg.task_status = current_task.task_status;
  
  p = true;
}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "execute_node");

  ros::NodeHandle nh;

  ros::Publisher e_pub = nh.advertise<rs_autonomy::EInfo>("/EInfo", 3);
  ros::Subscriber e_sub = nh.subscribe<rs_autonomy::PInfo>("/PInfo",
                                                         3,
							 pInfoSubCallback);

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    if (p) {
      e_pub.publish(e_msg);
      p = false;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

