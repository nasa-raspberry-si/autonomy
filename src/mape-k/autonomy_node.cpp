// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/AuInfo.h>
#include <rs_autonomy/KInfo.h>

//Others
#include "message_passing_support.h" // for msg_queue_size

class AuInfoListener {
  public:
    ros::Publisher& pub;
    rs_autonomy::AuInfo current_task;
    AuInfoListener(ros::Publisher& pub):pub(pub) {}; 
    void callback(const rs_autonomy::KInfo current_task);
};

void AuInfoListener::callback(const rs_autonomy::KInfo current_task)
{
  ROS_INFO_STREAM("[Autonomy Node - Listener] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  this->current_task.task_name = current_task.task_name + "-Au";
  this->current_task.task_status = current_task.task_status;

  this->pub.publish(this->current_task);
}


int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "autonomy_node");

  ros::NodeHandle nh;

  ros::Publisher au_pub = nh.advertise<rs_autonomy::AuInfo>("/AuInfo", msg_queue_size);

  AuInfoListener listener(au_pub);
  ros::Subscriber au_sub = nh.subscribe<rs_autonomy::KInfo>("/KInfo",
		  					 msg_queue_size,
							 &AuInfoListener::callback,
							 &listener);

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

