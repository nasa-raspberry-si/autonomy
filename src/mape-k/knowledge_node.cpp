// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/KInfo.h>
#include <rs_autonomy/AInfo.h>

//Others
#include "message_passing_support.h" // for msg_queue_size

class AInfoListener {
  public:
    ros::Publisher& pub;
    rs_autonomy::KInfo current_task;
    AInfoListener(ros::Publisher& pub):pub(pub) {};
    void callback(const rs_autonomy::AInfo current_task);
};

void AInfoListener::callback(const rs_autonomy::AInfo current_task)
{
  ROS_INFO_STREAM("[Knowledge Node - Listener] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  this->current_task.task_name = current_task.task_name + "-K";
  this->current_task.task_status = current_task.task_status;

  this->pub.publish(this->current_task);
}


int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "knowledge_node");

  ros::NodeHandle nh;

  ros::Publisher k_pub = nh.advertise<rs_autonomy::KInfo>("/KInfo", msg_queue_size);
  AInfoListener listener(k_pub);
  ros::Subscriber k_sub = nh.subscribe<rs_autonomy::AInfo>("/AInfo",
		  					 msg_queue_size,
							 &AInfoListener::callback,
							 &listener);

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

