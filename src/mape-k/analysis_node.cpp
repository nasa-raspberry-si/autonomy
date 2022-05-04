// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/MInfo.h>
#include <rs_autonomy/AInfo.h>

//Others
#include "message_passing_support.h" // for msg_queue_size

class MInfoListener {
  public:
    ros::Publisher pub;
    rs_autonomy::AInfo current_task;
    void callback(const rs_autonomy::MInfo current_task);
};

void MInfoListener::callback(const rs_autonomy::MInfo current_task)
{
  ROS_INFO_STREAM("[Analysis Node - Listener] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  this->current_task.task_name = current_task.task_name;
  this->current_task.task_status = current_task.task_status;

  this->pub.publish(this->current_task);
}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "analysis_node");

  ros::NodeHandle nh;

  MInfoListener listener;
  listener.pub =  nh.advertise<rs_autonomy::AInfo>("/AInfo", msg_queue_size); 
  ros::Subscriber a_sub = nh.subscribe<rs_autonomy::MInfo>("/MInfo",
                                                         msg_queue_size,
                                                         &MInfoListener::callback,
                                                         &listener);


  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

