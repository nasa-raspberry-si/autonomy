// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/PInfo.h>
#include <rs_autonomy/EInfo.h>

//Others
#include "message_passing_support.h" // for msg_queue_size

class PInfoListener {
  public:
    ros::Publisher pub;
    rs_autonomy::EInfo current_task;
    void callback(const rs_autonomy::PInfo current_task);
};

void PInfoListener::callback(const rs_autonomy::PInfo current_task)
{
  ROS_INFO_STREAM("[Execute Node - Listener] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  this->current_task.task_name = current_task.task_name + "-E";
  this->current_task.task_status = current_task.task_status;

  this->pub.publish(this->current_task);
}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "planner_node");

  ros::NodeHandle nh;

  PInfoListener listener;
  listener.pub =  nh.advertise<rs_autonomy::EInfo>("/EInfo", msg_queue_size);
  ros::Subscriber e_sub = nh.subscribe<rs_autonomy::PInfo>("/PInfo",
                                                         msg_queue_size,
                                                         &PInfoListener::callback,
                                                         &listener);


  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

