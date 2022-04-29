// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <ow_plexil/CurrentTask.h>
#include <rs_autonomy/MInfo.h>

//Others
#include "message_passing_support.h" // for msg_queue_size

/*
bool t = false;
rs_autonomy::MInfo m_msg;

void taskSubCallback(const ow_plexil::CurrentTask current_task)
{
  ROS_INFO_STREAM("[Monitor Node] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  m_msg.task_name = current_task.task_name + "-M";
  m_msg.task_status = current_task.task_status;

  t = true;
}
*/


class CurrentTaskListener {
  public:
    ros::Publisher pub;
    rs_autonomy::MInfo current_task;
    void callback(const ow_plexil::CurrentTask current_task);
};

void CurrentTaskListener::callback(const ow_plexil::CurrentTask current_task)
{
  ROS_INFO_STREAM("[Monitor Node - Listener] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  this->current_task.task_name = current_task.task_name + "-M";
  this->current_task.task_status = current_task.task_status;

  this->pub.publish(this->current_task);
}


int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "monitor_node");

  ros::NodeHandle nh;

  /*
  ros::Publisher m_pub = nh.advertise<rs_autonomy::MInfo>("/MInfo", 10);
  ros::Subscriber m_sub = nh.subscribe<ow_plexil::CurrentTask>("/CurrentTask",
                                                         10,
							 taskSubCallback);
  */

  CurrentTaskListener listener;
  listener.pub =  nh.advertise<rs_autonomy::MInfo>("/MInfo", msg_queue_size); 
  ros::Subscriber ml_sub = nh.subscribe<ow_plexil::CurrentTask>("/CurrentTask",
                                                         msg_queue_size,
							 &CurrentTaskListener::callback,
							 &listener);
  


  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    /*
    if(t) {
      m_pub.publish(m_msg);
      t = false;
    }
    */
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

