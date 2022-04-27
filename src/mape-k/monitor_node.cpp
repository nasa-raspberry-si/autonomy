// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <ow_plexil/CurrentTask.h>
#include <rs_autonomy/MInfo.h>


bool t = false;
rs_autonomy::MInfo m_msg;

void taskSubCallback(const ow_plexil::CurrentTask current_task)
{
  ROS_INFO_STREAM("[Monitor Node] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  m_msg.task_name = current_task.task_name + "-M";
  m_msg.task_status = current_task.task_status;

  t = true;
}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "monitor_node");

  ros::NodeHandle nh;

  ros::Publisher m_pub = nh.advertise<rs_autonomy::MInfo>("/MInfo", 3);
  ros::Subscriber m_sub = nh.subscribe<ow_plexil::CurrentTask>("/CurrentTask",
                                                         3,
							 taskSubCallback);


  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {

    if(t) {
      m_pub.publish(m_msg);
      t = false;
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

