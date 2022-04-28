// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/AInfo.h>
#include <rs_autonomy/PInfo.h>



class AInfoListener {
  public:
    ros::Publisher pub;
    rs_autonomy::PInfo current_task;
    void callback(const rs_autonomy::AInfo current_task);
};

void AInfoListener::callback(const rs_autonomy::AInfo current_task)
{
  ROS_INFO_STREAM("[Planner Node - Listener] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  this->current_task.task_name = current_task.task_name + "-P";
  this->current_task.task_status = current_task.task_status;

  this->pub.publish(this->current_task);
}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "planner_node");

  ros::NodeHandle nh;

  AInfoListener listener;
  listener.pub =  nh.advertise<rs_autonomy::PInfo>("/PInfo", 3);
  ros::Subscriber p_sub = nh.subscribe<rs_autonomy::AInfo>("/AInfo",
                                                         3,
                                                         &AInfoListener::callback,
                                                         &listener);


  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

