// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <ow_plexil/CurrentPlan.h>
#include <ow_plexil/CurrentOperation.h>
#include <rs_autonomy/MInfo.h>

//Others
#include "message_passing_support.h" // for msg_queue_size

/*
bool t = false;
rs_autonomy::MInfo m_msg;

void planSubCallback(const ow_plexil::CurrentPlan current_plan)
{
  ROS_INFO_STREAM("[Monitor Node] current plan: " << current_plan.plan_name << ", status: " << current_plan.plan_status);

  m_msg.plan_name = current_plan.plan_name;
  m_msg.plan_status = current_plan.plan_status;

  t = true;
}
*/


class CurrentPlanListener {
  public:
    ros::Publisher pub;
    rs_autonomy::MInfo current_plan;
    void callback(const ow_plexil::CurrentPlan current_plan);
};

void CurrentPlanListener::callback(const ow_plexil::CurrentPlan current_plan)
{
  ROS_INFO_STREAM("[Monitor Node - Listener] current plan: " << current_plan.plan_name << ", status: " << current_plan.plan_status);

  this->current_plan.task_name = current_plan.plan_name;
  this->current_plan.task_status = current_plan.plan_status;

  this->pub.publish(this->current_plan);
}


int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "monitor_node");

  ros::NodeHandle nh;

  /*
  ros::Publisher m_pub = nh.advertise<rs_autonomy::MInfo>("/MInfo", 10);
  ros::Subscriber m_sub = nh.subscribe<ow_plexil::CurrentPlan>("/CurrentPlan",
                                                         10,
							 planSubCallback);
  */

  CurrentPlanListener listener;
  listener.pub =  nh.advertise<rs_autonomy::MInfo>("/MInfo", msg_queue_size); 
  ros::Subscriber ml_sub = nh.subscribe<ow_plexil::CurrentPlan>("/current_plan_status",
                                                         msg_queue_size,
							 &CurrentPlanListener::callback,
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

