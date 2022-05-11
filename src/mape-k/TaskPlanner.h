#ifndef TaskPlanner_H
#define TaskPlanner_H


// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/AdaptationInstruction.h>
#include <rs_autonomy/PlannerInstruction.h>
#include <rs_autonomy/TaskPlanning.h>
#include <rs_autonomy/HighLevelPlan.h>

//Others
#include "message_passing_support.h" // for msg_queue_size
#include <string>


class TaskPlanner {
  public:
    TaskPlanner(ros::NodeHandle *nh)
    {
      planner_inst_pub = nh->advertise<rs_autonomy::PlannerInstruction>(
		      "/Planner/PlannerInstruction",
		      msg_queue_size);
      high_level_plan_pub = nh->advertise<rs_autonomy::HighLevelPlan>(
		      "/Planner/HighLevelPlan",
		      msg_queue_size);
      adap_inst_sub = nh->subscribe<rs_autonomy::AdaptationInstruction>(
		      "/Analysis/AdaptationInstruction",
		      msg_queue_size,
		      &TaskPlanner::callback_adap_inst_sub,
		      this);
      task_planning_service_client = nh->serviceClient<rs_autonomy::TaskPlanning>("/task_planning"); 
    }

    // publisher
    ros::Publisher planner_inst_pub;
    ros::Publisher high_level_plan_pub;
    rs_autonomy::PlannerInstruction planner_instruction;

    // callback for subscriber
    void callback_adap_inst_sub(const rs_autonomy::AdaptationInstruction adpt_inst);

    // ROS clients for calling '/task_planning' service
    rs_autonomy::TaskPlanning task_planning;
    ros::ServiceClient task_planning_service_client;

    // For one task, we use one plan template and fill it with
    // runtime info to turn it into a complete PLEXIL plan (*.plp file)
    // Since one task may needs run several plans, we format the names
    // of these plans as "<task_name><plan_id>", e.g., "Exca1", "Exca2".
    std::string task_name = "";
    std::string current_plan_name = "";
    int current_plan_id = 0; // 0 indiates no plan has been tried for the task
  private:
    ros::Subscriber adap_inst_sub;
};


#endif
