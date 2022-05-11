#ifndef TaskExecutor_H
#define TaskExecutor_H



// The role of Execute node:
// * Wait for instruction from the planner node
// * Pull info from the knowledge node when needed
// * Do plan translation if the instruction from the planner node asks to run a new plan
// * Send commands to the system under test to drive the lander

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <string>

// OW
#include <rs_autonomy/ArmFaultConfig.h>
#include <rs_autonomy/PlannerInstruction.h>
#include <rs_autonomy/PlanTranslation.h>
#include <ow_plexil/PlanSelection.h>

//Others
#include "message_passing_support.h" // for msg_queue_size


class TaskExecutor {
  public:
    TaskExecutor(ros::NodeHandle *nh)
    {
      // ROS service clients
      plan_selection_service_client = nh->serviceClient<ow_plexil::PlanSelection>("/plexil_plan_selection");
      fault_management_service_client = nh->serviceClient<rs_autonomy::ArmFaultConfig>("/arm_fault_management");
      plan_translation_service_client = nh->serviceClient<rs_autonomy::PlanTranslation>("/plan_translation");
      // callbacks for subscribers
      plan_inst_sub = nh->subscribe<rs_autonomy::PlannerInstruction>(
    		      "/Planner/PlannerInstruction",
    		      msg_queue_size,
    		      &TaskExecutor::callback_planner_inst,
    		      this);
    }

    // ROS client for calling '/plan_translation' service
    rs_autonomy::PlanTranslation plan_translation;
    ros::ServiceClient plan_translation_service_client;

    ow_plexil::PlanSelection execute_instruction;
    ros::ServiceClient plan_selection_service_client;

    rs_autonomy::ArmFaultConfig clear_arm_faults = create_fault_clear_req();
    rs_autonomy::ArmFaultConfig create_fault_clear_req();
    ros::ServiceClient fault_management_service_client;

    void callback_planner_inst(const rs_autonomy::PlannerInstruction planner_instruction);
  private:
    ros::Subscriber plan_inst_sub;
};

#endif
