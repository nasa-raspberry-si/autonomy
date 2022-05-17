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
#include <vector>

// OW
#include <rs_autonomy/ArmFaultConfig.h> // for calling fault management service to clear faults
#include <rs_autonomy/PlannerInstruction.h>
#include <rs_autonomy/PlanTranslation.h>
#include <rs_autonomy/ArmFault.h>
#include <ow_plexil/PlanSelection.h>
#include <ow_plexil/CurrentPlan.h>

//Others
#include "message_passing_support.h" // for msg_queue_size


class TaskExecutor {
  public:
    TaskExecutor(ros::NodeHandle *nh)
    {
      has_arm_fault = false;

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
      current_plan_sub = nh->subscribe<ow_plexil::CurrentPlan>(
                  "/Monitor/CurrentPlan",
                  msg_queue_size,
                  &TaskExecutor::callback_current_plan,
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

    std::string current_plan_name;
    std::string current_plan_status;
    bool has_arm_fault;

    void callback_arm_fault_status(const rs_autonomy::ArmFault fault_msg);
    void callback_current_plan(const ow_plexil::CurrentPlan current_plan);
    void callback_planner_inst(const rs_autonomy::PlannerInstruction planner_instruction);
    void send_inst_to_plexil_executive(std::string command, std::string plan);
    void terminate_plan(std::string plan);
    void clear_arm_fault();
    void handle_exec_commands();

  private:
    std::vector<std::string> exec_commands;
    std::vector<std::string> exec_plans;
    ros::Subscriber plan_inst_sub;
    ros::Subscriber current_plan_sub;
};

#endif
