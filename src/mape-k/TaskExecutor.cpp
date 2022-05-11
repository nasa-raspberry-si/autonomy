#include "TaskExecutor.h"

#include <rs_autonomy/ArmFaultConfig.h>

rs_autonomy::ArmFaultConfig TaskExecutor::create_fault_clear_req()
{
  rs_autonomy::ArmFaultConfig srv;
  srv.request.action = "Clear";
  srv.request.arm_fault_var_ids = { 1, 2, 3, 4, 5, 6 };
  srv.request.values = {false, false, false, false, false, false};
  return srv;
}

void TaskExecutor::callback_planner_inst(const rs_autonomy::PlannerInstruction planner_inst)
{
  // FIXME: Clear Arm Fault
  // * This is a workaround to simulate the arm fault has been cleared. It is to
  //   ensure the lander in testbeds will stop halting after the termination of
  //   the current plan, which mimic a resolution of the arm fault.
  // * In reality, some handeling for the lander resolves the fault. 
  // * The workaround is to publish no-fault tags to ROS topics by the lander.
  if (planner_inst.command == "ClearArmFault")
  {
    // set fault tag to be false to ROS topic published by the lander
    if(fault_management_service_client.call(clear_arm_faults))
    {
      ROS_INFO("[Execute Node] The request of clearing arm faults is successfully recieved.");
    }
    else
    {
      ROS_ERROR("[Execute Node] Unable to contact /arm_fault_management service.");
    }
    
    return;
  }

  // If it is a new plan, translate the high-level plan to a plexil plan.
  if (planner_inst.command == "ADD" && planner_inst.aux_info != "")
  { 
    // When the command is "ADD", the aux_info represents the high-level plan
    //
    // The plan_name in planner_inst is the name of the root node in the,
    // plexil plan, e.g., Exca. While the plan name sent to be executed on the lander
    // should be the full file name of the compiled plexil plan, e.g., Exca.plx.
    plan_translation.request.task_name = planner_inst.task_name;
    plan_translation.request.plan_name = planner_inst.plan_name;
    plan_translation.request.high_level_plan = planner_inst.aux_info;

    // After calling translation service successfully, we get a
    // PLEXIL plan (*.plp file) and a compiled PLEXIL plan (*.plx file)
    //
    // The *.plx is the name sent to ow_exec node / owlat_exec node for execution.
    if(plan_translation_service_client.call(plan_translation))
    {
      ROS_INFO("[Execute Node] The plan translation is performed.");
    }
    else
    {
      ROS_ERROR("[Execute Node] unable to contact /plan_translation service. Failed to translate the high-level plan to a PLEXIL plan.");
    }
  }

  // Synthesize the execute instruction and send to PLEXIL executive
  // FIXME: The current autonomy design leads to that there is only one plexil 
  //        plan in the plan array, plans.
  std::vector<std::string> plan_array;
  plan_array.push_back(planner_inst.plan_name + ".plx");

  execute_instruction.request.command = planner_inst.command;
  execute_instruction.request.plans = plan_array;

  ROS_INFO_STREAM(
    "[Execute Node] sending execute instruction to the PLEXIL executive: "
    << "\n\tcommand: " << execute_instruction.request.command
    << "\n\tplans:" << execute_instruction.request.plans[0]);

  // Send the execute instruction to the ROS service /plexil_plan_selection 
  if(plan_selection_service_client.call(execute_instruction))
  {
    ROS_INFO("[Execute Node] The execute instruction is successfully recieved.");
  }
  else
  {
    ROS_ERROR("[Execute Node] Unable to contact plan_selection_server.");
  }
}

