#include "TaskExecutor.h"

#include <rs_autonomy/ArmFaultConfig.h>

rs_autonomy::ArmFaultConfig TaskExecutor::create_fault_clear_req()
{
  rs_autonomy::ArmFaultConfig srv;
  srv.request.action = "Clear";
  srv.request.arm_fault_var_ids = {
	  "dist_pitch_joint_locked_failure",
          "hand_yaw_joint_locked_failure",
          "prox_pitch_joint_locked_failure",
          "scoop_yaw_joint_locked_failure",
          "shou_pitch_joint_locked_failure",
          "shou_yaw_joint_locked_failure"

  };
  srv.request.values = {
	  "False",
	  "False",
	  "False",
	  "False",
	  "False",
	  "False"
  };
  return srv;
}

void TaskExecutor::callback_arm_fault_status(const rs_autonomy::ArmFault fault_msg)
{
  has_arm_fault = fault_msg.has_a_fault;
}

void TaskExecutor::callback_current_plan(const ow_plexil::CurrentPlan current_plan)
{
  current_plan_name = current_plan.plan_name;
  current_plan_status = current_plan.plan_status;
}

void TaskExecutor::callback_planner_inst(const rs_autonomy::PlannerInstruction planner_inst)
{
  std::string command = "";
  std::string plan = "";
  bool is_valid_cmd = true;
  if (planner_inst.command == "ClearArmFault")
  {
    command = "ClearArmFault";
    plan = "";
  }
  else if (planner_inst.command == "TERMINATE")
  {
    command = "TERMINATE";
    plan = planner_inst.plan_name + ".plx";
  }
  // ADD command:
  // If it is a new plan, translate the high-level plan to a plexil plan.
  // If it is a ManualXXX plan from the Earth, just pass it to the lander
  else if (planner_inst.command == "ADD")
  { 
    if (planner_inst.aux_info != "")
    {
		// When the command is "ADD", a non-empty aux_info represents the high-level plan
		// The plan_name in planner_inst is the name of the root node in the, plexil plan, e.g., Exca.
		plan_translation.request.task_name = planner_inst.task_name;
		plan_translation.request.plan_name = planner_inst.plan_name;
		plan_translation.request.high_level_plan = planner_inst.aux_info;

		// After calling translation service successfully, we get a
		// PLEXIL plan (*.plp file) and a compiled PLEXIL plan (*.plx file)
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

    command = "ADD";
    plan = planner_inst.plan_name + ".plx";
  }
  else
  {
    ROS_ERROR_STREAM("[Execute Node] Received an invalid command: " << planner_inst.command);
    is_valid_cmd = false;
  }

  if (is_valid_cmd)
  {
    exec_commands.push_back(command);
    exec_plans.push_back(plan);

    ROS_INFO_STREAM(
      "[Execute Node] Received an instruction from the planner node:"
      << "\n\tcommand: " << command
      << "\n\tplan:" << plan);
  }
}

void TaskExecutor::send_inst_to_plexil_executive(std::string command, std::string plan)
{
  // Synthesize the execute instruction and send to PLEXIL executive
  std::vector<std::string> plan_array;
  plan_array.push_back(plan);

  execute_instruction.request.command = command;
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

// Terminate the currently executing plan
void TaskExecutor::terminate_plan(std::string plan)
{
  send_inst_to_plexil_executive("TERMINATE", plan);

  ros::Rate rate(10);
  //wait for the plan to be fully terminated
  while(current_plan_status!="Terminated" && current_plan_status!="Completed_Failure")
  {
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("[Execute Node] The current plan is terminated. Continue to process the next command.");
}

// Clear Arm Fault 
void TaskExecutor::clear_arm_fault()
{
  // FIXME: Clear Arm Fault
  // * This is a workaround to simulate the arm fault has been cleared. It is to
  //   ensure the lander in testbeds will stop halting after the termination of
  //   the current plan, which mimic a resolution of the arm fault.
  // * In reality, some handeling for the lander resolves the fault. 
  // * The workaround is to publish no-fault tags to ROS topics by the lander.

  // set fault tag to be false to ROS topic published by the lander
  if(fault_management_service_client.call(clear_arm_faults))
  {
	ROS_INFO("[Execute Node] The request of clearing arm faults is successfully recieved.");
  }
  else
  {
	ROS_ERROR("[Execute Node] Unable to contact /arm_fault_management service.");
  }

  // wait for the arm fault signal to be cleared
  ros::Rate rate(10);
  while(has_arm_fault)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // wait 3 secondsfor the fault to be fully resolved in the system
  int timepass = 0;
  while(timepass<30)
  {
    ros::spinOnce();
    rate.sleep();
    timepass+=1;
    if(timepass % 10 == 0)
    {
      ROS_INFO("[Execute Node] Waiting the arm fault to fully resolved in %i seconds", (3-timepass/10));
    }
  }
  ROS_INFO("[Execute Node] The arm fault has been cleared now.");
}

void TaskExecutor::handle_exec_commands()
{
  ros::Rate rate(1);
  int length = 0;
  while(ros::ok())
  {
    if (exec_commands.size() > 0) {
      if (exec_commands[0] == "ClearArmFault")
      {
        clear_arm_fault();
      }
      else if (exec_commands[0] == "TERMINATE")
      {
        terminate_plan(exec_plans[0]);
      }
      else // (exec_commands[0] == "ADD")
      {
        send_inst_to_plexil_executive("ADD", exec_plans[0]);
      }

      exec_commands.erase(exec_commands.begin());
      exec_plans.erase(exec_plans.begin());
    }
    // if no commands yet
    ros::spinOnce();
    rate.sleep();
  }
}
