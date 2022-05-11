#include "TaskPlanner.h"

void TaskPlanner::callback_adap_inst_sub(const rs_autonomy::AdaptationInstruction adpt_inst)
{
  for (auto command : adpt_inst.adaptation_commands)
  {
    bool is_valid_cmd = true;
    if (command == "TerminatePlan")
    {
      planner_instruction.command = "TERMINATE";
      planner_instruction.plan_name = current_plan_name;
      planner_instruction.aux_info = "";
    }
    else if (command == "ClearArmFault")
    {
      planner_instruction.command = "ClearArmFault";
      planner_instruction.plan_name = current_plan_name;
      planner_instruction.aux_info = "";
    }
    else if (command == "Unstow") // send a PLEXIL plan for unstowing the arm
    {
      planner_instruction.command = "ADD";
      planner_instruction.plan_name = "Unstow";
      planner_instruction.aux_info = "";
    }
    else if (command.find("ManualPlan") != std::string::npos)
    {
      planner_instruction.command = "ADD";
      planner_instruction.plan_name = command; // the command variable is the plan name
      planner_instruction.aux_info = "";
    }
    else if (command == "Planning") // current task and new task
    { 
      if (task_name == adpt_inst.task_name)
      {
        current_plan_id += 1;
      }
      else
      {
	task_name = adpt_inst.task_name;
        current_plan_id = 1; // frist plan for the new task
      }
      current_plan_name = task_name + "Plan" + std::to_string(current_plan_id);

      if (task_planning_service_client.call(task_planning))
      {
        ROS_INFO_STREAM("[Planner Node] high-level plan is " << task_planning.response.high_level_plan);

        planner_instruction.command = "ADD";
        planner_instruction.plan_name = current_plan_name;
        planner_instruction.aux_info = task_planning.response.high_level_plan;
	rs_autonomy::HighLevelPlan hlp_msg;
	hlp_msg.plan = task_planning.response.high_level_plan;
	high_level_plan_pub.publish(hlp_msg);
      }
      else
      {
        // FIXME: notify the failure to the analysis componenet
	ROS_ERROR("[Planner Node] unable to talk to /task_planning service. Planning fails.");
      }
    }
    else
    {
      is_valid_cmd = false;
      ROS_INFO_STREAM("[Planner Node] receives an unknown adaptation command, " << command);
    }

    if (is_valid_cmd)
    {
      ROS_INFO_STREAM("[Planner Node] receives an adaptation command: " << command);
      this->planner_inst_pub.publish(this->planner_instruction);

      ROS_INFO_STREAM(
        "[Planner Node - Listener] send a planner instruction for the current task: "
        << "\n\tcommand: " << this->planner_instruction.command
        << "\n\tplan_name:" << this->planner_instruction.plan_name
        << "\n\taux_info: " << this->planner_instruction.aux_info
      );
    }
  }
  
}
