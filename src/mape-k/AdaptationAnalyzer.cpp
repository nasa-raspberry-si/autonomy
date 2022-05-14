#include "AdaptationAnalyzer.h"
#include <rs_autonomy/CurrentTask.h> // send it to the mission control componenet


void AdaptationAnalyzer::callback_current_plan(const ow_plexil::CurrentPlan current_plan)
{
  ROS_INFO_STREAM("[Analysis Node] the current plan, " << current_plan.plan_name << ", status: " << current_plan.plan_status);
  plan_name = current_plan.plan_name;
  plan_status = current_plan.plan_status;

  if (plan_status == "Completed_Success")
  {
    current_task_status = "Completed_Success";
  }
}

void AdaptationAnalyzer::callback_high_level_plan(const rs_autonomy::HighLevelPlan msg)
{
  ROS_INFO_STREAM("[Analysis Node] the synthesized high-level plan, " << msg.plan);
  // For excavation scenario:
  //   - The high-level plan for excavation scenario has the format as
  //     "[excavation_loc_ID,dump_loc_ID]"
  //   - the aux_info is a string with a format of 
  //     "excavation_loc_ID,dump_loc_ID"
  plan_aux_info = msg.plan.substr(1, msg.plan.length()-2);
}

void AdaptationAnalyzer::callback_next_task(const rs_autonomy::NextTask next_task)
{
  ROS_INFO_STREAM("[Analysis Node] a new task is requested by the Mission control center.");
  terminate_current_task = next_task.terminate_current_task;
  next_task_name = next_task.name;
  next_task_aux_info = next_task.aux_info;
  next_task_model_names = next_task.model_names;
  has_new_task = true;
}

void AdaptationAnalyzer::callback_arm_fault_status(const rs_autonomy::ArmFault arm_fault)
{
  ROS_INFO_STREAM("[Analysis Node] a change of arm fault status is notified");
  has_arm_fault = arm_fault.has_a_fault;
  plan_status = "StopByArmFault";
  update_local_vars();
}

void AdaptationAnalyzer::callback_current_operation(const ow_plexil::CurrentOperation current_op)
{
  ROS_INFO("[Analysis Node] the current operation %s, status: %s", current_op.op_name.c_str(), current_op.op_status.c_str());
  current_op_name = current_op.op_name;
  current_op_status = current_op.op_status;
  update_local_vars();
}

void AdaptationAnalyzer::callback_vibration_level_changed(const rs_autonomy::VibrationLevel vl)
{
  ROS_INFO("[Analysis Node] the vibration level is changed from level %d to level %d.", vibration_level, vl.level);
  vibration_level = vl.level;
  wait_for_quake_off = !wait_for_quake_off;
}

void AdaptationAnalyzer::callback_earth_inst(const rs_autonomy::EarthInstruction earth_inst)
{
  if (earth_inst.plan_name != "") {
    // Assume the plan name follows the format, ManualPlanXXX.plx
    manual_planname = earth_inst.plan_name;
    has_manual_plan = true;
    ROS_INFO_STREAM("[Analysis Node] receive an instruction from the Earth to run " << manual_planname);
  }
  else
  {
    ROS_INFO_STREAM("[Analysis Node] receive an instruction from the Earth to run a new plan, but the plan name is empty.");
  }
}

// FIXME:
// This is to assist the simulation of excavation failure.
// It should be replaced when a more realistic simulation is available
void AdaptationAnalyzer::update_local_vars()
{
  if (has_arm_fault)
  {
    if (current_op_name == "Digging" && current_op_status == "starts")
    {
      current_digging_failed = true;
      num_digging_failures += 1;
    }
  }
}

// ROS service request: Update Models
// Currently supported models: "SciVal", "ExcaProb"
void AdaptationAnalyzer::update_models(std::vector<std::string> model_names)
{
  model_update_inst.request.task_name = current_task_name;
  model_update_inst.request.model_names = model_names;

  if(model_update_service_client.call(model_update_inst))
  {
    ROS_INFO("The Model Update is performed.");
  }
  else
  {
    ROS_INFO("Can not talk to the Model Update service, /update_models.");
  }
}

// ROS service request: Runtime Information Maintenance
// Input Parameters
// actions         aux_info (format: str1,str2,...)
// "Initialize"    "#xloc,dloc"
// "Update"        "model1", "model1,model2", ...
// "Remove"        "item1_ID", ...
void AdaptationAnalyzer::maintain_rtInfo(std::string action, std::string aux_info)
{
  rtInfo_maintenance_inst.request.task_name = current_task_name;
  rtInfo_maintenance_inst.request.action = action;
  rtInfo_maintenance_inst.request.aux_info = aux_info;
  if(rtInfo_maintenence_service_client.call(rtInfo_maintenance_inst))
  {
    ROS_INFO("The Runtime Info Update is performed.");
  }
  else
  {
    ROS_INFO("Can not talk to the Runtime Info Update service, /runtime_info_maintenance");
  }
}

void AdaptationAnalyzer::update_task_control_vars()
{
  current_task_name = next_task_name;
  next_task_name = "";
  current_task_aux_info = next_task_aux_info;
  next_task_aux_info = "";
  
  current_task_model_names = next_task_model_names;
  next_task_model_names.clear(); 

  terminate_current_task = false; // reset it to false to allow the task to be carried out
  has_new_task = false; // notify the Mission componenet that it can send a new task

  current_task_status = "Ready"; // reset it to "Ready", indicating the task is not starting yet.

  rs_autonomy::CurrentTask current_task_msg;
  current_task_msg.name = current_task_name;
  current_task_msg.status = current_task_status;
  current_task_status_pub.publish(current_task_msg);
}

void AdaptationAnalyzer::initialize_rtInfo()
{
  update_models(current_task_model_names);
  std::string action = "Initialize";
  maintain_rtInfo(action, current_task_aux_info);
}

// Transition the lander to a safey pose (Unstow pose) with arm fault cleared
void AdaptationAnalyzer::transition_to_safe_pose()
{
  ROS_INFO_STREAM("[Analysis Node] Transitioning the arm to a safe pose");
  adpt_inst.adaptation_commands = { "TerminatePlan", "Unstow"};
  if (has_arm_fault)
  {
    // Assume that before Unstow operation, the arm fault has been cleared
    adpt_inst.adaptation_commands = { "TerminatePlan", "ClearArmFault", "Unstow"};
  }
  adpt_inst.task_name = current_task_name;
  adpt_inst_pub.publish(adpt_inst);
}

void AdaptationAnalyzer::planning()
{
  adpt_inst.task_name = current_task_name;
  adpt_inst.adaptation_commands = { "Planning"};
  adpt_inst_pub.publish(adpt_inst);
}

// Update models and prepare the initial runtime info for the current task
void AdaptationAnalyzer::initialize_task(bool syn_plan, bool terminate_current_task)
{
  // if the current task has been started, there should a plan is running for it,
  // so terminate the plan and transition the lander to the safe pose, Unstow pose.
  if (current_task_status == "Started")
  {
    ROS_INFO_STREAM("[Analysis Node] terminating the current plan of the current task"
                    << current_task_name);

    transition_to_safe_pose();
    if(terminate_current_task)
    {
      rs_autonomy::CurrentTask current_task_msg;
      current_task_msg.name = current_task_name;
      current_task_msg.status = "Terminated";
      current_task_status_pub.publish(current_task_msg);
      ROS_INFO_STREAM("[Analysis Node] the current task is terminated");
    }
  }

  // if there is a new task, update task control variables to transition to the new task 
  if (has_new_task)
  {
    ROS_INFO_STREAM("[Analysis Node] transitioning to the new task");
    update_task_control_vars();
    ROS_INFO_STREAM("[Analysis Node] ready to do the new task: " << current_task_name);
  }

  if (!wait_for_quake_off)
  {
    // Update all models (maybe not necessary) and runtime info for the new task
    initialize_rtInfo();

    if (syn_plan) // Synthesize a plan to run
    {
      ROS_INFO_STREAM("[Analysis Node] request a plan for the task " << current_task_name);
      planning();

      if(current_task_status == "Ready")
      {
        current_task_status = "Started";
        rs_autonomy::CurrentTask current_task_msg;
        current_task_msg.name = current_task_name;
        current_task_msg.status = current_task_status;
        current_task_status_pub.publish(current_task_msg);
      }
      /*
      ROS_INFO_STREAM("[Analysis Node] the task "
                      << current_task_name
                      <<" is ready to run a plan "
                      << plan_name);
      */
    }
  }
  else
  {
    ROS_INFO_STREAM("[Analysis Node] the lander is waiting for the quake to pass.");
  }
}

void AdaptationAnalyzer::adaptation_analysis()
{
  // Determine which adaptation should be triggered
  if (has_manual_plan && manual_planname != "") // A manual plan from the Earth
  {
    ROS_INFO_STREAM("[Analysis Node] Has a plan from the Earth control center");
    // The substring, "ManualPlan", in manual_planname, will make
    // the planner componenet to give an instruction to the execute
    // compoenent to run the manual plan.
    // Assume the manual plan from the Earth still try to complete
    // the current task, but not starts a new task.
    transition_to_safe_pose();

    adpt_inst.task_name = current_task_name;
    adpt_inst.adaptation_commands = { manual_planname };
    adpt_inst_pub.publish(adpt_inst);
  }
  else if (!wait_for_quake_off) // The lander is in active mode
  {
    if (vibration_level == 1) // An earthquake is detected. 
    {
      ROS_INFO_STREAM("[Analysis Node] A quake is detected. The vibration level is changed to 1");

      // Transition the lander to a safe mode
      wait_for_quake_off = !wait_for_quake_off; // change from false to true
      bool syn_plan = false;
      initialize_task(syn_plan);
      // Currently, leave re-planning when the vibration level comes back to 0
    }
    else if (terminate_current_task && has_new_task) // a new task is requsted to do instead of the current one
    {
      ROS_INFO_STREAM("[Analysis Node] a new task is requested and the current task will be termianted.");
 
      bool syn_plan = true;
      initialize_task(syn_plan, terminate_current_task); 
    }
    else if (num_digging_failures == 3) // the belief of models drops too low, such initialize the task
    {
      ROS_INFO_STREAM("[Analysis Node] the belief of both models drops below a threshold. A model update request is issued.");
 
      bool syn_plan = true;
      initialize_task(syn_plan);
    }
    else if (num_digging_failures == 2) // the belief of models drops a little, update models and rtInfo
    {
      ROS_INFO_STREAM("[Analysis Node] the belief of excavation probability model drops below a threshold. A model update request is issued.");
 
      transition_to_safe_pose();

      // Update Excavation-Probability Model and Runtime Info
      std::vector<std::string> model_names = {"ExcaProb"};
      update_models(model_names);
      std::string action = "Update";
      std::string aux_info = "ExcaProb";
      maintain_rtInfo(action, aux_info);

      planning();
    }
    else if (current_digging_failed || has_arm_fault)
    {
      ROS_INFO_STREAM("[Analysis Node] the digging fails while the belief of models are still ok. Update the runtime information by removing the current excavalocation.");
 
      transition_to_safe_pose();

      // FIXME: here is for excavaion sceanrio only
      std::size_t pos = plan_aux_info.find(","); // pos is the location of first, ","
      std::string aux_info = plan_aux_info.substr(pos); // the ID of excavation location
      std::string action = "Remove";
      maintain_rtInfo(action, aux_info);

      planning();
    }
    // 1. start the first task
    // 2. start the next task when current task is completed
    else if (has_new_task && (current_task_name=="" || current_task_status=="Completed_Success")) 
    {
      if (current_task_name=="")
      {
        ROS_INFO("[Analysis Node] Starts to do the first task for the mission");
      }
      else
      {
        ROS_INFO_STREAM("[Analysis Node] The current task is finished successfully. Starts to do a new task.");
      }
      bool syn_plan = true;
      initialize_task(syn_plan);
    }
  }
  else // The lander is in active mode due to a previously detected earthquake
  {
    if (vibration_level == 0) // the earthquake has gone
    {
      ROS_INFO_STREAM("[Analysis Node] The quake has stopped. The vibration level comes back to the normal leve 0");
      // clear the waiting tag
      wait_for_quake_off = !wait_for_quake_off; // change from true to false

      // update runtime info and synthesize a plan
      initialize_rtInfo(); 
      planning();
    }
  }

}

