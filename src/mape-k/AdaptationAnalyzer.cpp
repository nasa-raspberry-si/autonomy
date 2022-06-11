#include "AdaptationAnalyzer.h"
#include <rs_autonomy/CurrentTask.h> // send it to the mission control componenet


void AdaptationAnalyzer::callback_current_plan(const ow_plexil::CurrentPlan current_plan)
{
  if (is_debug)
  {
    ROS_INFO_STREAM("[Analysis Node] the current plan, " << current_plan.plan_name << ", status: " << current_plan.plan_status);
  }

  // A new plan starts, so reset "plan_retries"
  if (plan_name != current_plan.plan_name)
  {
    plan_retries = 0;
  }

  plan_name = current_plan.plan_name;
  plan_status = current_plan.plan_status;

  if (plan_status == "Completed_Success")
  {
    current_task_status = "Completed_Success";
  }
  else if (plan_status == "Terminated")
  {
    terminating_current_plan = false;
  }

}

void AdaptationAnalyzer::callback_current_operation(const ow_plexil::CurrentOperation current_op)
{
  if (is_debug)
  {
    ROS_INFO("[Analysis Node] the current operation %s, status: %s", current_op.op_name.c_str(), current_op.op_status.c_str());
  }
  current_op_name = current_op.op_name;
  current_op_status = current_op.op_status;
  update_local_vars();
}

void AdaptationAnalyzer::callback_high_level_plan(const rs_autonomy::HighLevelPlan msg)
{
  // FIXME:
  // The location IDs published in this topic are actually keys in the runtime
  // information dictionary.
  // Their actuall location names are stored in the attribute, 'name', in each
  // location in the runtime information dictionary.
  // Will disable the message out.
  ROS_INFO_STREAM("[Analysis Node] the synthesized high-level plan (loc info are keys in runtime info dictionary, actual names can be found in the runtime info dictionary): " << msg.plan);
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
  ROS_INFO_STREAM("[Analysis Node] a change of arm fault status is notified. Is there an arm fault: " + std::to_string(arm_fault.has_a_fault));
  has_arm_fault = arm_fault.has_a_fault;
  if(has_arm_fault)
  {
    plan_status = "StopByArmFault";
  }
  else
  {
    /*
    int timepassed = 0;
    int waittime = 30; // maximum waiting time 30 seconds
    ros::Rate rate(10);
    while(timepassed < waittime)
    {
      ros::spinOnce();
      rate.sleep();
      timepassed+=1;
      if(timepassed % 10 == 0)
      {
        ROS_INFO("[Analysis Node] waiting for the arm fault clear signal to be fully reflected in the lander system in %i seconds", (waittime/10 - timepassed/10));
      }
    }
    */
    // The previous arm fault has been cleared.
    // The status of arm fault changes from true to false
    clearing_arm_fault = false;
  }
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

// FIXME:
// This is to assist the simulation of excavation failure.
// It should be replaced when a more realistic simulation is available
void AdaptationAnalyzer::update_local_vars()
{
  if (has_arm_fault)
  {
    if (current_op_name == "Grind" && current_op_status == "Started")
    {
      current_digging_failed = true;
      num_digging_failures += 1;
      digging_failure_adaptation_on = true;
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

void AdaptationAnalyzer::initialize_rtInfo()
{
  update_models(current_task_model_names);
  std::string action = "Initialize";
  maintain_rtInfo(action, current_task_aux_info);
}

// Clear Arm Fault
void AdaptationAnalyzer::clear_arm_fault()
{
  // Due to some bug in the fault handling in ow_simulator, when there is an effective
  // arm fault, it will take some time for the corresponding ROS action to finish in the 
  // ABORTED state such that PLEXIL executive could be ready for following plans.
  //
  // In current excavation scenario (the excavation plan template):
  //   * The ABORTED state of either GuardedMove and Grind will cause the current plan to finish with a status of Completed_Failure
  //   * The ABORTED state of DigCircular will not cause the plan to fail immediately because a Deliver action follows it. So, autonomy will send a TerminatePlan signal at the time.
  // 
  // Here, it is assumed that the SUT will stop and cause the PLEXIL plan to fail
  // when an arm fault is detected.

  int timepassed = 0;
  int waittime = 900; // maximum waiting time 90 seconds
  ros::Rate rate(10);
  while((plan_status!= "Completed_Failure") && (timepassed < waittime))
  {
	ros::spinOnce();
	rate.sleep();
	timepassed+=1;
	if(timepassed % 50 == 0)
	{
	ROS_INFO("[Analysis Node] waiting the arm-fault-interrupted current plan to finish in %i seconds", (waittime/10 - timepassed/10));
	}
  }

  clearing_arm_fault = true;
  adpt_inst.adaptation_commands = {"ClearArmFault"};
  adpt_inst.task_name = current_task_name;
  adpt_inst_pub.publish(adpt_inst);
}

// Terminating the current plan
void AdaptationAnalyzer::terminate_current_plan()
{
  ROS_INFO_STREAM("[Analysis Node] Need to terminate the current plan");

  // If there is an arm fault, clear it first
  if (has_arm_fault) {
    clear_arm_fault();
  }

  // If the plan does not finish, send the instruction, TerminatePlan
  if (plan_status != "Completed_Failure"
		&& plan_status != "Completed_Success"
		&& plan_status != "Terminated")
  {
    adpt_inst.adaptation_commands = { "TerminatePlan"};
    terminating_current_plan = true;

    adpt_inst.task_name = current_task_name;
    adpt_inst_pub.publish(adpt_inst);
  }
}

void AdaptationAnalyzer::planning()
{
  adpt_inst.task_name = current_task_name;
  adpt_inst.adaptation_commands = { "Planning"};
  adpt_inst_pub.publish(adpt_inst);
}

// Update models and prepare the initial runtime info for the current task
void AdaptationAnalyzer::initialize_task(bool terminate_current_task)
{
  // if the current task has been started, there should a plan is running for it,
  // so terminate the plan and transition the lander to the safe pose, Unstow pose.
  if (current_task_status == "Started")
  {
    ROS_INFO_STREAM("[Analysis Node] terminating the current plan of the current task"
                    << current_task_name);

    terminate_current_plan();
    if(terminate_current_task)
    {
      rs_autonomy::CurrentTask current_task_msg;
      current_task_msg.name = current_task_name;
      current_task_msg.status = "Terminated";
      current_task_status_pub.publish(current_task_msg);
      ROS_INFO_STREAM("[Analysis Node] the current task is terminated");
    }
  }

  ROS_INFO_STREAM("[Analysis Node] transitioning to the new task");
  update_task_control_vars();
  ROS_INFO_STREAM("[Analysis Node] ready to do the new task: " << current_task_name);

  // Update all models (maybe not necessary) and runtime info for the new task
  initialize_rtInfo();

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
}

void AdaptationAnalyzer::adaptation_analysis()
{
  if (terminating_current_plan || clearing_arm_fault)
  {
    return;
  }

  // Start adaptation analysis when
  // the lander is ready: not clearing_arm_fault
  // the PLEXIL executive is ready: not terminating_current_plan. That is,
  //   the PLEXIL executive finishes with the current plan


  // Determine which adaptation should be triggered
  if (has_manual_plan && manual_planname != "") // A manual plan from the Earth
  {
    ROS_INFO_STREAM("[Analysis Node] Has a plan from the Earth control center");
    // The substring, "ManualPlan", in manual_planname, will make
    // the planner componenet to give an instruction to the execute
    // compoenent to run the manual plan.
    // Assume the manual plan from the Earth still try to complete
    // the current task, but not starts a new task.
    terminate_current_plan();

    adpt_inst.task_name = current_task_name;
    adpt_inst.adaptation_commands = { manual_planname };
    adpt_inst_pub.publish(adpt_inst);

	// reset to avoid infinite loop
    has_manual_plan = false;
    manual_planname = "";
  }
  else if (!wait_for_quake_off) // The lander is in active mode
  {
    if (vibration_level == 1) // An earthquake is detected. 
    {
      ROS_INFO_STREAM("[Analysis Node] A quake is detected. The vibration level is changed to 1");

      // Transition the lander to a safe mode
      wait_for_quake_off = !wait_for_quake_off; // change from false to true
      // Terminate the current plan
      terminate_current_plan();
      // Leave re-planning when the vibration level comes back to 0
    }
    else if (terminate_current_task && has_new_task) // a new task is requsted to do instead of the current one
    {
      ROS_INFO_STREAM("[Analysis Node] a new task is requested and the current task will be termianted.");
      // terminate the current task
      // initialzation for the new task
      initialize_task(terminate_current_task);
    }
    // Use as a case to indicate the beliefs of both ExcaProb and SciVal models drops significantly
    // Therefore, both of the models should be updated
    else if (digging_failure_adaptation_on && num_digging_failures == 3)
    {
      ROS_INFO_STREAM("[Analysis Node] digging fails for 3 times. The beliefs of both ExcaProb and SciVal models drop too much. They need to be updated.");
 
      // Update all machine-learning models
      terminate_current_plan();

      // Update Excavation-Probability Model and Science-value Model
      // It will regenerate the entire runtime information with a randomly chosen
      // number of excavation locations ([8, 16]) and the number of dump locations
      // ([3, 6]) since all models will be updated and lists of appropriate locations
      // for excavation and dumpping will be changed.
      std::vector<std::string> model_names = {"ExcaProb", "SciVal"};
      update_models(model_names);
      // Update Runtime info
      std::string action = "Update";
      std::string aux_info = "ExcaProb,SciVal";
      maintain_rtInfo(action, aux_info);

      // Request synthesizing a new plan
      planning();

      digging_failure_adaptation_on = false;
    }
    // the belief of models drops a little, update models and rtInfo
    else if (digging_failure_adaptation_on && num_digging_failures == 2) 
    {
      ROS_INFO_STREAM("[Analysis Node] digging fails twice. The belief of the ExcaProb model drops too much. It needs to be updated.");
 
      terminate_current_plan();
      
      // Remove the location where excavation just failed.
      std::size_t pos = plan_aux_info.find(","); // pos is the location of first, ","
      std::string aux_info = plan_aux_info.substr(7, pos-7); // the ID of excavation location
      std::string action = "Remove";
      maintain_rtInfo(action, aux_info);

      // Update Excavation-Probability Model and Runtime Info for the left excavation locations
      std::vector<std::string> model_names = {"ExcaProb"};
      update_models(model_names);
      action = "Update";
      aux_info = "ExcaProb";
      maintain_rtInfo(action, aux_info);

      planning();

      digging_failure_adaptation_on = false;
    }
    // The first failure of digging: arm fault is detected during Grind operation
    else if (digging_failure_adaptation_on && num_digging_failures == 1)
    {
      ROS_INFO_STREAM("[Analysis Node] The digging fails once while the beliefs of models are still ok. Update the runtime information by removing the current excavation location.");
 
      terminate_current_plan();

      // FIXME: here is for excavaion sceanrio only
      std::size_t pos = plan_aux_info.find(","); // pos is the location of first, ","
      std::string aux_info = plan_aux_info.substr(7, pos-7); // the ID of excavation location
      std::string action = "Remove";
      // ROS_INFO_STREAM("[Analysis Node] plan_aux_info for Remove action: " + plan_aux_info); 
      // ROS_INFO_STREAM("[Analysis Node] aux_info for Remove action: " + aux_info);
      maintain_rtInfo(action, aux_info);

      planning();

      digging_failure_adaptation_on = false;
    }
    // Cases:
    // a. an arm fault is detected in operations (e.g., GuardedMove) other than Grind operation
    else if (has_arm_fault)
    {
      ROS_INFO_STREAM("[Analysis Node] An arm fault is notified during an arm operation other than Grind. Update the runtime information by removing the current excavation location.");
      terminate_current_plan();
 
      std::size_t pos = plan_aux_info.find(","); // pos is the location of first, ","
      std::string aux_info = plan_aux_info.substr(7, pos-7); // the ID of excavation location
      std::string action = "Remove";
      ROS_INFO_STREAM("[Analysis Node] plan_aux_info for Remove action: " + plan_aux_info); 
      ROS_INFO_STREAM("[Analysis Node] aux_info for Remove action: " + aux_info);
      maintain_rtInfo(action, aux_info);

      planning();
    }
    // Cases:
    // a. start the first task
    // b. start the next task when current task is completed successfully
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
      // Initialize runtime info for the task
      // Request synthesizing a plan
      initialize_task();
    }
    else if (plan_status=="Plan_Registration_Timeout" && plan_retries<3)
    {
      plan_retries += 1;
      std::string plan_tries_str = "";
      if (plan_retries == 1)
      {
        plan_tries_str = "(1st time)";
      }
      else if (plan_retries == 2)
      {
        plan_tries_str = "(2nd time)";
      }
      else // plan_tries == 3
      {
        plan_tries_str == "(3rd time)";
      }

      ROS_INFO("[Analysis Node] Wait for some ROS actions' states to finish before retrying the timed-out plan, %s.plx", plan_name.c_str());
      int timepassed = 0;
      int waittime = 50; // maximum waiting time 5 seconds
      ros::Rate rate(10);
      while(timepassed < waittime)
      {
        ros::spinOnce();
        rate.sleep();
        timepassed+=1;
        if(timepassed % 10 == 0)
        {
          ROS_INFO("[Analysis Node] Waiting in %d seconds", (waittime-timepassed)/10);
        }
      }
      
      ROS_INFO("[Analysis Node] %s Retry the plan, %s.plx, whose plan registration was timed out", plan_tries_str.c_str(), plan_name.c_str());
      
      adpt_inst.task_name = current_task_name;
      adpt_inst.adaptation_commands = { "RetryPlan" };
      adpt_inst_pub.publish(adpt_inst);
    }
  }
  else // The lander is in active mode due to a previously detected earthquake
  {
    if (vibration_level == 0) // the earthquake has gone
    {
      ROS_INFO_STREAM("[Analysis Node] The quake has stopped. The vibration level comes back to the normal leve 0");
      // clear the waiting tag
      wait_for_quake_off = !wait_for_quake_off; // change from true to false

      // update machine-learning models and runtime info to reflect the updated terrain
      // and surface structure after the quake.
      // synthesize a plan using the updated runtime info
      initialize_rtInfo(); 
      planning();
    }
  }

}

