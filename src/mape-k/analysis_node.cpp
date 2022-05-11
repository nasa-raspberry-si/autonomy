// Role of The Analysis Component
// * Pull information from Monitor componenet
// * Send requests to Knowledge component to update models and maintain runtime info
// * Determine what adaptation is needed for the current task
// * Send adaptation instruction to the Plan componenet
// * Comunicate with the Mission componenet to report the task status and/or start a new task

// C++
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Autonomy
#include <rs_autonomy/RTInfoMaintenanceInstruction.h>
#include <rs_autonomy/ModelUpdateInstruction.h>

#include <rs_autonomy/AdaptationInstruction.h> // send it to the planner componenet
#include <rs_autonomy/CurrentTask.h> // send it to the mission control componenet
#include <rs_autonomy/NextTask.h> // published by the Mission Control componenet

// published by the monitor component
#include <rs_autonomy/VibrationLevel.h>
#include <rs_autonomy/EarthInstruction.h>
#include <rs_autonomy/ArmFault.h>
#include <ow_plexil/CurrentOperation.h>
#include <ow_plexil/CurrentPlan.h>

//Others
#include "message_passing_support.h" // for msg_queue_size



// Currently, the adaptation_analysis implements seven cases in excavation scenario
// More details can be found in rs_autonomy/doc/analysis_node_description.txt
//
// FIXME: for other excavation cases and cases in other task (sample identification)
//        this function needs to be updated accordingly
class AdaptationAnalyzer {
  public:
    AdaptationAnalyzer(ros::NodeHandle *nh)
    {
      // Publisher for sending adaptation commands to the planner component
      adpt_inst_pub = nh->advertise<rs_autonomy::AdaptationInstruction>(
 		      "/AdaptationInstruction", msg_queue_size);
      current_task_status_pub = nh->advertise<rs_autonomy::CurrentTask>(
		      "/Analysis/CurrentTask", msg_queue_size);

      // callbacks for subscribered ROS topics
      next_task_sub = nh->subscribe<rs_autonomy::NextTask>(
    		      "/Mission/NextTask",
    		      msg_queue_size,
    		      &AdaptationAnalyzer::callback_next_task,
    		      this);
      current_plan_sub = nh->subscribe<ow_plexil::CurrentPlan>(
    		      "/Monitor/CurrentPlan",
    		      msg_queue_size,
    		      &AdaptationAnalyzer::callback_current_plan,
    		      this);
      arm_fault_sub = nh->subscribe<rs_autonomy::ArmFault>(
    		      "/Monitor/ArmFaultStatus",
    		      msg_queue_size,
    		      &AdaptationAnalyzer::callback_arm_fault_status,
    		      this);
      current_operation_sub = nh->subscribe<ow_plexil::CurrentOperation>(
    		      "/Monitor/CurrentOperation",
    		      msg_queue_size,
    		      &AdaptationAnalyzer::callback_current_operation,
    		      this);
      vibration_level_sub = nh->subscribe<rs_autonomy::VibrationLevel>(
    		      "/Monitor/VibrationLevelChanged",
    		      msg_queue_size,
    		      &AdaptationAnalyzer::callback_vibration_level_changed,
    		      this);
      earth_inst_sub = nh->subscribe<rs_autonomy::EarthInstruction>(
    		      "/Monitor/EarthInstruction",
    		      msg_queue_size,
    		      &AdaptationAnalyzer::callback_earth_inst,
    		      this);
      
      // ROS clients for update models and runtime information
      rtInfo_maintenence_service_client = nh->serviceClient<rs_autonomy::RTInfoMaintenanceInstruction>("//runtime_info_maintenance");
      model_update_service_client = nh->serviceClient<rs_autonomy::ModelUpdateInstruction>("/update_models");

    }

    // Publisher and message for sending the instruction to the plan component
    ros::Publisher adpt_inst_pub;
    ros::Publisher current_task_status_pub;
    rs_autonomy::AdaptationInstruction adpt_inst;

    // Variables that capture necessary info for adaptation_analysis()
    // They could be modifed by externally through ROS topics
    //// NextTask
    bool terminate_current_task = false;
    std::string next_task_name = "";
    std::string next_task_aux_info = "";
    std::vector<std::string> model_names; // models used for the task
    //// ArmFault
    bool has_arm_fault = false;
    //// CurrentOperation
    std::string current_op_name = "":
    std::string current_op_status = "";
    //// Vibration caused by earthquake
    int vibration_level = 0;
    //// A manual plan from the Earth center 
    //// manual_planname has the format, ManualPlanXXX.plx
    //// This eases message passing for it in the implementation
    std::string manual_planname = "";
    //// CurrentPlan
    std::string plan_name = "";
    std::string plan_status = ""; 
    std::string plan_aux_info = "";
    
    // Updated in the corresponding callback functions
    bool has_manual_plan = false;
    bool wait_for_quake_off = false;
    bool has_new_task = false;
    std::string current_task_name = "";
    std::string current_task_aux_info = "";
    //// TBD (3 status): "started", "completed", "terminated" 
    std::string current_task_status = "";

    // The following member variables depends on the variables above
    // and should be updated accordingly by calling update_local_vars()
    int num_digging_failures = 0;
    bool current_digging_failed = false;

    // constantly update num_digging_failures and current_digging_failed
    // when a related topic value is changed
    void update_local_vars();

    void update_task_control_vars();
    void update_models(std::vector<std::string> model_names);
    void maintain_rtInfo(std::string action, std::string aux_info);
    //// syn_plan indicates to synthesize a plan
    void initialize_task(bool syn_plan=true, bool terminate_current_task=false);
    void initialize_rtInfo();
    void transition_to_safe_pose();
    void planning();

    // callbacks for subscribered ROS topics
    void callback_current_operation(const ow_plexil::CurrentOperation current_op);
    void callback_current_plan(const ow_plexil::CurrentPlan current_plan);
    void callback_next_task(const rs_autonomy::NextTask next_task);
    void callback_arm_fault_status(const rs_autonomy::ArmFault arm_fault);
    void callback_vibration_level_changed(const rs_autonomy::VibrationLevel vl);
    void callback_earth_inst(const rs_autonomy::EarthInstruction earth_inst);

    // ROS clients for update models and runtime information
    rs_autonomy::ModelUpdateInstruction model_update_inst;
    rs_autonomy::RTInfoMaintenanceInstruction rtInfo_maintenance_inst;
    ros::ServiceClient rtInfo_maintenence_service_client;
    ros::ServiceClient model_update_service_client;

    // constantly checking if an adaptation is needed
    void adaptation_analysis();

  private:
    ros::Subscriber next_task_sub;
    ros::Subscriber current_plan_sub;
    ros::Subscriber arm_fault_sub;
    ros::Subscriber current_operation_sub;
    ros::Subscriber vibration_level_sub;
    ros::Subscriber earth_inst_sub;
 
};

void AdaptationAnalyzer::callback_current_plan(const ow_plexil::CurrentPlan current_plan)
{
  ROS_INFO_STREAM("[Analysis Node] the current plan, " << current_plan.plan_name << ", status: " << current_plan.plan_status);
  plan_name = current_plan.plan_name;
  plan_status = current_plan.plan_status;
  // for excavation scenario, the aux_info is a string with a format of 
  // "excavation_loc_ID,dump_loc_ID"
  plan_aux_info = current_plan.aux_info;
}

void AdaptationAnalyzer::callback_next_task(const rs_autonomy::NextTask next_task)
{
  ROS_INFO_STREAM("[Analysis Node] a new task is requested by the Mission control center.");
  terminate_current_task = next_task.terminate_current_task;
  next_task_name = next_task.name;
  next_task_aux_info = next_task.aux_info;
  model_names = next_task.model_names;
  has_new_task = true;
}

void AdaptationAnalyzer::callback_arm_fault_status(const rs_autonomy::ArmFault arm_fault)
{
  ROS_INFO_STREAM("[Analysis Node] a change of arm fault status is notified");
  has_arm_fault = arm_fault.has_a_fault;
  update_local_vars();
}

void AdaptationAnalyzer::callback_current_operation(const ow_plexil::CurrentOperation.h current_op)
{
  ROS_INFO_STREAM("[Analysis Node] the current operation " << current_op.op_name << ", status: " << current_op.op_status);
  current_op_name = current_op.op_name;
  current_op_status = current_op.op_status;
  update_local_vars();
}

void AdaptationAnalyzer::callback_vibration_level_changed(const rs_autonomy::VibrationLevel vl)
{
  ROS_INFO_STREAM("[Analysis Node] the vibration level is changed from level " << vibration_level << " to level " << v_level);
  vibration_level = vl.level;
  wait_for_quake_off = !wait_for_quake_off;


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
    ROS_INFO_STREAM("[Analysis Node] receive an instruction from the Earth to run a new plan, but the plan name is empty.";
  }
}

// FIXME:
// This is to assist the simulation of excavation failure.
// It should be replaced when a more realistic simulation is available
void AdaptationAnalyzer::update_local_vars(){
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
  model_update_inst.task_name = task_name;
  model_update_inst.model_names = model_names;

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
  rtInfo_maintenance_inst.task_name = current_task_name;
  rtInfo_maintenance_inst.action = action;
  rtInfo_maintenance_inst.aux_info = aux_info;
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

  CurrentTask current_task_msg;
  current_task_msg.name = current_task_name;
  current_task_msg.status = current_task_status;
  current_task_status_pub.publish(current_task_msg);
}

void MonitorListener::initialize_rtInfo()
{
  update_models(current_task_model_names)
  action = "Initialize";
  maintain_rtInfo(action, current_task_aux_info);
}

// Transition the lander to a safey pose (Unstow pose) with arm fault cleared
void MonitorListener::transition_to_safe_pose()
{
  adpt_inst.adaptation_commands = { "TerminatePlan", "Unstow"};
  if (has_arm_fault)
  {
    // Assume that before Unstow operation, the arm fault has been cleared
    adpt_inst.adaptation_commands = { "TerminatePlan", "ClearArmFault", "Unstow"};
  }
  adpt_inst.task_name = current_task_name;
  adpt_inst_pub.publish(adpt_inst);
}

void MonitorListener::planning()
{
  adpt_inst.task_name = current_task_name;
  adpt_inst.adaptation_commands = { "Planning"};
  adpt_inst_pub.publish(adpt_inst);
}

// Update models and prepare the initial runtime info for the current task
void MonitorListener::initialize_task(bool syn_plan=true, bool terminate_current_task=false)
{
  // if the current task has been started, there should a plan is running for it,
  // so terminate the plan and transition the lander to the safe pose, Unstow pose.
  if (current_task_status == "Started")
  {
    transition_to_safe_pose();
    if(terminate_current_task)
    {
      CurrentTask current_task_msg;
      current_task_msg.name = current_task_name;
      current_task_msg.status = "Terminated";
      current_task_status_pub.publish(current_task_msg);
    }
  }

  // if there is a new task, update task control variables to transition to the new task 
  if (has_new_task)
  {
    update_task_control_vars();
  }

  if (!wait_for_quake_off)
  {
    // Update all models (maybe not necessary) and runtime info for the new task
    initialize_rtInfo();

    if (syn_plan) // Synthesize a plan to run
    {
      planning();
      if(current_task_status == "Ready")
      {
        current_task_status = "Started";
	CurrentTask current_task_msg;
	current_task_msg.name = current_task_name;
	current_task_msg.status = current_task_status;
        current_task_status_pub.publish(current_task_msg);
      }
    }
  }
}

void AdaptationAnalyzer::adaptation_analysis()
{
    bool has_arm_fault = false;
    std::string current_op_name = "":
    std::string current_op_status = "";
    int vibration_level = 0;
    std::string manual_planname = ""; 
    bool has_manual_plan = false;
    bool wait_for_quake_off = false;
    int num_digging_failures = 0;
    bool current_digging_failed = false;

  // Determine which adaptation should be triggered
  if (has_manual_plan && manual_planname != "") // A manual plan from the Earth
  {
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
  else if (!wait_for_quake_off) { // The lander is in active mode
    if (vibration_level == 1) // An earthquake is detected. 
    {
      // Transition the lander to a safe mode
      wait_for_quake_off = !wait_for_quake_off; // change from false to true
      bool syn_plan = false;
      initialize_task(syn_plan);
      // Currently, leave re-planning when the vibration level comes back to 0
    }
    else if (terminate_current_task && has_new_task) // a new task is requsted to do instead of the current one
    {
      bool syn_plan = true;
      initialize_task(syn_plan, terminate_current_task); 
    }
    else if (num_digging_failures == 3) // the belief of models drops two low, such initialize the task
    {
      bool syn_plan = true;
      initialize_task(syn_plan);
    }
    else if (num_digging_failures == 2) // the belief of models drops a little, update models and rtInfo
    {
      transition_to_safe_pose();

      // Update Excavation-Probability Model and Runtime Info
      std::vector<std::string> model_names = {"ExcaProb";}
      update_models(model_names);
      std::string action = "Update";
      std::string axu_info = "ExcaProb";
      maintain_rtInfo(action, aux_info);

      planning();
    }
    else if (current_digging_failed || has_arm_fault)
    {
      transition_to_safe_pose();

      // FIXME: here is for excavaion sceanrio only
      std::size_t pos = plan_aux_info.find(","); // pos is the location of first, ","
      std::string aux_info = plan_aux_info.substr(pos); // the ID of excavation location
      std::string action = "Remove";
      maintain_rtInfo(action, aux_info);

      planning();
    }
    // When a plan ends with "Complete_Success" status
    else if (current_task_status=="Completed_Success" && has_new_task) 
    {
      bool syn_plan = true;
      initialize_task(syn_plan);
    }
  }
  else // The lander is in active mode due to a previously detected earthquake
  {
    if (vibration_level == 0) // the earthquake has gone
    {
      // clear the waiting tag
      wait_for_quake_off = !wait_for_quake_off; // change from true to false

      // update runtime info and synthesize a plan
      initialize_rtInfo(); 
      planning();
    }
  }

}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "analysis_node");

  ros::NodeHandle nh;

  AdaptationAnalyzer adaptation_analyzer(&nh);

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    // determine if some adaptation should be performed
    adaptation_analyzer.adaptation_analysis();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

