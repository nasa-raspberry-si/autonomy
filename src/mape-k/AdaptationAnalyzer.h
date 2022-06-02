#ifndef AdaptationAnalyzer_H
#define AdaptationAnalyzer_H



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

// published by the planner componenet
#include <rs_autonomy/HighLevelPlan.h>

//Others
#include "message_passing_support.h" // for msg_queue_size


// Currently, the adaptation_analysis implements seven cases in excavation scenario
// More details can be found in rs_autonomy/doc/analysis_node_description.txt
//
// FIXME: for other excavation cases and cases in other task (sample identification)
//        this function needs to be updated accordingly
class AdaptationAnalyzer {
  public:
    AdaptationAnalyzer(ros::NodeHandle *nh, bool is_debug=false)
    {
      terminating_current_plan = false;
      clearing_arm_fault = false;

      this->is_debug = is_debug;

      // Publisher for sending adaptation commands to the planner component
      adpt_inst_pub = nh->advertise<rs_autonomy::AdaptationInstruction>(
 		      "/Analysis/AdaptationInstruction", msg_queue_size);
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
      high_level_plan_sub = nh->subscribe<rs_autonomy::HighLevelPlan>(
    		      "/Planner/HighLevelPlan",
    		      msg_queue_size,
    		      &AdaptationAnalyzer::callback_high_level_plan,
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
    std::vector<std::string> next_task_model_names; // models used for the task
    //// ArmFault
    bool has_arm_fault = false;
    //// CurrentOperation
    std::string current_op_name = "";
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
    int plan_retries = 0;
    
    // Updated in the corresponding callback functions
    bool has_manual_plan = false;
    bool wait_for_quake_off = false;
    bool has_new_task = false;
    std::string current_task_name = "";
    std::string current_task_aux_info = "";
    std::vector<std::string> current_task_model_names;
    //// TBD (3 status): "started", "completed", "terminated" 
    std::string current_task_status = "";

    // The following member variables depends on the variables above
    // and should be updated accordingly by calling update_local_vars()
    int num_digging_failures = 0;
    bool digging_failure_adaptation_on = true;
    bool current_digging_failed = false;

    // constantly update num_digging_failures and current_digging_failed
    // when a related topic value is changed
    void update_local_vars();

    void update_task_control_vars();
    void update_models(std::vector<std::string> model_names);
    void maintain_rtInfo(std::string action, std::string aux_info);
    void initialize_task(bool terminate_current_task=false);
    void initialize_rtInfo();
    void transition_to_safe_pose();
    void planning();
    void terminate_current_plan();
    void clear_arm_fault();

    // callbacks for subscribered ROS topics
    void callback_current_operation(const ow_plexil::CurrentOperation current_op);
    void callback_current_plan(const ow_plexil::CurrentPlan current_plan);
    void callback_next_task(const rs_autonomy::NextTask next_task);
    void callback_arm_fault_status(const rs_autonomy::ArmFault arm_fault);
    void callback_vibration_level_changed(const rs_autonomy::VibrationLevel vl);
    void callback_earth_inst(const rs_autonomy::EarthInstruction earth_inst);
    void callback_high_level_plan(const rs_autonomy::HighLevelPlan msg);

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
    ros::Subscriber high_level_plan_sub; 
    
    // The following two variables are used to safe-guard the adaptation_analysis()
    bool terminating_current_plan;
    bool clearing_arm_fault;

    bool is_debug;
};


#endif
