#ifndef SysEnvMonitor_H 
#define SysEnvMonitor_H


// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <ow_faults_detection/ArmFaults.h>
#include <ow_plexil/CurrentPlan.h>
#include <ow_plexil/CurrentOperation.h>
#include <rs_autonomy/VibrationLevel.h>
#include <rs_autonomy/EarthInstruction.h>
#include <rs_autonomy/ArmFault.h>

//Others
#include "message_passing_support.h" // for msg_queue_size
int msg_queue_size = 50;

class SysEnvMonitor {
  public:
    SysEnvMonitor(ros::NodeHandle *nh)
    {
      // Initialize publishers
      current_plan_pub = nh->advertise<ow_plexil::CurrentPlan>(
		  "/Monitor/CurrentPlan", msg_queue_size);
      current_op_pub = nh->advertise<ow_plexil::CurrentOperation>(
		  "/Monitor/CurrentOperation", msg_queue_size);
      vibration_level_changed_pub = nh->advertise<rs_autonomy::VibrationLevel>(
		  "/Monitor/VibrationLevelChanged", msg_queue_size);
      arm_fault_changed_pub = nh->advertise<rs_autonomy::ArmFault>(
		  "/Monitor/ArmFaultStatus", msg_queue_size);

      // Initialize subscribers and bind callbacks
      // The messages from /faults/arm_faults_status and /Env/VibrationLevel
      // may be relatively fast, so set a large size of queue.
      arm_fault_sub = nh->subscribe<ow_faults_detection::ArmFaults>(
		      "/faults/arm_faults_status",
    		      100,
		      &SysEnvMonitor::callback_arm_fault_status,
		      this);
      vibration_level_sub = nh->subscribe<rs_autonomy::VibrationLevel>(
    		      "/Env/VibrationLevel",
		      100,
		      &SysEnvMonitor::callback_vibration_level,
		      this);
      current_op_sub = nh->subscribe<ow_plexil::CurrentOperation>(
    		      "/CurrentOperation",
    		      msg_queue_size,
    		      &SysEnvMonitor::callback_current_op,
    		      this);
      current_plan_sub = nh->subscribe<ow_plexil::CurrentPlan>(
    		      "/CurrentPlan",
    		      msg_queue_size,
    		      &SysEnvMonitor::callback_current_plan,
    		      this);
    }


    // publishers
    ros::Publisher current_plan_pub;
    ros::Publisher current_op_pub;
    ros::Publisher vibration_level_pub;
    ros::Publisher arm_fault_changed_pub;
    ros::Publisher vibration_level_changed_pub;

    // callbacks
    void callback_current_plan(const ow_plexil::CurrentPlan current_plan);
    void callback_current_op(const ow_plexil::CurrentOperation current_op);
    void callback_vibration_level(const rs_autonomy::VibrationLevel vl);
    void callback_arm_fault_status(const ow_faults_detection::ArmFaults::ConstPtr& msg);

    // monitored variables
    int vibration_level = 0; // 0 - normal, 1 - earthquake
    int arm_fault_status = 0; // 0 - no fault, 1 arm fault

    rs_autonomy::VibrationLevel vl_msg;
    rs_autonomy::ArmFault arm_fault_msg;

  private:
    ros::Subscriber arm_fault_sub;
    ros::Subscriber vibration_level_sub;
    ros::Subscriber current_op_sub;
    ros::Subscriber current_plan_sub;
};


#endif
