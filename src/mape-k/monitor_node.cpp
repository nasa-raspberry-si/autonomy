// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
// FIXME: Topic: /faults/arm_faults_status
// - r9:ow_faults_detection/ArmFaults (#include <ow_faults_detection/ArmFaults.h>)
// - r8:ow_faults/ArmFaults (#include <ow_faults/ArmFaults.h>)
#include <ow_faults/ArmFaults.h>
#include <ow_plexil/CurrentPlan.h>
#include <ow_plexil/CurrentOperation.h>
#include <rs_autonomy/VibrationLevel.h>
#include <rs_autonomy/EarthInstruction.h>
#include <rs_autonomy/ArmFault.h>


//Others
#include "message_passing_support.h" // for msg_queue_size






class SysEnvMonitor {
  public:
    // publishers
    ros::Publisher current_plan_pub;
    ros::Publisher current_op_pub;
    ros::Publisher vibration_level_pub;
    ros::Publisher arm_fault_changed_pub;

    // callbacks
    void callback_current_plan(const ow_plexil::CurrentPlan current_plan);
    void callback_current_op(const ow_plexil::CurrentOperation current_op);
    void callback_vibration_level(const rs_autonomy::VibrationLevel vl);
    // * arm fault message type:
    //   - r9:ow_faults_detection/ArmFaults (#include <ow_faults_detection/ArmFaults.h>)
    //   - r8:ow_faults/ArmFaults (#include <ow_faults/ArmFaults.h>)
    void callback_arm_fault_status(const ow_faults::ArmFaults::ConstPtr& msg);

    // monitored variables
    int vibration_level = 0; // 0 - normal, 1 - earthquake
    int arm_fault_status = 0; // 0 - no fault, 1 arm fault

    rs_autonomy::VibrationLevel vl_msg;
    rs_autonomy::ArmFault arm_fault_msg;
};

void SysEnvMonitor::callback_arm_fault_status(const ow_faults::ArmFaults::ConstPtr& msg)
{
  if (arm_fault_status != msg->value) 
  {
    arm_fault_status = msg->value;
    arm_fault_msg.has_a_fault = (arm_fault_status==1) ? true : false;
    arm_fault_changed_pub.publish(arm_fault_msg);
  }
}

void SysEnvMonitor::callback_current_plan(const ow_plexil::CurrentPlan current_plan)
{
  current_plan_pub.publish(current_plan);
}

void SysEnvMonitor::callback_current_op(const ow_plexil::CurrentOperation current_op)
{
  current_plan_op.publish(current_op);
}

void SysEnvMonitor::callback_vibration_level(const rs_autonomy::VibrationLevel vl)
{
  if (vl.level != vibration_level)
  {
    vl_msg.level = vl.level;
    vibration_level = vl.level;
    vibration_level_pub.publish(vl_msg);
  }
}

void SysEnvMonitor::callback_arm_fault(const ???)
{
  if (vl.level != vibration_level)
  {
    vl_msg.level = vl.level;
    vibration_level = vl.level;
    vibration_level_pub.publish(vl_msg);
  }
}

void CurrentPlanListener::callback(const ow_plexil::CurrentPlan current_plan)
{
  ROS_INFO_STREAM("[Monitor Node - Listener] current plan: " << current_plan.plan_name << ", status: " << current_plan.plan_status);

  this->current_plan.task_name = current_plan.plan_name;
  this->current_plan.task_status = current_plan.plan_status;

  this->pub.publish(this->current_plan);
}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "monitor_node");

  ros::NodeHandle nh;


  SysEnvMonitor monitor;
  // Initialize publishers
  monitor.current_plan_pub = nh.advertise<ow_plexil::CurrentPlan>(
		  "/Monitor/CurrentPlan", msg_queue_size);
  monitor.current_op_pub = nh.advertise<ow_plexil::CurrentOperation>(
		  "/Monitor/CurrentOperation", msg_queue_size);
  monitor.vibration_level_changed_pub = nh.advertise<rs_autonomy::VibrationLevel>(
		  "/Monitor/VibrationLevelChanged", msg_queue_size);
  monitor.arm_fault_changed_pub = nh.advertise<ow_>(
		  "/Monitor/ArmFaultStatus", msg_queue_size);

  // Initialize subscribers and bind callbacks
  // The messages from /faults/arm_faults_status and /Env/VibrationLevel
  // may be relatively fast, so set a large size of queue.
  ros::Subscriber arm_fault_sub = nh.subscribe<ow_faults::ArmFaults>(
		  "/faults/arm_faults_status",
		  100,
		  SysEnvMonitor::callback_arm_fault_status,
		  &monitor);
  ros::Subscriber vibration_level_sub = nh.subscribe<rs_autonomy::VibrationLevel>(
		  "/Env/VibrationLevel",
		  100,
		  SysEnvMonitor::callback_vibration_level,
		  &monitor);
  ros::Subscriber current_op_sub = nh.subscribe<ow_plexil::CurrentOperation>(
		  "/CurrentOperation",
		  msg_queue_size,
		  SysEnvMonitor::callback_current_op,
		  &monitor);
  ros::Subscriber current_plan_sub = nh.subscribe<ow_plexil::CurrentPlan>(
		  "/current_plan_status",
		  msg_queue_size,
		  SysEnvMonitor::callback_current_plan,
		  &monitor);



  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    /*
    if(t) {
      m_pub.publish(m_msg);
      t = false;
    }
    */
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

