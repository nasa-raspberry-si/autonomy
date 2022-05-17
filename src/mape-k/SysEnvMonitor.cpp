#include "SysEnvMonitor.h"

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
  ROS_INFO("[Monitor Node] the current plan %s, status: %s", current_plan.plan_name.c_str(), current_plan.plan_status.c_str());

  current_plan_pub.publish(current_plan);
}

void SysEnvMonitor::callback_current_op(const ow_plexil::CurrentOperation current_op)
{
  ROS_INFO("[Monitor Node] the current operation %s, status: %s", current_op.op_name.c_str(), current_op.op_status.c_str());

  current_op_pub.publish(current_op);
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

