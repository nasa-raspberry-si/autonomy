// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/AInfo.h>
#include <rs_autonomy/PInfo.h>
#include <rs_autonomy/PlannerInstruction.h>

//Others
#include "message_passing_support.h" // for msg_queue_size

class AInfoListener {
  public:
    ros::Publisher pub;
    rs_autonomy::PInfo current_task;
    void callback(const rs_autonomy::AInfo current_task);
};

void AInfoListener::callback(const rs_autonomy::AInfo current_task)
{
  ROS_INFO_STREAM("[Planner Node - Listener] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  this->current_task.task_name = current_task.task_name;
  this->current_task.task_status = current_task.task_status;

  this->pub.publish(this->current_task);
}

class AdaptationInstructionListener {
  public:
    ros::Publisher pub;
    rs_autonomy::PlannerInstruction planner_instruction;
    void callback(const rs_autonomy::AInfo current_task);
};

void AdaptationInstructionListener::callback(const rs_autonomy::AInfo current_task)
{
  //ROS_INFO_STREAM("[Planner Node - Listener] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  // For testing execute node
  if (current_task.task_name == "GuardedMove"
       && current_task.task_status.find("starts") != std::string::npos)
  {
    this->planner_instruction.command = "SUSPEND";
    this->planner_instruction.plan_name = current_task.task_name;
    this->planner_instruction.high_level_plan = ""; // no need to provide, so use the emtpy string
  }
  else if (current_task.task_name == "GuardedMove"
       && current_task.task_status.find("finishes") != std::string::npos)
  {
    this->planner_instruction.command = "RESUME";
    this->planner_instruction.plan_name = current_task.task_name;
    this->planner_instruction.high_level_plan = ""; // no need to provide, so use the emtpy string
  }
  else if (current_task.task_name == "GroundDetection"
    && current_task.task_status.find("finishes") != std::string::npos)
  {
    this->planner_instruction.command = "TERMINATE";
    this->planner_instruction.plan_name = current_task.task_name;
    this->planner_instruction.high_level_plan = ""; // no need to provide, so use the emtpy string
  }
  else
  {
    this->planner_instruction.command = "NoAction";
    this->planner_instruction.plan_name = current_task.task_name;
    this->planner_instruction.high_level_plan = ""; // no need to provide, so use the emtpy string  
  }

  ROS_INFO_STREAM(
    "[Planner Node - Listener] planner instruction for the current task: "
    << "\n\tcommand: " << this->planner_instruction.command
    << "\n\tplan_name:" << this->planner_instruction.plan_name
    << "\n\thigh_level_plan: " << this->planner_instruction.high_level_plan
    );

  this->pub.publish(this->planner_instruction);
}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "planner_node");

  ros::NodeHandle nh;

  AInfoListener listener;
  listener.pub =  nh.advertise<rs_autonomy::PInfo>("/PInfo", msg_queue_size);
  ros::Subscriber p_sub = nh.subscribe<rs_autonomy::AInfo>("/AInfo",
                                                         msg_queue_size,
                                                         &AInfoListener::callback,
                                                         &listener);

  AdaptationInstructionListener adptListener;
  adptListener.pub =  nh.advertise<rs_autonomy::PlannerInstruction>("/PlannerInstruction", msg_queue_size);
  ros::Subscriber pl_sub = nh.subscribe<rs_autonomy::AInfo>("/AInfo",
                                                         msg_queue_size,
                                                         &AdaptationInstructionListener::callback,
                                                         &adptListener);



  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

