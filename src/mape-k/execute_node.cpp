// The role of Execute node:
// * Wait for instruction from the planner node
// * Pull info from the knowledge node when needed
// * Do plan translation if the instruction from the planner node asks to run a new plan
// * Send commands to the system under test to drive the lander

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <string>

// OW
#include <rs_autonomy/PInfo.h>
#include <rs_autonomy/EInfo.h>
#include <rs_autonomy/PlannerInstruction.h>
#include <ow_plexil/PlanSelection.h>

//Others
#include "message_passing_support.h" // for msg_queue_size

// TBD
static void planTranslator(std::string plan_name, std::string high_level_plan)
{
  ROS_INFO_STREAM(
    "[Execute Node] Translating the high-level plan to a PLEXIL plan:"
    << "\n\t plan_name: " << plan_name
    << "\n\t high-level plan: " << high_level_plan);
}

class PInfoListener {
  public:
    ros::Publisher pub;
    rs_autonomy::EInfo current_task;
    void callback(const rs_autonomy::PInfo current_task);
};

void PInfoListener::callback(const rs_autonomy::PInfo current_task)
{
  ROS_INFO_STREAM("[Execute Node - Listener] current task: " << current_task.task_name << ", status: " << current_task.task_status);

  this->current_task.task_name = current_task.task_name;
  this->current_task.task_status = current_task.task_status;

  this->pub.publish(this->current_task);
}

class PlannerInstructionListener {
  public:
    ros::Publisher pub;
    ow_plexil::PlanSelection execute_instruction;
    ros::ServiceClient planSelectionServiceClient;

    void callback(const rs_autonomy::PlannerInstruction planner_instruction);
};

void PlannerInstructionListener::callback(const rs_autonomy::PlannerInstruction planner_instruction)
{
  if (planner_instruction.command == "NoAction")
  {
    return;
  }

  // If it is a new plan, translate the high-level plan to a plexil plan.
  if (planner_instruction.command == "ADD")
  {
    // TBD: how to deal translation failure.
    planTranslator(planner_instruction.plan_name, planner_instruction.high_level_plan);        
  }

  // Synthesize the execute instruction
  std::vector<std::string> plan_array;
  //// plan_name in planner_instruction is the name of the root node in the plexil plan,
  //// e.g., Exca. While the plan name sent to be executed on the lander should be full
  //// file name of the plexil plan, e.g., Exca.plx.
  plan_array.push_back(planner_instruction.plan_name + ".plx");

  this->execute_instruction.request.command = planner_instruction.command;
  this->execute_instruction.request.plans = plan_array;

  // FIXME: The current autonomy design leads to that there is only one plexil 
  // plan in the plan array, plans.
  ROS_INFO_STREAM(
    "[Execute Node - Listener] sending execute instruction for the current task: "
    << "\n\tcommand: " << this->execute_instruction.request.command
    << "\n\tplans:" << this->execute_instruction.request.plans[0]
    );

  // Send the execute instruction to the ROS service /plexil_plan_selection 
  if(planSelectionServiceClient.call(execute_instruction))
  {
    ROS_INFO("[Execute Node - Listener] The execute instruction is successfully recieved.");
  }
  else
  {
    ROS_ERROR("[Execute Node - Listener] The execute instruction is not received because it is unable to contact plan_selection_server.");
  }
}


int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "planner_node");

  ros::NodeHandle nh;

  PInfoListener listener;
  listener.pub =  nh.advertise<rs_autonomy::EInfo>("/EInfo", msg_queue_size);
  ros::Subscriber e_sub = nh.subscribe<rs_autonomy::PInfo>("/PInfo",
                                                         msg_queue_size,
                                                         &PInfoListener::callback,
                                                         &listener);


  PlannerInstructionListener piListener;
  piListener.planSelectionServiceClient = nh.serviceClient<ow_plexil::PlanSelection>("/plexil_plan_selection");
  ros::Subscriber el_sub = nh.subscribe<rs_autonomy::PlannerInstruction>("/PlannerInstruction",
                                                         msg_queue_size,
                                                         &PlannerInstructionListener::callback,
                                                         &piListener);


  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

