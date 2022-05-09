// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/AdaptationInstruction.h>
#include <rs_autonomy/PlannerInstruction.h>
#include <rs_autonomy/TaskPlanning.h>

//Others
#include "message_passing_support.h" // for msg_queue_size
#include <string>


class AdaptationInstructionListener {
  public:
    // publisher
    ros::Publisher planner_inst_pub;
    rs_autonomy::PlannerInstruction planner_instruction;

    // callback for subscriber
    void callback_adap_inst_sub(const rs_autonomy::AdaptationInstruction adpt_inst);

    // ROS clients for calling '/task_planning' service
    rs_autonomy::TaskPlanning task_planning;
    ros::ServiceClient task_planning_service_client;

    // For one task, we use one plan template and fill it with
    // runtime info to turn it into a complete PLEXIL plan (*.plp file)
    // Since one task may needs run several plans, we format the names
    // of these plans as "<task_name><plan_id>", e.g., "Exca1", "Exca2".
    std::string task_name = "";
    std::string current_plan_name = "";
    int current_plan_id = 0; // 0 indiates no plan has been tried for the task
};

void AdaptationInstructionListener::callback_adap_inst_sub(const rs_autonomy::AdaptationInstruction adpt_inst)
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
      planner_instruction.plan_name = Unstow;
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
      }
      else
      {
        // FIXME: notify the failure to the analysis componenet
	ROS_ERROR("[Planner Node] unable to talk to /task_planning service. Planning fails.")
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
      )
    }
  }
  
}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "planner_node");

  ros::NodeHandle nh;

  AdaptationInstructionListener adptListener;

  adptListener.planner_inst_pub =  nh.advertise<rs_autonomy::PlannerInstruction>(
		  "/PlannerInstruction",
		  msg_queue_size);

  ros::Subscriber adap_inst_sub = nh.subscribe<rs_autonomy::AdaptationInstruction>(
		  "/AdaptationInstruction",
		  msg_queue_size,
		  &AdaptationInstructionListener::callback_adap_inst_sub,
		  &adptListener);

  adptListener.task_planning_service_client = nh.serviceClient<rs_autonomy::TaskPlanning>("/task_planning"); 

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

