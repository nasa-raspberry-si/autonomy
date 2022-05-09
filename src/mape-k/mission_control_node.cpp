/* Mission Control Node
 * 1. Accept a mission specification file
 * 2. Determine which task to do next
 * 3. Send the determiend new task to the analysis componenet
 * 4. Monitor the progress of the mission
*/

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <rs_autonomy/NextTask.h>
#include <rs_autonomy/CurrentTask.h>

//Others
#include "message_passing_support.h" // for msg_queue_size

#include <sstream>
#include <string>
#include <fstream>
#include <map>
#include <vector>
#include <cstdlib>

class Task {
  public:
    Task(std::string name, std::string status, std::string aux_info) {
      this->name = name;
      this->status = status;
      this->aux_info = aux_info;
    }
    std::string name = "";
    // Types of Task Status
    // "Created"
    // "Sent" // sent to the analysis componenet
    // "Ready" // ready to do the task in the analysis component
    // "Started"
    // "Terminated"
    // "Completed_Success"
    // "Completed_Failure"
    std::string status = "";
    std::string aux_info = "";
};

class MissionController {
  public:
    ros::Publisher& new_task_pub;
    void callback_current_task_status(const rs_autonomy::CurrentTask msg);

    // FIXME: the following members are used to demonstrate running several
    //        excavation tasks. They should be modified accordingly when
    //        new tasks are designed for the mission.
    std::vector<Task> past_task_list; 
    Task current_task = Task();
    std::vector<std::string> model_names = {"SciVal", "ExcaProb"}
    int current_task_idx = -1;
    std::vector<std::string> task_names_list;
    std::map<std::string, std::string> mission_spec; // a dictionary of tasks
    void load_mission_spec(std::string mission_spec_fp); // load from a file    
    void prepare_task_to_run();
    void report_mission_result();
    bool prepare_evaluation_task_dir();
};

bool MissionController::prepare_evaluation_task_dir(std::string task_name)
{
  char *eval_root_dir = getenv("EVALUATION_ROOT_DIR");
  if(eval_root_dir == NULL) {
    ROS_ERROR("Environment variable $PLEXIL_PLAN_DIR is not set.");
    return false;
  }

  // Create a directory for organizing the evaluation result for the task, task_name
  if (task_name.find("Evaluation")!=std::string::nops)
  {
    std::string eval_task_dir;
    eval_task_dir = = str(eval_root_dir) + "/Tasks/" + task_name;
    std::string command;
    command = "mkdir -p "+ eval_task_dir;  
    system(command.c_str());
    result = true;
  }
  else
  {
    ROS_INFO("[Mission Control Node] Unsupported task: " << task_name << ". Can't prepare planing tuility files for it");
    result = false;
  }

  return result;

}
void MissionController::report_mission_result()
{
  ROS_INFO("[Mission Control Node] Mission Result");
  for(auto task : past_task_list)
  {
    ROS_INFO_STREAM("Task: "<<task.name<<", Status: "<<task.status<<"Aux_Info: "<<task.aux_info);
  }
}

// Assume the mission specification file has the following format
// === Mission Specification File ===
// Task:<task_name>,Aux_Info:<aux_info>
//                 ...
// For example,
// Task:Task1_Excavation|Aux_Info:8,4
// Task:Task2_Excavation|Aux_Info:10,6
//
// Note: AuxInfo:8,4: 8 excavation locations and 4 dump locations
void MissionController::load_mission_spec(std::string mission_spec_fp)
{
  std::ifstream infile(mission_spec_fp);

  std::string line;
 
  ROS_INFO_STEAM("[Mission Control Node] Loading mission specification: " << mission_spec_fp);

  while (std::getline(infile, line))
  {
    std::string delimiter = "|";
    std::size_t pos = line.find(delimiter); // pos is 0-based index
    std::string task_name = line.substr(5, pos-5); // "Task:" has a length of 5
    std::string task_aux_info = line.substr(pos+10); // "Aux_Info:" has a length of 9
    task_name_list.push_back(task_name);
    mission_spec.insert( std::pair<std::string,std::string>(task_name, task_aux_info) ); 
    ROS_INFO_STEAM("\ttask: " << task_name << "aux_info: " << task_aux_info);
  }
}

void MissionController::prepare_task_to_run()
{
  current_task_idx += 1;
  std::string task_name = task_names_list[current_task_idx];
  std::string aux_info = mission_spec[task_name];
  std::string status = "Created";


  bool is_successful = prepare_evaluation_task_dir(task_name);
 
  if (is_successful)
  {
    current_task = Task(task_name, status, aux_info);
    // Create a NextTask message and send it to the analysis componenet
    rs_autonomy::NextTask new_task;
    new_task.name = task_name;
    new_task.aux_info = aux_info;
    new_task.model_names = model_names;
    new_task.terminate_current_task = false;
    new_task_pub.publish(new_task);
    current_task.status = "Sent";
  }
  else
  {
    ROS_ERROR("[Mission Control Node] failed to prepare to do the task " + task_name.);
  }
}

void MissionController::callback_current_task_status(const rs_autonomy::CurrentTask msg)
{
  ROS_INFO_STREAM("[Mission Control Node] current task: " << msg.name << ", status: " << msg.status);

  if (msg.status == "Started")
  {
    current_task.status = msg.status;
  }
  // The current task has finished. The mission controller needs to determine what to do next
  else if (msg.status == "Completed_Success"
		  || msg.status == "Completed_Failure"
		  || msg.status == "Terminated")
  {
   if ((current_task_idx+1) == task_names_list.size())
   {
     ROS_INFO("[Mission Control Node] All tasks have been run.");
     report_mission_result();
   }
   else
   {
    past_task_list.push_back(current_task);
    prepare_task_to_run();
   }
  }


  this->pub.publish(this->current_task);
}


class Task {
  public:
    std::string name = "";
    // Types of Task Status
    // "Sent" // sent to the analysis componenet
    // "Starts"
    // "Terminated"
    // "Completed_Success"
    // "Completed_Failure"
    std::string status = "";
    std::string aux_info = "";
};

class MissionController {
  public:
    ros::Publisher& next_task_pub;
    rs_autonomy::NextTask next_task;
    std::vector<Task> past_task_list; 
    Task current_task = Task();
    std::vector<std::string> model_names = {"SciVal", "ExcaProb"}
    void callback_current_task_status(const rs_autonomy::CurrentTask msg);

    // FIXME: the following members are used to demonstrate running several
    //        excavation tasks. They should be modified accordingly when
    //        new tasks are designed for the mission.
    int current_task_idx = -1;
    std::vector<std::string> task_names_list;
    std::map<std::string, std::string> mission_spec; // a dictionary of tasks
    void load_mission_spec(std::string mission_spec_fp); // load from a file    
    void determine_next_task();
};



int main(int argc, char* argv[])
{
  std::string mission_spec_fp = "None"; 

  if (argc==2 && std::string(argv[1]).compare("None") != 0)
  {
    mission_spec_fp = std::string(argv[1]);
  }
  else
  {
    ROS_ERROR("[Mission Control Node] The mission specification filepath is not given.");
    return 1;
  }

  // Initialization
  ros::init(argc, argv, "autonomy_node");
  ros::NodeHandle nh;
  MissionController mission_controller;
  mission_controller.new_task_pub = nh.advertise<rs_autonomy::NextTask>(
		  "/Mission/NextTask", msg_queue_size)
  ros::Subscriber current_task_sub = nh.subscribe<rs_autonomy::CurrentTask>(
		  "/Analysis/CurrentTask",
		  msg_queue_size,
		  &MissionController::callback_current_task_status,
		  &mission_controller);

  // Load the mission specification and start to run the first task
  mission_controller.load_mission_spec(mission_spec_fp);
  mission_controller.prepare_task_to_run();

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}

