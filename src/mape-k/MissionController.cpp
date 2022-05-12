#include "MissionController.h"

bool MissionController::prepare_evaluation_task_dir(std::string task_name)
{
  bool result = false;

  // Create a directory for organizing the evaluation result for the task, task_name
  if (task_name.find("Excavation")!=std::string::npos)
  {
    std::string eval_task_dir = eval_root_dir + "/Tasks/" + task_name;
    std::string command;
    command = "mkdir -p "+ eval_task_dir;  
    system(command.c_str());
    result = true;
  }
  else
  {
    ROS_INFO("[Mission Control Node] Unsupported task: %s. Can't prepare planing tuility files for it", task_name.c_str());
    result = false;
  }

  return result;

}
void MissionController::report_mission_result()
{
  ROS_INFO("[Mission Control Node] Mission Result");
  for(auto task : past_task_list)
  {
    ROS_INFO("Task: %s, Status: %s, Aux_Info: %s.",
		    task.name.c_str(), task.status.c_str(), task.aux_info.c_str());
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
 
  ROS_INFO("[Mission Control Node] Loading mission specification: %s", mission_spec_fp.c_str());

  while (std::getline(infile, line))
  {
    std::string delimiter = "|";
    std::size_t pos = line.find(delimiter); // pos is 0-based index
    std::string task_name = line.substr(5, pos-5); // "Task:" has a length of 5
    std::string task_aux_info = line.substr(pos+10); // "Aux_Info:" has a length of 9
    task_names_list.push_back(task_name);
    mission_spec.insert( std::pair<std::string,std::string>(task_name, task_aux_info) ); 
    ROS_INFO("\ttask: %s, aux_info: %s", task_name.c_str(), task_aux_info.c_str());
  }
}

void MissionController::prepare_new_task_to_run()
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
    ROS_ERROR_STREAM("[Mission Control Node] failed to prepare to do the task " << task_name);
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
     prepare_new_task_to_run();
    }
  }

}

