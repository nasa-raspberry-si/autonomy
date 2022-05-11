#ifndef MissionController_H
#define MissionController_H


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
    MissionController(ros::NodeHandle *nh)
    {
      new_task_pub = nh->advertise<rs_autonomy::NextTask>(
    		      "/Mission/NextTask", msg_queue_size);

      current_task_sub = nh->subscribe<rs_autonomy::CurrentTask>(
    		      "/Analysis/CurrentTask",
    		      msg_queue_size,
    		      &MissionController::callback_current_task_status,
    		      this);
    }

    ros::Publisher new_task_pub;
    void callback_current_task_status(const rs_autonomy::CurrentTask msg);

    // FIXME: the following members are used to demonstrate running several
    //        excavation tasks. They should be modified accordingly when
    //        new tasks are designed for the mission.
    std::vector<Task> past_task_list; 
    Task current_task = Task("", "", "");
    std::vector<std::string> model_names = {"SciVal", "ExcaProb"};
    int current_task_idx = -1;
    std::vector<std::string> task_names_list;
    std::map<std::string, std::string> mission_spec; // a dictionary of tasks
    void load_mission_spec(std::string mission_spec_fp); // load from a file    
    void prepare_new_task_to_run();
    void report_mission_result();
    bool prepare_evaluation_task_dir(std::string task_name);
  private:
    ros::Subscriber current_task_sub;
};


#endif
