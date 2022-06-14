We enhanced the [ow_plexil](https://github.com/nasa/ow_autonomy) package as follows in order to have it properly communicate with the autonomy:

   - Enable the autonomy to terminate an executing plan by
      * Adding a TerminatePlan Lookup
      * Processing the TERMINATE signal in the callback function of the ROS service, /plexil_plan_selection
   - Make the ow_plexil to publish the states of the current plan and the current operation by
      * Adding support for PLEXIL Update node in a format of having three variables, checkpoint_type, checkpoint_name and checkpoint_status. Below is an example of sending out the state of GuardedMove operation to the autonomy. </br>
      `Update checkpoint_type="Operation", checkpoint_name="GuardedMove", checkpoint_status=OpOutcome;`
      * Adding two ROS messages, CurrentPlan.msg and CurrentOperation.msg
      * Adding the Lookup, OpState(String operation_name), to check the state of the operation (i.e., the state of the sent ROS action) when it finishes. If the state is "SUCCEEDED", then set the operation to be successful; otherwise, set it as a failure. Currently, the Lookup, OpState(String) supports several operations related to excavation, including, GuardedMove, Grind, DigCicular and Discard.
   - Implement a transition diagram of plan status as shown in Figure 1.
      * The change of the plan status to each of "Started", "Completed_Failure" and "Completed_Success" is triggered from an executing PLEXIL plan
      * The change of the plan status to every other status is triggered in PlexilPlanSelection componenet in the ow_exec_node.

<div align="center">
   
| ![](https://user-images.githubusercontent.com/5262552/173477732-cec3b0e0-6d6e-4be7-9a50-5875f2179e08.png)| 
|:--:| 
| *Figure 1: The transition diagram of plan status.* |
</div>
