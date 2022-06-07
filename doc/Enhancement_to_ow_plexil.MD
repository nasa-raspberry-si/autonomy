We enhanced the [ow_plexil](https://github.com/nasa/ow_autonomy) package as follows in order to have it properly communicate with the autonomy:

   - Enable the autonomy to terminate an executing plan by
      * Adding a TerminatePlan Lookup
      * Processing the TERMINATE signal in the callback function of the ROS service, /plexil_plan_selection
   - Make the ow_plexil to publish the states of the current plan and the current operation by
      * Adding support for PLEXIL Update node in a format of having three variables, checkpoint_type, checkpoint_name and checkpoint_status. Below is an example of sending out the state of GuardedMove operation to the autonomy. </br>
      `Update checkpoint_type="Operation", checkpoint_name="GuardedMove", checkpoint_status=OpOutcome;`
      * Adding two ROS messages, CurrentPlan.msg and CurrentOperation.msg
