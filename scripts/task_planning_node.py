#!/usr/bin/env python

import os
import rospy
from prism_planning.planning_service import TaskPlanningService

# Environment Variables
# 1 Assist loading run-time info file and dumping synthesized PLEXIL plan to a file
evaluation_root_dir = os.environ["EVALUATION_ROOT_DIR"]

def task_planning():
    rospy.init_node('task_planning_node')
    rospy.loginfo("[Task Planning Node] initialized")

    task_planning_service = TaskPlanningService(evaluation_root_dir)

    rospy.loginfo("[Task Planning Node] Ready to provide task planning service")
    rospy.spin()

if __name__ == "__main__":
    task_planning()
