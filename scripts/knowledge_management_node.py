#!/usr/bin/env python

import os
import rospy

# knowledge-update package
from knowledge_update.maintain_knowledge import KnowledgeManagementService

# Environment Variables
# 1 Assist loading run-time info file and dumping synthesized PLEXIL plan to a file
evaluation_root_dir = os.environ["EVALUATION_ROOT_DIR"]


def knowledge_maintainence():
    global evaluation_root_dir

    rospy.init_node('knowledge_node')
    rospy.loginfo("[Knowledge Node] initialized")

    knowledge_management_service = KnowledgeManagementService(evaluation_root_dir)

    rospy.loginfo("[Knowledge Node] Ready to conduct knowledge maintainence")
    rospy.spin()


if __name__ == "__main__":
    knowledge_maintainence()
