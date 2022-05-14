#!/usr/bin/env python

import os
import rospy
# PLEXIL plan translation
from plan_translation.plan_translation_service import PlanTranslationService

# Environment Variables
# 1 Assist loading run-time info file and dumping synthesized PLEXIL plan to a file
evaluation_root_dir = os.environ["EVALUATION_ROOT_DIR"]
# 2 Assist compiling C++-like '#include' headers in synthesized PLEXIL plan (*.plp)
#   ow_simulator : '#include "plan-interface.h"'
#   owlat        : '#include "owlat-interface.h"'
ow_plexil_lib_source_dir = os.environ["OW_PLEXIL_LIB_SOURCE_DIR"]
# 3 Assist moving the compiled PLEXIL plan (*.plx) to the destination
plexil_lib_compiled_plan_dir = os.environ["PLEXIL_LIB_COMPILED_PLAN_DIR"]


def plan_translation():
    rospy.init_node('plan_translation_node')
    rospy.loginfo("[Plan Translation Node] initialized")

    plan_translation_service = PlanTranslationService(
        evaluation_root_dir,
        ow_plexil_lib_source_dir,
        plexil_lib_compiled_plan_dir)

    rospy.loginfo("[Plan Translation Node] Ready to provide PLEXIL plan translation service")
    rospy.spin()


if __name__ == "__main__":
    plan_translation()
