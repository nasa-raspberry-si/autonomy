set (HEADERS
  joint_support.h
  subscriber.h
  action_support.h
  adapter_support.h
  PlexilInterface.h
  OwExecutive.h
  OwInterface.h
  CommonAdapter.h
  OwAdapter.h
  PlexilPlanSelection.h
  TerminalPlanSelection.h
)

set (SOURCES
  subscriber.cpp
  action_support.cpp
  adapter_support.cpp
  PlexilInterface.cpp
  OwExecutive.cpp
  OwInterface.cpp
  CommonAdapter.cpp
  OwAdapter.cpp
  PlexilPlanSelection.cpp
  TerminalPlanSelection.cpp
)


add_definitions(-DUSING_ROS)



add_executable(monitor_node monitor_node.cpp)
add_dependencies(monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(analysis_node analysis_node.cpp)
add_dependencies(analysis_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(planner_node planner_node.cpp)
add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(execute_node execute_node.cpp)
add_dependencies(execute_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(knowledge_node knowledge_node.cpp)
add_dependencies(knowledge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(autonomy_node autonomy_node.cpp)
add_dependencies(autonomy_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(monitor_node
  ${catkin_LIBRARIES}
)

target_link_libraries(analysis_node
  ${catkin_LIBRARIES}
)

target_link_libraries(planner_node
  ${catkin_LIBRARIES}
)

target_link_libraries(execute_node
  ${catkin_LIBRARIES}
)

target_link_libraries(knowledge_node
  ${catkin_LIBRARIES}
  ${LIB_NAME})

target_link_libraries(autonomy_node
  ${catkin_LIBRARIES}
  ${LIB_NAME})




install(TARGETS monitor_node analysis_node planner_node execute_node knowledge_node autonomy_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


