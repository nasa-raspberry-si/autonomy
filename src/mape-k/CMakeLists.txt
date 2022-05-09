add_definitions(-DUSING_ROS)



add_executable(monitor_node monitor_node.cpp)
add_dependencies(monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(analysis_node analysis_node.cpp)
add_dependencies(analysis_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(planner_node planner_node.cpp)
add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(execute_node execute_node.cpp)
add_dependencies(execute_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(mission_control_node mission_control_node.cpp)
add_dependencies(mission_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



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

target_link_libraries(mission_control_node
  ${catkin_LIBRARIES}
)



install(TARGETS monitor_node analysis_node planner_node execute_node mission_control_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


