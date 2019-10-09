set(TARGET_NAME publisher)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${irg_spice_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
)

add_executable(${TARGET_NAME} ephemeris_publisher_node.cpp ephemeris.cpp)

target_link_libraries(${TARGET_NAME}
  ${rclcpp_LIBRARIES}
  ${irg_spice_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
)

ament_target_dependencies(${TARGET_NAME} rclcpp tf2 tf2_ros irg_spice)

install(TARGETS ${TARGET_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
