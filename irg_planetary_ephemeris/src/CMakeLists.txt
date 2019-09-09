include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${irg_spice_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
)

add_executable(publisher ephemeris_publisher_node.cpp ephemeris.cpp handle-options.cpp)

target_link_libraries(publisher
  ${rclcpp_LIBRARIES}
  ${irg_spice_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
)

ament_target_dependencies(publisher rclcpp tf2 tf2_ros irg_spice)
