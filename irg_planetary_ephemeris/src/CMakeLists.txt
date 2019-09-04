include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
)

add_executable(publisher ephemeris_publisher_node.cpp ephemeris.cpp handle-options.cpp)

target_link_libraries(publisher
  ${catkin_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
)
