include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(publisher ephemeris_publisher_node.cpp ephemeris.cpp handle-options.cpp)
