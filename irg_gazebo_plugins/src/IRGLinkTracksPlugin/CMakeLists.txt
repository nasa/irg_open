set (HEADERS
  LinkTracksPlugin.h
)

set (SOURCES
  LinkTracksPlugin.cpp
)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

add_library(link_tracks_plugin SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(link_tracks_plugin ${GAZEBO_libraries} ${rclcpp_LIBRARIES})
set_target_properties(link_tracks_plugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")
