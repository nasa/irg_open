set(TARGET_NAME IRGVisibilityPlugin)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED 
  VisibilityPlugin.h
  VisibilityPlugin.cpp
)

target_link_libraries(${TARGET_NAME}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

