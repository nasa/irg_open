set(TARGET_NAME IRGGlobalShaderParamPlugin)

set (HEADERS
  GlobalShaderParamPlugin.h
)

set (SOURCES
  GlobalShaderParamPlugin.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${TARGET_NAME}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

# because we're using custom ros message
add_dependencies(${TARGET_NAME}  ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
