set(TARGET_NAME IRGCameraSim)

find_package(gazebo_ros REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${TARGET_NAME}SensorPlugin SHARED 
  CameraCompositor.h
  CameraCompositorListener.h
  CameraSimSensorPlugin.h
  CameraCompositor.cpp
  CameraCompositorListener.cpp
  CameraSimSensorPlugin.cpp
)

add_library(${TARGET_NAME}VisualPlugin SHARED
  CameraCompositor.h
  CameraCompositorListener.h
  CameraSimVisualPlugin.h
  CameraCompositor.cpp
  CameraCompositorListener.cpp
  CameraSimVisualPlugin.cpp
)

ament_target_dependencies(${TARGET_NAME}SensorPlugin "gazebo_ros")
target_link_libraries(${TARGET_NAME}SensorPlugin ${GAZEBO_libraries})
set_target_properties(${TARGET_NAME}SensorPlugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

ament_target_dependencies(${TARGET_NAME}VisualPlugin "gazebo_ros")
target_link_libraries(${TARGET_NAME}VisualPlugin ${GAZEBO_libraries})
set_target_properties(${TARGET_NAME}VisualPlugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS
  ${TARGET_NAME}SensorPlugin
  ${TARGET_NAME}VisualPlugin
  LIBRARY DESTINATION lib
)

