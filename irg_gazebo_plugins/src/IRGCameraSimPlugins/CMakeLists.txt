set(TARGET_NAME IRGCameraSim)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${TARGET_NAME}SensorPlugin SHARED 
  CameraCompositorListener.h
  CameraSimBase.h
  CameraSimSensorPlugin.h
  CameraCompositorListener.cpp
  CameraSimBase.cpp
  CameraSimSensorPlugin.cpp
)

add_library(${TARGET_NAME}VisualPlugin SHARED
  CameraCompositorListener.h
  CameraSimBase.h
  CameraSimVisualPlugin.h
  CameraCompositorListener.cpp
  CameraSimBase.cpp
  CameraSimVisualPlugin.cpp
)

target_link_libraries(${TARGET_NAME}SensorPlugin ${GAZEBO_libraries})
set_target_properties(${TARGET_NAME}SensorPlugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

target_link_libraries(${TARGET_NAME}VisualPlugin ${GAZEBO_libraries})
set_target_properties(${TARGET_NAME}VisualPlugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

