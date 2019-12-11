set(TARGET_NAME IRGCameraSim)

include_directories(
  ${catkin_INCLUDE_DIRS}
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

target_link_libraries(${TARGET_NAME}SensorPlugin ${roscpp_LIBRARIES} ${GAZEBO_LIBRARIES})
set_target_properties(${TARGET_NAME}SensorPlugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

target_link_libraries(${TARGET_NAME}VisualPlugin ${roscpp_LIBRARIES} ${GAZEBO_LIBRARIES})
set_target_properties(${TARGET_NAME}VisualPlugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

