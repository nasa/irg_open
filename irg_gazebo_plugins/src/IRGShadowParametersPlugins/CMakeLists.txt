set(TARGET_NAME IRGShadowParameters)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${TARGET_NAME}SensorPlugin SHARED 
  ShadowParametersPluginBase.h
  ShadowParametersSensorPlugin.h
  ShadowParametersPluginBase.cpp
  ShadowParametersSensorPlugin.cpp
)

add_library(${TARGET_NAME}VisualPlugin SHARED 
  ShadowParametersPluginBase.h
  ShadowParametersVisualPlugin.h
  ShadowParametersPluginBase.cpp
  ShadowParametersVisualPlugin.cpp
)

target_link_libraries(${TARGET_NAME}SensorPlugin ${GAZEBO_LIBRARIES})
set_target_properties(${TARGET_NAME}SensorPlugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

target_link_libraries(${TARGET_NAME}VisualPlugin ${GAZEBO_LIBRARIES})
set_target_properties(${TARGET_NAME}VisualPlugin PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

