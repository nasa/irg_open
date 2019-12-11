set(TARGET_NAME IRGCameraNoisePlugin)

set (HEADERS
  CameraNoisePlugin.h
  NoiseCompositorListener.h
)

set (SOURCES
  CameraNoisePlugin.cpp
  NoiseCompositorListener.cpp
)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${TARGET_NAME} ${GAZEBO_LIBRARIES})
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
