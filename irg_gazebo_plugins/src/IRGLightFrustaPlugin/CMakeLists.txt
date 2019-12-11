set(TARGET_NAME IRGLightFrustaPlugin)

set (HEADERS
  LightFrustaPlugin.h
)

set (SOURCES
  LightFrustaPlugin.cpp
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${TARGET_NAME}
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
