set(TARGET_NAME IRGCelestialBodyPlugin)

set (HEADERS
  CelestialBodyPlugin.h
)

set (SOURCES
  CelestialBodyPlugin.cpp
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

ament_target_dependencies(${TARGET_NAME} tf2_ros )
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
