set(TARGET_NAME IRGCelestialBodyPlugin)

set (HEADERS
  CelestialBodyPlugin.h
)

set (SOURCES
  CelestialBodyPlugin.cpp
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${TARGET_NAME}
  ${GAZEBO_LIBRARIES}
)
if(ENABLE_PROFILER)
  target_link_libraries(${TARGET_NAME}
    ${ignition-common3_LIBRARIES}
  )
endif()

ament_target_dependencies(${TARGET_NAME} rclcpp tf2_ros )
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
