set(TARGET_NAME IRGVisibilityPlugin)

add_library(${TARGET_NAME} SHARED 
  VisibilityPlugin.h
  VisibilityPlugin.cpp
)

target_link_libraries(${TARGET_NAME}
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(${TARGET_NAME}
  rclcpp
)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
