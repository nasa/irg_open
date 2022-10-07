set(TARGET_NAME IRGNearFarClipPlugin)

set (HEADERS
  NearFarClipPlugin.h
)

set (SOURCES
  NearFarClipPlugin.cpp
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${TARGET_NAME}
  ${GAZEBO_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
)
ament_target_dependencies(${TARGET_NAME}
  rclcpp
)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
