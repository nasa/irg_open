set(TARGET_NAME IRGIrradianceMapPlugin)

set (HEADERS
  IrradianceMapPlugin.h
  CubemapFilter.h
)

set (SOURCES
  IrradianceMapPlugin.cpp
  CubemapFilter.cpp
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)
target_link_libraries(${TARGET_NAME}
  ${GAZEBO_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${OGRE_RenderSystem_GL_LIBRARIES}
)
ament_target_dependencies(${TARGET_NAME}
  rclcpp
  std_msgs
)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
