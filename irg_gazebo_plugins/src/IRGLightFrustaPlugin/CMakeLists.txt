set(TARGET_NAME IRGLightFrustaPlugin)

set (HEADERS
  LightFrustaPlugin.h
)

set (SOURCES
  LightFrustaPlugin.cpp
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${TARGET_NAME}
  ${GAZEBO_LIBRARIES}
)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
