set(TARGET_NAME IRGIrradianceMapPlugin)

set (HEADERS
  IrradianceMapPlugin.h
  CubemapFilter.h
)

set (SOURCES
  IrradianceMapPlugin.cpp
  CubemapFilter.cpp
)

find_package(GLEW REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${TARGET_NAME}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${GLEW_LIBRARIES}
)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")
