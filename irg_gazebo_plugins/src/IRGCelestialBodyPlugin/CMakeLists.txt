set(TARGET_NAME IRGCelestialBodyPlugin)

set (HEADERS
  CelestialBodyPlugin.h
)

set (SOURCES
  CelestialBodyPlugin.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${TARGET_NAME}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")
