set(TARGET irg_rqt_tools)

set(SOURCES 
  ImageTrigger.cpp
  ImageTriggerWidget.cpp
  PanTilt.cpp
  Utils.cpp
)

set(HEADERS
  ImageTrigger.h
  ImageTriggerWidget.h
  PanTilt.h
  Utils.h
)

set(MOC_HEADERS
  ImageTrigger.h
  ImageTriggerWidget.h
  PanTilt.h
)

set(UIC_SOURCES 
  ImageTrigger.ui 
  ImageTriggerWidget.ui 
  PanTilt.ui
)

qt5_wrap_ui(UIC_FILES ${UIC_SOURCES})
qt5_wrap_cpp(MOC_FILES ${MOC_HEADERS})

include_directories(
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(QT_LINK_LIBRARIES Qt5::Widgets)

include_directories( 
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(${TARGET} SHARED
  ${SOURCES}
  ${HEADERS}
  ${MOC_HEADERS}
  ${MOC_FILES}
  ${UIC_FILES}
)

target_link_libraries(${TARGET}
  ${QT_LINK_LIBRARIES}
)

ament_target_dependencies(${TARGET}
  rclcpp
  rqt_gui_cpp
  std_msgs
)

install(
  TARGETS ${TARGET}
  EXPORT ${TARGET}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${TARGET})
