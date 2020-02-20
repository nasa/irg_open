set(TARGET link_irg_rqt_tools)

set(SOURCES 
  main.cpp
)

include_directories( 
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/../irg_rqt_tools
)

add_executable(${TARGET}
  ${SOURCES}
  ${HEADERS}
)

target_link_libraries(${TARGET}
  irg_rqt_tools
)

