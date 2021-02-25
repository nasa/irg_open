cmake_minimum_required(VERSION 3.5)
project(irg_tools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

add_subdirectory(src)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

ament_package()
