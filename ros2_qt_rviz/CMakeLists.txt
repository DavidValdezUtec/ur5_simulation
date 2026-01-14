cmake_minimum_required(VERSION 3.8)
project(ros2_qt_rviz)

# 1. Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# 2. Configuración de Qt
set(CMAKE_AUTOMOC ON)

# 3. Definir el ejecutable
add_executable(my_viz_node 
  src/main.cpp 
  src/simple_viz.cpp
  include/ros2_qt_rviz/simple_viz.hpp 
)

# 4. Include directories
target_include_directories(my_viz_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 5. Linkear librerías (CRÍTICO)
target_link_libraries(my_viz_node
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins # Necesario para cargar Grid, RobotModel, etc.
  Qt5::Widgets
)

install(TARGETS my_viz_node DESTINATION lib/${PROJECT_NAME})
ament_package()