cmake_minimum_required(VERSION 3.10)
project(ur5_interfaz_library)

# Use C++17 and position-independent code for shared libs
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# —————————————————————————————
# Enable Qt AUTOMOC for any file with Q_OBJECT
# —————————————————————————————
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# —————————————————————————————
# Find all the packages we need
# —————————————————————————————
find_package(ament_cmake            REQUIRED)
find_package(rclcpp                 REQUIRED)
find_package(std_msgs               REQUIRED)
find_package(sensor_msgs            REQUIRED)
find_package(rviz_common            REQUIRED)
find_package(rviz_rendering         REQUIRED)
find_package(rviz_default_plugins   REQUIRED)
find_package(Qt5                    REQUIRED COMPONENTS Widgets)
find_package(Python3                REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11               REQUIRED)

# —————————————————————————————
# Build the reusable RvizWidget library
# (we list the header so AUTOMOC sees Q_OBJECT)
# —————————————————————————————
add_library(rviz_widget SHARED
  src/rviz_widget.cpp
  include/ur5_interfaz_panel/rviz_widget.hpp
)
target_link_libraries(rviz_widget
  Qt5::Widgets
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
)
# Export the include directory for consumers
target_include_directories(rviz_widget PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# —————————————————————————————
# Build the PyBind11-based Python module
# Output .so right into the build folder for easy import
# —————————————————————————————
pybind11_add_module(rviz_widget_py
  src/python_bindings.cpp
)
# Force the output filename to be rviz_widget_py.so
set_target_properties(rviz_widget_py PROPERTIES
  PREFIX ""
  SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
target_link_libraries(rviz_widget_py
  PRIVATE
    rviz_widget
    Qt5::Widgets
    pybind11::module
)
target_include_directories(rviz_widget_py PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${pybind11_INCLUDE_DIRS}
)

# —————————————————————————————
# Ament package install rules
# —————————————————————————————
install(TARGETS rviz_widget rviz_widget_py
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

# Detect Python version and install Python files in standard site-packages
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(PYTHON_INSTALL_DIR lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)

# Install Python module files in the standard Python path
install(FILES
  src/RvizWrapper.py
  src/__init__.py
  DESTINATION ${PYTHON_INSTALL_DIR}/ur5_interfaz_library
)

# Also install the .so in the Python module directory so it can be imported
install(FILES
  ${CMAKE_BINARY_DIR}/rviz_widget_py.so
  DESTINATION ${PYTHON_INSTALL_DIR}/ur5_interfaz_library
)

# Required for ament_cmake packages
ament_package()
