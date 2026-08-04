cmake_minimum_required(VERSION 3.8)
project(ur5_torque)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(omni_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(feedback_node
  src/feedback.cpp
)

target_include_directories(feedback_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
)

ament_target_dependencies(feedback_node
  rclcpp
  geometry_msgs
  omni_msgs
  sensor_msgs
  ament_index_cpp
  pinocchio
  Eigen3
)

target_link_libraries(feedback_node
  ${PINOCCHIO_LIBRARIES}
  Eigen3::Eigen
  pinocchio::pinocchio
)

install(TARGETS feedback_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
