cmake_minimum_required(VERSION 3.8)
project(torque_geo_ur)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(omni_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)


set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
add_executable(torque ${SRC_DIR}/torque.cpp)

target_include_directories(
    torque   PRIVATE ${EIGEN3_INCLUDE_DIRS}
    )
ament_target_dependencies(torque 
        rclcpp 
        trajectory_msgs 
        sensor_msgs
        omni_msgs
        geometry_msgs
    )
target_link_libraries(torque 
    ${EIGEN3_LIBRARIES} 
    pinocchio::pinocchio
)

install(DIRECTORY 
    src DESTINATION share/${PROJECT_NAME})
    
install(TARGETS torque
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
