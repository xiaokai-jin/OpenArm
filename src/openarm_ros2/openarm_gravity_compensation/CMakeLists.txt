cmake_minimum_required(VERSION 3.22)
project(openarm_gravity_compensation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

add_library(gravity_dynamics SHARED
  src/dynamics.cpp
)

target_include_directories(gravity_dynamics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(gravity_dynamics
  PUBLIC
    kdl_parser::kdl_parser
    orocos-kdl
    urdfdom::urdfdom_model
)

add_executable(gravity_compensation_node
  src/gravity_compensation_node.cpp
)

target_link_libraries(gravity_compensation_node gravity_dynamics)
ament_target_dependencies(gravity_compensation_node rclcpp sensor_msgs std_msgs)

install(TARGETS gravity_dynamics gravity_compensation_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
