cmake_minimum_required(VERSION 3.8)
project(serial_communicate_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED) #for custom msgs
find_package(serial_communicate_cpp REQUIRED)


add_executable(from_leonardo src/pub_node.cpp)
ament_target_dependencies(from_leonardo rclcpp custom_msgs) #custom_msgs!

install(TARGETS 
  from_leonardo
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(to_leonardo src/sub_node.cpp)
ament_target_dependencies(to_leonardo rclcpp custom_msgs)

install(TARGETS
  #from_leonardo  #someone who publish data for leonardo
  to_leonardo
  DESTINATION lib/${PROJECT_NAME})

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
