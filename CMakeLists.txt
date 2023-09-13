cmake_minimum_required(VERSION 3.8)
project(trinamic_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

#add_library(trinamic_control
#  src/${PROJECT_NAME}/trinamic_joint/trinamic_joint.cpp
#)
#ament_target_dependencies(trinamic_control
#  rclcpp
#)

add_executable(trinamic_control_node
  src/trinamic_control_node.cpp
)
target_link_libraries(trinamic_control_node
#  trinamic_control
  yaml-cpp
)
ament_target_dependencies(trinamic_control_node
  rclcpp
  yaml_cpp_vendor
)

install(TARGETS
  trinamic_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/${PROJECT_NAME}_test.cpp
    #test/${PROJECT_NAME}/trinamic_message/trinamic_message_test.cpp
    #test/${PROJECT_NAME}/trinamic_message/trinamic_message_command_test.cpp
    #test/${PROJECT_NAME}/trinamic_message/trinamic_message_response_test.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    std_msgs
  )
  #target_link_libraries(${PROJECT_NAME}_test trinamic_control)
endif()

include_directories(include)

ament_package()
