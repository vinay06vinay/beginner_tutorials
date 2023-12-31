cmake_minimum_required(VERSION 3.5)
project(beginner_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)


# publisher executable
add_executable(talker src/beginner_publisher.cpp)
ament_target_dependencies(talker 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros

)
set(srv_files
  "srv/CustomService.srv"
  )
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
"rosidl_typesupport_cpp" 
)

add_executable(listener src/beginner_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
target_link_libraries(talker "${cpp_typesupport_target}")
target_link_libraries(listener "${cpp_typesupport_target}")


install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)



if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # the following lines skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  # Add a gtest executable
  set(TESTFILES
    test/tf_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs)

  install(TARGETS
  ${PROJECT_NAME}_test
  DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
