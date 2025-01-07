cmake_minimum_required(VERSION 3.8)
project(camera_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(camera_tools REQUIRED)

add_executable(server src/change_exposure_server.cpp)
ament_target_dependencies(server rclcpp camera_tools)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
        server
        DESTINATION lib/${PROJECT_NAME})

ament_package()
