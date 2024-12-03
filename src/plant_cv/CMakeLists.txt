cmake_minimum_required(VERSION 3.8)
project(plant_cv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add executable target with source files
add_executable(arducam_driver src/arducam_driver.cpp)

# Link libraries for the target
ament_target_dependencies(arducam_driver
        "rclcpp"
        "sensor_msgs"
        "cv_bridge"
        "OpenCV"
)
# Install the executable
install(TARGETS arducam_driver
        DESTINATION lib/${PROJECT_NAME})

add_executable(c2_cam_viewer src/c2_cam_viewer.cpp)
ament_target_dependencies(c2_cam_viewer rclcpp sensor_msgs cv_bridge OpenCV)
install(TARGETS c2_cam_viewer DESTINATION lib/${PROJECT_NAME})

ament_package()
