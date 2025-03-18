cmake_minimum_required(VERSION 3.8)
project(camera_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(camera_tools_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)


# Add the change_exposure_server executable
add_executable(change_exposure_server src/change_exposure_server.cpp)
ament_target_dependencies(change_exposure_server
        rclcpp camera_tools_interfaces)

# Add the hdr_photo executable
add_executable(hdr_photo_server src/hdr_photo.cpp)  # Add the new hdr_photo executable
ament_target_dependencies(hdr_photo_server
        rclcpp
        sensor_msgs
        cv_bridge
        OpenCV
        std_srvs
        camera_tools_interfaces
)

add_executable(take_picture_server src/take_picture_server.cpp)
ament_target_dependencies(take_picture_server
        rclcpp
        sensor_msgs
        OpenCV
        cv_bridge
        camera_tools_interfaces
)

add_executable(take_picture_client src/take_picture_client.cpp)
ament_target_dependencies(take_picture_client
        rclcpp
        OpenCV
        cv_bridge
        camera_tools_interfaces
)


# Testing dependencies (if any tests were added)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

# Install the targets
install(TARGETS
        take_picture_client
        change_exposure_server
        hdr_photo_server
        take_picture_server
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
        scripts/hdr_photo.py
        scripts/take_and_store_picture.py
        scripts/take_picture_server.py
        scripts/color_calibrate.py
        scripts/image_top_level.py
        scripts/clarity_service.py
        scripts/dom.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()