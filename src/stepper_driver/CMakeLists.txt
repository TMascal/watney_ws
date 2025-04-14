cmake_minimum_required(VERSION 3.8)
project(stepper_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_library(GPIOD_LIBRARY NAMES gpiod)

if(NOT GPIOD_LIBRARY)
  message(FATAL_ERROR "gpiod library not found. Please ensure it’s installed and visible to the linker.")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DeployCameraDistance.srv"
)

# Create Cpp executable
add_executable(deploy_camera_node src/deploy_camera_node.cpp)
ament_target_dependencies(deploy_camera_node rclcpp std_srvs)

target_include_directories(deploy_camera_node PRIVATE /usr/local/include)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(deploy_camera_node ${cpp_typesupport_target} ${GPIOD_LIBRARY})

# Install Cpp executables
install(TARGETS
  deploy_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# python compatible
# ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()