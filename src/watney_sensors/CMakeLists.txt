cmake_minimum_required(VERSION 3.8)
project(watney_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include serialib header directory
include_directories(/usr/local/include)

# Create the executable from your source file(s)
add_executable(h2l_node src/h2l_node.cpp)

# Link libraries and set target dependencies
ament_target_dependencies(h2l_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  nlohmann_json
)

# Find and link the installed serialib library
find_library(SERIALIB_LIBRARY
  NAMES serialib
  PATHS /usr/local/lib /usr/lib
  REQUIRED
)

if (NOT SERIALIB_LIBRARY)
  message(FATAL_ERROR "serialib library not found. Please install serialib.")
endif()

target_link_libraries(h2l_node ${SERIALIB_LIBRARY})

# Install the executable
install(TARGETS
  h2l_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
