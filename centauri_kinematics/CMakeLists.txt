cmake_minimum_required(VERSION 3.8)
project(centauri_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(centauri_interfaces REQUIRED)

# Install custom nodes
ament_python_install_package(scripts)
install(PROGRAMS
  scripts/kinematics_server.py
  scripts/kinematics_client.py
  scripts/home_server.py
  scripts/home_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
