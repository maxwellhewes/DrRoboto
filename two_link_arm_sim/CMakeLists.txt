cmake_minimum_required(VERSION 3.8)
project(two_link_arm_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  urdf
  worlds
  config
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  src/pose_controller.py
  src/simple_joint_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
