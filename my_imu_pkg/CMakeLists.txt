cmake_minimum_required(VERSION 3.5)
project(my_imu_pkg)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/read_all_data.py
  scripts/imu_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
