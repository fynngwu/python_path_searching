cmake_minimum_required(VERSION 3.8)
project(python_path_searching)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts to lib/${PROJECT_NAME}
install(PROGRAMS
  path_searching/astar_planner_node.py
  path_searching/omnidirectional_tracker_node.py
  path_searching/path_decision_node.py
  path_searching/pure_pursuit_node.py
  path_searching/odom_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
