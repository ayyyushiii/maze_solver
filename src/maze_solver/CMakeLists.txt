cmake_minimum_required(VERSION 3.8)
project(maze_solver)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/maze_solver.py
  scripts/lidar_processor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
