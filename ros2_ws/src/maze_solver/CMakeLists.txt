cmake_minimum_required(VERSION 3.5)
project(maze_solver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

include_directories(
  include
)

add_executable(maze_solver_node
    src/maze_solver_node_main.cpp
    src/maze_solver_node.cpp
    src/astar.cpp
)

ament_target_dependencies(maze_solver_node
  rclcpp
  cg_interfaces
)

install(TARGETS
  maze_solver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
