cmake_minimum_required(VERSION 3.5)
project(maze_solver)

# 1. Encontrar dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

include_directories(
  include
)

# =========================================================
# Executável da Parte 1 (BFS com Mapa Estático)
# =========================================================
add_executable(maze_solver_pt1
    src/maze_solver_node_pt1.cpp
)

ament_target_dependencies(maze_solver_pt1
  rclcpp
  cg_interfaces
)

# =========================================================
# Executável da Parte 2 (Mapeamento DFS + BFS)
# =========================================================
add_executable(maze_solver_pt2
    src/maze_solver_node_pt2.cpp
)

ament_target_dependencies(maze_solver_pt2
  rclcpp
  cg_interfaces
)

# =========================================================
# Instalação (Torna visível para o ros2 run)
# =========================================================
install(TARGETS
  maze_solver_pt1
  maze_solver_pt2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()