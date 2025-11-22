#include <rclcpp/rclcpp.hpp>
#include "maze_solver/maze_solver_node.hpp" 

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Cria a instância do Node modularizado
    auto node = std::make_shared<MazeSolverNode>();

    // Mantém o node rodando até shutdown
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
