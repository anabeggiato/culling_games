# Maze Solve para projeto Culling Games

Esse projeto cria um pacote ROS chamado maze-solver para resolver o desafio do culling games.

Para rodar o projeto, é necessário rodar `colcon build` seguido de `source install/setup.zsh` (ou .bash dependendo do tipo de terminal) na raiz do projeto. Após isso, é necessário rodar `ros2 run cg maze` para encontrar o labirinto.

Em um segundo terminal, rodar `colcon build` seguido de `source install/setup.zsh` (ou .bash dependendo do tipo de terminal) também na raiz do projeto. Em seguida, rodar o comando `ros2 run maze_solver maze_solver_pt1` se quiser o algoritmo que roda com um mapa pré recebiso ou `ros2 run maze_solver maze_solver_pt2` para executar o algoritmo que mapeia o ambiente antes de rodar o bfs.
