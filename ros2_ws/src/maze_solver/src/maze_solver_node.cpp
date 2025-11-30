// inclusão da biblioteca de client do ros
#include "rclcpp/rclcpp.hpp"
// incluclusão da biblioteca de servidor
#include "cg_interfaces/srv/get_map.hpp"

// ouras dependencias encessárias
#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <queue>

using namespace std::chrono_literals;

std::shared_ptr<cg_interfaces::srv::GetMap::Response> GetMap()
{
    // criação de um nó ROS chamado "get_map_client" e retorna um ponteiro compartilhado armazenado na variavel node
    auto node = rclcpp::Node::make_shared("get_map_client");

    // cria um cliente ROS para o serviço GetMap
    // o métodocreate_client cria um objeto do tipo Client<GetMap> associado ao nome do serviço "get_map"
    // o cliente criado é armazenado em um ponteiro compartilhado chamado client
    // este cliente permite que o nó envie requisições ao servidor desse serviço
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client =
        node->create_client<cg_interfaces::srv::GetMap>("get_map");

    // cria um objeto request vazio, assim como um ponteiro compartilhado para esse objeto, o qual é do tipo GetMap::Request.
    // Esse ponteiro é armazenado na variável request
    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();

    // espera um segundo caso o client ainda n tenha respondido
    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            // retorna erro caso o servidor retorne um erro
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        // tenta conexão novamente caso o servidor não responda
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // amrazena o retorno do request e armazena na variavel result
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // log de sucesso mostrando o primeiro item do mapa retornado pelo servidor
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mapa recebido com sucesso");
        return result.get();
    }
    else
    {
        // log em casod e erro
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_map");
        return nullptr;
    }
}

std::vector<std::vector<std::string>> BuildMap(const std::shared_ptr<cg_interfaces::srv::GetMap::Response> map_infos)
{
    // define a quantidade de linhas e colunas da matriz de acordo com o recebido do getmap
    int linhas = map_infos->occupancy_grid_shape[0];
    int colunas = map_infos->occupancy_grid_shape[1];

    // cria uma matriz de strings com tamanho linhasxcolunas previamente preenchida com 'x'
    std::vector<std::vector<std::string>> map(linhas, std::vector<std::string>(colunas, "x"));

    // pega a string do mapa e adicona em uma variável (pra não ter que usar map_infos->occupancy_grid_flattened toda vez)
    const auto &flattened = map_infos->occupancy_grid_flattened;

    // percorre a string do mapa e vai adionando cada caractere do mapa em seu respectivo lugar da matriz
    for (int i = 0; i < (linhas); i++)
    {
        for (int j = 0; j < colunas; j++)
        {
            int index = i * colunas + j;
            map[i][j] = flattened[index];
        }
    }

    // PRINT MAP
    for (int i = 0; i < (linhas); i++)
    {
        for (int j = 0; j < colunas; j++)
        {
            std::cout << map[i][j] << "  ";
        }
        std::cout << std::endl;
    }

    return map;
}

void Bfs(std::vector<std::vector<std::string>> &map, int &linhas, int &colunas)
{
    // encontrar a posição do robo e do target
    int robot_x, robot_y, target_x, target_y;
    for (int i = 0; i < (linhas); i++)
    {
        for (int j = 0; j < colunas; j++)
        {
            if (map[i][j] == "r")
            {
                robot_x = i;
                robot_y = j;
            }
            else if (map[i][j] == "t")
            {
                target_x = i;
                target_y = j;
            }
        }
    }

    // criar a fila que será usada pelo algoritmo
    std::queue<std::pair<int, int>> a_visitar;

    // criar uma matriz de posições já visitadas
    std::vector<std::pair<int, int>> visitados;

    // criar lista de pais (a posição atual é sempre o pai de seu vizinho)
    //  cada pai[i][j] = {pai_linha, pai_coluna}
    std::vector<std::vector<std::pair<int, int>>> pai(linhas, std::vector<std::pair<int, int>>(colunas, {-1, -1}));
    pai[robot_x][robot_y] = std::make_pair(robot_x, robot_y);

    std::pair<int, int> robot_position = std::make_pair(robot_x, robot_y);
    std::pair<int, int> target_position = std::make_pair(target_x, target_y);

    // inserir posição inicial do robo na fila
    a_visitar.push(robot_position);
    visitados.push_back(robot_position);

    std::cout << "Posição inicial do robô: " << robot_x << ", " << robot_y << std::endl;
    std::cout << "Posição do target: " << target_x << ", " << target_y << std::endl;

    std::cout << "Inicializando BFS... " << std::endl;
    // enquanto a fila estiver vazia
    while (!a_visitar.empty())
    {
        // remover a posição atual
        auto posicao_atual = a_visitar.front();
        a_visitar.pop();

        // parar caso seja o target
        if (posicao_atual == target_position)
        {
            std::cout << "Target alcançado! " << std::endl;
            break;
        }

        // não sendo o target, verificar os vizinhos
        // caso o vizinho ainda n tenha sido visitado e não esteja bloqueado, marcar como visitado, registrar o pai (posição atual) e enfileirar

        // ---- VIZINHO DE CIMA ----
        int nx = posicao_atual.first - 1;
        int ny = posicao_atual.second;

        if (nx >= 0)
        {
            if (map[nx][ny] != "b")
            {
                auto vizinho = std::make_pair(nx, ny);

                if (find(visitados.begin(), visitados.end(), vizinho) == visitados.end())
                {
                    pai[nx][ny] = posicao_atual;
                    visitados.push_back(vizinho);
                    a_visitar.push(vizinho);
                }
            }
        }

        // ---- VIZINHO DE BAIXO ----
        nx = posicao_atual.first + 1;
        ny = posicao_atual.second;

        if (nx < map.size())
        {
            if (map[nx][ny] != "b")
            {
                auto vizinho = std::make_pair(nx, ny);

                if (find(visitados.begin(), visitados.end(), vizinho) == visitados.end())
                {
                    pai[nx][ny] = posicao_atual;
                    visitados.push_back(vizinho);
                    a_visitar.push(vizinho);
                }
            }
        }

        // ---- VIZINHO DA ESQUERDA ----
        nx = posicao_atual.first;
        ny = posicao_atual.second - 1;

        if (ny >= 0)
        {
            if (map[nx][ny] != "b")
            {
                auto vizinho = std::make_pair(nx, ny);

                if (find(visitados.begin(), visitados.end(), vizinho) == visitados.end())
                {
                    pai[nx][ny] = posicao_atual;
                    visitados.push_back(vizinho);
                    a_visitar.push(vizinho);
                }
            }
        }

        // ---- VIZINHO DA DIREITA ----
        nx = posicao_atual.first;
        ny = posicao_atual.second + 1;

        if (ny < map[0].size())
        {
            if (map[nx][ny] != "b")
            {
                auto vizinho = std::make_pair(nx, ny);

                if (find(visitados.begin(), visitados.end(), vizinho) == visitados.end())
                {
                    pai[nx][ny] = posicao_atual;
                    visitados.push_back(vizinho);
                    a_visitar.push(vizinho);
                }
            }
        }
    }

    // reconstruir o caminho usando a tabela de pais (percorrendo pais partindo do target e indo até o robô)

    if (pai[target_x][target_y].first == -1)
    {
        std::cout << "Nenhum caminho encontrado." << std::endl;
        return;
    }

    std::vector<std::pair<int, int>> caminho;

    // começando no target
    auto atual = target_position;

    // enquanto o pai não for ele mesmo (posição inicial)
    while (atual != pai[atual.first][atual.second])
    {
        caminho.push_back(atual);
        atual = pai[atual.first][atual.second];
    }

    // adicionar a posição inicial
    caminho.push_back(atual);

    // inverter o caminho para ficar do início até o target
    std::reverse(caminho.begin(), caminho.end());

    // mostrar o caminho encontrado
    std::cout << "Caminho encontrado:" << std::endl;
    for (auto &p : caminho)
    {
        std::cout << "(" << p.first << ", " << p.second << ")" << std::endl;
    }
}

int main(int argc, char **argv)
{
    // inicializa o sistema ros
    rclcpp::init(argc, argv);

    // recebe a string do mapa, armazena em uma variavel e chama a função de transformar em uma matriz
    auto mapa = GetMap();
    auto map = BuildMap(mapa);

    // roda o algoritmo
    int linhas = 29;
    int colunas = 29;
    Bfs(map, linhas, colunas);

    // encerra o ros
    rclcpp::shutdown();

    return 0;
}