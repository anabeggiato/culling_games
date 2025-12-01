// inclusão da biblioteca de client do ros
#include "rclcpp/rclcpp.hpp"
// incluclusão da biblioteca de servidor
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

// ouras dependencias encessárias
#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <queue>
#include <thread>

using namespace std::chrono_literals;

// FUNÇÃO GETMAP
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

// FUNÇÃO BUILD MAP
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
    // for (int i = 0; i < (linhas); i++)
    // {
    //     for (int j = 0; j < colunas; j++)
    //     {
    //         std::cout << map[i][j] << "  ";
    //     }
    //     std::cout << std::endl;
    // }

    return map;
}

// ALGORITMO PARA PERCORRER O MAPA
std::vector<std::pair<int, int>> Bfs(std::vector<std::vector<std::string>> &map, int &linhas, int &colunas)
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
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

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

        for (int i = 0; i < 4; i++)
        {
            int nx = posicao_atual.first + dx[i];
            int ny = posicao_atual.second + dy[i];

            // 1. Verificar se está dentro dos limites do mapa
            if (nx >= 0 && nx < linhas && ny >= 0 && ny < colunas)
            {
                // 2. Verificar se não é bloqueio ('b')
                if (map[nx][ny] != "b")
                {
                    auto vizinho = std::make_pair(nx, ny);

                    // 3. Verificar se já foi visitado (mantendo sua lógica original com std::find)
                    if (find(visitados.begin(), visitados.end(), vizinho) == visitados.end())
                    {
                        pai[nx][ny] = posicao_atual;
                        visitados.push_back(vizinho);
                        a_visitar.push(vizinho);
                    }
                }
            }
        }
    }

    // reconstruir o caminho usando a tabela de pais (percorrendo pais partindo do target e indo até o robô)
    if (pai[target_x][target_y].first == -1)
    {
        std::cout << "Nenhum caminho encontrado." << std::endl;
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
    // std::cout << "Caminho encontrado:" << std::endl;
    // for (auto &p : caminho)
    // {
    //     std::cout << "(" << p.first << ", " << p.second << ")" << std::endl;
    // }

    return caminho;
}

//ALGORITMO PARA O ROBO SEGUIR O CAMINHO ENCONTRADO
void ExecutePath(std::vector<std::pair<int, int>> &caminho)
{
    // 1. Criação do nó e do cliente (Idêntico ao seu exemplo)
    auto node = rclcpp::Node::make_shared("move_client_node");
    
    // Supondo que o nome do serviço seja "move_command" (ajuste se for diferente no seu launch file)
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client =
        node->create_client<cg_interfaces::srv::MoveCmd>("move_command");

    // 2. Loop de espera pelo serviço (Idêntico ao seu exemplo)
    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrompido enquanto aguardava o serviço. Saindo.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Serviço MoveCmd indisponível, aguardando...");
    }

    // 3. Iterar sobre o caminho para enviar comandos passo a passo
    // O loop vai até size() - 1 porque comparamos o atual com o próximo
    for (size_t i = 0; i < caminho.size() - 1; ++i)
    {
        std::pair<int, int> atual = caminho[i];
        std::pair<int, int> proximo = caminho[i + 1];
        std::string direcao;

        // Determinar a string de direção baseada na diferença de coordenadas
        if (proximo.first < atual.first)       direcao = "up";
        else if (proximo.first > atual.first)  direcao = "down";
        else if (proximo.second < atual.second) direcao = "left";
        else if (proximo.second > atual.second) direcao = "right";

        // Cria a requisição (Idêntico ao seu exemplo, mas dentro do loop)
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direcao;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Movendo para: %s", direcao.c_str());

        // Envia a requisição assíncrona
        auto result_future = client->async_send_request(request);

        // Espera pela resposta DESTE passo específico
        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            
            // Verifica se o movimento foi bem sucedido (campo 'success' da resposta)
            if (response->success)
            {
                // Podemos acessar robot_pos ou target_pos se necessário
                // response->robot_pos
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Movimento executado com sucesso.");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Falha ao mover o robô! Bloqueio encontrado?");
                break; // Para a execução se o robô falhar em mover
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Falha ao chamar o serviço MoveCmd");
            break;
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execução do caminho finalizada.");
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
    auto caminho = Bfs(map, linhas, colunas);
    ExecutePath(caminho);

    // encerra o ros
    rclcpp::shutdown();

    return 0;
}