#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <map>
#include <queue>
#include <algorithm>
#include <climits>

using namespace std::chrono_literals;

// Definição de tipos para facilitar leitura
using Coordinate = std::pair<int, int>;
using MapType = std::map<Coordinate, std::string>;

// Variáveis globais de direção (constantes)
const std::string DIR_NAMES[4] = {"up", "down", "left", "right"};
const int DX[4] = {-1, 1, 0, 0};
const int DY[4] = {0, 0, -1, 1};

// ==========================================
// FUNÇÕES AUXILIARES DE MOVIMENTO E LÓGICA
// ==========================================

std::string GetOppositeDir(std::string dir) {
    if (dir == "up") return "down";
    if (dir == "down") return "up";
    if (dir == "left") return "right";
    if (dir == "right") return "left";
    return "";
}

// Tenta mover o robô e retorna true se conseguiu (e detecta se é target)
bool MoveRobot(rclcpp::Node::SharedPtr node, 
               rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client, 
               std::string direction, 
               bool &found_target) 
{
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direction;

    auto result_future = client->async_send_request(request);
    
    // Espera a resposta
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result_future.get();
        if (response->success) {
            // Verifica se a posição atual do robô é igual à do target (se o serviço retornar isso)
            // Assumindo que o serviço preenche robot_pos e target_pos
            if (response->robot_pos == response->target_pos) {
                found_target = true;
            }
            
            // Delay físico obrigatório
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return true;
        }
    }
    return false;
}

// ==========================================
// FASE 1: EXPLORAÇÃO (DFS RECURSIVO)
// ==========================================

void DfsExplore(Coordinate u, 
                MapType &internal_map, 
                rclcpp::Node::SharedPtr node, 
                rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client)
{
    // Para cada um dos 4 vizinhos
    for (int i = 0; i < 4; i++)
    {
        Coordinate vizinho = {u.first + DX[i], u.second + DY[i]};

        // Se eu ainda não conheço esse vizinho (não está no mapa)
        if (internal_map.find(vizinho) == internal_map.end())
        {
            bool achou_target = false;
            
            // Tenta mover fisicamente
            bool moveu = MoveRobot(node, client, DIR_NAMES[i], achou_target);

            if (moveu)
            {
                // Se moveu, marca no mapa
                if (achou_target) {
                    internal_map[vizinho] = "t"; // Target
                    RCLCPP_INFO(node->get_logger(), "TARGET ENCONTRADO NA POSIÇÃO RELATIVA (%d, %d)!", vizinho.first, vizinho.second);
                } else {
                    internal_map[vizinho] = "f"; // Free (livre)
                }

                // === RECURSÃO (Vai fundo) ===
                DfsExplore(vizinho, internal_map, node, client);

                // === BACKTRACKING (Volta fisicamente) ===
                bool lixo; // Não precisamos checar target na volta
                MoveRobot(node, client, GetOppositeDir(DIR_NAMES[i]), lixo);
            }
            else
            {
                // Se bateu, é parede
                internal_map[vizinho] = "b"; // Blocked
            }
        }
    }
}

// ==========================================
// MAIN (Fluxo Principal)
// ==========================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("maze_solver_simple");
    auto client = node->create_client<cg_interfaces::srv::MoveCmd>("move_command");

    // 1. Esperar o serviço ficar online
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) return 0;
        RCLCPP_INFO(node->get_logger(), "Aguardando serviço...");
    }

    // Estruturas de dados principais
    MapType internal_map;
    Coordinate start_pos = {0, 0};
    internal_map[start_pos] = "f"; // Posição inicial é livre

    RCLCPP_INFO(node->get_logger(), ">>> INICIANDO MAPEAMENTO (DFS) <<<");
    
    // Roda a exploração. O robô vai andar tudo e voltar pro (0,0) no final.
    DfsExplore(start_pos, internal_map, node, client);

    RCLCPP_INFO(node->get_logger(), ">>> MAPEAMENTO FINALIZADO. MAPA DESCOBERTO: %lu celulas", internal_map.size());

    // ---------------------------------------------------------
    // CONVERSÃO: Map (esparso) -> Grid (Matriz para o BFS)
    // ---------------------------------------------------------
    
    // Achar os limites do mapa (min/max X e Y)
    int min_x = INT_MAX, max_x = INT_MIN, min_y = INT_MAX, max_y = INT_MIN;
    Coordinate target_relative = {-999, -999}; // Pra saber onde o target ficou no grid

    for (auto const& [pos, val] : internal_map) {
        if (pos.first < min_x) min_x = pos.first;
        if (pos.first > max_x) max_x = pos.first;
        if (pos.second < min_y) min_y = pos.second;
        if (pos.second > max_y) max_y = pos.second;
        
        if (val == "t") target_relative = pos;
    }

    int rows = max_x - min_x + 1;
    int cols = max_y - min_y + 1;

    // Criar Grid
    std::vector<std::vector<std::string>> grid(rows, std::vector<std::string>(cols, "b"));
    
    // Preencher Grid
    for (auto const& [pos, val] : internal_map) {
        grid[pos.first - min_x][pos.second - min_y] = val;
    }

    // Ajustar Start e Target para coordenadas da matriz (0 a N)
    Coordinate start_grid = {0 - min_x, 0 - min_y};
    Coordinate target_grid = {-1, -1};
    
    if (target_relative.first != -999) {
        target_grid = {target_relative.first - min_x, target_relative.second - min_y};
    }

    RCLCPP_INFO(node->get_logger(), "Grid gerado: %dx%d. Start no Grid: (%d, %d)", rows, cols, start_grid.first, start_grid.second);

    // ---------------------------------------------------------
    // FASE 2: CÁLCULO DE ROTA (BFS)
    // ---------------------------------------------------------
    
    if (target_grid.first == -1) {
        RCLCPP_ERROR(node->get_logger(), "Target nao foi encontrado durante a exploracao! Abortando BFS.");
        rclcpp::shutdown();
        return 0;
    }

    std::queue<Coordinate> fila;
    std::vector<std::vector<bool>> visitado(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Coordinate>> pai(rows, std::vector<Coordinate>(cols, {-1, -1}));

    fila.push(start_grid);
    visitado[start_grid.first][start_grid.second] = true;
    pai[start_grid.first][start_grid.second] = start_grid;

    bool encontrou = false;

    while(!fila.empty()) {
        Coordinate atual = fila.front();
        fila.pop();

        if (atual == target_grid) {
            encontrou = true;
            break;
        }

        for(int i=0; i<4; i++) {
            int nx = atual.first + DX[i];
            int ny = atual.second + DY[i];

            if(nx >= 0 && nx < rows && ny >= 0 && ny < cols) {
                if(!visitado[nx][ny] && grid[nx][ny] != "b") {
                    visitado[nx][ny] = true;
                    pai[nx][ny] = atual;
                    fila.push({nx, ny});
                }
            }
        }
    }

    if (!encontrou) {
        RCLCPP_ERROR(node->get_logger(), "Caminho impossivel para o target.");
    } else {
        // Reconstruir caminho
        std::vector<Coordinate> caminho;
        Coordinate p = target_grid;
        while(p != start_grid) {
            caminho.push_back(p);
            p = pai[p.first][p.second];
        }
        caminho.push_back(start_grid);
        std::reverse(caminho.begin(), caminho.end());

        RCLCPP_INFO(node->get_logger(), "Caminho Otimizado Encontrado com %lu passos!", caminho.size());
        
        // Aqui você poderia chamar MoveRobot seguindo esse vetor 'caminho' 
        // para executar a rota final mais rápida.
    }

    rclcpp::shutdown();
    return 0;
}