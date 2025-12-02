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

// Definição de tipos
using Coordinate = std::pair<int, int>;
using MapType = std::map<Coordinate, std::string>;

// Constantes de direção
const std::string DIR_NAMES[4] = {"up", "down", "left", "right"};
const int DX[4] = {-1, 1, 0, 0};
const int DY[4] = {0, 0, -1, 1};

// ==========================================
// VARIÁVEIS GLOBAIS
// ==========================================

Coordinate predicted_target = {-999, -999};

// ==========================================
// FUNÇÕES AUXILIARES
// ==========================================

std::string GetOppositeDir(std::string dir)
{
    if (dir == "up")
        return "down";
    if (dir == "down")
        return "up";
    if (dir == "left")
        return "right";
    if (dir == "right")
        return "left";
    return "";
}

// Determina a string de direção baseada na diferença de coordenadas
std::string GetDirName(int dx, int dy)
{
    if (dx == -1 && dy == 0)
        return "up";
    if (dx == 1 && dy == 0)
        return "down";
    if (dx == 0 && dy == -1)
        return "left";
    if (dx == 0 && dy == 1)
        return "right";
    return "";
}

// Tenta mover o robô e calcula a posição relativa do target
bool MoveRobot(rclcpp::Node::SharedPtr node,
               rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client,
               std::string direction,
               Coordinate dest_internal_pos)
{
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direction;

    auto result_future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result_future.get();
        if (response->success)
        {

            int abs_robot_row = response->robot_pos[0];
            int abs_robot_col = response->robot_pos[1];
            int abs_target_row = response->target_pos[0];
            int abs_target_col = response->target_pos[1];

            int diff_row = abs_target_row - abs_robot_row;
            int diff_col = abs_target_col - abs_robot_col;

            predicted_target = {dest_internal_pos.first + diff_row, dest_internal_pos.second + diff_col};

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return true;
        }
    }
    return false;
}

// ==========================================
// FASE 1: EXPLORAÇÃO (DFS)
// ==========================================

void DfsExplore(Coordinate u,
                MapType &internal_map,
                rclcpp::Node::SharedPtr node,
                rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client)
{
    for (int i = 0; i < 4; i++)
    {
        Coordinate vizinho = {u.first + DX[i], u.second + DY[i]};

        if (internal_map.find(vizinho) == internal_map.end())
        {
            // Proteção contra Target
            if (predicted_target.first != -999 && vizinho == predicted_target)
            {
                internal_map[vizinho] = "t";
                continue;
            }

            bool moveu = MoveRobot(node, client, DIR_NAMES[i], vizinho);

            if (moveu)
            {
                internal_map[vizinho] = "f";
                DfsExplore(vizinho, internal_map, node, client);
                MoveRobot(node, client, GetOppositeDir(DIR_NAMES[i]), u);
            }
            else
            {
                internal_map[vizinho] = "b";
            }
        }
    }
}

// ==========================================
// MAIN
// ==========================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("maze_solver_complete");
    auto client = node->create_client<cg_interfaces::srv::MoveCmd>("move_command");

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
            return 0;
        RCLCPP_INFO(node->get_logger(), "Aguardando servico move_command...");
    }

    // --- FASE 1: Mapeamento ---
    MapType internal_map;
    Coordinate start_pos = {0, 0};
    internal_map[start_pos] = "f";

    RCLCPP_INFO(node->get_logger(), ">>> FASE 1: INICIANDO MAPEAMENTO (DFS) <<<");
    DfsExplore(start_pos, internal_map, node, client);
    RCLCPP_INFO(node->get_logger(), ">>> MAPEAMENTO FINALIZADO <<<");

    // --- Conversão Map -> Grid ---
    int min_x = INT_MAX, max_x = INT_MIN, min_y = INT_MAX, max_y = INT_MIN;
    Coordinate target_relative = {-999, -999};

    for (auto const &[pos, val] : internal_map)
    {
        if (pos.first < min_x)
            min_x = pos.first;
        if (pos.first > max_x)
            max_x = pos.first;
        if (pos.second < min_y)
            min_y = pos.second;
        if (pos.second > max_y)
            max_y = pos.second;
        if (val == "t")
            target_relative = pos;
    }

    if (target_relative.first == -999 && predicted_target.first != -999)
    {
        target_relative = predicted_target;
    }

    int rows = max_x - min_x + 1;
    int cols = max_y - min_y + 1;

    std::vector<std::vector<std::string>> grid(rows, std::vector<std::string>(cols, "b"));

    for (auto const &[pos, val] : internal_map)
    {
        grid[pos.first - min_x][pos.second - min_y] = val;
    }

    if (target_relative.first != -999)
    {
        int tx = target_relative.first - min_x;
        int ty = target_relative.second - min_y;
        if (tx >= 0 && tx < rows && ty >= 0 && ty < cols)
            grid[tx][ty] = "t";
    }

    Coordinate start_grid = {0 - min_x, 0 - min_y};
    Coordinate target_grid = {-1, -1};

    if (target_relative.first != -999)
    {
        target_grid = {target_relative.first - min_x, target_relative.second - min_y};
    }

    // --- FASE 2: BFS (Cálculo) ---
    if (target_grid.first == -1)
    {
        RCLCPP_ERROR(node->get_logger(), "Target nao encontrado! Abortando.");
        rclcpp::shutdown();
        return 0;
    }

    std::queue<Coordinate> fila;
    std::vector<std::vector<bool>> visitado(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Coordinate>> pai(rows, std::vector<Coordinate>(cols, {-1, -1}));

    fila.push(start_grid);
    visitado[start_grid.first][start_grid.second] = true;
    bool encontrou = false;

    while (!fila.empty())
    {
        Coordinate atual = fila.front();
        fila.pop();

        if (atual == target_grid)
        {
            encontrou = true;
            break;
        }

        for (int i = 0; i < 4; i++)
        {
            int nx = atual.first + DX[i];
            int ny = atual.second + DY[i];

            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols)
            {
                if (!visitado[nx][ny] && grid[nx][ny] != "b")
                {
                    visitado[nx][ny] = true;
                    pai[nx][ny] = atual;
                    fila.push({nx, ny});
                }
            }
        }
    }

    std::vector<Coordinate> caminho;
    if (encontrou)
    {
        Coordinate p = target_grid;
        while (p != start_grid)
        {
            caminho.push_back(p);
            p = pai[p.first][p.second];
        }
        caminho.push_back(start_grid);
        std::reverse(caminho.begin(), caminho.end());
        RCLCPP_INFO(node->get_logger(), "Caminho BFS calculado com %lu passos.", caminho.size());
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Nenhum caminho possivel.");
        rclcpp::shutdown();
        return 0;
    }

    // --- FASE 3: EXECUÇÃO DO CAMINHO ---
    RCLCPP_INFO(node->get_logger(), ">>> FASE 3: EXECUTANDO O CAMINHO OTIMIZADO <<<");


    for (size_t i = 1; i < caminho.size(); i++)
    {
        Coordinate anterior = caminho[i - 1];
        Coordinate proximo = caminho[i];

        // Calcula a direção baseada na diferença no Grid
        int dx = proximo.first - anterior.first;
        int dy = proximo.second - anterior.second;

        std::string direction = GetDirName(dx, dy);

        if (direction != "")
        {
            Coordinate next_internal = {proximo.first + min_x, proximo.second + min_y};

            RCLCPP_INFO(node->get_logger(), "Movendo para passo %lu/%lu (%s)", i, caminho.size() - 1, direction.c_str());

            MoveRobot(node, client, direction, next_internal);
        }
    }

    RCLCPP_INFO(node->get_logger(), ">>> SUCESSO! Robo posicionado adjacente ao Target! <<<");

    rclcpp::shutdown();
    return 0;
}