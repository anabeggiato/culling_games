#include "maze_solver/astar.hpp"
#include <queue>
#include <map>
#include <rclcpp/rclcpp.hpp>

AStar::AStar(const std::vector<std::vector<char>> &map) : map_(map) {}

std::pair<int,int> AStar::computeNextMove(int start_x,int start_y,int goal_x,int goal_y) {
    int rows = map_.size();
    int cols = map_[0].size();

    std::priority_queue<AStarNode,std::vector<AStarNode>,CompareAStar> open_list;
    std::map<std::pair<int,int>,std::pair<int,int>> came_from;
    std::map<std::pair<int,int>,int> g_score;

    auto start = std::make_pair(start_x,start_y);
    g_score[start] = 0;
    open_list.push(AStarNode(start_x,start_y,0,manhattan(start_x,start_y,goal_x,goal_y)));

    std::vector<std::pair<int,int>> directions = {{-1,0},{1,0},{0,-1},{0,1}};

    while(!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        if(current.x == goal_x && current.y == goal_y) {
            std::pair<int,int> cur = {goal_x,goal_y};
            std::pair<int,int> prev = came_from[cur];
            while(prev != start) {
                cur = prev;
                prev = came_from[cur];
            }
            return cur;
        }

        for(auto &dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;
            if(nx<0||ny<0||nx>=rows||ny>=cols) continue;
            if (nx != goal_x || ny != goal_y) {
                char cell_content = map_[nx][ny];
                if (cell_content != 'f' && cell_content != 'r') continue;
            }
            
            int tentative_g = current.g + 1;
            auto neighbor = std::make_pair(nx,ny);
            if(g_score.find(neighbor)==g_score.end() || tentative_g<g_score[neighbor]) {
                g_score[neighbor] = tentative_g;
                int h = manhattan(nx,ny,goal_x,goal_y);
                open_list.push(AStarNode(nx,ny,tentative_g,h));
                came_from[neighbor] = {current.x,current.y};
            }
        }
    }

    // Retorna posição inicial se não encontrou caminho
    return start;
}
