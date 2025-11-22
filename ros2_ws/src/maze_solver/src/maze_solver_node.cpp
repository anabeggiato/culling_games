#include "maze_solver/maze_solver_node.hpp"
#include <sstream>

MazeSolverNode::MazeSolverNode() : Node("maze_solver")
{
    map_client_ = this->create_client<GetMapSrv>("get_map");
    move_client_ = this->create_client<MoveCmdSrv>("move_cmd");

    sensor_sub_ = this->create_subscription<RobotSensorsMsg>(
        "/culling_games/robot_sensors", 10,
        std::bind(&MazeSolverNode::sensorCallback, this, std::placeholders::_1));

    requestMap();
}

void MazeSolverNode::requestMap() {
    auto request = std::make_shared<GetMapSrv::Request>();
    if(!map_client_->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Serviço GetMap não disponível!");
        return;
    }

    map_client_->async_send_request(request,
        [this](rclcpp::Client<GetMapSrv>::SharedFuture future){
            auto response = future.get();
            int rows = response->occupancy_grid_shape[0];
            int cols = response->occupancy_grid_shape[1];
            map_.resize(rows, std::vector<char>(cols));

            bool robot_found = false, target_found = false;
            std::stringstream map_output;
            map_output << "\n--- MAPA RECEBIDO (" << rows << "x" << cols << ") ---\n";

            for(int i=0;i<rows;i++){
                for(int j=0;j<cols;j++){
                    char cell = response->occupancy_grid_flattened[i*cols+j][0];
                    map_[i][j] = cell;

                    if(cell=='r'){ robot_x_=i; robot_y_=j; robot_found=true; map_output<<"\033[1;34mr\033[0m "; }
                    else if(cell=='t'){ goal_x_=i; goal_y_=j; target_found=true; map_output<<"\033[1;32mt\033[0m "; }
                    else map_output<<cell<<" ";
                }
                map_output<<"\n";
            }

            if(!robot_found){ RCLCPP_WARN(this->get_logger(), "Robô não encontrado! Usando (0,0)."); robot_x_=0; robot_y_=0; }
            if(!target_found){ RCLCPP_WARN(this->get_logger(), "Alvo não encontrado! Usando fallback (28,28)."); goal_x_=rows-1; goal_y_=cols-1; }

            RCLCPP_INFO(this->get_logger(), map_output.str().c_str());
            RCLCPP_INFO(this->get_logger(), "Mapa: %d x %d | Robô: (%d,%d) | Alvo: (%d,%d)", rows,cols,robot_x_,robot_y_,goal_x_,goal_y_);

            pathfinder_ = std::make_unique<AStar>(map_); // Inicializa A* com o mapa
        }
    );
}

void MazeSolverNode::sensorCallback(RobotSensorsMsg::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Sensores: Up:%s Down:%s Left:%s Right:%s", 
                msg->up.c_str(), msg->down.c_str(), msg->left.c_str(), msg->right.c_str());

    if(!pathfinder_){ RCLCPP_WARN(this->get_logger(), "Mapa ainda não carregado."); return; }

    if(robot_x_ == goal_x_ && robot_y_ == goal_y_){
        RCLCPP_INFO(this->get_logger(), "🎉 ALVO ALCANÇADO em (%d,%d)!", goal_x_,goal_y_);
        return;
    }

    auto next_step = pathfinder_->computeNextMove(robot_x_, robot_y_, goal_x_, goal_y_);
    int nx = next_step.first, ny = next_step.second;

    if(nx==robot_x_ && ny==robot_y_){
        RCLCPP_WARN(this->get_logger(), "A* não encontrou caminho. Robô em (%d,%d)", robot_x_,robot_y_);
        return;
    }

    std::string direction;
    if(nx>robot_x_) direction="down";
    else if(nx<robot_x_) direction="up";
    else if(ny>robot_y_) direction="right";
    else if(ny<robot_y_) direction="left";

    sendMoveCommand(direction, nx, ny);
    RCLCPP_INFO(this->get_logger(), "Próximo passo: (%d,%d) | Direção: %s", nx,ny,direction.c_str());
}

void MazeSolverNode::sendMoveCommand(const std::string &direction, int target_x, int target_y)
{
    auto request = std::make_shared<MoveCmdSrv::Request>();
    request->direction = direction;

    if(!move_client_->service_is_ready()){ RCLCPP_WARN(this->get_logger(),"MoveCmd não pronto."); return; }

    move_client_->async_send_request(request, [this,target_x,target_y](rclcpp::Client<MoveCmdSrv>::SharedFuture future){
        auto response = future.get();
        if(response->success){
            robot_x_=target_x; robot_y_=target_y;
            RCLCPP_INFO(this->get_logger(), "Movimento bem-sucedido: (%d,%d)", robot_x_,robot_y_);
        } else RCLCPP_ERROR(this->get_logger(), "Movimento falhou!");
    });
}
