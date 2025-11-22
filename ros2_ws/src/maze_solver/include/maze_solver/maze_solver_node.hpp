#pragma once
#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/msg/robot_sensors.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <vector>
#include <string>
#include <utility>
#include <memory>
#include "astar.hpp"

using RobotSensorsMsg = cg_interfaces::msg::RobotSensors;
using MoveCmdSrv = cg_interfaces::srv::MoveCmd;
using GetMapSrv = cg_interfaces::srv::GetMap;

class MazeSolverNode : public rclcpp::Node
{
public:
    MazeSolverNode();

private:
    rclcpp::Client<GetMapSrv>::SharedPtr map_client_;
    rclcpp::Client<MoveCmdSrv>::SharedPtr move_client_;
    rclcpp::Subscription<RobotSensorsMsg>::SharedPtr sensor_sub_;

    std::vector<std::vector<char>> map_;
    int robot_x_, robot_y_;
    int goal_x_, goal_y_;

    void requestMap();
    void sensorCallback(RobotSensorsMsg::SharedPtr msg);
    void sendMoveCommand(const std::string &direction, int target_x, int target_y);

    std::unique_ptr<AStar> pathfinder_;
};
