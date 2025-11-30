//inclusão da biblioteca de client do ros
#include "rclcpp/rclcpp.hpp"
//incluclusão da biblioteca de servidor
#include "cg_interfaces/srv/get_map.hpp"

//ouras dependencias encessárias
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;


int main(int argc, char **argv)
{
    // inicializa o sistema ros
  rclcpp::init(argc, argv);

  // criação de um nó ROS chamado "add_two_ints_client" e retorna um ponteiro compartilhado armazenado na variavel node
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
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
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
    //log de sucesso mostrando a largura retornada pelo servidor
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "First item: %s", result.get()->occupancy_grid_flattened[0].c_str());
  } else {
    //log em casod e erro
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_map");
  }

  rclcpp::shutdown();
  return 0;
}