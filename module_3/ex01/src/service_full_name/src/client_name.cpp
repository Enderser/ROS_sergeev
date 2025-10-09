#include "rclcpp/rclcpp.hpp"
#include "service_full_name/srv/summ_full_name.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using SummFullName = service_full_name::srv::SummFullName;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (argc != 4) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: ros2 run service_full_name client_name <surname> <name> <second_name>");
    return 1;
  }

  auto node = rclcpp::Node::make_shared("client_name");
  auto client = node->create_client<SummFullName>("SummFullName");

  auto request = std::make_shared<SummFullName::Request>();
  request->surname = argv[1];
  request->name = argv[2];
  request->second_name = argv[3];

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Full name: %s", response->full_name.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service SummFullName");
  }

  rclcpp::shutdown();
  return 0;
}
