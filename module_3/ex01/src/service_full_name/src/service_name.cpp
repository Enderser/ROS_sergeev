#include "rclcpp/rclcpp.hpp"
#include "service_full_name/srv/summ_full_name.hpp"
#include <memory>

using SummFullName = service_full_name::srv::SummFullName;

void handle_summ_full_name(
  const std::shared_ptr<SummFullName::Request> request,
  std::shared_ptr<SummFullName::Response> response)
{
  response->full_name = request->surname + " " + request->name + " " + request->second_name;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Incoming request: '%s' '%s' '%s'",
              request->surname.c_str(),
              request->name.c_str(),
              request->second_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Sending back response: '%s'",
              response->full_name.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("service_name");
  auto service = node->create_service<SummFullName>("SummFullName", &handle_summ_full_name);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service 'SummFullName' ready.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
