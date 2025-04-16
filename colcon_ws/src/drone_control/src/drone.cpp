#include "drone.hpp"

void DRONE::setMode(const std::string &mode)
{
    // Create a request object
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;      // Set to 0 to only use custom_mode
    request->custom_mode = mode; // Mode to set, e.g., "ALT_HOLD"

    // Call the service asynchronously
    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(this->get_logger(), "Set mode successful.");
    }else{
        RCLCPP_INFO(this->get_logger(), "Set mode failed.");
    }

    // Optional: Add a timeout in case the service call takes too long
    try {
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Service call to /mavros/set_mode timed out");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while waiting for service response: %s", e.what());
    }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto drone_node = std::make_shared<DRONE>();
  drone_node->main_loop(); // This will spin the node.
  rclcpp::shutdown();
  return 0;
}
