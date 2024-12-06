#include "rclcpp/rclcpp.hpp"
#include "uclv_aruco_detection_interfaces/srv/pose_service.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

class PoseServiceClient : public rclcpp::Node
{
public:
  PoseServiceClient() : Node("pose_service_client")
  {
    client_ = this->create_client<uclv_aruco_detection_interfaces::srv::PoseService>("pose_conversion_service");
    timer_ = this->create_wall_timer(500ms, std::bind(&PoseServiceClient::send_request, this));
  }

private:
  void send_request()
  {
    auto request = std::make_shared<uclv_aruco_detection_interfaces::srv::PoseService::Request>();
    // Fill in the request data here

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("dual_arm_control");
    request->yaml_file_path.data = package_share_directory + "/config/config.yaml";
    request->object_name.data = "resin_block_1";

    while (!client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto result = client_->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Service call succeeded");
      // Process the result here
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service ekf");
    }

    // close the node
    rclcpp::shutdown();
  }

  rclcpp::Client<uclv_aruco_detection_interfaces::srv::PoseService>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseServiceClient>());
  rclcpp::shutdown();
  return 0;
}