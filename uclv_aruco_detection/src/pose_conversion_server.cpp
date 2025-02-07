// This ROS2 node implements a service server that reads the estimated poses from aruco markers and use them to estiamte
// the frame poses. The input topic is /aruco_marker_poses. It publishes aruco_msgs/msg/MarkerArray.msg The estimated
// frame poses are published to the topic /pose_conversion. The transformations are read by the configuration file
// defined by the parameter "config_file"

#include <rclcpp/rclcpp.hpp>
#include <aruco_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "uclv_aruco_detection_interfaces/srv/pose_service.hpp"

class PoseConversionServer : public rclcpp::Node
{
public:
  PoseConversionServer() : Node("pose_conversion_server")
  {
    // declare parameters
    this->declare_parameter<std::string>("additional_transformation_topic", "");
    additional_transformation_topic_ = this->get_parameter("additional_transformation_topic").as_string();

    this->declare_parameter<std::string>("frame_id", "");
    frame_id_ = this->get_parameter("frame_id").as_string();

    std::cout << "additional_transformation_topic : " << additional_transformation_topic_ << std::endl;

    if (additional_transformation_topic_ != "")
    {
      std::cout << "Subscribing to additional transformation topic  " << additional_transformation_topic_ << std::endl;
      // subscriber
      additional_transformation_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          additional_transformation_topic_, rclcpp::SensorDataQoS(),
          std::bind(&PoseConversionServer::additional_transformation_callback, this, std::placeholders::_1));
    }

    // Create the service server
    server_ = this->create_service<uclv_aruco_detection_interfaces::srv::PoseService>(
        "pose_conversion_service",
        std::bind(&PoseConversionServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "PoseConversion service server ready!");
  }

private:
  void additional_transformation_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // store the additional transformation
    additional_transformation_ = *msg;
  }

  void reset_server()
  {
    server_initialized_ = false;
    frame_publishers_.clear();
    fixed_transformations_.clear();
    frame_names.clear();
    aruco_ids_.clear();
    aruco_sub_.reset();
  }
  void handle_service_request(const std::shared_ptr<uclv_aruco_detection_interfaces::srv::PoseService::Request> request,
                              std::shared_ptr<uclv_aruco_detection_interfaces::srv::PoseService::Response> response)
  {
    if (server_initialized_)
    {
      RCLCPP_WARN(this->get_logger(), "Server already initialized : the server will be reset");
      reset_server();
    }

    RCLCPP_INFO(this->get_logger(), "Received request");

    // load arucoid_T_frameid transformations
    this->obj_name_ = request->object_name.data.c_str();
    load_fixed_transformations(request->yaml_file_path.data.c_str());

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);

    // instantiate the pusblishers for each frame
    for (auto frame_name : frame_names)
    {
      frame_publishers_.push_back(
          this->create_publisher<geometry_msgs::msg::PoseStamped>(this->obj_name_ + "/" + frame_name + "/pose", rclcpp::SensorDataQoS()));
    }

    // subscriber
    aruco_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
        "/aruco_marker_poses", qos, std::bind(&PoseConversionServer::aruco_callback, this, std::placeholders::_1));

    // return the response
    server_initialized_ = true;
    response->success = server_initialized_;
  }

  void load_fixed_transformations(const std::string& config_file)
  {
    RCLCPP_INFO(this->get_logger(), "Loading fixed transformations from %s", config_file.c_str());
    YAML::Node config = YAML::LoadFile(config_file);
    for (const auto& transformation : config[this->obj_name_]["aruco_transforms"])
    {
      int id = transformation["aruco_id"].as<int>();
      std::vector<double> translation = transformation["frame_transform"]["translation"].as<std::vector<double>>();
      std::vector<double> quaternion = transformation["frame_transform"]["quaternion"].as<std::vector<double>>();

      tf2::Transform tf;
      tf.setOrigin(tf2::Vector3(translation[0], translation[1], translation[2]));
      tf.setRotation(tf2::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]));

      fixed_transformations_.push_back(tf);
      frame_names.push_back(transformation["name"].as<std::string>());
      aruco_ids_.push_back(id);

      RCLCPP_INFO(this->get_logger(), "Loaded transformation for aruco id %d", id);
      RCLCPP_INFO(this->get_logger(), "Translation: %f %f %f", translation[0], translation[1], translation[2]);
      RCLCPP_INFO(this->get_logger(), "Quaternion: %f %f %f %f", quaternion[0], quaternion[1], quaternion[2],
                  quaternion[3]);
      RCLCPP_INFO(this->get_logger(), "Frame: %s", transformation["name"].as<std::string>().c_str());
    }
  }
  void aruco_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg)
  {
    for (auto marker : msg->markers)
    {
      // Find the transformation for the current ArUco ID
      auto it = std::find(aruco_ids_.begin(), aruco_ids_.end(), marker.id);
      if (it == aruco_ids_.end())
      {
        // RCLCPP_WARN(this->get_logger(), "ArUco ID %d not found in the config file", marker.id);
        continue;
      }
      int idx = std::distance(aruco_ids_.begin(), it);
      tf2::Transform aruco_T_frame = fixed_transformations_[idx];

      // Convert marker pose to tf2::Transform
      tf2::Transform referenceframe_T_aruco;

      tf2::fromMsg(marker.pose.pose, referenceframe_T_aruco);

      // Apply the fixed transformation
      tf2::Transform referenceframe_T_frame = referenceframe_T_aruco * aruco_T_frame;

      // Convert back to geometry_msgs::Pose
      geometry_msgs::msg::PoseStamped frame_pose;
      frame_pose.header = marker.header;

      // apply additional transformation if available
      if (additional_transformation_topic_ != "")
      {
        tf2::Transform additional_transformation;
        tf2::fromMsg(additional_transformation_.pose, additional_transformation);
        referenceframe_T_frame = additional_transformation * referenceframe_T_frame;
        frame_pose.header.frame_id = additional_transformation_.header.frame_id;
      }

      if (frame_id_ != "")
      {
        frame_pose.header.frame_id = frame_id_;
      }

      tf2::toMsg(referenceframe_T_frame, frame_pose.pose);

      frame_publishers_[idx]->publish(frame_pose);
    }
  }

  rclcpp::Service<uclv_aruco_detection_interfaces::srv::PoseService>::SharedPtr server_;

  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr aruco_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr additional_transformation_sub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> frame_publishers_;
  std::string obj_name_;
  std::vector<tf2::Transform> fixed_transformations_;  // vector of fixed transformations from aruco_id to frame_id
  std::vector<std::string> frame_names;                // vector of frame names
  std::vector<int> aruco_ids_;                         // vector of aruco ids

  bool server_initialized_ = false;
  std::string additional_transformation_topic_;  // topic to read additional transformation to apply to the poses
  geometry_msgs::msg::PoseStamped additional_transformation_;

  std::string frame_id_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseConversionServer>());
  rclcpp::shutdown();
  return 0;
}
