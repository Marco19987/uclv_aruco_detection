// This ROS2 node reads the estimated poses from aruco markers and use them to estiamte the frame poses.
// The input topic is /aruco_marker_poses. It publishes aruco_msgs/msg/MarkerArray.msg
// The estimated frame poses are published to the topic /pose_conversion.
// The transformations are read by the configuration file defined by the parameter "config_file"

#include <rclcpp/rclcpp.hpp>
#include <aruco_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>


class PoseConversion : public rclcpp::Node
{
public:
    PoseConversion() : Node("pose_conversion")
    {
        // define parameters
        this->declare_parameter<std::string>("object_name", "object");
        this->get_parameter("object_name", this->obj_name_);

        this->declare_parameter<std::string>("config_file", "config/config.yaml");
        std::string config_file;
        this->get_parameter("config_file", config_file);

        // load arucoid_T_frameid trandormations
        load_fixed_transformations(config_file);

        // define the subscriber and publisher
        aruco_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
            "/aruco_marker_poses", 1, std::bind(&PoseConversion::aruco_callback, this, std::placeholders::_1));

        // instantiate the pusblishers for each frame
        for (auto frame_name : frame_names)
        {
            frame_publishers_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(this->obj_name_ + "/" + frame_name + "/pose", 1));
        }
    }

private:

    void load_fixed_transformations(const std::string &config_file)
    {
        RCLCPP_INFO(this->get_logger(), "Loading fixed transformations from %s", config_file.c_str());
        YAML::Node config = YAML::LoadFile(config_file);
        for (const auto &transformation : config[this->obj_name_])
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
            RCLCPP_INFO(this->get_logger(), "Quaternion: %f %f %f %f", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
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
                RCLCPP_WARN(this->get_logger(), "ArUco ID %d not found in the config file", marker.id);
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
            tf2::toMsg(referenceframe_T_frame, frame_pose.pose);

            frame_publishers_[idx]->publish(frame_pose);
        }
    }

    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr aruco_sub_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> frame_publishers_;
    std::string obj_name_;
    std::vector<tf2::Transform> fixed_transformations_;     // vector of fixed transformations from aruco_id to frame_id
    std::vector<std::string> frame_names;                  // vector of frame names
    std::vector<int> aruco_ids_;                          // vector of aruco ids    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseConversion>());
    rclcpp::shutdown();
    return 0;
}
