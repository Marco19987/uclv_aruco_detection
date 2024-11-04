// This ROS2 node reads the poses of the same frame measured by different ArUco markers and publishes the average pose.


#include <rclcpp/rclcpp.hpp>
#include <aruco_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>


class PoseElaboration : public rclcpp::Node
{
public:
    PoseElaboration() : Node("pose_elaboration_node")
    {
        // define parameters
        this->declare_parameter<std::string>("object_name", "object");
        this->get_parameter("object_name", this->obj_name_);

        this->declare_parameter<std::string>("config_file", "config/config.yaml");
        std::string config_file;
        this->get_parameter("config_file", config_file);

        // load arucoid_T_frameid transformations
        read_config_file(config_file);

        // initialize the elaborated pose
        elaborated_pose.pose.position.x = 0;
        elaborated_pose.pose.position.y = 0;
        elaborated_pose.pose.position.z = 0;
        elaborated_pose.pose.orientation.x = 0;
        elaborated_pose.pose.orientation.y = 0;
        elaborated_pose.pose.orientation.z = 0;
        elaborated_pose.pose.orientation.w = 1;

        // initialize the poses_read vector
        for (std::vector<bool>::size_type i = 0; i < frame_names.size(); i++)
        {
            poses_read.push_back(false);
            poses_read_msg.push_back(geometry_msgs::msg::PoseStamped());

        }
  

        // define the timer
        float node_frequency_ = 60.0;
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / node_frequency_)),
                                         std::bind(&PoseElaboration::compute_elaborated_pose, this));

        // define the subscriber and publisher
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(this->obj_name_ +  "/elaborated_pose",1);

        // instantiate the subscribers for each frame
        int index = 0;
        for (auto frame_name : frame_names)
        {
            pose_subscribers_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
                this->obj_name_ + "/" + frame_name + "/pose", 1, [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->pose_callback(msg, index);
                }));
            index++;
        }
    }

private:

    void read_config_file(const std::string &config_file)
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
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const int &index)
    {
        RCLCPP_INFO(this->get_logger(), "Received pose from %s", frame_names[index].c_str());
        poses_read_msg[index] = *msg;
        poses_read[index] = true;
    }

    void compute_elaborated_pose()
    {
        geometry_msgs::msg::PoseStamped elaborated_pose_tmp;
         elaborated_pose_tmp.pose.orientation.w = 0;
        // average the poses corresponding true values in the vector poses_read
        std::vector<bool>::size_type n = 0;
        for (std::vector<bool>::size_type i = 0; i < poses_read.size(); i++)
        {
            if (poses_read[i])
            {
                RCLCPP_INFO(this->get_logger(), "Pose from %s has been read", frame_names[i].c_str());
                n++;
                elaborated_pose_tmp.pose.position.x += poses_read_msg[i].pose.position.x;
                elaborated_pose_tmp.pose.position.y += poses_read_msg[i].pose.position.y;
                elaborated_pose_tmp.pose.position.z += poses_read_msg[i].pose.position.z;
                elaborated_pose_tmp.pose.orientation.x += poses_read_msg[i].pose.orientation.x;
                elaborated_pose_tmp.pose.orientation.y += poses_read_msg[i].pose.orientation.y;
                elaborated_pose_tmp.pose.orientation.z += poses_read_msg[i].pose.orientation.z;
                elaborated_pose_tmp.pose.orientation.w += poses_read_msg[i].pose.orientation.w;  
                
            }
        }
        if (n > 0)
        {
            RCLCPP_INFO(this->get_logger(), "n = %d", n);   
            elaborated_pose_tmp.pose.position.x /= n;
            elaborated_pose_tmp.pose.position.y /= n;
            elaborated_pose_tmp.pose.position.z /= n;
            elaborated_pose_tmp.pose.orientation.x /= n;
            elaborated_pose_tmp.pose.orientation.y /= n;
            elaborated_pose_tmp.pose.orientation.z /= n;
            elaborated_pose_tmp.pose.orientation.w /= n;
            
            // normalize the quaternion
            double norm = sqrt(elaborated_pose_tmp.pose.orientation.x * elaborated_pose_tmp.pose.orientation.x +
                               elaborated_pose_tmp.pose.orientation.y * elaborated_pose_tmp.pose.orientation.y +
                               elaborated_pose_tmp.pose.orientation.z * elaborated_pose_tmp.pose.orientation.z +
                               elaborated_pose_tmp.pose.orientation.w * elaborated_pose_tmp.pose.orientation.w);
            elaborated_pose_tmp.pose.orientation.x /= norm;
            elaborated_pose_tmp.pose.orientation.y /= norm;
            elaborated_pose_tmp.pose.orientation.z /= norm;
            elaborated_pose_tmp.pose.orientation.w /= norm;
            
            
            elaborated_pose_tmp.header.stamp = rclcpp::Clock().now();
            // find the first element of poses_read_msg that has been read
            for (std::vector<bool>::size_type i = 0; i < poses_read.size(); i++)
            {
                if (poses_read[i])
                {
                    elaborated_pose_tmp.header.frame_id = poses_read_msg[i].header.frame_id;
                    break;
                }
            }
            elaborated_pose = elaborated_pose_tmp;

        }


        pose_publisher_->publish(elaborated_pose);

        // reset the poses_read vector
        for (std::vector<bool>::size_type i = 0; i < poses_read.size(); i++)
        {
            poses_read[i] = false;
        }

        
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subscribers_;
    std::vector<geometry_msgs::msg::PoseStamped> poses_read_msg;          // vector of poses read from each aruco
    std::vector<bool> poses_read;          // vector of bool indicating if the pose has been read
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::string obj_name_;
    geometry_msgs::msg::PoseStamped elaborated_pose;  // result of the pose elaboration

    std::vector<tf2::Transform> fixed_transformations_;     // vector of fixed transformations from aruco_id to frame_id
    std::vector<std::string> frame_names;                  // vector of frame names
    std::vector<int> aruco_ids_;                          // vector of aruco ids    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseElaboration>());
    rclcpp::shutdown();
    return 0;
}
