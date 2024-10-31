// test node. Publisher for aruco markers

#include <rclcpp/rclcpp.hpp>
#include <aruco_msgs/msg/marker_array.hpp>

class ArucoMarkerArrayPubTest : public rclcpp::Node
{
public:
    ArucoMarkerArrayPubTest() : Node("aruco_marker_array_pub_test")
    {
        aruco_pub_ = this->create_publisher<aruco_msgs::msg::MarkerArray>("/aruco_marker_poses", 1);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArucoMarkerArrayPubTest::timer_callback, this));
    }


private:

    void timer_callback()
    {
        aruco_msgs::msg::MarkerArray msg;
        aruco_msgs::msg::Marker marker;
        marker.id = 1;
        marker.header.frame_id = "test_frame";
        marker.pose.pose.position.x = 0.1;
        marker.pose.pose.position.y = 0.2;
        marker.pose.pose.position.z = 0.3;
        marker.pose.pose.orientation.x = 0.1;
        marker.pose.pose.orientation.y = 0.2;
        marker.pose.pose.orientation.z = 0.3;
        marker.pose.pose.orientation.w = 0.4;
        msg.markers.push_back(marker);
        aruco_pub_->publish(msg);
    }

    rclcpp::Publisher<aruco_msgs::msg::MarkerArray>::SharedPtr aruco_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoMarkerArrayPubTest>());
    rclcpp::shutdown();
    return 0;
}




