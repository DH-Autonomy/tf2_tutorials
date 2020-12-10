#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <turtlesim/msg/pose.hpp>

class TFBroadcaster : public rclcpp::Node
{
    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);

    // Initialize the subscription to turtle's pose.
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
	
    // Initialize TransformBroadcaster 
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;

    public:
        std::string robot_name;
        TFBroadcaster(std::string input);
        void createSubscription(const std::string& input);
};

std::string robot_name;