#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>

class FrameTF2Broadcaster : public rclcpp::Node
{
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb;
	geometry_msgs::msg::TransformStamped transformStamped;
	tf2::Quaternion q;
	rclcpp::Rate rate(10.0);
    void runFrameTF2Broadcaster();
    
    public:
        FrameTF2Broadcaster(std::string input);
}
