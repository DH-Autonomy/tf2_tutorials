#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/spawn.hpp>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node
{
	rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
	std::string turtle_1;
	std::string turtle_2;
	geometry_msgs::msg::Twist vel_msg;
	geometry_msgs::msg::TransformStamped transformStamped;

	public:
		TFListener(std::string& turtle_1, std::string& turtle_2);
		void createClient();
		void createROS2Publisher(const std::string& turtle_name);
		void runTFListener();
		int spawnTurtle();
};

rclcpp::Rate rate(10.0);