#include <tf2_tutorials/turtle_tf2_broadcaster.h>

// Constructor
TFBroadcaster::TFBroadcaster(std::string input) : 
Node("my_tf2_broadcaster"), robot_name(input)
{
	RCLCPP_INFO(this->get_logger(), "Initialized TFBroadcaster with %s", robot_name);
	createSubscription(robot_name);
	br = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void TFBroadcaster::createSubscription(const std::string& input){
	// Constructor that initializes a subscription to /pose with the callback poseCallback
	std::function<void(const turtlesim::msg::Pose::SharedPtr)> callback = std::bind(&TFBroadcaster::poseCallback,this,std::placeholders::_1);
	subscription_ = this->create_subscription<turtlesim::msg::Pose>(input + "/pose",10,callback);
}

void TFBroadcaster::poseCallback(const turtlesim::msg::Pose::SharedPtr msg){
	geometry_msgs::msg::TransformStamped transformStamped;
	transformStamped.header.stamp = this->now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = robot_name;
	transformStamped.transform.translation.x = msg->x;
	transformStamped.transform.translation.y = msg->y;
	transformStamped.transform.translation.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();
	RCLCPP_INFO(this->get_logger(),"translation x = %f, translation y = %f", msg->x, msg->y);
			
	br->sendTransform(transformStamped);
}

int main(int argc, char** argv){
	if (argc != 2) {
		std::cout<<"Missing: robot name"<<std::endl; 
		return -1;
	}
	else {
		// Naively accept robot name for now.
		robot_name = argv[1];
	}
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TFBroadcaster>(robot_name));
	rclcpp::shutdown();	
	return 0;
}
