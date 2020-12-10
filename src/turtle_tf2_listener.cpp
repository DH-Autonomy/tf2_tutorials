#include <tf2_tutorials/turtle_tf2_listener.h>

using namespace std::chrono_literals;

TFListener::TFListener(std::string& arg1, std::string& arg2) :
Node("my_tf2_listener"), turtle_1(arg1), turtle_2(arg2)
{
	createClient();
}

int TFListener::spawnTurtle(){
	auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
	request->x = 4;
	request->y = 2;
	request->theta = 0;
	request->name = turtle_2;

	while (!client->wait_for_service(1s)){
		if (!rclcpp::ok()){
			RCLCPP_ERROR(this->get_logger(), "Some form of interruption during service, exiting.");
			return -1;
		}
		RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
	}
	auto result = client->async_send_request(request);
	return 0;
}

void TFListener::createClient(){
	// Creates a client whose only purpose is to spawn a turtle using turtlesim.
	client = this->create_client<turtlesim::srv::Spawn>("spawn");
	int success = spawnTurtle();
	if (success == 0){
		// next step
		RCLCPP_INFO(this->get_logger(), "Service available! Spawned %s. Starting publisher...",turtle_2);
		createROS2Publisher(turtle_2);
		RCLCPP_INFO(this->get_logger(), "Publisher created! Now running TFListener");
		runTFListener();
	}
}

void TFListener::createROS2Publisher(const std::string& turtle_name){
	publisher = this->create_publisher<geometry_msgs::msg::Twist>("/" + turtle_name + "/cmd_vel",10);
}

void TFListener::runTFListener(){
	tf2_ros::Buffer tfBuffer(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)); // read source code - it actually requires rclcpp::Clock.
	tf2_ros::TransformListener tfListener(tfBuffer);

	while(rclcpp::ok()){
		try {
			transformStamped = tfBuffer.lookupTransform(turtle_2, turtle_1, rclcpp::Time(0));
		}
		catch (tf2::TransformException &ex) {
			RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "%s", ex.what());
			std::chrono::seconds sec(1);
			rclcpp::sleep_for(sec);
			continue;
		}
		vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);
		vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x,2) + pow(transformStamped.transform.translation.y,2));
	  	publisher->publish(vel_msg);
		rate.sleep();
	}
}

int main(int argc, char** argv){
	std::string arg_1;
	std::string arg_2;
	if(argc != 3){
		std::cout<<"Wrong format. Only accepts two arguments: 'robot1' 'robot2'"<<std::endl;
		return -1;
	} else {
		arg_1 = argv[1];
		arg_2 = argv[2];
	}
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TFListener>(arg_1, arg_2));
	rclcpp::shutdown();	
	return 0;
}

