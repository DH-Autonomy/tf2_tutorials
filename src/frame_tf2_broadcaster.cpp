#include <tf2_tutorials/frame_tf2_broadcaster.h>

FrameTF2Broadcaster::FrameTF2Broadcaster() : 
Node("frame_tf2_broadcaster"), rate(10.0)
{
	RCLCPP_INFO(this->get_logger(), "Initialized frame_tf2_broadcaster");
	tfb = std::make_shared<tf2_ros::TransformBroadcaster>(this);
	
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "carrot1";
	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = 2.0;
	transformStamped.transform.translation.z = 0.0;
	
	q.setRPY(0,0,0);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	runFrameTF2Broadcaster();
}

void FrameTF2Broadcaster::runFrameTF2Broadcaster(){
	while(rclcpp::ok()){
		transformStamped.header.stamp = this->now();

		transformStamped.transform.translation.x = 2.0*sin(this->now().seconds());
		transformStamped.transform.translation.y = 2.0*cos(this->now().seconds());

		tfb->sendTransform(transformStamped);
		rate.sleep();
		printf("sending\n");
	}
}


int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FrameTF2Broadcaster>());	
	rclcpp::shutdown();	
}
