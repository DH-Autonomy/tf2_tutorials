#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("my_tf2_broadcaster");
	// ros::init(argc, argv, "my_tf2_broadcaster");
	// ros::NodeHandle node;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tfb;
	tfb = std::make_shared<tf2_ros::TransformBroadcaster>(node);

	//tf2_ros::TransformBroadcaster tfb;
	geometry_msgs::msg::TransformStamped transformStamped;

	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "carrot1";
	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = 2.0;
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0,0,0);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	// ros::Rate rate(10.0);
	rclcpp::Rate rate(10.0);
	
	//while (node.ok()){
	while(rclcpp::ok()){
		transformStamped.header.stamp = node->now();

		transformStamped.transform.translation.x = 2.0*sin(node->now().seconds());
		transformStamped.transform.translation.y = 2.0*cos(node->now().seconds());

		tfb->sendTransform(transformStamped);
		rate.sleep();
		printf("sending\n");
	}
}
