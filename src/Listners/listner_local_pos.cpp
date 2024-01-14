#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

void OdomCallback(const  nav_msgs::Odometry::ConstPtr& msg)
{
	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;
	double z = msg->pose.pose.position.z;

		ROS_INFO("x: %0.2f, y: %0.2f, z: %0.2f", x, y, z);	

	sleep(1);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener_node");
	ros::NodeHandle nh;
	ros::Subscriber str_pub = nh.subscribe("/mavros/global_position/local", 2, OdomCallback);

	ros::spin();

	return 0;
}