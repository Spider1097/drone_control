#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

void OdomCallback(const  sensor_msgs::NavSatFix::ConstPtr& msg)
{
        double x= msg->latitude;
        double y= msg->longitude;
        double z= msg->altitude;

		ROS_INFO("latitude: %f, longitude: %f, altitude: %f", x, y, z);	

		sleep(1);
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener_node");
	ros::NodeHandle nh;
	ros::Subscriber str_pub = nh.subscribe("mavros/global_position/global", 2, OdomCallback);

	ros::spin();

	return 0;
}
