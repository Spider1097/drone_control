#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <drone_control/send_value.h>

int main(int argc, char **argv){
    ros::init(argc,argv,"talker");
    ros::NodeHandle n;

    ros::Publisher pub=n.advertise<drone_control::send_value>("/send_value",1000);
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        drone_control::send_value send_value;
        send_value.accept=1;
        pub.publish(send_value);
        ros::spinOnce();
        loop_rate.sleep();
    }
    

}
