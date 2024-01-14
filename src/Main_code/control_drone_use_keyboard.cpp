
#include <declaration_all_function.hpp>
#include <mavros_msgs/RCIn.h>
#include <thread>
#include <chrono>
#include <conio.h>

Myclass_info info;

float speed_x=0;
float speed_y=0;
float speed_z=0;
float angl_x=0;
float angl_y=0;
float angl_z=0;
int closee=0;

char key;

void error_close(){
  while(ros::ok()){
 key = getch();

 if (key == 'a') {
        speed_x=speed_x+0.1;
 }

 if (key == 'd') {
        speed_y=speed_y+0.1;
 }

 if (key == 'w') {
        speed_z=speed_z+0.1;
 }

 if (key == 's') {
        speed_z=speed_z-0.1;
 }

 if (key == 'q') {
        angl_x=angl_x+0.1;
 }

 if (key == 'e') {
        angl_y=angl_y+0.1;
 }
 if (key == 'p') {
        closee=1;
 }

  }
}

float speed_xw=0;
float speed_yw=0;
float speed_zw=0;

void speed(const  geometry_msgs::TwistStamped::ConstPtr& msg)
{
    speed_xw=msg->twist.linear.x;
    speed_yw=msg->twist.linear.y;
    speed_zw=msg->twist.linear.z;

//ROS_INFO("x:%lf y:%lf z:%lf",speed_x,speed_y,speed_z);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    Set_publishers_subscribers(nh);

ros::Subscriber state_sub = nh.subscribe<geometry_msgs::TwistStamped> ("/mavros/local_position/velocity_body", 100, speed);
ros::Publisher pub_speed = nh.advertise < geometry_msgs::Twist> ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    std::thread th(error_close);
    th.detach();
    
    wait_for_connect();
    wait_for_connect_GPS_value();
    time_refresh_data(10);
    info.value_what_want_see(true,true,false);

    ros::Rate rate(20.0);

 std::string pat="/home/anton/Desktop/aa";

    read_file(pat,5,false);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3;

    geometry_msgs::Twist spee;
    spee.linear.x=0;
    spee.linear.y=0;
    spee.linear.z=0;

    pub_speed.publish(spee);

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    set_mode_client.call(offb_set_mode);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode == "OFFBOARD" ){
            
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                        
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

if(closee==1){
    set_destination_local(0, 0, 1, 0);
    if( check_waypoint_reached_local()==1){
        land();
        break; 
    }
    
}

spee.linear.x=speed_x;
spee.linear.y=speed_y;
spee.linear.z=speed_z;
spee.angular.x=angl_x;
spee.angular.y=angl_y;

    pub_speed.publish(spee);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}