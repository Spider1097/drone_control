#include <declaration_all_function.hpp>

Myclass_info info;

int array[3][3] = { {1,1,2},
                    {2,2,2},
                    {0,0,4} };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    Set_publishers_subscribers(nh);
    
    wait_for_connect();
    wait_for_connect_GPS_value();
    time_refresh_data(10);
    info.value_what_want_see(true,true,false);

    ros::Rate rate(20.0);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3;


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



if(circle(4,3,0.01)==1){
    break;
}



        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}