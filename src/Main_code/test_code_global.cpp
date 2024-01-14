
#include <declaration_all_function.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test2");
    ros::NodeHandle nh;

    Set_publishers_subscribers(nh);

    wait_for_connect();
    wait_for_connect_GPS_value();

    ros::Rate rate(20.0);

    pose_global.pose.position.latitude = home_value.global_pose_latitude;
    pose_global.pose.position.longitude = home_value.global_pose_longitude;
    pose_global.pose.position.altitude = home_value.global_pose_altitude+2;

    for(int i = 100; ros::ok() && i > 0; --i){
        global_pos_pub.publish(pose_global);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

 while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

          global_pos_pub.publish(pose_global);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}