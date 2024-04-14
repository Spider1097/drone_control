#include <declaration_all_function.hpp>

float position_latitude[4] = {0};
float position_longitude[4] = {0};
float position_hight[4] = {0};

int counter_curent=0;

Myclass_info info;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    Set_publishers_subscribers(nh);

    wait_for_connect();
    wait_for_connect_GPS_value();
    time_refresh_data(10);
    info.value_what_want_see(true, true, false);

    ros::Rate rate(20.0);

    while (ros::ok())
    {

        set_destination_global(position_latitude[counter_curent], position_longitude[counter_curent], home_value.global_pose_altitude+position_hight[counter_curent], 0);
     
        if(check_waypoint_reached_global_latitude_longitude()==1){

            while(timer3(3)==0){
            send_value.accept=counter_curent;
            data_send_value_pub.publish(send_value);
            }
            
            counter_curent++;
        }

        publish_data(true);
        publish_date_to_topic(false, false, false, false);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}