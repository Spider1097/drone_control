#include <declaration_all_function.hpp>

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
     
        publish_data(true);
        publish_date_to_topic(true, true, true, true);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}