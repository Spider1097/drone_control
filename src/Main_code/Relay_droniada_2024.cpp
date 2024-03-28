#include <declaration_all_function.hpp>

Myclass_info info;

struct drone_waypoint
{
    float lat, lon, alt, heading;
};

struct drone_value
{
    double latitude = 0;
    double longitude = 0;
    double altitude = 0;

    int count_waypoints = 0;
    int counter = 0;
    int open = 0;
    int choice = 0;
    bool close = false;
};
drone_value drone_value;

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

    std::vector<drone_waypoint> waypointList;
    drone_waypoint nextWayPoint;

    ROS_INFO("How many waypoints?: ");

    std::cin >> drone_value.count_waypoints;

    for (int i = 0; i < drone_value.count_waypoints; i++)
    {

        ROS_INFO("Set latitude: ");
        std::cin >> drone_value.latitude;
        ROS_INFO("Set longitude: ");
        std::cin >> drone_value.longitude;
        ROS_INFO("Set altitude in meters: ");
        std::cin >> drone_value.altitude;
        ROS_INFO("-------------------------------------------");

        nextWayPoint.lat = drone_value.latitude;
        nextWayPoint.lon = drone_value.longitude;
        nextWayPoint.alt = home_value.global_pose_altitude + drone_value.altitude;
        nextWayPoint.heading = 0;
        waypointList.push_back(nextWayPoint);
    }

    nextWayPoint.lat = home_value.global_pose_latitude;
    nextWayPoint.lon = home_value.global_pose_longitude;
    nextWayPoint.alt = home_value.global_pose_altitude + 3;
    nextWayPoint.heading = 0;
    waypointList.push_back(nextWayPoint);

    // for(int i=0; i<waypointList.size() i++){
    //     ROS_INFO("latitude: %lf, longitude: %lf, altitude: %lf", waypointList[i].lat, waypointList[i].lon, waypointList[i].alt);
    // }

    pose_global.pose.position.latitude = home_value.global_pose_latitude;
    pose_global.pose.position.longitude = home_value.global_pose_longitude;
    pose_global.pose.position.altitude = home_value.global_pose_altitude + 3;

    for (int i = 100; ros::ok() && i > 0; --i)
    {
        global_pos_pub.publish(pose_global);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    set_mode_client.call(offb_set_mode);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (current_state.mode == "OFFBOARD")
        {

            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {

                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        publish_data(true);
        publish_date_to_topic(true, true, true, true);

        if (drone_value.close == true)
        {
            break;
        }

        if (drone_value.open == 0)
        {
            global_pos_pub.publish(pose_global);
            if (check_waypoint_reached_global() == 1)
            {
                drone_value.open = 1;
            }
        }

        if (drone_value.open == 1)
        {
            switch (drone_value.choice)
            {
            case 0:
                // ROS_INFO("latitude: %lf, longitude: %lf, altitude: %lf", waypointList[drone_value.counter].lat, waypointList[drone_value.counter].lon, waypointList[drone_value.counter].alt);
                set_destination_global(waypointList[drone_value.counter].lat, waypointList[drone_value.counter].lon, waypointList[drone_value.counter].alt, waypointList[drone_value.counter].heading);
                drone_value.choice = 1;
                break;
            case 1:
                if (check_waypoint_reached_global() == 1)
                {
                    if (drone_value.counter + 1 < waypointList.size())
                    {
                        drone_value.choice = 2;
                    }
                    else
                    {
                        drone_value.choice = 3;
                    }
                }
                break;
            case 2:
                // ROS_INFO("latitude: %lf, longitude: %lf, altitude: %lf", waypointList[drone_value.counter].lat, waypointList[drone_value.counter].lon, waypointList[drone_value.counter].alt);
                set_destination_global(waypointList[drone_value.counter].lat, waypointList[drone_value.counter].lon, waypointList[drone_value.counter].alt, waypointList[drone_value.counter].heading);
                send_value.accept = 1;
                publish_date_to_topic(true, true, true, true);
                if (timer2(10) == 1)
                {
                    drone_value.counter++;
                    drone_value.choice = 0;
                    send_value.accept = 0;
                    publish_date_to_topic(true, true, true, true);
                }
                break;
            case 3:
                if (go_down_drone_curent_pose(0.4) == 1)
                {
                    drone_value.choice = 4;
                }
                break;
            case 4:
                land();
                drone_value.close = true;
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}