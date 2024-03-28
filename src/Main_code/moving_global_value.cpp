

#include <declaration_all_function.hpp>

Myclass_info info;

struct drone_waypoint
{
    float lat, lon, alt, heading;
};

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
    nextWayPoint.lat = 47.397850;
    nextWayPoint.lon = home_value.global_pose_longitude;
    nextWayPoint.alt = home_value.global_pose_altitude + 2;
    nextWayPoint.heading = 90;
    waypointList.push_back(nextWayPoint);
    nextWayPoint.lat = home_value.global_pose_latitude + 2 * (9.0090090090090 / 1000000);
    nextWayPoint.lon = home_value.global_pose_longitude + 2 * (1.6129032258064517 / 100000);
    nextWayPoint.alt = home_value.global_pose_altitude + 2;
    nextWayPoint.heading = 90;
    waypointList.push_back(nextWayPoint);
    nextWayPoint.lat = home_value.global_pose_latitude - 2 * (9.0090090090090 / 1000000);
    nextWayPoint.lon = home_value.global_pose_longitude - 2 * (1.6129032258064517 / 100000);
    nextWayPoint.alt = home_value.global_pose_altitude + 2;
    nextWayPoint.heading = -180;
    waypointList.push_back(nextWayPoint);
    nextWayPoint.lat = home_value.global_pose_latitude;
    nextWayPoint.lon = home_value.global_pose_longitude;
    nextWayPoint.alt = home_value.global_pose_altitude + 2;
    nextWayPoint.heading = 0;
    waypointList.push_back(nextWayPoint);

    pose_global.pose.position.latitude = home_value.global_pose_latitude;
    pose_global.pose.position.longitude = home_value.global_pose_longitude;
    pose_global.pose.position.altitude = home_value.global_pose_altitude + 2;

    for (int i = 100; ros::ok() && i > 0; --i)
    {
        global_pos_pub.publish(pose_global);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int counter = 1;
    int open = 0;

    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
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

        if (open == 0)
        {
            global_pos_pub.publish(pose_global);
            if (check_waypoint_reached_global() == 1)
            {
                open = 1;
            }
        }

        if (check_waypoint_reached_global() == 1 && open == 1)
        {
            if (counter < waypointList.size())
            {
                ROS_INFO("lat:%f lon:%f alt:%f",waypointList[counter].lat,waypointList[counter].lon,waypointList[counter].alt);
                set_destination_global(waypointList[counter].lat, waypointList[counter].lon, waypointList[counter].alt, waypointList[counter].heading);
                counter++;
            }
            else
            {
                land();
                break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
