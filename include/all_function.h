//////////////////////////////////section for library//////////////////////////////////

// cpp functions
#include <ros/ros.h>
#include <cmath>
#include <math.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
// cpp functions

/*time*/
#include <ctime>
#include <iomanip>
/*time*/

/*file txt*/
#include <iostream>
#include <cmath>
#include <string>
#include <fstream>
/*file txt*/

/*read buttons PC*/
#include <conio.h>
/*read buttons PC*/

/*wielowatkowosc*/
#include <thread>
#include <chrono>
/*wielowatkowosc*/

/*ros messange from pixhawk*/
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include "GeographicLib/Geoid.hpp"
#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/RCIn.h>
/*ros messange from pixhawk*/

/*custom msm*/
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
/*custom msm*/

//////////////////////////////////section for library//////////////////////////////////

/*custtom messenge where you sending date*/
#include <drone_control/drone_data.h>
#include <drone_control/mqtt_data.h>
#include <drone_control/send_value.h>
drone_control::drone_data drone_data;
drone_control::mqtt_data mqtt_data;
drone_control::send_value send_value;
/*custtom messenge where you sending date*/

/*all_subcriber, servicec and clinet what are using*/
ros::Publisher data_drone_pub;
ros::Publisher data_mqtt_pub;
ros::Publisher data_send_value_pub;

ros::Publisher global_pos_pub;
ros::Publisher local_pos_pub;
ros::Publisher speed_drone_pub;

ros::Subscriber state_sub;
ros::Subscriber global_pos_sub;
ros::Subscriber local_pos_sub;
ros::Subscriber velocity_value_sub;

ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;
/*all_subcriber, servicec and clinet what are using*/

/*all msgs are using*/
mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
geographic_msgs::GeoPoseStamped pose_global;
mavros_msgs::CommandTOL srv_land;
/*all msgs are using*/

//////////////////////////////////section for value//////////////////////////////////
bool global_position_received = false;

/*close tages*/
struct close_tag
{
    int close_pose_local = 0;
    int close_pose_global = 0;
    bool close_back_home_position_local = false;
    bool close_back_home_position_global = false;
    bool back_choice_position_local = false;
    bool back_choice_position_global = false;
    bool go_down_global = false;
    bool go_up_global = false;
};
close_tag close_tag;
/*close tages*/

/*convert value*/
struct convert_value
{
    float radius_earth = 0;
    double dLat = 0;
    double dLon = 0;
    double count = 0;
    double angle = 0;
    float result = 0;
};
convert_value convert_value;
/*convert value*/

/*Home value*/
struct home_value
{
    float home_pose_x = 0;
    float home_pose_y = 0;

    double global_pose_latitude = 0;
    double global_pose_longitude = 0;
    double global_pose_altitude = 0;

    double save_current_high = 0;
};
home_value home_value;
/*Home value*/

/*GPS value and else*/
struct call_back
{
    double latitude = 0;
    double longitude = 0;
    double altitude = 0;

    float position_x = 0;
    float position_y = 0;
    float position_z = 0;

    float linear_velocity_x = 0;
    float linear_velocity_y = 0;
    float linear_velocity_z = 0;
    float angular_velocity_x = 0;
    float angular_velocity_y = 0;
    float angular_velocity_z = 0;
};
call_back call_back;
/*GPS value and else*/

/*info from call_backs*/
struct open_info_call_back
{
    bool global_value = false;
    bool local_value = false;
    bool value_velocity = false;
};
open_info_call_back open_info;
/*info from call_backs*/

/*Time value refresh*/
struct TiMe
{
    int second_refresh = 0;
    int second_last = 0;
    int refresh_time = 0;
    int period_time = 0;

    int second_refresh1 = 0;
    int second_last1 = 0;
    int refresh_time1 = 0;
    int period_time1 = 0;

    int second_refresh2 = 0;
    int second_last2 = 0;
    int refresh_time2 = 0;
    int period_time2 = 0;

    int second_refresh3 = 0;
    int second_last3 = 0;
    int refresh_time3 = 0;
    int period_time3 = 0;
};
TiMe Time;
/*Time value refresh*/

/*file value */
struct txt_file
{
    std::string path = " ";
    float x_position_file[100] = {0};
    float y_position_file[100] = {0};
    float z_position_file[100] = {0};
};
txt_file filee;
/*file value */

/*tolerance value */
struct tolerance
{
    float pos_local_tolerance = 0.1;
    float heading_local_tolerance = 0.1;
    float pos_global_tolerance = 0.000001;
    float heading_global_tolerance = 0.01;
};
tolerance tolerance;
/*tolerance value */

/*distance value */
struct distance
{
    float current_pose_local_distans = 0;
    float current_pose_local_hight = 0;
    float current_pose_global_distans = 0;
    float current_pose_global_hight = 0;
    float deltaX = 0;
    float deltaY = 0;
    float deltaZ = 0;
    float dMag = 0;
    float convert_distances = 0;
};
distance distance;
/*distance value */

/*mission value*/
struct mission_data
{
    float teta = 0;
    float counter = 0;
    float circle_x = 0;
    float circle_y = 0;
};
mission_data mission_data;
/*mission value*/

//////////////////////////////////section for value//////////////////////////////////

/*geographic value needs if position of z work on local value*/
GeographicLib::Geoid _egm96("egm96-5");
double calc_geoid_height(double lat, double lon)
{
    return _egm96(lat, lon);
}
double amsl_to_ellipsoid_height(double lat, double lon, double amsl)
{
    return amsl + GeographicLib::Geoid::GEOIDTOELLIPSOID * calc_geoid_height(lat, lon);
}
double ellipsoid_height_to_amsl(double lat, double lon, double ellipsoid_height)
{
    return ellipsoid_height + GeographicLib::Geoid::ELLIPSOIDTOGEOID * calc_geoid_height(lat, lon);
}
/*geographic value needs if position of z work on local value*/

//////////////////////////////////section for time(timers)//////////////////////////////////

int time_refresh_data(int value_period)
{
    Time.period_time = value_period;

    Time.second_last = Time.second_refresh;

    time_t now = time(0);
    tm *ltm = localtime(&now);
    Time.second_refresh = ltm->tm_sec;

    if (Time.second_refresh != Time.second_last)
    {
        Time.refresh_time++;
    }

    if (Time.refresh_time == Time.period_time)
    {
        Time.refresh_time = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

int timer(int value_period)
{
    Time.period_time1 = value_period;

    Time.second_last1 = Time.second_refresh1;

    time_t now = time(0);
    tm *ltm = localtime(&now);
    Time.second_refresh1 = ltm->tm_sec;

    if (Time.second_refresh1 != Time.second_last1)
    {
        Time.refresh_time1++;
    }

    if (Time.refresh_time1 == Time.period_time1)
    {
        Time.refresh_time1 = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

int timer2(int value_period)
{
    Time.period_time2 = value_period;

    Time.second_last2 = Time.second_refresh2;

    time_t now = time(0);
    tm *ltm = localtime(&now);
    Time.second_refresh2 = ltm->tm_sec;

    if (Time.second_refresh2 != Time.second_last2)
    {
        Time.refresh_time2++;
    }

    if (Time.refresh_time2 == Time.period_time2)
    {
        Time.refresh_time2 = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

int timer3(int value_period)
{
    Time.period_time3 = value_period;

    Time.second_last3 = Time.second_refresh3;

    time_t now = time(0);
    tm *ltm = localtime(&now);
    Time.second_refresh3 = ltm->tm_sec;

    if (Time.second_refresh3 != Time.second_last3)
    {
        Time.refresh_time3++;
    }

    if (Time.refresh_time3 == Time.period_time3)
    {
        Time.refresh_time3 = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

//////////////////////////////////section for time//////////////////////////////////

//////////////////////////////////section for necessary connection//////////////////////////////////

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int wait_for_connect()
{
    ROS_INFO("Waiting for FCU connection");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    if (current_state.connected)
    {
        ROS_INFO("Connected to FCU");
        return 0;
    }
    else
    {
        ROS_INFO("Error connecting to drone");
        return -1;
    }
}

int wait_for_connect_GPS_value()
{
    ROS_INFO("Waiting for GPS connection");
    while (ros::ok() && global_position_received == false)
    {

        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    if (global_position_received == true)
    {
        ROS_INFO("Connected to GPS");
        return 0;
    }
    else
    {
        ROS_INFO("Error connecting to drone");
        return -1;
    }
}

//////////////////////////////////section for necessary connection//////////////////////////////////

//////////////////////////////////section for converters/////////////////////////////////////

int convert_global_distans_to_local(double set_latitude, double set_longitude)
{

    convert_value.radius_earth = 6378.137;
    convert_value.dLat = set_latitude * M_PI / 180 - call_back.latitude * M_PI / 180;
    convert_value.dLon = set_longitude * M_PI / 180 - call_back.longitude * M_PI / 180;
    convert_value.count = sin(convert_value.dLat / 2) * sin(convert_value.dLat / 2) +
                          cos(call_back.latitude * M_PI / 180) * cos(set_latitude * M_PI / 180) *
                              sin(convert_value.dLon / 2) * sin(convert_value.dLon / 2);
    convert_value.angle = 2 * atan2(sqrt(convert_value.count), sqrt(1 - convert_value.count));
    convert_value.result = convert_value.radius_earth * convert_value.angle * 1000;

    return convert_value.result;
}

//////////////////////////////////section for converters/////////////////////////////////////

//////////////////////////////////section for check distans//////////////////////////////////

int check_waypoint_reached_local()
{

    local_pos_pub.publish(pose);

    distance.current_pose_local_distans = sqrt(pow(pose.pose.position.x - call_back.position_x, 2) + pow(pose.pose.position.y - call_back.position_y, 2));
    distance.current_pose_local_hight = pose.pose.position.z - call_back.position_z;

    if (distance.current_pose_local_distans < tolerance.pos_local_tolerance && distance.current_pose_local_hight < tolerance.heading_local_tolerance)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int check_waypoint_reached_global()
{

    global_pos_pub.publish(pose_global);

    distance.deltaX = (pose_global.pose.position.latitude - call_back.latitude);
    distance.deltaY = (pose_global.pose.position.longitude - call_back.longitude);
    distance.deltaZ = (pose_global.pose.position.altitude - call_back.altitude);
    distance.dMag = sqrt(pow(distance.deltaX, 2) + pow(distance.deltaY, 2));

    if (distance.dMag < tolerance.pos_global_tolerance && distance.deltaZ < tolerance.heading_global_tolerance)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int check_waypoint_reached_global_convert_to_local()
{

    global_pos_pub.publish(pose_global);

    distance.convert_distances = convert_global_distans_to_local(pose_global.pose.position.latitude, pose_global.pose.position.longitude);
    distance.deltaZ = (pose_global.pose.position.altitude - call_back.altitude);

    if (distance.convert_distances < tolerance.pos_local_tolerance && distance.deltaZ < tolerance.heading_global_tolerance)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//////////////////////////////////section for check distans//////////////////////////////////

//////////////////////////////////section for subcribers/////////////////////////////////////

void global_position_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    call_back.altitude = ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    call_back.latitude = msg->latitude;
    call_back.longitude = msg->longitude;

    global_position_received = true;

    // save home position global
    if (close_tag.close_pose_global < 6)
    {
        home_value.global_pose_latitude = call_back.latitude;
        home_value.global_pose_longitude = call_back.longitude;
        home_value.global_pose_altitude = call_back.altitude;
        // close tag
        close_tag.close_pose_global++;
        // close tag
    }
}

void local_position_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    call_back.position_x = msg->pose.pose.position.x;
    call_back.position_y = msg->pose.pose.position.y;
    call_back.position_z = msg->pose.pose.position.z;

    // save home position local
    if (close_tag.close_pose_local < 6)
    {
        home_value.home_pose_x = call_back.position_x;
        home_value.home_pose_y = call_back.position_y;
        // close tag
        close_tag.close_pose_local++;
        // close tag
    }
    // save home position local
}

void velocity_speed_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    call_back.linear_velocity_x = msg->twist.linear.x;
    call_back.linear_velocity_y = msg->twist.linear.y;
    call_back.linear_velocity_z = msg->twist.linear.z;
    call_back.angular_velocity_x = msg->twist.angular.x;
    call_back.angular_velocity_y = msg->twist.angular.y;
    call_back.angular_velocity_z = msg->twist.angular.z;
}

//////////////////////////////////section for subcribers//////////////////////////////////

//////////////////////////////////section for Publisher/////////////////////////////////////

void publish_data(bool wyswietlac)
{

    time_refresh_data(Time.period_time);

    if (Time.refresh_time == 0 && wyswietlac == true)
    {
        ROS_INFO("-------------------------------------------");
        if (open_info.global_value == true)
        {

            ROS_INFO("latitude: %lf, longitude: %lf, altitude: %lf", call_back.latitude, call_back.longitude, call_back.altitude);
        }
        if (open_info.local_value == true)
        {

            ROS_INFO("x: %0.2f, y: %0.2f, z: %0.2f", call_back.position_x, call_back.position_y, call_back.position_z);
        }
        if (open_info.value_velocity == true)
        {

            ROS_INFO("speed_x: %0.2f, speed_y: %0.2f, speed_z: %0.2f", call_back.linear_velocity_x, call_back.linear_velocity_y, call_back.linear_velocity_z);
            std::cout << std::endl;
            ROS_INFO("angular_x: %0.2f, angular_y: %0.2f, angular_z: %0.2f", call_back.angular_velocity_x, call_back.angular_velocity_y, call_back.angular_velocity_z);
        }
        Time.refresh_time++;
    }
}

void publish_date_to_topic(bool local_values, bool globa_values, bool velocity, bool curent_time)
{
    /* all_msg(drone_data)
    float64 pose_drone_x
    float64 pose_drone_y
    float64 pose_drone_z

    float64 latitude_drone
    float64 longitude_drone
    float64 altitude_drone

    float64 velocity_linear_drone_x
    float64 velocity_linear_drone_y
    float64 velocity_linear_drone_z
    float64 velocity_angular_drone_x
    float64 velocity_angular_drone_y
    float64 velocity_angular_drone_z
    all_msg(drone_data) */

    /* all_msg(send_value)
        uint8 hour
        uint8 minut
        uint8 sekun

        int8 accept
    all_msg(send_value) */

    if (local_values == true)
    {
        drone_data.pose_drone_x = call_back.position_x;
        drone_data.pose_drone_y = call_back.position_y;
        drone_data.pose_drone_z = call_back.position_z;
    }

    if (globa_values == true)
    {
        drone_data.latitude_drone = call_back.latitude;
        drone_data.longitude_drone = call_back.longitude;
        drone_data.altitude_drone = call_back.altitude;
    }

    if (velocity == true)
    {
        drone_data.velocity_linear_drone_x = call_back.linear_velocity_x;
        drone_data.velocity_linear_drone_y = call_back.linear_velocity_y;
        drone_data.velocity_linear_drone_z = call_back.linear_velocity_z;
        drone_data.velocity_angular_drone_x = call_back.angular_velocity_x;
        drone_data.velocity_angular_drone_y = call_back.angular_velocity_y;
        drone_data.velocity_angular_drone_z = call_back.angular_velocity_z;
    }

    if (curent_time == true)
    {
        time_t now = time(0);
        tm *ltm = localtime(&now);
        send_value.hour = ltm->tm_hour;
        send_value.minut = ltm->tm_min;
        send_value.second = ltm->tm_sec;
    }

    data_drone_pub.publish(drone_data);
    data_send_value_pub.publish(send_value);
}

void publish_any_value_to_topic(int number, std::string letter)
{

    /* all_msg(mqtt_data)
        int8 number
        string letter
    all_msg(mqtt_data) */

    for (int i = 0; i < 5; i++)
    {
        mqtt_data.number = number;
        mqtt_data.letter = letter;

        data_mqtt_pub.publish(mqtt_data);
    }
}

int set_destination_local_check(float x, float y, float z, float angle)
{

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    float yaw = angle * (M_PI / 180);
    float pitch = 0;
    float roll = 0;

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);

    float qw = cy * cr * cp + sy * sr * sp;
    float qx = cy * sr * cp - sy * cr * sp;
    float qy = cy * cr * sp + sy * sr * cp;
    float qz = sy * cr * cp - cy * sr * sp;

    pose.pose.orientation.w = qw;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;

    local_pos_pub.publish(pose);

    if (check_waypoint_reached_local() == 1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int set_destination_global_check(double latitude, double longitude, double altitude, float heading)
{

    pose_global.pose.position.latitude = latitude;
    pose_global.pose.position.longitude = longitude;
    pose_global.pose.position.altitude = altitude;

    float yaw = heading * (M_PI / 180);
    float pitch = 0;
    float roll = 0;

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);

    float qw = cy * cr * cp + sy * sr * sp;
    float qx = cy * sr * cp - sy * cr * sp;
    float qy = cy * cr * sp + sy * sr * cp;
    float qz = sy * cr * cp - cy * sr * sp;

    pose_global.pose.orientation.w = qw;
    pose_global.pose.orientation.x = qx;
    pose_global.pose.orientation.y = qy;
    pose_global.pose.orientation.z = qz;

    global_pos_pub.publish(pose_global);

    if (check_waypoint_reached_global() == 1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void set_destination_local(float x, float y, float z, float angle)
{

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    float yaw = angle * (M_PI / 180);
    float pitch = 0;
    float roll = 0;

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);

    float qw = cy * cr * cp + sy * sr * sp;
    float qx = cy * sr * cp - sy * cr * sp;
    float qy = cy * cr * sp + sy * sr * cp;
    float qz = sy * cr * cp - cy * sr * sp;

    pose.pose.orientation.w = qw;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;

    local_pos_pub.publish(pose);
}

void set_destination_global(double latitude, double longitude, double altitude, float heading)
{

    pose_global.pose.position.latitude = latitude;
    pose_global.pose.position.longitude = longitude;
    pose_global.pose.position.altitude = altitude;

    float yaw = heading * (M_PI / 180);
    float pitch = 0;
    float roll = 0;

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);

    float qw = cy * cr * cp + sy * sr * sp;
    float qx = cy * sr * cp - sy * cr * sp;
    float qy = cy * cr * sp + sy * sr * cp;
    float qz = sy * cr * cp - cy * sr * sp;

    pose_global.pose.orientation.w = qw;
    pose_global.pose.orientation.x = qx;
    pose_global.pose.orientation.y = qy;
    pose_global.pose.orientation.z = qz;

    global_pos_pub.publish(pose_global);
}

//////////////////////////////////section for Publisher/////////////////////////////////////

//////////////////////////////////section for classes////////////////////////////////////////

class Myclass_info
{

public:
    void value_what_want_see(bool global_info, bool local_info, bool velocity_info);
};

//////////////////////////////////section for classes/////////////////////////////////////////

//////////////////////////////////section for service/////////////////////////////////////////

int land()
{

    if (land_client.call(srv_land) && srv_land.response.success)
    {
        ROS_INFO_ONCE("land sent %d", srv_land.response.success);
        return 0;
    }
    else
    {
        ROS_ERROR("Landing failed");
        return -1;
    }
}

//////////////////////////////////section for service/////////////////////////////////////////

//////////////////////////////////section for functions/////////////////////////////////////

void read_file(std::string path, int colums, bool see)
{

    std::string remember;
    std::string x_pos_file;
    std::string y_pos_file;
    std::string z_pos_file;

    int p = 0;
    int zapys = 0;

    filee.path = path;

    std::ifstream fin;
    fin.open(path);

    if (!fin.is_open())
    {
        ROS_INFO("file does not exist");
    }
    else
    {

        /*start read and save value from file*/
        while (!fin.eof())
        {
            fin >> remember;
            p++;

            if (p == 1)
            {
                x_pos_file = remember;
                filee.x_position_file[zapys] = stod(x_pos_file);
            }
            if (p == 2)
            {
                y_pos_file = remember;
                filee.y_position_file[zapys] = stod(y_pos_file);
            }
            if (p == 3)
            {

                z_pos_file = remember;
                filee.z_position_file[zapys] = stod(z_pos_file);
                p = 0;
                zapys++;
            }
        }
        /*start read and save value from file*/
    }

    fin.close();

    if (see == true)
    {
        for (int i = 0; i < colums; i++)
        {

            std::cout << "x" << i << ": " << filee.x_position_file[i];

            std::cout << "  y" << i << ": " << filee.y_position_file[i];

            std::cout << "  z" << i << ": " << filee.z_position_file[i] << std::endl;
        }
    }

    zapys = 0;
    p = 0;
}

int start_position(float hight)
{

    pose.pose.position.x = call_back.position_x;
    pose.pose.position.y = call_back.position_y;
    pose.pose.position.z = hight;

    local_pos_pub.publish(pose);

    if (call_back.position_z > hight - 0.3)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int back_home_position_local(float high_home)
{

    if (close_tag.close_back_home_position_local == false)
    {
        pose.pose.position.x = home_value.home_pose_x;
        pose.pose.position.y = home_value.home_pose_y;
        pose.pose.position.z = high_home;
    }

    if (check_waypoint_reached_local() == 1)
    {

        close_tag.close_back_home_position_local = true;

        pose.pose.position.x = call_back.position_x;
        pose.pose.position.y = call_back.position_y;
        pose.pose.position.z = pose.pose.position.z - 0.01;

        if (call_back.position_z < 0.2)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

int back_choice_position_local(float x, float y, float high_home)
{

    if (close_tag.back_choice_position_local == false)
    {
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = high_home;
    }

    if (check_waypoint_reached_local() == 1)
    {

        close_tag.back_choice_position_local = true;

        pose.pose.position.x = call_back.position_x;
        pose.pose.position.y = call_back.position_y;
        pose.pose.position.z = pose.pose.position.z - 0.01;

        if (call_back.position_z < 0.25)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

int back_home_position_global(float high_home)
{

    if (close_tag.close_back_home_position_global == false)
    {
        pose_global.pose.position.latitude = home_value.global_pose_latitude;
        pose_global.pose.position.longitude = home_value.global_pose_longitude;
        pose_global.pose.position.altitude = high_home;
    }

    if (check_waypoint_reached_global() == 1)
    {

        close_tag.close_back_home_position_global = true;

        pose_global.pose.position.latitude = call_back.latitude;
        pose_global.pose.position.longitude = call_back.longitude;
        pose_global.pose.position.altitude = pose_global.pose.position.altitude - 0.01;

        if (call_back.position_z < 0.25)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

int back_choice_position_global(float latitude, float longitude, float high_home)
{

    if (close_tag.back_choice_position_global == false)
    {
        pose_global.pose.position.latitude = latitude;
        pose_global.pose.position.longitude = longitude;
        pose_global.pose.position.altitude = high_home;
    }

    if (check_waypoint_reached_global() == 1)
    {

        close_tag.back_choice_position_global = true;

        pose_global.pose.position.latitude = call_back.latitude;
        pose_global.pose.position.longitude = call_back.longitude;
        pose_global.pose.position.altitude = pose_global.pose.position.altitude - 0.01;

        if (call_back.position_z < 0.25)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

int go_up_drone_curent_pose(float high)
{

    if (close_tag.go_up_global == false)
    {
        home_value.save_current_high = high;
        close_tag.go_up_global = true;
    }

    pose_global.pose.position.latitude = call_back.latitude;
    pose_global.pose.position.longitude = call_back.longitude;
    pose_global.pose.position.altitude = pose_global.pose.position.altitude + 0.01;

    global_pos_pub.publish(pose_global);

    if (pose_global.pose.position.altitude - 0.2 > home_value.save_current_high)
    {
        close_tag.go_up_global = false;
        return 1;
    }
    else
    {
        return 0;
    }
}

int go_down_drone_curent_pose(float high)
{

    if (close_tag.go_down_global == false)
    {
        home_value.save_current_high = high;
        close_tag.go_down_global = true;
    }

    pose_global.pose.position.latitude = call_back.latitude;
    pose_global.pose.position.longitude = call_back.longitude;
    pose_global.pose.position.altitude = pose_global.pose.position.altitude - 0.01;

    global_pos_pub.publish(pose_global);

    if (pose_global.pose.position.altitude < home_value.global_pose_altitude+home_value.save_current_high + 0.2)
    {
        close_tag.go_down_global = false;
        return 1;
    }
    else
    {
        return 0;
    }
}

//////////////////////////////////section for functions/////////////////////////////////////

//////////////////////////////////section for missions drone/////////////////////////////////////

int circle(float radius, int how_many, float how_fast)
{

    mission_data.circle_x = radius * cos(mission_data.teta);
    mission_data.circle_y = radius * sin(mission_data.teta);

    set_destination_local(mission_data.circle_x, mission_data.circle_y, 2, 0);

    if (mission_data.teta >= 2 * M_PI)
    {
        mission_data.teta = 0;
        mission_data.counter++;
    }

    mission_data.teta += how_fast;

    if (how_many == mission_data.counter)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//////////////////////////////////section for missions drone/////////////////////////////////////

//////////////////////////////////section for topicks,service,subcriber declaration/////////////////////////////////////

/*put this funtion at the top of your code(here you have funtion and dependes for drone position and anothe else*/
int Set_publishers_subscribers(ros::NodeHandle controlnode)
{
    std::string ros_namespace;
    if (!controlnode.hasParam("namespace"))
    {
        ROS_INFO("using default namespace");
    }
    else
    {
        controlnode.getParam("namespace", ros_namespace);
        ROS_INFO("using namespace %s", ros_namespace.c_str());
    }

    /*custom messange declaration*/
    data_drone_pub = controlnode.advertise<drone_control::drone_data>("/drone_data", 1000);
    data_mqtt_pub = controlnode.advertise<drone_control::mqtt_data>("/mqtt_data", 1000);
    data_send_value_pub = controlnode.advertise<drone_control::send_value>("/send_value", 1000);
    /*custom messange declaration*/

    /*Subcribers*/
    state_sub = controlnode.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    global_pos_sub = controlnode.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, global_position_callback);
    local_pos_sub = controlnode.subscribe("/mavros/global_position/local", 10, local_position_callback);
    velocity_value_sub = controlnode.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 10, velocity_speed_callback);
    /*Subcribers*/

    /*Publishers*/
    global_pos_pub = controlnode.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
    local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    speed_drone_pub = controlnode.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    /*Publishers*/

    /*Service*/
    land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/land").c_str());
    arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    /*Service*/
}
/*put this funtion at at the top of your code(here you have funtion and dependes for drone position and anothe else*/

//////////////////////////////////section for topicks,service,subcriber declaration/////////////////////////////////////
