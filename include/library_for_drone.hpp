#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>
#include <math.h>
#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>

#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include "GeographicLib/Geoid.hpp"
#include <geographic_msgs/GeoPoseStamped.h>
#include <unistd.h>



/*time*/
#include <ctime>
#include <iomanip>
int hour;
int minut;
int second;
/*time*/

/* messenge where you sending date*/
#include <iq_gnc/drone_data.h>
iq_gnc::drone_data drone_data;
/* messenge where you sending date*/

/*all_subcriber, servicec and clinet what are using*/
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Subscriber global_pos_sub ;
ros::Publisher global_lla_pos_pub;
ros::ServiceClient land_client;
ros::Subscriber local_pos_sub;
ros::Publisher date_pub;
/*all_subcriber, servicec and clinet what are using*/

/*all msgs are using*/
mavros_msgs::State current_state;
geographic_msgs::GeoPoseStamped goal_position;
/*all msgs are using*/

/*value waht check if data is send*/
struct bool_value{
bool global_position_received = false;
bool drone_up =false;
bool land1= false;
bool save_height_down=false;
};
bool_value bool_value;
/*value waht check if data is send*/

/*value where is save actual position of drone */
struct call_back{
float latitude=0;
float longitude=0;
float altitude=0;
float position_x=0;
float position_y=0;
float position_z=0;
};
call_back call_back;
/*value where is save actual position of drone */

/*backhome value*/
struct home_value{
float home_position_latitude=0;
float home_position_longitude=0;
float home_position_altitude=0;
float save_actual_height=0;
float home_position_x=0;
float home_position_y=0;
float home_position_z=0;
int remember_home_position=0;
};
home_value home_value;
/*backhome value*/

/*value for setpoints*/
struct tolerance_and_control{
float pos_tolerance_global=0.00000001;
float heading_tolerance_global=0.00000001;
float start_hight=0;
int control_tolerance=0;
int how_many_waypoints=0;
int counter =1;
};
tolerance_and_control TaC;
/*value for setpoints*/

/*this value for check function reach*/
struct reach{
float deltaX=0;
float deltaY=0;
float deltaZ=0;
float dMag=0;
};
reach reach;
/*this value for check function reach*/

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/*geographic value needs if position of z work on local value*/
GeographicLib::Geoid _egm96("egm96-5"); 
double calc_geoid_height(double lat, double lon) {
    return _egm96(lat, lon);
}
double amsl_to_ellipsoid_height(double lat, double lon, double amsl) {
  return amsl + GeographicLib::Geoid::GEOIDTOELLIPSOID * calc_geoid_height(lat, lon);
}
double ellipsoid_height_to_amsl(double lat, double lon, double ellipsoid_height) {
  return ellipsoid_height + GeographicLib::Geoid::ELLIPSOIDTOGEOID * calc_geoid_height(lat, lon);
}
/*geographic value needs if position of z work on local value*/

/*send_value_from_drome_what_you_need*/

/*send_value_from_drome_what_you_need*/

void global_position_callback(const  sensor_msgs::NavSatFix::ConstPtr& msg){
call_back.altitude= ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
call_back.latitude= msg->latitude;
call_back.longitude= msg->longitude;
      
if(home_value.remember_home_position<5){
home_value.home_position_latitude=call_back.latitude;
home_value.home_position_longitude=call_back.longitude;
home_value.home_position_altitude=call_back.altitude;
home_value.remember_home_position++;
}

bool_value.global_position_received=true;

//ROS_INFO("latitude: %f, longitude: %f, altitude: %f", latitude, longitude, altitude); 
}

void local_position_callback(const  nav_msgs::Odometry::ConstPtr& msg)
{
call_back.position_x = msg->pose.pose.position.x;
call_back.position_y = msg->pose.pose.position.y;
call_back.position_z = msg->pose.pose.position.z;

if(home_value.remember_home_position<5){
home_value.home_position_x=call_back.position_x;
home_value.home_position_y=call_back.position_y;
home_value.home_position_z=call_back.position_z;
home_value.remember_home_position++;
}

//ROS_INFO("x: %0.2f, y: %0.2f, z: %0.2f", position_x, position_y, position_z);

}

/*funtion for set new setpoint for next road to drone*/
void set_destination_to_point(float latitude, float longitude, float altitude, float heading){

	goal_position.pose.position.latitude = latitude;
	goal_position.pose.position.longitude = longitude;
	goal_position.pose.position.altitude = altitude;

// 	ROS_INFO("%f",goal_position.pose.position.latitude);
// ROS_INFO("%f",goal_position.pose.position.longitude);
// ROS_INFO("%f",goal_position.pose.position.altitude);

float yaw = heading*(M_PI/180);
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

	goal_position.pose.orientation.w = qw;
	goal_position.pose.orientation.x = qx;
	goal_position.pose.orientation.y = qy;
	goal_position.pose.orientation.z = qz;

/*that funtion startet  when drone will up*/
	
	/*that funtion startet  when drone will up*/
	TaC.control_tolerance=0;
	
	global_lla_pos_pub .publish(goal_position);

}
/*funtion for set new setpoint for next road to drone*/

/*chece if drone reach the setpoint x and y (use this funtion if distance between two point no so big)*/
int chech_if_drone_reached_x_y(){

	global_lla_pos_pub .publish(goal_position);

	reach.deltaX = (goal_position.pose.position.latitude - call_back.latitude);
    reach.deltaY = (goal_position.pose.position.longitude - call_back.longitude);
    reach.deltaZ = (goal_position.pose.position.altitude-call_back.altitude); 
    reach.dMag = sqrt( pow(reach.deltaX, 2) + pow(reach.deltaY, 2) );

TaC.control_tolerance++;

/*need for what drone do not stand on one place and trying get a correct position*/
if(TaC.control_tolerance>500){
TaC.pos_tolerance_global=TaC.pos_tolerance_global*10;
}

if(TaC.control_tolerance>800){
TaC.pos_tolerance_global=TaC.pos_tolerance_global*100;
}

if(TaC.control_tolerance>1000){
TaC.pos_tolerance_global=TaC.pos_tolerance_global*1000;
}

if(TaC.control_tolerance>1300){
TaC.pos_tolerance_global=TaC.pos_tolerance_global*10000;
}
/*need for what drone do not stand on one place and trying get a correct position*/
if(reach.dMag>TaC.pos_tolerance_global){
return 1;
}
else{
return 0;

}
}
/*chece if drone reach the setpoint x and y (use this funtion if distance between two point no so big)*/

/*chece if drone reach the setpoint z(use this funtion if distance between two point no so big)*/
int chech_if_drone_reached_z(){

TaC.control_tolerance++;

/*need for what drone do not stand on one place and trying get a correct position*/
if(TaC.control_tolerance>800){
TaC.heading_tolerance_global=TaC.heading_tolerance_global*10;
}

if(TaC.control_tolerance>1300){
TaC.heading_tolerance_global=TaC.heading_tolerance_global*100;
}

if(TaC.control_tolerance>1800){
TaC.heading_tolerance_global=TaC.heading_tolerance_global*1000;
}

if(TaC.control_tolerance>2300){
TaC.heading_tolerance_global=TaC.heading_tolerance_global*10000;
}
/*need for what drone do not stand on one place and trying get a correct position*/
if(reach.deltaZ >TaC.heading_tolerance_global){
	return 1;
}
	else{
		return 0;
	}
}
/*chece if drone reach the setpoint z(use this funtion if distance between two point no so big)*/

/*funtion toling to drone go down on some value what you choise*/
int go_down_drone_smoothly(){
/*saving actual hight*/
if(bool_value.save_height_down == false){
	home_value.save_actual_height=goal_position.pose.position.altitude;
	bool_value.save_height_down=true;
}
/*saving actual hight*/
goal_position.pose.position.altitude=goal_position.pose.position.altitude-0.01;

global_lla_pos_pub.publish(goal_position);
/*this number close to home_position_altitude this value mean to what position drone will go*/
if(goal_position.pose.position.altitude<=home_value.home_position_altitude+3){


return 1;

}
else{
	return 0;
}
}
/*funtion toling to drone go down on some value what you choise*/

/*funtion toling to drone go up on some value what you choise*/
int go_up_drone_smoothly(){

goal_position.pose.position.latitude = call_back.latitude;
goal_position.pose.position.longitude = call_back.longitude;
 goal_position.pose.position.altitude = goal_position.pose.position.altitude+0.02;

global_lla_pos_pub.publish(goal_position);
/*this number close to home_position_altitude this value mean to what position drone will go*/
if(call_back.altitude>home_value.save_actual_height-0.18){
sleep(1);
 choise_what_operation=0;
 return 1;
}
else {
	return 0;
}
}
/*funtion toling to drone go up on some value what you choise*/

/*use this function then you want land your drone */
void go_down_drone_smoothly_land(){
	goal_position.pose.position.latitude = home_value.home_position_latitude;
goal_position.pose.position.longitude = home_value.home_position_longitude;
goal_position.pose.position.altitude=goal_position.pose.position.altitude-0.02;

global_lla_pos_pub.publish(goal_position);

}
/*use this function then you want land your drone */

/*landing function use by  serviceClient*/
int land(){
  mavros_msgs::CommandTOL srv_land;
  if(land_client.call(srv_land) && srv_land.response.success)
  {
    ROS_INFO("land sent %d", srv_land.response.success);
    return 0;
  }else{
    ROS_ERROR("Landing failed");
    return -1;
  }
}
/*landing function use by  serviceClient*/

/*put this funtion at at the top of your code(here you have funtion and dependes for drone position and anothe else*/
int Set_publishers_subscribers(ros::NodeHandle controlnode)
{

huj();

	std::string ros_namespace;
	if (!controlnode.hasParam("namespace"))
	{
		ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}

	state_sub = controlnode.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
    arming_client = controlnode.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");
    global_pos_sub = controlnode.subscribe < sensor_msgs::NavSatFix > ("mavros/global_position/global", 1, global_position_callback);
    global_lla_pos_pub  = controlnode.advertise < geographic_msgs::GeoPoseStamped > ("mavros/setpoint_position/global", 10);
	land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/land").c_str());
	local_pos_sub = controlnode.subscribe("/mavros/global_position/local", 2, local_position_callback);
	drone_date_pub = controlnode.advertise<drone_data::person_data>("position_time_data",1000);

	return 0;

}
/*put this funtion at at the top of your code(here you have funtion and dependes for drone position and anothe else*/
