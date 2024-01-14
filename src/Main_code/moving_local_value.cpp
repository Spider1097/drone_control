
#include <declaration_all_function.hpp>

Myclass_info info;


struct drone_waypoint {
    float x, y, z, psi;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    Set_publishers_subscribers(nh);

    wait_for_connect();
    wait_for_connect_GPS_value();
    time_refresh_data(10);
    info.value_what_want_see(true,true,false);

    ros::Rate rate(2.0);


   std::vector<drone_waypoint> waypointList;
	drone_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = -90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = -90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 180;
	waypointList.push_back(nextWayPoint);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

int counter = 0;


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

        ros::spinOnce();
        rate.sleep();

        if(check_waypoint_reached_local() == 1)
		{
			if (counter < waypointList.size())
			{
				set_destination_local(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
			}else{
				land();
				break;
			}	
		}	
		
    }

    return 0;
}