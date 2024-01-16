
#include <declaration_all_function.hpp>

/*
start value:
channels[0]=1493 / 1494
channels[1]=1494
channels[2]=982
channels[3]=1492/ 1494
channels[4]=1160
channels[5]=1160
channels[6]=982
channels[7]=1005
channels[8]=1494
channels[9]=1494
channels[10]=1494
channels[11]=1494
channels[12]=1494
channels[13]=1494
channels[14]=1494
channels[15]=2005
channels[16]=998
channels[17]=988
*/

float memory_chanel[5]={0};
float channel[5]={0};
int tag_close=0;
int counter=0;
int end_close=0;

int check(){
    for(int i=0; i<4; i++){
      //ROS_INFO("memory:%lf",memory_chanel[i]);
      //ROS_INFO("channel:%lf",channel[i]);
      if(memory_chanel[i]==channel[i]){
        counter++;
      }
    }
//ROS_INFO("counter: %d",counter);
  if (counter==4){
    counter=0;
    end_close++;
    return 1;
  }
  else{
  end_close=0;
    counter=0;
    return 0;
  }
}

void rcin_callback(const  mavros_msgs::RCIn::ConstPtr& msg)
{

 channel[0] = msg->channels[0];
 channel[1] = msg->channels[1];
 channel[2] = msg->channels[2];
 channel[3] = msg->channels[3];

if(tag_close<5){
  memory_chanel[0]=channel[0];
  memory_chanel[1]=channel[1];
  memory_chanel[2]=channel[2];
  memory_chanel[3]=channel[3];
  tag_close++;
}

//ROS_INFO("ch1: %f, ch2: %f, ch3: %f, ch4: %f", channel[0],channel[1],channel[2],channel[3]);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener_node");
	ros::NodeHandle nh;
	ros::Subscriber pilot_sub = nh.subscribe("mavros/rc/in", 2, rcin_callback);
  	ros::Publisher chatter_pub = nh.advertise<std_msgs::Float64>("chatter_node", 100);
	
    ros::Rate loop_rate(1);
    std_msgs::Float64 msg;
    msg.data=0;

 while (ros::ok())
  {
  
  if(time_refresh_data(2)==1){
        memory_chanel[0]=channel[0];
        memory_chanel[1]=channel[1];
        memory_chanel[2]=channel[2];
        memory_chanel[3]=channel[3];
    }
  
    if (timer(3)==1){
        if(check()==1 && end_close>4){
          msg.data=1;
        }
        else{
          msg.data=0;
        }
    }
    
    chatter_pub.publish(msg);
    
    ros::spinOnce();

    loop_rate.sleep();
    
  }
	return 0;
}
