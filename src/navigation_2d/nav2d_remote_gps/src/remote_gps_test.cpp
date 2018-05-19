#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>
#include <nav2d_operator/cmd.h>
#include <nav2d_navigator/commands.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>


//Pi
const double PI = 3.14159265359;

struct Target{
	double longitude, latitude;
	Target(){}
} target;

nav2d_operator::cmd cmd;

//Publishers
ros::Publisher cmd_pub = nh.advertise<nav2d_operator::cmd>("cmd", 100);

void gpsDataCallback(const sensor_msgs/NavSatFix::ConstPtr& msg)
{
	double distance = sqrt(pow(msg.longitude-target.longitude,2)+pow(msg.latitude-target.latitude,2));
	if(distance > 3)
		cmd.Velocity = 0.5;
	else
		cmd.Velocity = 0.2;
	cmd_pub.publish(cmd);
}

void gpsHeadingCallback(const std_msgs/Float64::ConstPtr& msg)
{
	cmd.Turn = msg.data/(2*PI);
	cmd_pub.publish(cmd);
}




int main(int argc, char** argv)
{
	//Node Initialization
	ros::init(argc, argv, "remote_gps");
	//Node handle
	ros::NodeHandle nh;

	//Subscribers
	//Gps data (longitude, latitude)
	ros::Subscriber sub = nh.subscribe("GPS_goal/gps_data", 100, gpsDataCallback);
	//Gps heading
	ros::Subscriber sub = nh.subscribe("GPS_goal/gps_heading", 100, gpsHeadingCallback);

	

	//Initialize cmd Mode
	cmd.Mode = 0;
	cmd.Turn = 0;
	cmd.Velocity = 0;

	//Printf the current position
	printf("The current position is: 0 0");
	//Read Goal in Map
	
	printf("Give me the coordinates (longitude, latitude) ");
	scanf("%lf%lf",&target.longitude,&target.latitude);

	
	ros::spin();
	
	return 0;
}
