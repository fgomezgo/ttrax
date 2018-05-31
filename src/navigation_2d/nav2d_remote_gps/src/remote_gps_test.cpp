#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>
#include <nav2d_operator/cmd.h>
#include <nav2d_navigator/commands.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>


//Pi
const double PI = 3.14159265359;
double Distance=100;
double x_me=0;
double y_me=0;
double ball_distance=0;
double ball_angle=0;
bool search=false;

struct Target{
	double longitude, latitude;
	Target(){}
} target;

struct Vector{
	double x,y;
	Vector(){}
	Vector(const double &_x, const double &_y){
		x = _x;
		y = _y;
	}
	//Vector's magnitude
	double magnitude(){
		return sqrt(x*x+y*y);
	}
};
//Methods for vectors

//Dot product
double dotProduct(const Vector &a, const Vector &b){
	return a.x*b.x+a.y*b.y;
}
//Cross product
double crossProduct(const Vector &a, const Vector &b){
	return a.x*b.y-a.y*b.x;
}
//Angle between 2 vectors
double angleBetween(Vector a, Vector b){
	return acos(dotProduct(a,b)/(a.magnitude()*b.magnitude()));
}


nav2d_operator::cmd cmd;


//Publishers
ros::Publisher cmd_pub;

//To calculate cmd's Turn

//Vector formed between current position and target
Vector toTarget = Vector(1,0);
Vector toHeading = Vector(1,0);


void gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

	if (ball_distance==0) {
		//Calculate vector between position and target
		toTarget = Vector(target.longitude-msg->longitude,target.latitude-msg->latitude);
		//ROS_INFO("Position x %lf y %lf", msg->longitude, msg->latitude);
		
		//Calculate distance between the 2 points
		double distance = toTarget.magnitude();
		
		//ROS_INFO("Distance %lf", distance);
		//Threshold to slowdown velocity
		if(distance > 0.75)
			cmd.Velocity = 0.5;
		else
			//cmd.Velocity = 0.16*distance;
			cmd.Velocity = 0.5*distance;
		cmd_pub.publish(cmd);

		//End if minError is reached
		if(distance < 0.1){
			cmd.Velocity = 0.0;
			cmd.Turn = 0.0;
			cmd_pub.publish(cmd);
			//endProgram = true;
		}
		Distance=distance;
	}
	else if (!search){
		x_me=msg->longitude;
		y_me=msg->latitude;
		search=true;
	}
}

void gpsHeadingCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if (ball_distance==0) {
		double heading = msg->data;
		//ROS_INFO("Heading %lf", heading);
		if(heading < 0)
			heading += 2*PI;
		//ROS_INFO("Heading %lf", heading);
		
		//Calculate angle between heading and toTarget
		toHeading = Vector(cos(heading),sin(heading));
		double angle = angleBetween(toTarget,toHeading);
		double cP = crossProduct(toTarget,toHeading);
		//ROS_INFO("Vector Target x %lf y %lf", toTarget.x, toTarget.y);
		//ROS_INFO("Vector Heading x %lf y %lf", toHeading.x, toHeading.y);
		if(angle < 2.8){
			if(cP < 0)
				angle *= -1.0;
		}
		//ROS_INFO("CrossProduct %lf", cP);
		//ROS_INFO("Angle %lf", angle);

		cmd.Turn = (angle)/(PI);
		if(Distance < 0.75) {
			cmd.Turn = 1.1*cmd.Turn;
			double sign = (cmd.Turn < 0)?-1:1;
			cmd.Turn = std::max(1.0,fabs(cmd.Turn));
			cmd.Turn *= sign;
		}

		//ROS_INFO("Turn %lf", double(cmd.Turn));

		cmd_pub.publish(cmd);
	}
}

/*void ballAngleCallback (const std_msgs::Float64::ConstPtr& msg) {
	ball_angle=msg->data;
}*/

void ballDistanceCallback (const std_msgs::Float32MultiArray::ConstPtr& msg) {
	ball_distance=msg->data[0];
}



int main(int argc, char** argv)
{
	//Node Initialization
	ros::init(argc, argv, "remote_gps");

	//Node handle
	ros::NodeHandle nh;

	//Publishers
	cmd_pub = nh.advertise<nav2d_operator::cmd>("cmd", 100);
	
	//Subscribers
	//Gps data (longitude, latitude)
	ros::Subscriber sub_gps_data = nh.subscribe("GPS_goal/gps_data", 100, gpsDataCallback);
	//Gps heading
	ros::Subscriber sub_gps_heading = nh.subscribe("GPS_goal/gps_heading", 100, gpsHeadingCallback);
	//Found ball
	ros::Subscriber ball_dist = nh.subscribe("ball_pos", 100, ballDistanceCallback);

	

	//Initialize cmd Mode
	cmd.Mode = 0;
	cmd.Turn = 0;
	cmd.Velocity = 0;

	//Printf the current position
	printf("The current position is: 0 0\n");
	//Read Goal in Map
	
	printf("Give me the coordinates (longitude, latitude) ");
	scanf("%lf%lf",&target.longitude,&target.latitude);
	
	double x_origin=target.longitude;
	double y_origin=target.latitude;
	double x_antipode;
	double y_antipode;
	bool back=true;
	bool once=false;

	double radio=3; //Radio to explore around point
	int N=16; //number of places to explore
	int i=0;

	//Define new targets in a circumference around the area
	while (i<=(N)){
		while (ball_distance!=0) {
			printf ("Go ahead \n");
			printf ("%f\n",ball_distance);
			//printf ("%f",ball_angle);
			cmd.Turn=0;
			cmd.Velocity=0.5;
			if (ball_distance<50) {
				cmd.Velocity=0;
				printf("Fund the ball \n");
			}
			cmd_pub.publish(cmd);
			if (cmd.Velocity==0)
				return 0;
			ball_distance=0;
			
			ros::spinOnce();	
		}

		while (search) {
			target.longitude=x_me;
			target.latitude=y_me;
			if (ball_distance!=0)
				break;
			if (Distance<0.1)
				search=false;
		}

		if (Distance<0.1 && back==true){
			back=false;
			once=true;
			i++;
			Distance=10;
		}
		else if (Distance<0.1 && back==false){
			back=true;
			once=true;
			target.longitude = x_origin;
			target.latitude = y_origin;
			Distance=10;
		}
	
		if (i%2!=0 && back==false) {
			//Get coordinates for each point according to the number of samples
			target.longitude = radio * cos( (2*PI/N)*((i-1)-((i-1)/2)) ) + x_origin;
			target.latitude = radio * sin( (2*PI/N)*((i-1)-((i-1)/2)) ) + y_origin;
			x_antipode=radio * cos( ((2*PI/N)*((i-1)-((i-1)/2)))-PI ) + x_origin;
			y_antipode=radio * sin( ((2*PI/N)*((i-1)-((i-1)/2)))-PI ) + y_origin;
		}
		else if (i%2==0 && back==false) {
			target.longitude = x_antipode;
			target.latitude = y_antipode;
		}

		//printf("Distance is %f\n ", Distance);
		if (once==true) {
			once=false;
			Distance=10;
			printf("Sample %i\n ", i);
			printf ("Target is x: %f   y:  %f \n-", target.longitude, target.latitude);
			printf ("My pos is x: %f   y:  %f \n-", x_me, y_me);
		}

		ros::spinOnce();
	}		
	return 0;
}
