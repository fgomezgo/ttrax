#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv){
    //Initialize node
    ros::init(argc, argv, "robot_tf_broadcast");

    //Node handle
    ros::NodeHandle node;
    //Node Publisher
    //Latitude and Longitude
    ros::Publisher gps_data = node.advertise<sensor_msgs::NavSatFix>("/GPS_goal/gps_data", 100);
    //Heading
    ros::Publisher heading_data = node.advertise<std_msgs::Float64>("/GPS_goal/gps_heading", 100);

    //tf listener
    tf::TransformListener listener;

    //Simulate GPS update freq of 1 Hz
    ros::Rate rate(1.0);

    //Variables to get rotation
    double roll,pitch,yaw;

    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/odom", "/base_link",  
                ros::Time(0), transform);
            //Publish data from transform
            sensor_msgs::NavSatFix msg_NavSat_Fix;
            std_msgs::Float64 msg_heading;
            msg_NavSat_Fix.latitude = transform.getOrigin().y();
            msg_NavSat_Fix.longitude = transform.getOrigin().x();

            transform.getBasis().getRPY(roll,pitch,yaw);

            msg_heading.data = yaw;
            gps_data.publish(msg_NavSat_Fix);
            heading_data.publish(msg_heading);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }

    return 0;
}