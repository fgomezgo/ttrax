#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>

int main(int argc, char** argv){
    //Initialize node
    ros::init(argc, argv, "robot_tf_broadcast");

    //Node handle
    ros::NodeHandle node;
    //Node Publisher
    ros::Publisher gps_data = node.advertise<sensor_msgs::NavSatFix>("gps_data", 100);

    //tf listener
    tf::TransformListener listener;

    //Simulate GPS update freq of 1 Hz
    ros::Rate rate(1.0);
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/base_link", "/map",  
                ros::Time(0), transform);
            //Publish data from transform
            sensor_msgs::NavSatFix msg;
            msg.latitude = transform.getOrigin().y();
            msg.longitude = transform.getOrigin().x();
            gps_data.publish(msg);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        
/*
        turtlesim::Velocity vel_msg;
        vel_msg.angular = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
        vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                    pow(transform.getOrigin().y(), 2));
        turtle_vel.publish(vel_msg);
*/
        rate.sleep();
    }

    return 0;
}