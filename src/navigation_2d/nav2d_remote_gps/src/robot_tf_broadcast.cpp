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
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/base_link", "/odom",  
                ros::Time(0), transform);
            //Publish data from transform
            sensor_msgs::NavSatFix msg_NavSat_Fix;
            std_msgs::Float64 msg_heading;
            msg_NavSat_Fix.latitude = transform.getOrigin().y();
            msg_NavSat_Fix.longitude = transform.getOrigin().x();

            //double yaw = atan2(2.0*(transform.getRotation().q.y*transform.getRotation().q.z + transform.getRotation().q.w*transform.getRotation().q.x), 
            //transform.getRotation().q.w*transform.getRotation().q.w - transform.getRotation().q.x*transform.getRotation().q.x - transform.getRotation().q.y*transform.getRotation().q.y + transform.getRotation().q.z*transform.getRotation().q.z);
            double yaw = atan2(0, 
            transform.getRotation().m_floats[3]*transform.getRotation().m_floats[3] + transform.getRotation().m_floats[2]*transform.getRotation().m_floats[2]);
            //msg_heading.data = transform.getRotation().getAngle();
            msg_heading.data = yaw;
            gps_data.publish(msg_NavSat_Fix);
            heading_data.publish(msg_heading);
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