#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <vector>

ros::Publisher pub;

// Imu callback function
void imuCb(ConstIMUPtr &_msg){
    
    sensor_msgs::Imu imu;


    imu.header.stamp.sec = _msg->stamp().sec();
    imu.header.stamp.nsec = _msg->stamp().nsec();
    
    imu.header.frame_id = _msg->entity_name();

    imu.orientation.x = _msg->orientation().x();
    imu.orientation.y = _msg->orientation().y();
    imu.orientation.z = _msg->orientation().z();
    imu.orientation.w = _msg->orientation().w();

    imu.angular_velocity.x = _msg->angular_velocity().x();
    imu.angular_velocity.y = _msg->angular_velocity().y();
    imu.angular_velocity.z = _msg->angular_velocity().z();

    imu.linear_acceleration.x = _msg->linear_acceleration().x();
    imu.linear_acceleration.y = _msg->linear_acceleration().y();
    imu.linear_acceleration.z = _msg->linear_acceleration().z();

    pub.publish(imu);

}

int main(int _argc, char **_argv){

    // Load Gazebo & ROS
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "s500_imu");

    // Create Gazebo node and init
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Create ROS node and init
    ros::NodeHandle n;
    pub = n.advertise<sensor_msgs::Imu>("imu_gazebo_to_ros", 100000);

    // Listen to Gazebo contacts topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/s500/imu_link/imu_sensor/imu", imuCb);

    
    while (ros::ok())
    {
        gazebo::common::Time::MSleep(20);

        // Spin ROS (needed for publisher) // (nope its actually for subscribers-calling callbacks ;-) )
        ros::spinOnce();


    // Maybe sure to shut everything down.

    }
    gazebo::client::shutdown();
}