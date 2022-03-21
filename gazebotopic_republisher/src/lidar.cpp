#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <iostream>
#include <vector>
#include <math.h>

ros::Publisher pub;

// Imu callback function
void lidarCb(ConstLaserScanStampedPtr &_msg){
    
    livox_ros_driver::CustomMsg custom_msg;
    livox_ros_driver::CustomPoint custom_point;


    custom_msg.header.stamp.sec = _msg->time().sec();
    custom_msg.header.stamp.nsec = _msg->time().nsec();
    custom_msg.header.frame_id = _msg->scan().frame();

    custom_msg.timebase = _msg->time().sec();

    custom_msg.point_num = (_msg->scan().count()) * (_msg->scan().vertical_count());

    //custom_msg.lidar_id = 1;

    //custom_msg.rsvd[0] = 0;
    //custom_msg.rsvd[1] = 0;
    //custom_msg.rsvd[2] = 0;


    // Calculation
    int no_horizontalsample = _msg->scan().count();
    int no_verticalsample = _msg->scan().vertical_count();
    float angle_per_horizontalsample = 1.22871179 / (no_horizontalsample - 1);
    float angle_per_verticalsample = 1.22871179 / (no_verticalsample - 1);

    int j = 0;

    for (int i = 0; i < custom_msg.point_num; ++i) {

        float sample_dist = _msg->scan().ranges(i);

        //Horizontal Calculation
        float horizontal_angle;
        if (((i + 1) % no_horizontalsample) == 0) {
            horizontal_angle = -0.61435590 + (angle_per_horizontalsample * j); //angle between x-axis & sample_dist on xy-plane
            j = 0;
        }
        else {
            horizontal_angle = -0.61435590 + (angle_per_horizontalsample * j); //angle between x-axis & sample_dist on xy-plane
            ++j;
        }

        //Vertical Calculation  
        int divide_vertical = ((i + 1) / float(no_horizontalsample) - 0.0000001); //if (i+1)/no_horizontalsample equals to a whole number, '-0.00000001' will cause the whole number to round down to the next whole number.
        float vertical_angle = 0.61435590 - (angle_per_verticalsample * divide_vertical); //angle between x-axis & sample_dist on xz-plane
        
        //Trigo calculation
        float x_axis = sample_dist * cos(fabsf(horizontal_angle));
        float y_axis = sample_dist * sin(fabsf(horizontal_angle));
        float z_axis = x_axis * tan(fabsf(vertical_angle));

        if (horizontal_angle < -0.00001) {
            custom_point.x = x_axis;
            custom_point.y = y_axis;
        }
        else if (horizontal_angle > 0.00001) {
            custom_point.x = x_axis;
            custom_point.y = y_axis * (-1);
        }
        else if (-0.00001 <= horizontal_angle && horizontal_angle <= 0.00001) {
            custom_point.x = x_axis;
            custom_point.y = 0;
        }

        if (vertical_angle < -0.00001) {
            custom_point.z = z_axis * (-1);
        }
        else if (vertical_angle > 0.00001) {
            custom_point.z = z_axis;
        }
        else if (-0.00001 <= vertical_angle && vertical_angle <= 0.00001) {
            custom_point.z = 0;
        }


        //custom_point.offset_time(i) = _msg->time().sec(); 

        //custom_point.reflectivity[i] = (0-255);
        //custom_point.tag = (0-255);
        custom_point.line = 0;

        custom_msg.points.push_back(custom_point);
    }

    pub.publish(custom_msg);

}

int main(int _argc, char **_argv){

    // Load Gazebo & ROS
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "s500_lidar");

    // Create Gazebo node and init
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Create ROS node and init
    ros::NodeHandle n;
    pub = n.advertise<livox_ros_driver::CustomMsg>("lidar_gazebo_to_ros", 100000);

    // Listen to Gazebo contacts topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/s500/livox_lipo_ass/livox/scan", lidarCb);

    
    while (ros::ok())
    {
        gazebo::common::Time::MSleep(20);

        // Spin ROS (needed for publisher) // (nope its actually for subscribers-calling callbacks ;-) )
        ros::spinOnce();


    // Maybe sure to shut everything down.

    }
    gazebo::client::shutdown();
}