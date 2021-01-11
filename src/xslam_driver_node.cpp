//
// Created by wangzhiyong on 2021/1/11.
//
#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <xslam/xslam_sdk.hpp>
#include <cmath>
#include "frequency_counter.hpp"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

ros::Publisher odom_pub;

void show_frame(xslam_frame*);


// Display the time and the 6 dof pose in the world coordinate frame using quaternion
void show_pose_quaternion(xslam_pose_quaternion* pose);


void lost(float timestamp);

void xslam_odom_callback();

int main(int argc, char** argv) {

    ros::init(argc,argv,"xslam_node");
    ros::NodeHandle nh;

    odom_pub = nh.advertise<nav_msgs::Odometry>("xslam/odom",10000);

    xslam_parse_arguments(argc, argv);

    // Replay mode using a recorded sequence : ./visual_odometry sequence_folder
    if(argc==2)
        xslam_set_replay_folder(argv[1]);

    xslam_disp_version();


    // set the function to call for each 6 dof pose with quaternion format
    xslam_6dof_quaternion_callback(&show_pose_quaternion);

    // set the lost callback
    xslam_lost_callback(&lost);

    // start visual odometry
    xslam_start_vo();

    std::cout << " Press Enter to stop the process" << std::endl;
    std::cin.get();
    std::cout << " Stop process ..." << std::endl;

    // stop visual odometry
    xslam_stop();

    // free ressources
    return xslam_free() == xslam_status::failure ? EXIT_FAILURE : EXIT_SUCCESS;



}

void show_frame(xslam_frame*){}
// Display the time and the 6 dof pose in the world coordinate frame

// Display the time and the 6 dof pose in the world coordinate frame using quaternion
void show_pose_quaternion(xslam_pose_quaternion* pose)
{
    static FrequencyCounter fc;
    static int cnt = 0;

    fc.tic();

    if(cnt++ % 300 == 0)
    {
        double fps = fc.fps();
        double distance_to_origin = std::sqrt(pose->x[0]*pose->x[0] + pose->x[1]*pose->x[1] + pose->x[2]*pose->x[2]);
        std::cout
                <<"[6DOF]: [" << std::setw(8) << pose->timestamp << std::setw(4) << " s],"
                << " fps=" << int(fps) << std::setprecision(5)
                << " p=(" << pose->x[0] << " " << pose->x[1] << " " << pose->x[2]
                << " ), q=(" << pose->quaternion[0] << " " << pose->quaternion[1] << " " << pose->quaternion[2] << " " << pose->quaternion[3]
                << " ), to origin(" << distance_to_origin << ")"
                << ", Confidence= " << (int)pose->confidence << std::endl;
    }

    //TODO add my msg there
    nav_msgs::Odometry temp_msg;
    temp_msg.header.frame_id = "odom";
    temp_msg.header.stamp.fromSec(pose->timestamp);
    temp_msg.child_frame_id = "xslam";
    temp_msg.pose.pose.position.x = pose->x[0];
    temp_msg.pose.pose.position.y = pose->x[0];
    temp_msg.pose.pose.position.z = pose->x[0];
    temp_msg.pose.pose.orientation.x = pose->quaternion[0];
    temp_msg.pose.pose.orientation.y = pose->quaternion[1];
    temp_msg.pose.pose.orientation.z = pose->quaternion[2];
    temp_msg.pose.pose.orientation.w = pose->quaternion[3];
    odom_pub.publish(temp_msg);
}


void lost(float timestamp)
{
    std::cout << "[LOST] Device is lost at timestamp " << timestamp << " sec." << std::endl;
}