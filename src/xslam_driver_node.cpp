//
// Created by wangzhiyong on 2021/1/11.
//
#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <xslam/xslam_sdk.hpp>
#include <cmath>
#include "frequency_counter.hpp"

ros::Publisher odom_pub;

void show_frame(xslam_frame*);

// Display the time and the 6 dof pose in the world coordinate frame
void show_pose(xslam_pose* pose);

// Display the time and the 6 dof pose in the world coordinate frame using quaternion
void show_pose_quaternion(xslam_pose_quaternion* pose);

// Function to call for each IMU info
void show_imu(xslam_imu* imu);

void lost(float timestamp);

int main(int argc, char** argv) {

    xslam_parse_arguments(argc, argv);

    // Replay mode using a recorded sequence : ./visual_odometry sequence_folder
    if(argc==2)
        xslam_set_replay_folder(argv[1]);

    xslam_disp_version();

    // set the function to call for each 6 dof pose, the protopy must be "(void)(xslam_pose*)"
    xslam_6dof_callback(&show_pose);

    // set the function to call for each 6 dof pose with quaternion format
    xslam_6dof_quaternion_callback(&show_pose_quaternion);

    // set the IMU callback
    xslam_imu_callback(&show_imu);

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
void show_pose(xslam_pose* pose)
{
    static FrequencyCounter fc;
    static int cnt = 0;

    fc.tic();

    if(cnt++ % 300 == 0)
    {
        double fps = fc.fps();
        double distance_to_origin = std::sqrt(pose->x*pose->x + pose->y*pose->y + pose->z*pose->z);
        std::cout
                <<"[6DOF]: [" << std::setw(8) << pose->timestamp << std::setw(4) << " s],"
                << " fps=" << int(fps) << std::setprecision(5)
                << " p=(" << pose->x << " " << pose->y << " " << pose->z
                << " ), r=(" << pose->pitch << " " << pose->yaw << " " << pose->roll << " ), to origin(" << distance_to_origin << ")"
                << ", Confidence= " << (int)pose->confidence << std::endl;
    }
}

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
}

// Function to call for each IMU info
void show_imu(xslam_imu* imu)
{
    static FrequencyCounter fc;
    static int cnt = 0;

    fc.tic();

    if(cnt++ % 100 == 0)
    {
        std::cout
                << "[IMU] : [" << std::setw(8) << imu->timestamp << std::setw(4) << " s],"
                << " fps=" << int(fc.fps())
                << " Gyro=(" << imu->gyro[0] << " " << imu->gyro[1] << " " << imu->gyro[2] << "),"
                << " Accel=("  << imu->accel[0] << " " << imu->accel[1] << " " << imu->accel[2] << "),"
                << " Magn=(" << imu->magn[0] << " " << imu->magn[1] << " " << imu->magn[2] << ")";
        if( imu->temperature > 0 ){
            std::cout << ", Temp=" << (imu->temperature - 273.15f);
        }
        std::cout << std::endl;
    }
}

void lost(float timestamp)
{
    std::cout << "[LOST] Device is lost at timestamp " << timestamp << " sec." << std::endl;
}