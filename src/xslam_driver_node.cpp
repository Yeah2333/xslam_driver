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
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <memory>
cv::Mat rgbImage(xslam_rgb *rgb);


std::string codec_name( xslam_rgb_codec codec );

void pub_pose(xslam_pose_quaternion* pose);
void pub_rgb(xslam_rgb* rgb);
void lost(float timestamp);


ros::Publisher odom_pub;
image_transport::Publisher image_pub;

int main(int argc, char** argv) {

    ros::init(argc,argv,"xslam_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_pub = it.advertise("xslam/image", 100);
    // image_pub = nh.advertise<sensor_msgs::Image>("xslam/image", 100);
    odom_pub = nh.advertise<nav_msgs::Odometry>("xslam/odom", 10000);

    xslam_disp_version();

    xslam_6dof_quaternion_callback(&pub_pose);
    xslam_rgb_callback(&pub_rgb);
    xslam_lost_callback(&lost);

    // start visual odometry
    xslam_start_vo();
    xslam_start_camera();
    // RGB_640x480      RGB_1280x720        RGB_1920x1080
    xslam_set_rgb_resolution( xslam_rgb_resolution::RGB_1920x1080);

    std::cout << " Press Enter to stop the process" << std::endl;
    std::cin.get();
    std::cout << " Stop process ..." << std::endl;

    // stop visual odometry
    xslam_stop();

    // free ressources
    return xslam_free() == xslam_status::failure ? EXIT_FAILURE : EXIT_SUCCESS;
}

void pub_pose(xslam_pose_quaternion* pose)
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

std::string codec_name( xslam_rgb_codec codec )
{
    switch( codec ){
    case YUYV: return "YUYV";
    case YUV420p: return "YUV420p";
    case JPEG: return "JPEG";
    }
    return "unknown";
}

void pub_rgb(xslam_rgb* rgb)
{
    // rgb->codec = xslam_rgb_codec::JPEG;
    cv::Mat image = rgbImage( rgb );
	//将Mat类型转为sensor_msgs::Image
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->header.frame_id = "image";
	//发布消息
	image_pub.publish(msg);
}
cv::Mat rgbImage(xslam_rgb *rgb)
{
    cv::Mat out;
    switch(rgb->codec){
       case xslam_rgb_codec::YUYV:{
           cv::Mat img( rgb->height, rgb->width, CV_8UC2, rgb->data );
           cv::cvtColor( img, out, cv::COLOR_YUV2RGB_YUYV );
           break;
       }
       case xslam_rgb_codec::YUV420p:{
           cv::Mat img( static_cast<int>(1.5*rgb->height), rgb->width, CV_8UC1, rgb->data );
           cv::cvtColor( img, out, cv::COLOR_YUV420p2RGB );
           break;
       }
       case xslam_rgb_codec::JPEG:{
           cv::Mat img( rgb->height, rgb->width, CV_8UC3, rgb->data );
           out = cv::imdecode( img, cv::IMREAD_COLOR );
           break;
       }
    }

    return out;
}


void lost(float timestamp)
{
    std::cout << "[LOST] Device is lost at timestamp " << timestamp << " sec." << std::endl;
}