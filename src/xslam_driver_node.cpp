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
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <memory>

// convert image by its encoding
cv::Mat rgbImage(xslam_rgb *rgb);
// get encoding string
std::string codec_name( xslam_rgb_codec codec );
// dist to color
std::tuple<int,int,int> color(double distance, double distance_min, double distance_max, double threshold);
// formate depth image
std::shared_ptr<unsigned char> tofImage(float *data, unsigned int width, unsigned int height, double distance);

/// callback functions for VISIO camera
void pub_pose(xslam_pose_quaternion* pose);
void pub_rgb(xslam_rgb* rgb);
void pub_depth(xslam_tof* tof);
void lost(float timestamp);

/// Publishers
ros::Publisher odom_pub;
image_transport::Publisher image_pub;
image_transport::Publisher depth_pub;




int main(int argc, char** argv) {
    ros::init(argc,argv,"xslam_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_pub = it.advertise("xslam/image", 100);
    depth_pub = it.advertise("xslam/depth", 100);
    odom_pub = nh.advertise<nav_msgs::Odometry>("xslam/odom", 10000);


    xslam_disp_version();
    // ====================================
    /// set callback functions for VISIO camera
    // ====================================
    xslam_6dof_quaternion_callback(&pub_pose);
    xslam_rgb_callback(&pub_rgb);
    xslam_tof_callback(&pub_depth);
    xslam_lost_callback(&lost);
    
    /// Configution of VISIO camera
    // resolution       RGB_640x480      RGB_1280x720        RGB_1920x1080
    xslam_set_rgb_resolution( xslam_rgb_resolution::RGB_1920x1080);
    // TOF      
    // framerate:       TOF_5Hz     TOF_10Hz        TOF_20Hz        TOF_25Hz        TOF_30Hz
    // mode:                TOF_LONG_RANGE  or  TOF_SHORT_RANGE
    // option:              disable_tof_median_filter  or  enable_tof_median_filter
    xslam_set_tof_framerate(TOF_10Hz);
    xslam_set_tof_mode(TOF_LONG_RANGE);
    // xslam_set_option(enable_tof_median_filter);

    // ====================================
    // start visual odometry
    // ====================================
    xslam_start_camera();
    xslam_start_vo();

    // ====================================
    // use 'while' to control freq
    // ====================================
    std::cout << " Press Enter to stop the process" << std::endl;
    std::cin.get();
    std::cout << " Stop process ..." << std::endl;

    // ====================================
    // stop visual odometry
    // ====================================
    xslam_stop();
    // free ressources
    return xslam_free() == xslam_status::failure ? EXIT_FAILURE : EXIT_SUCCESS;
}

void pub_pose(xslam_pose_quaternion* pose)
{
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

void pub_rgb(xslam_rgb* rgb)
{
    cv::Mat image = rgbImage( rgb );
	
    //convert from type cv::Mat to sensor_msgs::Image
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->header.frame_id = "image";
    msg->header.stamp.fromSec(rgb->timestamp);

	image_pub.publish(msg);
}

void pub_depth(xslam_tof* tof) {
    std::shared_ptr<unsigned char> tmp = tofImage( tof->data, tof->width, tof->height, 2.5 );
    cv::Mat im(tof->height, tof->width, CV_8UC3, tmp.get() );
    
    sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
    depth_msg->header.stamp.fromSec(tof->timestamp);
    depth_msg->encoding     = sensor_msgs::image_encodings::TYPE_8UC3;
    depth_msg->height           =  tof->height;
    depth_msg->width            =  tof->width;
    depth_msg->step               = depth_msg->width * 3;   // 8UC3
    depth_msg->data.resize(depth_msg->height * depth_msg->step);
    memcpy(reinterpret_cast<void*>(&depth_msg->data[0]), im.data, depth_msg->height * depth_msg->step);
    
    depth_pub.publish(depth_msg);
}

void lost(float timestamp)
{
    std::cout << "[LOST] Device is lost at timestamp " << timestamp << " sec." << std::endl;
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

std::string codec_name( xslam_rgb_codec codec )
{
    switch( codec ){
    case YUYV: return "YUYV";
    case YUV420p: return "YUV420p";
    case JPEG: return "JPEG";
    }
    return "unknown";
}

std::tuple<int,int,int> color(double distance, double distance_min, double distance_max, double threshold)
{
    double d = std::max(distance_min, std::min( distance, distance_max ) );
    d = (d - distance_min)/(distance_max - distance_min);
    if( distance <= threshold || distance > 9){
        return std::tuple<int,int,int>(0,0,0);
    }
    int b = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5-std::abs(1.0-4.0*(d-0.5))),1.0));
    int g = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5-std::abs(1.0-4.0*(d-0.25))),1.0));
    int r = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5-std::abs(1.0-4.0*d)),1.0));
    return std::tuple<int,int,int>(r,g,b);
}

std::shared_ptr<unsigned char> tofImage(float *data, unsigned int width, unsigned int height, double distance)
{
    std::shared_ptr<unsigned char> out( new unsigned char[width*height*3] );
    for (unsigned int i=0; i<width*height; i++) {
        auto c = color( data[i], 0.0f, distance, 0.0f);
        out.get()[i*3+0] = static_cast<unsigned char>(std::get<2>(c));
        out.get()[i*3+1] = static_cast<unsigned char>(std::get<1>(c));
        out.get()[i*3+2] = static_cast<unsigned char>(std::get<0>(c));
    }
    return out;
}


