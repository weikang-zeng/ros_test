#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image_topic", 1);

    cv::Mat img = cv::imread("ARtag1.png", cv::IMREAD_COLOR);
    if(img.empty())
    {
        ROS_ERROR("Failed to load image.");
        return -1;
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

    ros::Rate loop_rate(5);
    while (nh.ok())
    {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}