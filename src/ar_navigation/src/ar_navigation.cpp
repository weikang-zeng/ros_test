#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/common_functions.h>
#include <apriltag_ros/single_image_detector.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

ros::Publisher goal_pub;
apriltag_ros::SingleImageDetector* detector;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // Convert ROS Image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

// Convert OpenCV image to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
/*
    apriltag_ros::AprilTagDetectionArray detection_array = detector->detectTags(cv_ptr,nullptr);


    // Process the detected tags
    for (const auto& detection : detection_array.detections)
    {
        // Get the tag ID and corners
        int aprilTagId = detection.id[0];

        // Print the tag ID and corners
        ROS_INFO("Detected AprilTag ID: %d", aprilTagId);
        
    }*/

/*
    //Display the image
    cv::imshow("Received Image",image);
    cv::waitKey(1);
*/
/*    
    // Detect AR tags in the image
    cv::Ptr<aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    std::vector<int> markerIds;
    std::vector<std::vector<Point2f>> markerCorners;
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

    //Draw detected markers on the image
    cv::aruco::drawDetectedMarkers(image,markerCorners,markerIds);
    cv::imshow("Detected AR tags",image);
    cv::waitKey(1);


    // Print the IDs and corners of the detected AR tags
    for (size_t i = 0; i < markerIds.size(); ++i)
        {
            int arTagId = markerIds[i];
            std::vector<cv::Point2f> corners = markerCorners[i];
            ROS_INFO("Detected AR tag ID: %d", arTagId);
            for (size_t j = 0; j < corners.size(); ++j)
                {
                    ROS_INFO("Corner %zu: x = %f, y = %f", j, corners[j].x, corners[j].y);
                }
        }
*/


    /*    
    // Process detected markers
    for (size_t i = 0; i < markerIds.size(); ++i)
    {
      // Assuming you have defined the AR tag IDs and corresponding goals
      // Modify the code according to your AR tag IDs and goals
      int arTagId = markerIds[i];
      geometry_msgs::PoseStamped goal;

      if (arTagId == 20) // ARtag1
      {
        goal.header.frame_id = "map";
        goal.pose.position.x = 8.0;
        goal.pose.position.y = 0.0;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 0.0;
      }
      else if (arTagId == 21) // ARtag2
      {
        goal.header.frame_id = "map";
        goal.pose.position.x = 1.0;
        goal.pose.position.y = 1.0;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 0.0;
      }
      else if (arTagId == 22) // ARtag3
      {
        goal.header.frame_id = "map";
        goal.pose.position.x = -1.0;
        goal.pose.position.y = -1.0;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 0.0;
      }
      else
      {
        ROS_WARN("Unknown AR tag ID: %d", arTagId);
        continue;
      }

      // Publish the goal
      goal_pub.publish(goal);
      ROS_INFO("Published goal: position = (%f, %f, %f), orientation = (%f, %f, %f, %f)",
           goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
           goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);

      ROS_INFO("Published goal for AR tag ID: %d", arTagId);
    }*/
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_navigation");
  ros::NodeHandle nh;

  // Load the AR tag dictionary
  //dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);

  //Create SingleImageDetector
  detector = new apriltag_ros::SingleImageDetector(nh,nh);

  // Subscribe to the image topic
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("image_topic", 1, imageCallback);

  // Publish the navigation goal
  //goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  
  //ROS_INFO("Published goal for AR tag ID: %d", arTagId);
  ros::spin();

  delete detector;

  return 0;
}