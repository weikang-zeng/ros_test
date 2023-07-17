#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

ros::Publisher cmd_pub;

// Global variable to store the current source of control
std::string current_source = "/cmd_local";

void cmdLocalCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Traitez la commande locale
    if (current_source == "/cmd_local")
    {
        // Process the local command here
        ROS_INFO("Received local command: linear.x = %f, angular.z = %f", msg->linear.x, msg->angular.z);
    }
    puts("cmd_locale controlling");
    cmd_pub.publish(msg); // Republiez la commande sur le topic /cmd_vel
}

void cmdWebCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Traitez la commande web
    if (current_source == "/cmd_web")
    {
        // Process the web command here
        ROS_INFO("Received web command: linear.x = %f, angular.z = %f", msg->linear.x, msg->angular.z);
    }
    puts("cmd_web controlling");
    cmd_pub.publish(msg); // Republiez la commande sur le topic /cmd_vel
}

// Callback function for source of control change
void sourceChangeCallback(const std_msgs::String::ConstPtr& msg)
{
    current_source = msg->data;
    ROS_INFO("Source of control changed to: %s", current_source.c_str());
    puts("be careful source control change");
}



int main(int argc, char** argv)
{
    //initialisation du noeud ROS
    ros::init(argc, argv, "cmd_mux");
    //creer une poignee de noeud
    ros::NodeHandle nh;

    // Abonnez-vous aux topics des commandes de vitesse provenant des différentes sources
    ros::Subscriber cmd_local_sub = nh.subscribe("/cmd_local", 10, cmdLocalCallback);
    ros::Subscriber cmd_web_sub = nh.subscribe("/cmd_web", 10, cmdWebCallback);

    // Subscriber for source of control change
    ros::Subscriber source_change_sub = nh.subscribe("/control_source", 10, sourceChangeCallback);
    
    // Créez un éditeur pour publier les commandes de vitesse au robot Turtlebot 3
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //处理ros节点的消息和回调函数
    ros::spin();

    return 0;
}