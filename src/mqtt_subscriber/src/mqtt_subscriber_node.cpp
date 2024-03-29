#include "ros/ros.h"  //Include the ROS C++ client library
#include "geometry_msgs/Twist.h" // Include the Twist message from the geometry_msgs package
#include "mosquitto.h" //Include the mosquitto MQTT client library
#include <sstream> //Include the C++ Standard Library's stringstream class, which allows string parsing and formatting
#include <regex> //Include the C++ Standard Library's regular expression class

ros::Publisher cmd_vel_pub; //Declare a ros:Publisher objet that will publish to the cmd_vel topic

// Declare  callback function to be called when a connection with the MQTT broker is established.
void on_connect(struct mosquitto *mosq, void *obj, int rc)
{
    printf("ID: %d, Connect: %d\n", * (int *) obj, rc);
    if (rc)
    {
        printf("Connect fail with ID: %d, Code %d\n", * (int *) obj, rc);
    }
}

//Declare a callback function to be called when a message is received is from the MQTT broker
void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
    printf("Received message on topic %s: %s\n", message->topic, (char*) message->payload);

    geometry_msgs::Twist vel_msg;

    std::string payload((char*) message->payload);
    std::regex rgx_linear("linear:\n\\s*x:\\s*(-?\\d+(\\.\\d+)?).*\n\\s*y:\\s*(-?\\d+(\\.\\d+)?).*\n\\s*z:\\s*(-?\\d+(\\.\\d+)?)");
    std::regex rgx_angular("angular:\n\\s*x:\\s*(-?\\d+(\\.\\d+)?).*\n\\s*y:\\s*(-?\\d+(\\.\\d+)?).*\n\\s*z:\\s*(-?\\d+(\\.\\d+)?)");

    std::smatch matches_linear;
    std::smatch matches_angular;

    if (std::regex_search(payload, matches_linear, rgx_linear) && matches_linear.size() > 5) {
        vel_msg.linear.x = std::stod(matches_linear[1]);
        vel_msg.linear.y = std::stod(matches_linear[3]);
        vel_msg.linear.z = std::stod(matches_linear[5]);
    } else {
        ROS_ERROR("Linear values not found in payload");
        return;
    }

    if (std::regex_search(payload, matches_angular, rgx_angular) && matches_angular.size() > 5) {
        vel_msg.angular.x = std::stod(matches_angular[1]);
        vel_msg.angular.y = std::stod(matches_angular[3]);
        vel_msg.angular.z = std::stod(matches_angular[5]);
    } else {
        ROS_ERROR("Angular values not found in payload");
        return;
    }

    ROS_INFO("Publishing message to /cmd_web: linear x: %f, y: %f, z: %f; angular x: %f, y: %f, z: %f\n",
        vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z,
        vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z);

    cmd_vel_pub.publish(vel_msg);
}




/*
    std::istringstream iss((char*) message->payload);
    std::string token;
    std::string category;

    geometry_msgs::Twist twist_msg;
    while (std::getline(iss, token, ':')) {
        std::string field = token.substr(token.find_last_of(' ')+1);
        std::getline(iss, token, ',');
        std::cout << "Processing token: " << token << std::endl;
        double value = std::stod(token);

        if (field == "linear" || field =="angular"){
            category =field;
        }else{
            if(category == "linear"){
                if (field == "x") {
                    twist_msg.linear.x = value;
                } else if (field == "y") {
                    twist_msg.linear.y = value;
                } else if (field == "z") {
                    twist_msg.linear.z = value;
                }
            }else if (category =="angular"){
                if (field == "x") {
                    twist_msg.angular.x = value;
                } else if (field == "y") {
                    twist_msg.angular.y = value;
                } else if (field == "z") {
                    twist_msg.angular.z = value;
                }   
            }
        }
    }
        
    

    printf("Publishing message to /cmd_web: linear x: %f, y: %f, z: %f; angular x: %f, y: %f, z: %f\n",
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z);

    cmd_vel_pub.publish(twist_msg);
*/
/*
    double _linear, _angular;
    sscanf((char*) message->payload, "{linear: %lf, angular: %lf}", &_linear, &_angular);

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = _linear;
    vel_msg.angular.z = _angular;

    std::ostringstream os;
    os << "Publishing Twist message to /cmd_web: {linear:"<< _linear << ", angular: " << _angular << "}";
    printf("%s\n", os.str().c_str());

    cmd_vel_pub.publish(vel_msg);
}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mqtt_subscriber_node"); //Initialize the ROS node.
    ros::NodeHandle nh; //Create a ROS NodeHandle, which is used for creating publishers and subscribers

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_web", 10);

    int mqtt_client_id = 0; // MQTT client ID
    const char *mqtt_host = "localhost"; // MQTT broker host
    int mqtt_port = 1883; //Declare the MQTT broker port
    const char *mqtt_topic = "cmd_vel"; // MQTT topic 

    mosquitto_lib_init(); //Initalize the mosquitto library
    struct mosquitto *mqtt_client = mosquitto_new(NULL, true, &mqtt_client_id); //Creat a new mosquitto client instance
    if(!mqtt_client){ //Check if the client instance was created successfully
        fprintf(stderr, "Error: Out of memory.\n");
        return 1;
    }

    mosquitto_connect_callback_set(mqtt_client, on_connect); //Set the connection ccallback function
    mosquitto_message_callback_set(mqtt_client, on_message); //Set the message callback function

    if(mosquitto_connect(mqtt_client, mqtt_host, mqtt_port, 60)){ //connect to the MQTT
        fprintf(stderr, "Unable to connect.\n");
        return 1;
    }

    int mqtt_subscribe = mosquitto_subscribe(mqtt_client, NULL, mqtt_topic, 1); //Subscribe to the MQTT topic
    if(mqtt_subscribe){ // Check if the subscription was successful
        fprintf(stderr, "Unable to subscribe.\n");
        return 1;
    } else {
        printf("Subscribed to topic %s\n", mqtt_topic);
    }

    mosquitto_loop_start(mqtt_client); //Start the mosquitto network loop

    ros::spin(); // Start the ROS event loop

    mosquitto_disconnect(mqtt_client); 
    mosquitto_destroy(mqtt_client);
    mosquitto_lib_cleanup();

    return 0;
}


/*
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mosquitto.h>

// MQTT client
struct mosquitto *mqtt_client;

// Callback function for MQTT message
void mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    ROS_INFO_STREAM("Received message from topic:" << mosq);
    ROS_INFO_STREAM("Message content:" << message);
    if (message->payloadlen > 0)
    {
        std::string mqtt_message((char *)message->payload, message->payloadlen);
        // Process the MQTT message and extract linear and angular velocities
        std::string delimiter = ",";
        size_t pos = mqtt_message.find(delimiter);
        if (pos != std::string::npos)
        {
            std::string linear_vel_str = mqtt_message.substr(0, pos);
            std::string angular_vel_str = mqtt_message.substr(pos + delimiter.length(), std::string::npos);
            double linear_vel = std::stod(linear_vel_str);
            double angular_vel = std::stod(angular_vel_str);

            // Create a Twist message with the extracted velocities
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = linear_vel;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = angular_vel;

            // ROS publisher for /cmd_web topic
            ros::NodeHandle nh;
            ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            ROS_INFO("Received velocity command: linear = %f, angular = %f",linear_vel, angular_vel);
            ROS_INFO("Example format: '/cmd_local geometry_msgs/Twist\nlinear:\n  x: 0.2\n  y: 0.0\n  z: 0.0\nangular:\n  x: 0.0\n  y: 0.0\n  z: 0.0'");
            ROS_INFO("Enter velocity command (linear,angular):");
            pub.publish(cmd_vel);
        }
        else
        {
            ROS_WARN("Received invalid message format: %s", mqtt_message.c_str());

        }
    }
    else
    {
        ROS_WARN("Received empty message.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mqtt_subscriber");
    ros::NodeHandle nh;

    // Initialize the MQTT client
    mosquitto_lib_init();
    mqtt_client = mosquitto_new(NULL, true, NULL);
    if (!mqtt_client)
    {
        ROS_ERROR("Failed to initialize MQTT client.");
        return 1;
    }

    // Set the MQTT message callback
    mosquitto_message_callback_set(mqtt_client, mqtt_message_callback);

    // Connect to the MQTT broker
    const char *mqtt_broker = "localhost";  // Update with the MQTT broker's IP address
    int mqtt_port = 1883;                   // Update with the MQTT broker's port
    int mqtt_keepalive = 60;
    int mqtt_connect = mosquitto_connect(mqtt_client, mqtt_broker, mqtt_port, mqtt_keepalive);
    if (mqtt_connect != MOSQ_ERR_SUCCESS)
    {
        ROS_ERROR("Failed to connect to MQTT broker: %s", mosquitto_strerror(mqtt_connect));
        return 1;
    }

    // Subscribe to the MQTT topic
    const char *mqtt_topic = "cmd_vel";  // Update with the MQTT topic you want to subscribe to
    int mqtt_subscribe = mosquitto_subscribe(mqtt_client, NULL, mqtt_topic, 0);
    if (mqtt_subscribe != MOSQ_ERR_SUCCESS)
    {
        ROS_ERROR("Failed tosubscribe to MQTT topic: %s", mosquitto_strerror(mqtt_subscribe));
        return 1;
    }
    
    ROS_INFO("Subscribed to MQTT topic: %s", mqtt_topic);
    ROS_INFO("Waiting for velocity commands...");

    // ROS spin loop
    ros::spin();

    // Cleanup
    mosquitto_destroy(mqtt_client);
    mosquitto_lib_cleanup();

    return 0;
}*/