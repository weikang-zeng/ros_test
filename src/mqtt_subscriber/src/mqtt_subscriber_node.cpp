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
}