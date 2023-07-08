import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt

def mqtt_callback(client, userdata, msg):
    # Callback function for MQTT message
    message = msg.payload.decode('utf-8')
    # Process the MQTT message and extract linear and angular velocities
    linear_vel, angular_vel = message.split(',')
    # Create a Twist message with the extracted velocities
    cmd_vel = Twist()
    cmd_vel.linear.x = float(linear_vel)
    cmd_vel.angular.z = float(angular_vel)
    # Publish the Twist message to the ROS topic /cmd_web
    pub.publish(cmd_vel)

def ros_shutdown():
    # Function to be called on ROS shutdown
    client.disconnect()

if __name__ == '__main__':
    rospy.init_node('mqtt_subscriber')

    # ROS publisher for /cmd_web topic
    pub = rospy.Publisher('/cmd_web', Twist, queue_size=10)

    # MQTT client
    client = mqtt.Client()
    client.on_message = mqtt_callback
    client.connect('localhost', 1883)  # Update with the MQTT broker's IP and port

    # MQTT topic to subscribe
    mqtt_topic = 'cmd_vel'  # Update with the MQTT topic you want to subscribe to

    client.subscribe(mqtt_topic)
    client.loop_start()

    rospy.on_shutdown(ros_shutdown)

    rospy.spin()

