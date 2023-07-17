import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker.")

def on_disconnect(client, userdata, rc):
    print("Disconnected from MQTT broker.")

if __name__ == '__main__':
    client = mqtt.Client()

    client.connect('localhost', 1883)  # Update with the MQTT broker's IP and port

    client.loop_start()

    try:
        while True:


            # Get velocity command from user
            velocity_str = input("Enter velocity command (linear, angular): ")
   
            # Parse linear and angular velocities
            linear_vel, angular_vel = velocity_str.split(",")
            linear_vel = float(linear_vel.strip())
            angular_vel = float(angular_vel.strip())

            message = f"/cmd_vel geometry_msgs/Twist\nlinear:\n  x: {linear_vel}\n  y: 0.0\n  z: 0.0\nangular:\n  x: 0.0\n  y: 0.0\n  z: {angular_vel}"
            # Publish the MQTT message to the specified topic
            mqtt_topic = 'cmd_vel'  # Update with the MQTT topic you want to publish to


            client.publish(mqtt_topic, message)
            print("Sent velocity command:")
            print(mqtt_topic)
            print(message)
    finally:
        client.loop_stop()
        client.disconnect()
