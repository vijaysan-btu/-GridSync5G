```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import json
import yaml
import os
import numpy as np
from std_msgs.msg import Float32
import paho.mqtt.client as mqtt

class UDPJSONSenderNode(Node):
    def __init__(self):
        super().__init__('udp_json_sender')

        # Load configuration from YAML file
        config_file = os.getenv('CONFIG_FILE', os.path.expanduser('~/ros2_ws/src/comms_package/config/mqtt_config_four_topics.yaml'))
        try:
            with open(config_file, 'r') as file:
                self.config = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration file {config_file}: {e}")
            raise

        # UDP Configuration
        try:
            self.udp_ip = self.config['udp']['source_ip']
            self.udp_port = self.config['udp']['port']
            self.destination_ip = self.config['udp']['destination_ip']
        except KeyError as e:
            self.get_logger().error(f"Missing UDP configuration: {e}")
            raise

        # Initialize UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))  # Bind to source IP
        self.sock.settimeout(0.01)  # Non-blocking
        self.get_logger().info(f"UDP Sender initialized on {self.udp_ip}:{self.udp_port}")

        # MQTT Configuration
        try:
            self.mqtt_broker = self.config['mqtt']['broker']
            self.mqtt_port = self.config['mqtt']['port']
            self.mqtt_topics = self.config['mqtt']['topics']
        except KeyError as e:
            self.get_logger().error(f"Missing MQTT configuration: {e}")
            raise

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(client_id="udp_json_sender")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
        self.mqtt_client.loop_start()
        self.get_logger().info(f"Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")

        # Create ROS 2 publishers and MQTT subscriptions
        self.publishers = {}
        for mqtt_topic, topic_config in self.mqtt_topics.items():
            ros_topic = topic_config['ros_topic']
            self.publishers[ros_topic] = self.create_publisher(Float32, ros_topic, 10)
            self.mqtt_client.subscribe(mqtt_topic)  # Default QoS=0
            self.get_logger().info(f"Set up ROS publisher and MQTT subscription for {ros_topic} ({mqtt_topic})")

        # Start timer for sending data
        self.timer = self.create_timer(1.0, self.send_data)  # Send every 1s

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker, return code: {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        self.get_logger().info(f"Received from MQTT topic {msg.topic}: {msg.payload.decode()}")

    def send_data(self):
        try:
            # Sample data for the four parameters
            data = {
                'frequency': 50.0,  # Hz
                'power': 1000.0,    # W
                'voltage': 230.0,   # V
                'current': 10.0     # A
            }

            # Encode to JSON string
            json_str = json.dumps(data)
            # Convert to uint8 bytes for UDP
            bytes_data = np.frombuffer(json_str.encode('utf-8'), dtype=np.uint8)

            # Send JSON data via UDP
            self.sock.sendto(bytes_data, (self.destination_ip, self.udp_port))
            self.get_logger().info(f"Sent UDP data to {self.destination_ip}:{self.udp_port}: {json_str}")

            # Publish each parameter to its MQTT topic and ROS topic
            for key, value in data.items():
                mqtt_topic = f"phil/{key}/{'measured' if key == 'frequency' else 'active' if key == 'power' else 'phase1'}"
                ros_topic = f"/ros2mqtt/{key}/{'measured' if key == 'frequency' else 'active' if key == 'power' else 'phase1'}"
                if mqtt_topic in self.mqtt_topics:
                    self.mqtt_client.publish(mqtt_topic, str(value))  # Default QoS=0
                    self.get_logger().info(f"Published to MQTT topic {mnt_topic}: {value}")
                    if ros_topic in self.publishers:
                        float_msg = Float32()
                        float_msg.data = float(value)
                        self.publishers[ros_topic].publish(float_msg)
                        self.get_logger().info(f"Published to ROS topic {ros_topic}: {value}")

        except Exception as e:
            self.get_logger().error(f"Error sending data: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down UDP sender...")
        self.sock.close()
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.get_logger().info("Disconnected from MQTT broker")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UDPJSONSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected, shutting down.")
    except Exception as e:
        node.get_logger().error(f"Error in node: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```