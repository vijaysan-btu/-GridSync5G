#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import struct
import paho.mqtt.client as mqtt
import yaml
import os
from std_msgs.msg import Float32, String

class UDPMultiTopicServerNode(Node):
    def __init__(self):
        super().__init__('udp_fuelcell_server')

        # Load configuration from YAML file
        config_file = os.getenv('CONFIG_FILE', os.path.expanduser('~/ros2_ws/src/mqtt_python_client/config/mqtt_config_TC_Testrun.yaml'))
        module_name = os.getenv('MODULE_NAME', 'fuelcell')
        try:
            with open(config_file, 'r') as file:
                self.config = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration file {config_file}: {e}")
            raise

        # UDP Configuration
        try:
            self.udp_ip = self.config['udp']['modules'][module_name]['source_ip']
            self.udp_port = self.config['udp']['port']
            self.destination_ips = self.config['udp']['modules'][module_name]['destination_ips']
            self.destination_port = self.config['udp']['port']
        except KeyError as e:
            self.get_logger().error(f"Missing UDP configuration for module {module_name}: {e}")
            raise

        # Initialize UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(0.01)  # Non-blocking
        self.get_logger().info(f"UDP Server listening on {self.udp_ip}:{self.udp_port}")

        # Send initial message to all destination IPs
        init_message = f"Connection Established from {module_name}".encode()
        for dest_ip in self.destination_ips:
            self.sock.sendto(init_message, (dest_ip, self.destination_port))
            self.get_logger().info(f"Sent initial message to {dest_ip}:{self.destination_port}")

        # MQTT Configuration
        try:
            self.mqtt_broker = self.config['mqtt']['broker']
            self.mqtt_port = self.config['mqtt']['port']
            self.mqtt_topics = self.config['mqtt']['topics']
        except KeyError as e:
            self.get_logger().error(f"Missing MQTT configuration: {e}")
            raise

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(client_id=f"udp_server_{module_name}")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
        self.mqtt_client.loop_start()
        self.get_logger().info(f"Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")

        # Create ROS 2 publishers and subscriptions
        self.publishers = {}
        self.subscriptions = {}
        for mqtt_topic, topic_config in self.mqtt_topics.items():
            ros_topic = topic_config['ros_topic']
            msg_type = Float32 if 'voltage' in mqtt_topic or 'frequency' in mqtt_topic or 'power' in mqtt_topic else String
            self.publishers[ros_topic] = self.create_publisher(msg_type, ros_topic, 10)
            callback = self.create_subscription_callback(mqtt_topic)
            self.subscriptions[ros_topic] = self.create_subscription(msg_type, ros_topic, callback, 10)
            self.get_logger().info(f"Set up ROS 2 topic {ros_topic} for MQTT topic {mqtt_topic}")

        # Start receiving UDP messages
        self.timer = self.create_timer(0.05, self.receive_message)  # 20 Hz

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback for when the MQTT client connects to the broker."""
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
            for mqtt_topic in self.mqtt_topics.keys():
                client.subscribe(mqtt_topic, qos=self.mqtt_topics[mqtt_topic]['qos'])
                self.get_logger().info(f"Subscribed to MQTT topic: {mqtt_topic} with QoS {self.mqtt_topics[mqtt_topic]['qos']}")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker, return code: {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        """Callback for when a message is received from an MQTT topic."""
        self.get_logger().info(f"Received from MQTT topic {msg.topic}: {msg.payload.decode()}")
        try:
            # Convert message to appropriate type and publish to ROS 2 topic
            for ros_topic, topic_config in self.mqtt_topics.items():
                if topic_config['mqtt_topic'] == msg.topic:
                    if 'voltage' in msg.topic or 'frequency' in msg.topic or 'power' in msg.topic:
                        float_msg = Float32()
                        float_msg.data = float(msg.payload.decode())
                        self.publishers[ros_topic].publish(float_msg)
                    else:
                        string_msg = String()
                        string_msg.data = msg.payload.decode()
                        self.publishers[ros_topic].publish(string_msg)
                    break

            # Forward to UDP destinations
            for dest_ip in self.destination_ips:
                self.sock.sendto(msg.payload, (dest_ip, self.destination_port))
                self.get_logger().info(f"Forwarded MQTT message to {dest_ip}:{self.destination_port}")
        except ValueError as e:
            self.get_logger().error(f"Invalid data format for {msg.topic}: {e}")

    def create_subscription_callback(self, mqtt_topic):
        """Create a callback for ROS 2 subscription to publish to MQTT."""
        def callback(msg):
            data = str(msg.data)
            self.mqtt_client.publish(mqtt_topic, data, qos=self.mqtt_topics[mqtt_topic]['qos'])
            self.get_logger().info(f"Published to MQTT topic {mqtt_topic}: {data}")
        return callback

    def receive_message(self):
        """Receive and process UDP messages."""
        try:
            data, addr = self.sock.recvfrom(4)  # Expect 4 bytes for float
            self.get_logger().info(f"Received UDP data from {addr}: {data}")

            try:
                # Interpret data as float (e.g., for voltage, frequency, power)
                received_float = struct.unpack('f', data)[0]
                # Publish to module-specific topic (e.g., fuelcell/voltage/phase1)
                module_topic = f"/ros2mqtt/{os.getenv('MODULE_NAME', 'fuelcell')}/voltage/phase1"
                if module_topic in self.publishers:
                    float_msg = Float32()
                    float_msg.data = received_float
                    self.publishers[module_topic].publish(float_msg)
                    self.get_logger().info(f"Published to ROS 2 topic {module_topic}: {received_float}")

                    # Publish to MQTT
                    mqtt_topic = f"phil/{os.getenv('MODULE_NAME', 'fuelcell')}/voltage/phase1"
                    self.mqtt_client.publish(mqtt_topic, str(received_float), qos=self.mqtt_topics[mqtt_topic]['qos'])
                    self.get_logger().info(f"Published to MQTT topic {mqtt_topic}: {received_float}")

                    # Forward to other UDP destinations
                    for dest_ip in self.destination_ips:
                        if dest_ip != addr[0]:  # Avoid sending to sender
                            self.sock.sendto(data, (dest_ip, self.destination_port))
                            self.get_logger().info(f"Forwarded UDP data to {dest_ip}:{self.destination_port}")
                else:
                    self.get_logger().warn(f"No publisher for ROS 2 topic {module_topic}")
            except (struct.error, KeyError) as e:
                self.get_logger().warn(f"Invalid UDP data from {addr}: {e}")

        except socket.timeout:
            pass  # No data received
        except Exception as e:
            self.get_logger().error(f"Error receiving UDP data: {e}")

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info("Shutting down UDP server...")
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UDPMultiTopicServerNode()
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