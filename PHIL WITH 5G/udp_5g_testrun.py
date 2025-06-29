#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from mqtt_offered_client.mqtt_offered_client import PriorityMQTTClient
import socket
import struct
import yaml
import os

class UDPServerNode(Node):
    def __init__(self):
        super().__init__('udp_server')

        # Load configuration from YAML file
        config_file = os.getenv('CONFIG_FILE', 'configure_mqtt.yaml')
        module_name = os.getenv('MODULE_NAME', 'grid')  # Default to 'grid' if not set
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        # UDP Configuration
        self.udp_ip = config['udp']['modules'][module_name]['source_ip']
        self.udp_port = config['udp']['port']
        self.destination_ips = config['udp']['modules'][module_name]['destination_ips']
        self.destination_port = config['udp']['port']

        # Initialize UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(f"UDP Server listening on {self.udp_ip}:{self.udp_port}")

        # Send initial message to all destination IPs
        message = f"PoH says oh oh oh from {module_name}".encode()
        for dest_ip in self.destination_ips:
            if dest_ip != self.udp_ip:  # Avoid sending to self
                self.sock.sendto(message, (dest_ip, self.destination_port))
                self.get_logger().info(f"Sent message to {dest_ip}:{self.destination_port}")

        # Initialize PriorityMQTTClient
        self.mqtt_client = PriorityMQTTClient(
            broker=config['mqtt']['broker'],
            port=config['mqtt']['port']
        )

        # Topic mappings from MQTT configuration
        self.topic_mappings = {
            '/ros2mqtt/grid/frequency/reference': {'mqtt_topic': 'phil/grid/frequency/reference', 'qos': 2, 'type': Float32},
            '/ros2mqtt/grid/frequency/measured': {'mqtt_topic': 'phil/grid/frequency/measured', 'qos': 2, 'type': Float32},
            '/ros2mqtt/grid/voltage/phase1': {'mqtt_topic': 'phil/grid/voltage/phase1', 'qos': 1, 'type': Float32},
            '/ros2mqtt/supercap/mode': {'mqtt_topic': 'phil/supercap/mode', 'qos': 0, 'type': String},
            '/ros2mqtt/supercap/voltage/phase1': {'mqtt_topic': 'phil/supercap/voltage/phase1', 'qos': 1, 'type': Float32},
            '/ros2mqtt/battery/mode': {'mqtt_topic': 'phil/battery/mode', 'qos': 0, 'type': String},
            '/ros2mqtt/battery/voltage/phase1': {'mqtt_topic': 'phil/battery/voltage/phase1', 'qos': 1, 'type': Float32},
            '/ros2mqtt/battery/power/active': {'mqtt_topic': 'phil/battery/power/active', 'qos': 1, 'type': Float32},
            '/ros2mqtt/fuelcell/mode': {'mqtt_topic': 'phil/fuelcell/mode', 'qos': 0, 'type': String},
            '/ros2mqtt/fuelcell/voltage/phase1': {'mqtt_topic': 'phil/fuelcell/voltage/phase1', 'qos': 1, 'type': Float32},
            '/ros2mqtt/fuelcell/power/active': {'mqtt_topic': 'phil/fuelcell/power/active', 'qos': 1, 'type': Float32},
            '/ros2mqtt/component/status': {'mqtt_topic': 'phil/component/status', 'qos': 2, 'type': String},
            '/ros2mqtt/component/logs': {'mqtt_topic': 'phil/component/logs', 'qos': 0, 'type': String},
            '/ros2mqtt/alert/grid_failure': {'mqtt_topic': 'phil/alert/grid_failure', 'qos': 2, 'type': String},
            '/ros2mqtt/connectivity/status': {'mqtt_topic': 'phil/connectivity/status', 'qos': 0, 'type': String}
        }

        # Create publishers and subscriptions
        self.publishers = {}
        self.subscriptions = {}
        for ros_topic, config in self.topic_mappings.items():
            self.publishers[ros_topic] = self.create_publisher(config['type'], ros_topic, 10)
            callback = self.create_callback(config['mqtt_topic'], config['qos'])
            self.subscriptions[ros_topic] = self.create_subscription(
                config['type'], ros_topic, callback, 10
            )

        # Timer for receiving UDP messages
        self.timer = self.create_timer(0.01, self.receive_message)

    def create_callback(self, mqtt_topic, qos):
        """Create a callback function for a specific MQTT topic and QoS."""
        def callback(msg):
            self.mqtt_client.publish(mqtt_topic, str(msg.data), qos=qos)
            self.get_logger().info(f"Published to MQTT topic {mqtt_topic}: {msg.data}")
        return callback

    def receive_message(self):
        try:
            # Receive UDP data (4 bytes for float)
            data, addr = self.sock.recvfrom(4)
            self.get_logger().info(f"Received raw data from {addr}: {data}")

            # Interpret data as a float
            try:
                received_float = struct.unpack('f', data)[0]
                # Publish to ROS 2 topic (assuming supercap/voltage/phase1 for now)
                ros_topic = '/ros2mqtt/supercap/voltage/phase1'
                float_msg = Float32()
                float_msg.data = received_float
                if ros_topic in self.publishers:
                    self.publishers[ros_topic].publish(float_msg)
                    self.get_logger().info(f"Published to ROS 2 topic {ros_topic}: {received_float}")
                    # Forward data to other modules
                    for dest_ip in self.destination_ips:
                        if dest_ip != self.udp_ip and dest_ip != addr[0]:  # Avoid sending to self or sender
                            self.sock.sendto(data, (dest_ip, self.destination_port))
                            self.get_logger().info(f"Forwarded data to {dest_ip}:{self.destination_port}")
                else:
                    self.get_logger().warn(f"No publisher for ROS 2 topic {ros_topic}")
            except struct.error:
                self.get_logger().warn(f"Invalid float data from {addr}: {data}")

        except socket.timeout:
            pass  # No data received, continue
        except Exception as e:
            self.get_logger().error(f"Error receiving data: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down UDP server...")
        self.mqtt_client.stop()
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UDPServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()