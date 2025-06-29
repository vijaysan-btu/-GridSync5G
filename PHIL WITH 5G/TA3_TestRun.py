#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import socket
import struct

class UDPServerNode(Node):
    def __init__(self):
        super().__init__('udp_server')

        # Configuration
        self.udp_ip = "172.16.11.12"
        self.udp_port = 7000
        self.destination_ip = "172.16.11.70"
        self.destination_port = 7000

        # Initialize UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        self.get_logger().info(f"UDP Server listening on {self.udp_ip}:{self.udp_port}")

        # Send initial message
        self.sock.sendto("Connection Established".encode(), (self.destination_ip, self.destination_port))

        # ROS 2 Publisher (ROS topic 1: publish UDP data to MQTT via bridge)
        self.data_publisher = self.create_publisher(String, '/ros2mqtt/frequency', 10)

        # ROS 2 Subscriber (ROS topic 2: subscribe to data from MQTT via bridge)
        self.data_subscriber = self.create_subscription(
            String,
            '/mqtt2ros/frequency',
            self.subscriber_callback,
            10
        )

        # Start receiving UDP messages
        self.timer = self.create_timer(0.05, self.receive_message)  # Check every 0.05s (20 Hz)

    def receive_message(self):
        try:
            data, addr = self.sock.recvfrom(4)  # Expect 4 bytes for a float
            self.get_logger().info(f"Received raw data from {addr}: {data}")

            # Try to interpret data as a float
            try:
                received_float = struct.unpack('f', data)[0]
                string_msg = String()
                string_msg.data = f"UDP received: {received_float}"
                self.data_publisher.publish(string_msg)
                self.get_logger().info(f"msg Published to /ros2mqtt/frequency: {received_float}")
            except struct.error:
                self.get_logger().warn(f"Invalid float data from {addr}: {data}")

        except Exception as e:
            self.get_logger().error(f"Error receiving data: {e}")

    def subscriber_callback(self, msg):
        """Callback for the second ROS topic subscription."""
        self.get_logger().info(f"Received from /mqtt2ros/frequency: {msg.data}")

    def destroy_node(self):
        self.get_logger().info("Shutting down UDP server...")
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