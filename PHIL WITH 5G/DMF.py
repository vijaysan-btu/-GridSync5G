#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import struct
import paho.mqtt.client as mqtt

class UDPServerNode(Node):
    def __init__(self):
        super().__init__('udp_server')

        # UDP Configuration
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

        # MQTT Configuration
        self.mqtt_broker = "172.16.11.70"
        self.mqtt_port = 1883
        self.mqtt_topic = "frequency"  # Updated MQTT topic

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(client_id="udp_server_node")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
        self.mqtt_client.loop_start()

        # Start receiving UDP messages
        self.timer = self.create_timer(0.05, self.receive_message)  # Check every 0.05s (20 Hz)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback for when the MQTT client connects to the broker."""
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.subscribe(self.mqtt_topic)
            self.get_logger().info(f"Subscribed to MQTT topic: {self.mqtt_topic}")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker, return code: {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        """Callback for when a message is received from the MQTT topic."""
        self.get_logger().info(f"Received from MQTT topic {msg.topic}: {msg.payload.decode()}")

    def receive_message(self):
        """Receive UDP data and publish to MQTT topic."""
        try:
            data, addr = self.sock.recvfrom(4)  # Expect 4 bytes for a float
            self.get_logger().info(f"Received raw data from {addr}: {data}")

            # Try to interpret data as a float
            try:
                received_float = struct.unpack('f', data)[0]
                mqtt_message = f"UDP received: {received_float}"
                self.mqtt_client.publish(self.mqtt_topic, mqtt_message)
                self.get_logger().info(f"Published to MQTT topic {self.mqtt_topic}: {mqtt_message}")
            except struct.error:
                self.get_logger().warn(f"Invalid float data from {addr}: {data}")

        except Exception as e:
            self.get_logger().error(f"Error receiving UDP data: {e}")

    def destroy_node(self):
        """Clean up resources on node shutdown."""
        self.get_logger().info("Shutting down UDP server...")
        self.sock.close()
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.get_logger().info("Disconnected from MQTT broker")
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