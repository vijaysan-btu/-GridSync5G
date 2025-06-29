"""
this is a guideline for using the PHIL lab 5G modules (Dont execute this file).
Project 5G connected "sensing". Through a PC with simulink we define the 5G network topology. All the 5G modules SIM8200EA
(raspberry pi 4 based) are connected to all the DGs by the ethernet network 172.16.11.XXX.  
To install the software in RaspberryPis in every 5G module use the file "installation_simcomdriver_mqtt_ros_udp", which installs:
  - ubuntu20.04
  - SIMCOM8200 driver
  - MQTT paho 1.6.1
  - ROS foxy
  - ROS nodes for the application: mqtt and udp (this should be added manually)
"""

##################################  Procedure to use the 5G modules  ###################################################
# Turn on the main power switch of the 5G modules so they start bliking green.
# Turn on PC ORCCHIDEE in PHIL lab and connect via SSH to each module: e.g.:
ssh ubuntu@172.16.11.12
# password Nr12345$

#read current 5G IP (see wwan0 interface), (optional).
#verify all the modules have the same subnetmask /28, otherwise refer to "changing subnetmask" in this file.
ip a 

# verify the 5G connection to the server of each module (optional) or broker module (optional)
ping 192.168.88.10 -I wwan0
ping 192.168.6.21 -I wwan0

# run the mqtt 
ros2 run mqtt_python_client mqtt_python_node --ros-args --params-file ~/ros2_ws/src/mqtt_python_client/config/mqtt_config.yaml 

# run the udp client 
ros2 run udp_server udp_server

##################################  Other useful commands  ###################################################

# changing subnetmask, (e.g. changing 29 to 28 in module 192.168.6.23)
sudo ip addr del 192.168.6.23/29 dev wwan0
sudo ip addr add 192.168.6.23/28 dev wwan0

# dial with SIMCOM driver in case it is not dialing. (this should be work automattically since start-up) 
cd home/ubuntu/SIM8200_for_RPI/Goonline
sudo ./simcom-cm
 

# compile udp_server after making changes in code :
cd ~/ros2_ws/
colcon build
source ~/.bashrc

# check if there are ros2 topics running or listen to a topic (e.g. mqtt2ros/position)
ros2 topic list
ros2 topic echo /mqtt2ros/position

#edit our mqtt code:
cd src/mqtt_python_client/mqtt_python_client/
sudo nano mqtt_python_node.py
# (to save and exit ctrl+x. In SSH session, to select tex and copy use the mouse, not the keyboard.)
# ( to copy or edit files from windows, install mobaxterm in windows)

# to check stauts of mosquitto.service or rc.local
sudo systemctl status mosquitto.service

# check if mosquitto version
mosquitto -h

# change hostname:
sudo hostnamectl set-hostname 5GmoduleNN # select NN as per in the excel file, every 192.168.6.<NN> 5G SIM card is asociated to a hostname 
# if the hostname keep saying raspberrypi try "preserve_hostname: true" in /etc/cloud/cloud.cfg 

 
# Check if the module is receiving data using MQTT. Using a PC with simulink and ethernet connection. Use the file udp_sending_through_ethernet or the 
# file in the lab. Send a costant value and run the udp_server, logs will show the value. Also open 
# another terminal and see the constant value with 
ros2 topic echo /ros2mqtt/position 



################################# tasks to be added in start-up process for all the modules yet####################:
# 1 change the subnet mask to 28
# 2 start the nodes mqtt_python_node and udp_server
# 3 start the simcom-cm

# remember to check that all the modules have the same baschrc

# some pahts of interest:
#ros2_ws/src/mqtt_python_client/config/mqtt_config.yaml
# /etc/hosts  
# home/ubuntu/SIM8200_for_RPI/Goonline

#################################  last version (April 2025) of udp_node.py  ####################
'''
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String  # Import message types
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
        self.sock.sendto("hola erick".encode(), (self.destination_ip, self.destination_port))

        # ROS 2 Publisher
        self.data_publisher = self.create_publisher(String, '/ros2mqtt/position', 10)
        #self.string_publisher = self.create_publisher(String, '/ros2mqtt/position', 10)

        # Start receiving messages
        self.timer = self.create_timer(.05, self.receive_message)  # Check every 50s

    def receive_message(self):
        try:
            data, addr = self.sock.recvfrom(4)# here it was 1024, it think acts as buffer when we receive smaller packeges
            self.get_logger().info(f"Received raw data from {addr}: {data}")

            # Publish raw message as a String
            #string_msg = String()
            #string_msg.data = f"From {addr}: {data.decode(errors='ignore')}"
            #self.string_publisher.publish(string_msg)

            # Try to interpret data as a float
            try:
                received_float = struct.unpack('f', data)[0]
                float_msg = Float32()
                string_msg = String()
                float_msg = received_float
                string_msg.data = ("UDP received: " + str(received_float))
                self.data_publisher.publish(string_msg)
                self.get_logger().info(f"msg Published to ros2mqtt/position: {received_float}")
            except struct.error:
                self.get_logger().warn(f"Invalid float data from {addr}: {data}")

        except Exception as e:
            self.get_logger().error(f"Error receiving data: {e}")

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
'''


######################################## rc.local ########################################
# (05.05.2025) in the module that has the MQTT broker  you find the startup inicialization file etc/rc.local with this:

'''
#!/bin/sh -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.

# Your custom command or script
# sudo ip link set wwan0 up &

> /var/log/rc.local.log
echo "start of rc.local">> /var/log/rc.local.log

sudo /home/ubuntu/SIM8200_for_RPI/Goonline/simcom-cm &
sleep 1

echo "waiting for wwan0 to be ready...">> /var/log/rc.local.log
while ! ip link show wwan0 >/dev/null 2>&1; do sleep 2; done

IP=$(ip -4 addr show wwan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}/\d+' || echo"")
[ -n "$IP" ] && [ "$(echo "$IP" | cut -d'/' -f2)" != "28" ] && sudo ip addr del  "$IP" dev wwan0
sleep 5
echo "IP deleted in case the subnetmask is not 28">>/var/log/rc.local.log
sudo ip addr add 192.168.6.21/28 dev wwan0

echo "subnetmask 28 added">>/var/log/rc.local.log

sleep 3

echo "despues de bashrc">> /var/log/rc.local.log
source /opt/ros/foxy/setup.bash
echo "despues de foxy setup bash">> /var/log/rc.local.log
source /home/$USER/ros2_ws/install/setup.bash
echo "despues de ros2ws install stup bash">> /var/log/rc.local.log

sleep 1
echo "starting ros nodes: udp and mqtt">> /var/log/rc.local.log
ros2 run udp_server udp_server1 &
ros2 run mqtt_python_client mqtt_python_node --ros-args --params-file ~/ros2_ws/src/mqtt_python_client/config/mqtt_conf>
exit 0
'''

########################################  TROUBLESHOOTING ########################################
#note. if wwan0 doesnt appear, usb connector mght be loose
# reconnect usb cable and then reboot or run manually ./simcom-rc  


