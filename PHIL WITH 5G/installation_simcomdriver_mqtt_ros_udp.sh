"""
this is a guideline for installing everything step by step (Dont execute this file).
Project 5G connected sensing.
Instructions to install:
  - ubuntu20.04
  - SIMCOM8200 driver
  - MQTT
  - ROS foxy
  - ROS nodes for the application: mqtt and udp 
"""

##################################  OPERATING SISTEM INSTALLATION  ###################################################
# use SD card with raspi imager. Install ubuntu20.04 64bits. Select username ubuntu and Password Nr12345$, install sdcard
# and for the first time wait!!!! until we see SSH KEYS messages login and add to  /etc/netplan/50-cloud-init.yaml the following:

'''
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 172.16.11.11/24  # Static IP, change accorging to "SIMs" excel file
      gateway4: 172.16.11.1  
      nameservers:
        addresses:
          - 8.8.8.8
          - 8.8.4.4
  wifis:
    wlan0:
      optional: true
      dhcp4: true
      access-points:
        "Your_WiFi_SSID": # use your own phone hotspot for easy
          password: "Your_WiFi_Password"

'''
# note, if WiFI is not working can follow this suggestion of David Taylor: https://teams.microsoft.com/l/meetup-join/19%3ameeting_ZTEwY2JhMDktZWQ4YS00ZDE2LTgwY2EtNTNjODhlNzI2NWZh%40thread.v2/0?context=%7b%22Tid%22%3a%22f930300c-c97d-4019-be03-add650a171c4%22%2c%22Oid%22%3a%22808412e4-fd6e-4f8c-8c57-6bf5d4722254%22%7d
# change hostname:
sudo hostnamectl set-hostname 5GmoduleNN # select NN as per in the excel file, every 192.168.6.<NN> 5G SIM card is asociated to a hostname 
# if the hostname keep saying raspberrypi try "preserve_hostname: true" in /etc/cloud/cloud.cfg 
##################################  SIMCOM8200 DRIVER INSTALLATION  ###################################################

sudo apt-get update

sudo reboot

sudo apt-get update
sudo apt-get install net-tools
sudo apt-get install build-essential
sudo apt-get install p7zip-full
wget https://files.waveshare.com/upload/8/89/SIM8200_for_RPI.7z
7z x SIM8200_for_RPI.7z  -r -o./SIM8200_for_RPI

#in the SIM8200_for_RPI folder, 
#replace install.sh with  the below script for compatibility with ubuntu 20.04 (focal fossa)
#reason we use this because focal fossa has compatibility with both 32-bit and 64-bit arch
'''
#!/bin/bash

# Define the kernel headers directory
linuxheaders="linux-headers-"
uname_r=$(uname -r)
headerdir="/usr/src/$linuxheaders$uname_r"
echo "$linuxheaders$uname_r"

# Install kernel headers for Ubuntu 20.04
if [ ! -d "$headerdir" ]; then
  echo "Kernel headers not found, installing..."
  sudo apt-get update
  sudo apt-get install -y linux-headers-$(uname -r)
else
  echo "Kernel headers directory found at: $headerdir"
fi
  
cd option
make
mv /lib/modules/$(uname -r)/kernel/drivers/usb/serial/option.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial/option_bk.ko
cp option.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial/
cd ..

cd qmi_wwan_simcom/
make
cp qmi_wwan_simcom.ko /lib/modules/$(uname -r)/kernel/drivers/net/usb
cd ..

depmod
modprobe option
modprobe qmi_wwan_simcom
dmesg | grep "ttyUSB"
dmesg | grep "qmi_wwan_simcom"

# DNS file
mkdir -p /usr/share/udhcpc
sudo chmod 777 default.script
sudo cp default.script /usr/share/udhcpc
'''

cd SIM8200_for_RPI #go to this directory
chmod +x install.sh
sudo ./install.sh #install the script

sudo apt-get install minicom #install minicom

#for the next step ssh can not be used because the wifi will stop working, so monitor necessary
#now connect the Raspi4 and the SIMCOM8200 module using the USB cable or the connector; open the minicom window
sudo minicom -D /dev/ttyUSB2 #open the minicom window

#in te minicom window, reset the device to 9001 mode for the device to use the NDIS dial-up method; 9011 uses the RNDIS dial-up method
at+cusbcfg=usbid,1e0e,9001
#press ctrl+a then press q, then select YES to leave without reset and exit the minicom window


cd SIM8200_for_RPI  #go to this directory
cd Goonline
make
sudo ./simcom-cm #this will do the NDIS dial-up

###################### Run the dial-up method during the start-up  ###################################################

#Create the /etc/rc.local File:

sudo nano /etc/rc.local

#	copy the following so the file should look like this:

'''
#!/bin/sh -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.

# By default, this script does nothing.

# Your custom command or script
sudo /home/ubuntu/SIM8200_for_RPI/Goonline/simcom-cm &

exit 0

'''
#Save and Exit

#Make the /etc/rc.local File Executable:
sudo chmod +x /etc/rc.local

sudo reboot
#test . verify that wwan0 is there with the correct ip
ip a

#note. if wwan0 doesnt appear, usb connector mght be loose
# reconnect usb cable and then reboot or run manually ./simcom-rc  


##################################  ROS2 FOXY INSTALLATION  ###################################################
#install ros2 
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html #install the server version not the desktop or folllow the below steps:

locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verifiy for UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
#sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

#sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


sudo apt update
sudo apt upgrade

#sudo apt install ros-foxy-ros-base -y
sudo apt install ros-foxy-ros-base python3-argcomplete
sudo apt install ros-dev-tools

source /opt/ros/foxy/setup.bash

#once after installing the ros foxy use the below commands to automatically source ros2 foxy for every session,
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# at this moment ros2 run demo_nodes_cpp talker and ros2 run demo_nodes_py listener can be checked
## Note: if needed check the ROS DOMAIN is different for each module (only when checked with wifi),
## to avoid ROS nodes to communicate directly without the mqtt broker interference

# test 1 ros2 intsallation
printenv | grep ROS
#output 
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=foxy

sudo reboot # onnce check the ubuntu OS version after reboot uname -r and lsb_release -a
#afterh reboot could it be that the 50-config-init.cfg changes and wifi doesnt work. Edit again the file.

##################################  MQTT CLIENT INSTALLATION  ###################################################

sudo ufw allow 7400:7600/udp

sudo apt-get install mosquitto mosquitto-clients
sudo apt install python3-pip #added
pip3 install paho-mqtt
pip show paho-mqtt # check version
pip install paho-mqtt==1.6.1 #if it installs the 2.0 version, we downgrade because of the compatibility issues, 1.6.1 is more stable version

#enable mqtt client
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
sudo systemctl status mosquitto

cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python mqtt_python_client #create a package

#configure the setup.py and package.xml files, 
#the directory is already in the BTU Cloud named as my_python_node.zip; refer this folder to configure the setup.py and package.xml files
#also create the config folder manually and add the mqtt_config.yaml as in the my_python_node.zip file. 

#inside  mqtt_config.yaml change the host to :
      host: 192.168.6.21  # Replace with your MQTT broker IP (127.0.0.1 is for local machine)

cd ~/ros2_ws
rm -rf build/ install/ log/ #use only if needed

cd ~/ros2_ws
colcon build 
source ~/ros2_ws/install/setup.bash
ros2 run mqtt_python_client mqtt_python_node --ros-args --params-file ~/ros2_ws/src/mqtt_python_client/config/mqtt_config.yaml #runs ros2 node with the specific parameter file


#The mqtt_python_client in /ros2_ws/ src ; directory structure should look like this:
    [mqtt_python_node/]
        package.xml
        setup.cfg
        setup.py
        [config/] #create the config folder manually
            mqtt_config.yaml #then create this file
        [mqtt_python_client/]
            mqtt_python_node.py
            __init__.py
        [resource/]
            mqtt_python_client
           



#configuring the mosquitto broker to allow anonymous access
cd 
cd.. #go to home
cd.. #again to go to system files

ls /etc/mosquitto/conf.d/ # locate this file, if it exists then create 01-allow-anonymous.conf file in this directory to allow the mqtt to listen to all interfaces

sudo nano /etc/mosquitto/conf.d/01-allow-anonymous.conf

#add these lines in nano /etc/mosquitto/conf.d/01-allow-anonymous.conf
'''

listener 1883 0.0.0.0  # Listen on all interfaces (allows remote connections)
allow_anonymous true   # Allow anonymous access

'''
# save the file (ctrl+x, Y , enter)

sudo systemctl restart mosquitto
sudo systemctl status mosquitto

# test 2
#communiction commands to check the mqtt client and ros2
mosquitto_pub -h 192.168.160.14 -t "UAV/Position" -m "Sri Ranga Sai Nakka"
#new terminal
mosquitto_sub -h 192.168.160.14 -t "UAV/Position" -v
ros2 topic echo /mqtt2ros/position
ros2 topic pub /ros2mqtt/position std_msgs/String "data: 'Hello ros2mqtt'"

##################################  UDP SCRIPTS  ###################################################
# create udp_server package (optionally copy-paste folder from zip file and do .py file executable)
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python udp_server

cd ~/ros2_ws/src/udp_server/udp_server
nano udp_node.py

# add the pkg in the setup.py:       'udp_server = udp_server.udp_server:main'
# copy the following to the file (change "172.16.11.10" acording to SIMS excel file ), 

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
        self.udp_ip = "172.16.11.10" # change according to SIMS excel file
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
        self.timer = self.create_timer(0.05, self.receive_message)  # Check every 50ms

    def receive_message(self):
        try:
            data, addr = self.sock.recvfrom(1024)
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
                string_msg.data = ("Voltage" + str(received_float))
                self.data_publisher.publish(string_msg)
                self.get_logger().info(f"Published float: {received_float}")
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

chmod +x ~/ros2_ws/src/udp_server/udp_server/udp_node.py

# modify bashrc, add source commands at the end of  bashrc file
sudo nano ~/.bashrc
'''
source /opt/ros/foxy/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/ros2_ws/install/setup.bash
'''
ros2 run udp_server udp_server #for broker it is the same

# Test 3. using a PC with simulink and ethernet connection. Use the file udp_sending_through_ethernet or the 
# file in the lab. Send a costant value and run the udp_server, logs will show the value. Also open 
# another terminal and see the constant value with 
ros2 topic echo /ros2mqtt/position 

## Note: Everytime the broker and client1 needs to adjust their ip accordingly

in broker (192.168.6.21):
sudo ip addr del 192.168.6.21/30 dev wwan0
sudo ip addr add 192.168.6.21/29 dev wwan0
sudo ip route add default via 192.168.6.20 dev wwan0

in client1 (192.168.6.22):
sudo ip addr del 192.168.6.22/30 dev wwan0
sudo ip addr add 192.168.6.22/29 dev wwan0
sudo ip route add default via 192.168.6.20 dev wwan0

# test 4 for new client. location: PHIL. 
# connect two 5g modules: new client and broker. Run udp and mqtt in both sides. 
# Send from simulink a value to broker (.21), see the value in new client:
ros2 topic echo /mqtt2ros/position 
