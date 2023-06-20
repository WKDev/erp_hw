# send comands to erp

#!/usr/bin/env python3
#kroad-chson 230310

import rospy
import serial
import struct
import time, os
import binascii
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from pyfiglet import Figlet
import math
from erp42_msgs.msg import SerialFeedBack

# font examples are here : http://www.figlet.org/examples.html
f = Figlet(font='larry3d')


# port = rospy.get_param('port','/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0')
port = rospy.get_param('port',"/dev/serial/by-path/platform-2300000.pci-pci-0000:00:03.0-usb-0:2.13:1.0-port0")

# port = rospy.get_param('port','/dev/ttyUSB0')
baud = rospy.get_param('baud','115200')

# Initialize the node
rospy.init_node('pyERP_driver')
# rospy.init_node('imu_stat_publisher')

# Set up the IMU message
erp_msg = SerialFeedBack()
# erp_pub = rospy.Subscriber('erp42_serial', SerialFeedBack, cmdCallback)

    


# Main loop
while not rospy.is_shutdown():
    # Set up the serial port and message publisher
    ser = None

    try:
        rospy.loginfo('pyERP_driver')
        ser = serial.Serial(port, baud)
        time.sleep(0.5)
        i = 0
        j = 0
        while not rospy.is_shutdown():
            # Read data from the serial port
            # rospy.loginfo_once('trying to connect erp')
            # rospy.spin()
            barray=b''

            a_or_m = 1
            e_stop = 0
            gear = 1 # 0 for d, 1 for n , 2 for r 
            speed = 0
            steer = int(2000*math.sin(0.05*j))
            i = i+1
            j = j+1
            brake = 200
            alive = i
            if i == 255:
                i = 0

            # barray+=bytes.fromhex(hex(53))
            # barray+=bytes.fromhex(hex(54))
            # barray+=bytes.fromhex(hex(58))
            barray+= 0x53.to_bytes(1, byteorder='big')
            barray+= 0x54.to_bytes(1, byteorder='big')
            barray+= 0x58.to_bytes(1, byteorder='big')
            barray+= a_or_m.to_bytes(1, byteorder='big')
            barray+= e_stop.to_bytes(1, byteorder='big')
            barray+= gear.to_bytes(1, byteorder='big')
            barray+= speed.to_bytes(2, byteorder='big')
            barray+= steer.to_bytes(2, byteorder='big',signed=True)
            barray+= brake.to_bytes(1, byteorder='big')
            barray+= alive.to_bytes(1, byteorder='big')
            barray+= 0x0d.to_bytes(1, byteorder='big')
            barray+= 0x0a.to_bytes(1, byteorder='big')

            print(barray)

            ser.write(barray)

            rospy.sleep(0.01)


            




            


    except serial.SerialException as e:
        # rospy.logwarn(f'Unable to connect to serial port:{port}, baud:{baud}, retrying...')
        rospy.logwarn(e)
        time.sleep(1.0)

