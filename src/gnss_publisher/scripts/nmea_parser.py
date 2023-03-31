#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix

                
   


print("hi")
rospy.init_node('nmea_reader')
rospy.loginfo("hi")
nmea_pub = rospy.Publisher('nmea_data', NavSatFix, queue_size=10)

serial_port = '/dev/ttyACM0'
baud_rate = 38400
rate = rospy.Rate(10)

with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
    while True:
        try:
            line = ser.readline().decode('ascii', errors='replace')
            if line.startswith('$GP') or line.startswith('$GN'):
                try:
                    msg = pynmea2.parse(line)
                    
                    if isinstance(msg, pynmea2.types.talker.GGA):
                        #print(msg.latitude)
                        navsat_msg = NavSatFix()
                        navsat_msg.latitude = msg.latitude
                        navsat_msg.longitude = msg.longitude

                        nmea_pub.publish(navsat_msg)
                except pynmea2.ParseError as e:
                    print(f"Parse error: {e}") # Leep or change

        except KeyboardInterrupt:
            print("hi")
            pass