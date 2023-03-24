#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix

def nmea_reader():
    rospy.init_node('nmea_reader', anonymous=True)
    nmea_pub = rospy.Publisher('nmea_data', NavSatFix, queue_size=10)

    serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
    baud_rate = rospy.get_param('~baud_rate', 38400)


    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            while not rospy.is_shutdown():
                data = ser.readline().decode('ascii', errors='replace')
                if data.startswith('$'):
                    try:
                        msg = pynmea2.parse(data)
                        rospy.loginfo(msg)
                        if isinstance(msg, pynmea2.types.talker.GGA):
                            print(msg.latitude)
                            navsat_msg = NavSatFix()
                            navsat_msg.latitude = msg.latitude
                            navsat_msg.longitude = msg.longitude

                            nmea_pub.publish(navsat_msg)
                        
                    except pynmea2.ParseError as e:
                        rospy.logwarn("NMEA parsing error: %s", e)
    except serial.SerialException as e:
        rospy.logerr("Serial error: %s", e)

if __name__ == 'main':
    try:
        nmea_reader()
    except rospy.ROSInterruptException:
        pass