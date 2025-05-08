#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import serial

class OffsetToSerial:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        rospy.Subscriber('/lane/offset', Float32, self.offset_callback)

    def offset_callback(self, msg):
        offset = msg.data
        send_str = f"{offset:.2f}\n"
        self.ser.write(send_str.encode('utf-8'))
        rospy.loginfo(f"Sent to Arduino: {send_str.strip()}")

if __name__ == '__main__':
    rospy.init_node('offset_serial_node')
    try:
        OffsetToSerial()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
