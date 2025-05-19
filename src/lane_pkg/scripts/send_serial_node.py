#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import serial

class OffsetToSerial:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        rospy.Subscriber('/message/ArduinoROS', String, self.message_callback)

    def message_callback(self, msg):
        send_str = msg.data + "\n"  # 加上換行符號方便 Arduino 解析
        self.ser.write(send_str.encode('utf-8'))
        rospy.loginfo(f"Sent to Arduino: {send_str.strip()}")

if __name__ == '__main__':
    rospy.init_node('offset_serial_node')
    try:
        OffsetToSerial()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
