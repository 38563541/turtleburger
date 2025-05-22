#!/usr/bin/env python3
import rospy
import sys
import select
import termios
import tty
import serial
from std_msgs.msg import String  # 如果你要用 ROS topic

# 鍵盤初始化
settings = termios.tcgetattr(sys.stdin)

def get_key(timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node('keyboard_controller')
    
    # 設定 ROS publisher（如果不想用可以註解掉）
    pub = rospy.Publisher('/car_command', String, queue_size=10)

    # 設定 Serial 連線（根據你的裝置改 /dev/ttyUSB0 或 ttyACM0）
    try:
        ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # <- 確保你是用這個 port
        rospy.loginfo("已連線到 Arduino")
    except serial.SerialException as e:
        rospy.logerr(f"無法連接 Arduino: {e}")
        return

    rospy.loginfo("使用 WASD 控制小車，按 q 退出")

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key:
                if key in ['w', 'a', 's', 'd','x']:
                    rospy.loginfo(f"Pressed: {key}")
                    pub.publish(key)        # 傳 ROS topic
                    ser.write(key.encode()) # 傳給 Arduino
                elif key == 'q':
                    key=='x'
                    pub.publish(key)        # 傳 ROS topic
                    ser.write(key.encode()) # 傳給 Arduino
                    rospy.loginfo("退出鍵盤控制")
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        ser.close()

if __name__ == '__main__':
    main()
