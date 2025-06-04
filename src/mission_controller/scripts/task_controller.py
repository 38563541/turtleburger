#!/usr/bin/env python
import rospy
import subprocess
import signal
from std_msgs.msg import String  # 根據你 Arduino topic 的型別調整

class TaskController:
    def __init__(self):
        rospy.init_node('task_controller')

        # 啟動 A launch
        rospy.loginfo("Launching A (lane)...")
        self.lane_process = subprocess.Popen(['roslaunch', 'lane_pkg', 'lane.launch'])

        rospy.Subscriber("/message/ArduinoToROS", String, self.msg_callback)
        self.transitioned = False

    def msg_callback(self, msg):
        if not self.transitioned and msg.data == "GO_NAV":  # 你要比對的訊息內容
            rospy.loginfo("Received GO_NAV, switching to navigation task...")

            # 結束 A 階段
            self.lane_process.send_signal(signal.SIGINT)
            self.lane_process.wait()

            # 啟動 B 階段
            rospy.loginfo("Launching B (navigation)...")
            self.nav_process = subprocess.Popen(['roslaunch', 'navigation', 'navigation.launch'])

            self.transitioned = True

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        TaskController().spin()
    except rospy.ROSInterruptException:
        pass
