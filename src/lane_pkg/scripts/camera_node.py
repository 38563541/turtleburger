#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('camera_node')
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    rate = rospy.Rate(30)  # 30 FPS

    # 替換成你影片實際路徑
    cap = cv2.VideoCapture('/home/yourname/catkin_ws/src/lane_pkg/scripts/inputvideo.mp4')
    bridge = CvBridge()

    if not cap.isOpened():
        rospy.logerr("❌ Cannot open video file")
        return

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.loginfo("✅ End of video or failed to read frame")
            break

        # 轉成 ROS Image 訊息
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
