#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('camera_node', anonymous=True)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    # 嘗試打開 webcam（預設為 /dev/video0）
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("無法開啟攝影機。請確認攝影機是否接好。")
        return

    rospy.loginfo("Camera Node 啟動，開始發送影像...")

    rate = rospy.Rate(30)  # 30 FPS

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("無法讀取影像幀，跳過此幀...")
            continue

        # 將 OpenCV 的 BGR 影像轉換為 ROS Image message
        ros_img = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_pub.publish(ros_img)

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
