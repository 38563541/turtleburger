#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
from datetime import datetime
import rospkg

class VideoSaver:
    def __init__(self):
        rospy.init_node('video_saver', anonymous=True)
        self.bridge = CvBridge()

        # 獲取套件路徑，預設輸出至 lane_pkg/video/
        pkg_path = rospkg.RosPack().get_path('lane_pkg')
        default_output_dir = os.path.join(pkg_path, 'video')
        self.output_dir = rospy.get_param('~output_dir', default_output_dir)

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # 動態建立影片檔名
        time_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_filename = f"output_video_{time_str}.mp4"
        self.output_path = os.path.join(self.output_dir, output_filename)

        self.video_writer = None
        self.frame_width = None
        self.frame_height = None
        self.fps = rospy.get_param('~fps', 30)

        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        rospy.loginfo(f"[video_saver] Saving video to: {self.output_path}")
        rospy.spin()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", str(e))
            return

        if self.video_writer is None:
            self.frame_height, self.frame_width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(self.output_path, fourcc, self.fps,
                                                (self.frame_width, self.frame_height))

        self.video_writer.write(frame)

    def __del__(self):
        if self.video_writer:
            self.video_writer.release()
            rospy.loginfo("[video_saver] Video saved.")

if __name__ == '__main__':
    try:
        VideoSaver()
    except rospy.ROSInterruptException:
        pass
