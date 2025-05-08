#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class LaneDetectionNode:
    def __init__(self):
        rospy.init_node('lane_detection_node')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.offset_pub = rospy.Publisher('/lane/offset', Float32, queue_size=10)

        # 調參區
        self.top_width_ratio = 0.75
        self.bottom_width_ratio = 1.0
        self.height_start_ratio = 0.3
        self.canny_thresh1 = 50
        self.canny_thresh2 = 150
        self.hough_thresh = 30
        self.min_line_len = 30
        self.max_line_gap = 40

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        image_center = width // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, self.canny_thresh1, self.canny_thresh2)

        # ROI
        top_width = int(width * self.top_width_ratio)
        bottom_width = int(width * self.bottom_width_ratio)
        y_start = int(height * self.height_start_ratio)
        y_end = height
        trap_pts = np.array([
            [(width - top_width) // 2, y_start],
            [(width + top_width) // 2, y_start],
            [width, y_end],
            [0, y_end]
        ])
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, [trap_pts], 255)
        masked = cv2.bitwise_and(edges, mask)

        # 偵測線段
        lines = cv2.HoughLinesP(masked, 1, np.pi / 180, self.hough_thresh,
                                minLineLength=self.min_line_len, maxLineGap=self.max_line_gap)

        left_points = []
        right_points = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else np.inf
                x_avg = (x1 + x2) / 2

                if slope < -0.2 and x_avg < image_center:
                    left_points.extend([(x1, y1), (x2, y2)])
                elif slope > 0.2 and x_avg > image_center:
                    right_points.extend([(x1, y1), (x2, y2)])

        lane_center = None
        y_mid = (y_start + y_end) // 2

        if left_points:
            left_fit = np.polyfit([p[1] for p in left_points], [p[0] for p in left_points], 1)
            left_x = left_fit[0] * y_mid + left_fit[1]
            for y in np.linspace(y_start, y_end, 50):
                x = int(left_fit[0] * y + left_fit[1])
                cv2.circle(frame, (x, int(y)), 2, (255, 0, 0), -1)
        else:
            left_x = None

        if right_points:
            right_fit = np.polyfit([p[1] for p in right_points], [p[0] for p in right_points], 1)
            right_x = right_fit[0] * y_mid + right_fit[1]
            for y in np.linspace(y_start, y_end, 50):
                x = int(right_fit[0] * y + right_fit[1])
                cv2.circle(frame, (x, int(y)), 2, (0, 0, 255), -1)
        else:
            right_x = None

        # 決定 lane_center
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2
        elif left_x is not None:
            lane_center = (left_x + width) / 2
        elif right_x is not None:
            lane_center = right_x / 2

        # 發佈 offset
        if lane_center is not None:
            offset = float(lane_center - image_center)
            self.offset_pub.publish(Float32(offset))
            cv2.line(frame, (int(lane_center), 0), (int(lane_center), height), (0, 255, 0), 2)
            cv2.line(frame, (image_center, 0), (image_center, height), (0, 255, 255), 2)
            cv2.putText(frame, f'Offset: {offset:.2f}px', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(frame, 'No lanes detected', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 顯示影像
        cv2.polylines(frame, [trap_pts], isClosed=True, color=(0, 165, 255), thickness=2)
        cv2.imshow("Lane Detection", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        LaneDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
