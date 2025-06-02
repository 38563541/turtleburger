#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import threading
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

bridge = CvBridge()
latest_frame = None
lock = threading.Lock()

pub_message = None
pub_image = None

# === 參數設定 ===
top_width_ratio = 1
bottom_width_ratio = 1.0
height_start_ratio = 0.65
canny_threshold1 = 50
canny_threshold2 = 150
hough_threshold = 30
min_line_length = 30
max_line_gap = 40

triangle_region_top = 0
triangle_region_bottom = 0.7
triangle_region_left = 0.2
triangle_region_right = 0.8

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, [vertices], 255)
    return cv2.bitwise_and(img, mask)

def is_isosceles_and_pointing(pts, side_ratio_tolerance=0.15, max_sharp_angle_deg=50):
    def dist(p1, p2):
        return np.linalg.norm(p1 - p2)

    a = dist(pts[0], pts[1])
    b = dist(pts[1], pts[2])
    c = dist(pts[2], pts[0])

    is_isosceles = (
        abs(a - b) / max(a, b) < side_ratio_tolerance or
        abs(b - c) / max(b, c) < side_ratio_tolerance or
        abs(c - a) / max(c, a) < side_ratio_tolerance
    )
    if not is_isosceles:
        return False, None

    def angle(p1, p2, p3):
        v1 = p1 - p2
        v2 = p3 - p2
        cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        return np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))

    angles = [
        angle(pts[1], pts[0], pts[2]),
        angle(pts[0], pts[1], pts[2]),
        angle(pts[0], pts[2], pts[1]),
    ]

    if any(ang >= 90 for ang in angles):
        return False, None
    if all(ang > max_sharp_angle_deg for ang in angles):
        return False, None

    min_idx = np.argmin(angles)
    tip = pts[min_idx]
    return True, tip

def detect_triangles(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

    height, width = bw.shape
    x1 = int(width * triangle_region_left)
    x2 = int(width * triangle_region_right)
    y1 = int(height * triangle_region_top)
    y2 = int(height * triangle_region_bottom)

    roi_mask = np.zeros_like(bw)
    roi_mask[y1:y2, x1:x2] = 255
    roi_bw = cv2.bitwise_and(bw, roi_mask)

    kernel = np.ones((3,3), np.uint8)
    roi_bw = cv2.morphologyEx(roi_bw, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(roi_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detected = []
    state = 0

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        if len(approx) == 3 and cv2.contourArea(cnt) > 3000:
            pts = approx[:, 0, :]
            state = cv2.contourArea(cnt)

            mask = np.zeros_like(gray)
            cv2.drawContours(mask, [cnt], 0, 255, 2)
            surrounding = cv2.bitwise_and(gray, mask)
            surrounding_mean = np.mean(surrounding[surrounding > 0])

            if surrounding_mean > 54:
                is_iso, tip = is_isosceles_and_pointing(pts, side_ratio_tolerance=0.1, max_sharp_angle_deg=50)
                if not is_iso:
                    continue

                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                direction = "Left" if tip[0] < cx else "Right"

                detected.append({
                    "contour": approx,
                    "cx": cx,
                    "cy": cy,
                    "dir": direction,
                    "tip": tuple(tip)
                })
    return detected, (x1, y1, x2, y2), state

def detect_lanes(frame, state):
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray_frame, 50, 255, cv2.THRESH_BINARY_INV)
    edges = cv2.Canny(bw, canny_threshold1, canny_threshold2)

    y_start = int(frame.shape[0] * height_start_ratio)
    y_end = frame.shape[0]
    top_width = int(frame.shape[1] * top_width_ratio)
    bottom_width = int(frame.shape[1] * bottom_width_ratio)

    trap_pts = np.array([
        [(frame.shape[1] - top_width) // 2, y_start],
        [(frame.shape[1] + top_width) // 2, y_start],
        [frame.shape[1], y_end],
        [0, y_end]
    ])

    masked_edges = region_of_interest(edges, trap_pts)

    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=hough_threshold,
                             minLineLength=min_line_length, maxLineGap=max_line_gap)

    left_points, right_points = [], []
    center = frame.shape[1] // 2
    for line in lines if lines is not None else []:
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else np.inf
        x_avg = (x1 + x2) / 2
        if slope < -0.45 and x_avg < center:
            left_points += [(x1, y1), (x2, y2)]
        elif slope > 0.45 and x_avg > center:
            right_points += [(x1, y1), (x2, y2)]

    results = {}
    y_mid = 0.5 * y_start + 0.5 * y_end

    if left_points:
        left_pts = np.array(left_points)
        left_fit = np.polyfit(left_pts[:, 1], left_pts[:, 0], 1)
        results['left_fit'] = left_fit
    if right_points:
        right_pts = np.array(right_points)
        right_fit = np.polyfit(right_pts[:, 1], right_pts[:, 0], 1)
        results['right_fit'] = right_fit

    if 'left_fit' in results and 'right_fit' in results:
        results['lane_center'] = (results['left_fit'][0]*y_mid + results['left_fit'][1] + results['right_fit'][0]*y_mid + results['right_fit'][1]) / 2
    elif 'left_fit' in results:
        results['lane_center'] = (results['left_fit'][0]*y_mid + results['left_fit'][1])*0.58 + 0.42*frame.shape[1]
    elif 'right_fit' in results:
        results['lane_center'] = (results['right_fit'][0]*y_mid + results['right_fit'][1])*0.55

    return results

def draw_all(frame, triangle_info, triangle_roi, lane_info):
    x1, y1, x2, y2 = triangle_roi
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 2)

    for tri in triangle_info:
        cv2.drawContours(frame, [tri['contour']], 0, (0, 255, 255), 3)
        cv2.putText(frame, tri['dir'], (tri['cx'] - 30, tri['cy']), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    y_start = int(frame.shape[0] * height_start_ratio)
    y_end = frame.shape[0]

    if 'left_fit' in lane_info:
        left_fit = lane_info['left_fit']
        x1 = int(left_fit[0] * y_start + left_fit[1])
        x2 = int(left_fit[0] * y_end + left_fit[1])
        cv2.line(frame, (x1, y_start), (x2, y_end), (255, 0, 0), 2)  # 藍色左線

    if 'right_fit' in lane_info:
        right_fit = lane_info['right_fit']
        x1 = int(right_fit[0] * y_start + right_fit[1])
        x2 = int(right_fit[0] * y_end + right_fit[1])
        cv2.line(frame, (x1, y_start), (x2, y_end), (0, 0, 255), 2)  # 紅色右線

    if 'lane_center' in lane_info:
        offset = lane_info['lane_center'] - (frame.shape[1] // 2)
        cv2.line(frame, (int(lane_info['lane_center']), 0), (int(lane_info['lane_center']), frame.shape[0]), (0, 255, 0), 2)
        cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (0, 255, 255), 2)
        cv2.putText(frame, f'Offset: {offset:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        direction_char = 'X'
        if triangle_info:
            first_triangle_dir = triangle_info[0].get('dir') 
            if first_triangle_dir == "Left":
                direction_char = 'L'
            elif first_triangle_dir == "Right":
                direction_char = 'R'

        message_to_send = f"N{int(offset)}T{direction_char}"
        pub_message.publish(message_to_send)
    else:
        cv2.putText(frame, 'No lanes detected', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    directions = [tri['dir'] for tri in triangle_info] if triangle_info else ['Not Detected']
    cv2.putText(frame, f'Triangle(s): {", ".join(directions)}', (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 100, 0), 2)
    return frame


def image_callback(msg):
    global latest_frame
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with lock:
            latest_frame = frame
    except Exception as e:
        rospy.logerr(f"CV bridge error: {e}")

def main():
    global pub_message, pub_image
    rospy.init_node('lane_triangle_node_opt', anonymous=True)
    pub_message = rospy.Publisher('/message/ArduinoROS', String, queue_size=10)
    pub_image = rospy.Publisher('/processed_image', Image, queue_size=10)
    rospy.Subscriber('/camera/image_raw', Image, image_callback)
    rospy.loginfo("Optimized Lane & Triangle Detection Node Started")

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        frame = None
        with lock:
            if latest_frame is not None:
                frame = latest_frame.copy()

        if frame is not None:
            triangles, tri_roi, state = detect_triangles(frame)
            lanes = detect_lanes(frame, state)
            output_frame = draw_all(frame, triangles, tri_roi, lanes)
            image_msg = bridge.cv2_to_imgmsg(output_frame, encoding='bgr8')
            pub_image.publish(image_msg)
            cv2.imshow("Processed Frame", output_frame)
            cv2.waitKey(1)

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()