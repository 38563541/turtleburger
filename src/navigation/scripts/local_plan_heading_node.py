#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

class LocalPlanHeadingNode:
    def __init__(self):
        rospy.init_node("local_plan_heading_node")

        self.topic_name = rospy.get_param("~publish_topic", "/message/ArduinoROS")
        self.pub_message = rospy.Publisher(self.topic_name, String, queue_size=10)

        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.3)
        self.local_plan = []
        self.goal_reached = False

        rospy.Subscriber("/slam_out_pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/move_base/TrajectoryPlannerROS/local_plan", Path, self.local_plan_cb)

        rospy.loginfo("[local_plan_heading_node] Running, publishing to: %s", self.topic_name)
        rospy.spin()

    def local_plan_cb(self, msg):
        if not msg.poses:
            self.local_plan = []
        else:
            self.local_plan = msg.poses

    def pose_cb(self, msg):
        if not self.local_plan:
            return

        current = msg.pose
        # 用第一個或第二個路徑點作為目標方向
        if len(self.local_plan) > 1:
            target_pose = self.local_plan[1].pose
        else:
            target_pose = self.local_plan[0].pose

        dx = target_pose.position.x - current.position.x
        dy = target_pose.position.y - current.position.y
        distance = math.hypot(dx, dy)

        # 若與最後一個 local_plan 目標點距離小於容差，就停下來
        final_target = self.local_plan[-1].pose
        fx = final_target.position.x - current.position.x
        fy = final_target.position.y - current.position.y
        final_dist = math.hypot(fx, fy)

        if final_dist <= self.goal_tolerance:
            if not self.goal_reached:
                rospy.loginfo("\u2705 Reached final goal. Sending stop.")
                self.pub_message.publish("mode:s\n")
                self.goal_reached = True
            return
        else:
            self.goal_reached = False

        # 計算 heading error
        yaw_target = math.atan2(dy, dx)
        orientation_q = current.orientation
        _, _, yaw_current = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

        heading_error = yaw_target - yaw_current
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        command_str = f"mode:1,offset:{heading_error:.3f}\\n"
        self.pub_message.publish(command_str)

if __name__ == '__main__':
    try:
        LocalPlanHeadingNode()
    except rospy.ROSInterruptException:
        pass
