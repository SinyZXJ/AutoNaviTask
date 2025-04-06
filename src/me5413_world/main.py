#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations
import re
import os
import sys
from collections import defaultdict
import subprocess

def publish_nav_goal(x, y, yaw_deg):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.init_node('nav_goal_sender', anonymous=True)
    rospy.sleep(1.0)  # Wait for RViz subscriber to connect

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0

    # Convert yaw angle (degrees) to quaternion (rotation around Z-axis)
    yaw_rad = yaw_deg * 3.1415926 / 180.0
    q = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]

    rospy.loginfo(f"🚀 Published navigation goal: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}°")
    print(f"Published goal: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}°")
    pub.publish(goal)
    rospy.sleep(1.0)  # Ensure message is published


if __name__ == "__main__":
    # Step 1: Move to the first goal
    rospy.sleep(1.0)  # Wait for RViz subscriber
    publish_nav_goal(x=20.0, y=-22, yaw_deg=180)

    # Step 2: Visit locations 2 to 6 sequentially
    rospy.sleep(80)
    publish_nav_goal(x=19, y=-14, yaw_deg=180)
    rospy.sleep(20)
    publish_nav_goal(x=19, y=-8, yaw_deg=180)
    rospy.sleep(20)
    publish_nav_goal(x=19, y=-2, yaw_deg=180)
    rospy.sleep(20)
    publish_nav_goal(x=13, y=-2, yaw_deg=270)
    rospy.sleep(10)
    publish_nav_goal(x=9, y=-2, yaw_deg=270)

    # Step 3 and 4: Execute Control.py
    control_path = os.path.join(os.path.dirname(__file__),"bridge/scripts/Control.py")
    try:
        print(f"Running control script: {control_path}")
        subprocess.run(["python3", control_path], check=True)
    except subprocess.CalledProcessError as e:
        rospy.logwarn(f"Failed to run Control.py: {e}")
        sys.exit(1)  # Exit on failure
    
    rospy.sleep(30)

    # Step 5: Execute final navigation
    navigation_final_path = os.path.join(os.path.dirname(__file__),"navigation_final.py")
    try:
        print(f"Running final navigation script: {navigation_final_path}")
        subprocess.run(["python3", navigation_final_path], check=True)
    except subprocess.CalledProcessError as e:
        rospy.logwarn(f"Failed to run navigation_final.py: {e}")
        sys.exit(1)

    # Optional extra delay to ensure script completion
    rospy.sleep(1.0)
    rospy.sleep(1.0)
    rospy.sleep(1.0)

############
'''
Navigation Sequence:
1:   (x=20.0, y=-22, yaw_deg=180)
2:   (x=19, y=-14, yaw_deg=180)
3:   (x=19, y=-8, yaw_deg=180)
4:   (x=19, y=-2, yaw_deg=180)
5:   (x=13, y=-2, yaw_deg=270)
6:   (x=9, y=-2, yaw_deg=270)
'''
