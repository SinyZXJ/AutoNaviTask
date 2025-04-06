#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
import threading
import tf2_ros
import tf
from lidar_camera_easyocr import run_node  # Launch recognition node
from lidar_camera_easyocr import box_history
from std_msgs.msg import UInt8
from std_msgs.msg import String

target_detected_flag = False
min_digit = 1
recognition_timer = None

def recognition_state_callback(msg):
    global target_detected_flag
    if msg.data == 2:
        target_detected_flag = True
        rospy.loginfo("📩 Target recognized")


def move_to_target(pub, tf_buffer):
    TARGET_LABEL = min_digit
    rate = rospy.Rate(10)
    reached = False

    while not rospy.is_shutdown() and not reached:
        # ✅ Wait until target label is recognized
        if TARGET_LABEL not in box_history or len(box_history[TARGET_LABEL]) == 0:
            rospy.loginfo(f"⏳ Waiting for detection of box {TARGET_LABEL}...")
            rate.sleep()
            continue

        target = box_history[TARGET_LABEL][-1]
        base = get_base_link_pose(tf_buffer)
        if base is None:
            rate.sleep()
            continue

        # ✅ Calculate pose difference between current and target
        x, y, yaw = base
        dx = target[0] - x
        dy = target[1] - y
        dist = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(angle_to_target - yaw), math.cos(angle_to_target - yaw))

        cmd = Twist()

        # ✅ Control logic: prioritize rotation, then move forward
        if dist > 0.4:
            if abs(angle_diff) > math.radians(20):
                cmd.linear.x = 0.0
                cmd.angular.z = 1 * angle_diff
            else:
                cmd.linear.x = 1.2
                cmd.angular.z = 0.5 * angle_diff
        else:
            rospy.loginfo(f"✅ Arrived at box {TARGET_LABEL}, current position x={x:.2f}, y={y:.2f}")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            reached = True

        pub.publish(cmd)
        rate.sleep()

def get_base_link_pose(tf_buffer):
    try:
        trans = tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0), rospy.Duration(1.0))
        translation = trans.transform.translation
        rotation = trans.transform.rotation
        euler = tf.transformations.euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        rospy.loginfo(f"📍 base_link relative to odom: x={translation.x:.2f}, y={translation.y:.2f}, yaw={math.degrees(euler[2]):.2f}°")
        return translation.x, translation.y, euler[2]
    except Exception as e:
        rospy.logwarn(f"❌ Failed to get TF: {e}")
        return None


def rotate(pub, angle_deg, tf_buffer, angular_speed=0.5):
    global target_detected_flag, TARGET_LABEL  # Ensure these variables are available

    # 🔒 Check if the target is already detected
    if target_detected_flag:
        rospy.logwarn(f"❗Target label {TARGET_LABEL} already detected, skipping rotation")
        return

    twist = Twist()
    direction = 1 if angle_deg > 0 else -1
    twist.angular.z = direction * abs(angular_speed)
    angle_rad = math.radians(abs(angle_deg))
    duration = angle_rad / abs(angular_speed)

    rospy.loginfo(f"↪️ Start rotating {angle_deg}°, estimated duration: {duration:.2f}s")
    start_time = rospy.Time.now().to_sec()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start_time

        if target_detected_flag:
            rospy.loginfo(f"🛑 Target {TARGET_LABEL} detected, stopping rotation immediately")
            break
        if elapsed >= duration:
            break

        pub.publish(twist)
        get_base_link_pose(tf_buffer)  # Get and log current pose
        rate.sleep()

    # Stop rotating
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.loginfo("✅ Rotation stopped")

if __name__ == "__main__":
    rospy.init_node("jackal_rotate_and_detect")

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    target_label_pub = rospy.Publisher("/target_label", String, queue_size=1)
    recognition_state_pub = rospy.Publisher("/recognition_state", UInt8, queue_size=1)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber("/recognition_state", UInt8, recognition_state_callback)

    # Start recognition node in a separate thread
    threading.Thread(target=run_node, daemon=True).start()

    rospy.sleep(1.0)
    # Publish target label and enable recognition
    TARGET_LABEL = min_digit
    target_label_pub.publish(String(data=TARGET_LABEL))
    recognition_state_pub.publish(UInt8(data=1))  # Enable recognition

    rospy.sleep(2.0)
    
    # Rotate to search for target
    rotate(cmd_pub, 60, tf_buffer)
    rotate(cmd_pub, -120, tf_buffer)
    
    # Move toward target
    move_to_target(cmd_pub, tf_buffer)

    rospy.loginfo("✅ Rotation + recognition sequence completed")
