#!/usr/bin/env python
import rospy
import cv2
import time
import numpy as np
import easyocr
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from collections import defaultdict
from datetime import datetime
from math import sqrt

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_msgs.msg import UInt8

import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

bridge = CvBridge()
img = None
cloud_data = None
box_pub = None
label_pub = None
reader = easyocr.Reader(['en'], gpu=True)
current_target_label = None
recognition_enabled = False  # Recognition is off by default
state_pub = None  # Publisher for recognition state

# Camera intrinsic matrix
K = np.array([[554.2547, 0.0, 320.5],
              [0.0, 554.2547, 256.5],
              [0.0, 0.0, 1.0]])

# LiDAR to camera coordinate transformation
translation = np.array([0, -0.105, -0.242])
quat = [0.500, -0.500, 0.500, 0.500]
rotation_matrix = R.from_quat(quat).as_matrix()

box_history = defaultdict(list)
DIST_THRESHOLD = 2  # meters

def target_label_callback(msg):
    global current_target_label
    current_target_label = msg.data
    rospy.loginfo(f"🎯 Received target label: {current_target_label}")

def image_callback(msg):
    global img
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

def lidar_callback(msg):
    global cloud_data
    cloud_data = msg

def transform_point(p_lidar):
    p = np.array(p_lidar).reshape((3, 1))
    p_cam = rotation_matrix @ p + translation.reshape((3, 1))
    return p_cam.flatten()

def project_point(X, Y, Z):
    if Z <= 0:
        return None
    uv = K @ np.array([[X / Z], [Y / Z], [1]])
    u, v = int(uv[0][0]), int(uv[1][0])
    if 0 <= u < 640 and 0 <= v < 512:
        return (u, v)
    return None

def state_callback(msg):
    global recognition_enabled
    if msg.data == 2:
        recognition_enabled = False
        rospy.loginfo("🔴 Recognition disabled (state code 2 received)")
    elif msg.data == 1:
        recognition_enabled = True
        rospy.loginfo("🟢 Recognition enabled (state code 1 received)")

def detect_boxes(image):
    boxes = []
    seen = set()

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        area = w * h
        if area < 300 or h < 20 or w < 20 or h / w > 5 or w / h > 5 or area > 30000:
            continue

        roi = image[y:y + h, x:x + w]
        roi_resized = cv2.resize(roi, (100, 100))
        result = reader.readtext(roi_resized, detail=0)
        if result:
            text = result[0].strip()
            if not text.isdigit():
                continue

            center = (x + w // 2, y + h // 2)
            key = (text, center)
            if key in seen:
                continue
            seen.add(key)

            boxes.append({
                "polygon": np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]]),
                "label": text,
                "center": center,
                "bbox": (x, y, w, h)
            })

    return boxes

def euclidean(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def transform_xy_to_odom_from_lidar(xy_lidar, tf_buffer):
    point_stamped = geometry_msgs.msg.PointStamped()
    point_stamped.header.frame_id = "velodyne"  # ✅ Replace with your actual LiDAR frame ID
    point_stamped.header.stamp = rospy.Time.now()

    point_stamped.point.x = xy_lidar[0]
    point_stamped.point.y = xy_lidar[1]
    point_stamped.point.z = 0.0  # Only care about 2D position

    try:
        transform = tf_buffer.lookup_transform("odom", point_stamped.header.frame_id,
                                               rospy.Time.now(), rospy.Duration(0.5))
        transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        return np.array([transformed_point.point.x, transformed_point.point.y])
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
        rospy.logwarn(f"⚠️ Failed to get transform from {point_stamped.header.frame_id} to odom: {e}")
        return None

def periodic_match(event):
    global img, cloud_data, tf_buffer, recognition_enabled, target_detected
    global box_pub, label_pub, state_pub

    if not recognition_enabled:
        rospy.loginfo("🛑 Recognition is disabled")
        return
    if img is None or cloud_data is None:
        rospy.logwarn("Image or point cloud not ready")
        return

    t0 = time.time()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    image_vis = img.copy()

    t_ocr_start = time.time()
    boxes = detect_boxes(image_vis)
    t_ocr_end = time.time()

    box_points = defaultdict(list)
    new_labels = []
    new_label_coords = {}

    t_pc_start = time.time()
    for pt in point_cloud2.read_points(cloud_data, skip_nans=True, field_names=("x", "y", "z")):
        if pt[0] <= 0:
            continue

        cam_xyz = transform_point(pt)
        u_v = project_point(*cam_xyz)
        if u_v is None:
            continue

        u, v = u_v
        for box in boxes:
            if cv2.pointPolygonTest(box["polygon"], (u, v), False) >= 0:
                box_points[box["label"]].append(pt)
                cv2.circle(image_vis, (u, v), 2, (0, 255, 0), -1)
    t_pc_end = time.time()

    t_draw_start = time.time()

    filtered_boxes = []
    for label, pts in box_points.items():
        candidate_boxes = [b for b in boxes if b["label"] == label]
        if not candidate_boxes:
            continue
        def box_distance(box):
            arr = np.array(box_points[box["label"]])
            avg = np.mean(arr, axis=0)
            return avg[2]
        best_box = min(candidate_boxes, key=box_distance)
        filtered_boxes.append(best_box)

    for box in filtered_boxes:
        x, y, w, h = box["bbox"]
        label = box["label"]
        cv2.rectangle(image_vis, (x, y), (x + w, y + h), (255, 255, 0), 2)
        cv2.putText(image_vis, f"Box {label}", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    for label, pts in box_points.items():
        arr = np.array(pts)
        xy_avg = np.mean(arr[:, :2], axis=0)

        odom_xy = transform_xy_to_odom_from_lidar(xy_avg, tf_buffer)
        if odom_xy is None:
            continue

        existing = box_history[label]
        should_add = all(euclidean(odom_xy, prev) > DIST_THRESHOLD for prev in existing)

        if should_add:
            box_history[label].append(odom_xy)
            new_labels.append(label)
            new_label_coords[label] = odom_xy
            rospy.loginfo(f"📦 Box {label} ✅ New ODOM position (X,Y) = ({odom_xy[0]:.2f}, {odom_xy[1]:.2f})")

        if current_target_label is not None and label == current_target_label:
            target_detected = True
            recognition_enabled = False
            rospy.loginfo(f"🎯 Target label {current_target_label} detected. Stopping rotation and disabling recognition.")
            if state_pub:
                state_pub.publish(UInt8(data=2))  # ✅ State code 2 means target found

        else:
            rospy.loginfo(f"📦 Box {label} ⛔ Too close to previous detection, not added")

    for i, label in enumerate(new_labels):
        cv2.putText(image_vis, f"New Box: {label}", (10, 30 + 25 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    for label, poses in box_history.items():
        if not poses:
            continue
        latest_pose = poses[-1]
        pt = PointStamped()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = label
        pt.point.x = latest_pose[0]
        pt.point.y = latest_pose[1]
        pt.point.z = 0.0
        box_pub.publish(pt)

        rospy.loginfo(f"📤 Published box {label} position: ({pt.point.x:.2f}, {pt.point.y:.2f})")

    t_draw_end = time.time()
    t_total = time.time()
    rospy.loginfo(f"⏱️ Time | OCR: {t_ocr_end - t_ocr_start:.2f}s | PointCloud: {t_pc_end - t_pc_start:.2f}s | Drawing: {t_draw_end - t_draw_start:.2f}s | Total: {t_total - t0:.2f}s")

def run_node():
    global tf_buffer, tf_listener, recognition_time
    global box_pub, label_pub, target_pub, state_pub

    rospy.Subscriber("/front/image_raw", Image, image_callback)
    rospy.Subscriber("/mid/points", PointCloud2, lidar_callback)
    rospy.Subscriber("/recognition_state", UInt8, state_callback)
    rospy.Subscriber("/target_label", String, target_label_callback)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    box_pub = rospy.Publisher("/recognized_boxes", PointStamped, queue_size=10)
    state_pub = rospy.Publisher("/recognition_state", UInt8, queue_size=1)

    recognition_timer = rospy.Timer(rospy.Duration(1), periodic_match)
    rospy.loginfo("🟢 lidar_camera_easyocr recognition thread started")

__all__ = ['box_history']
