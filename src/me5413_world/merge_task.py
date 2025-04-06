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
import cv2

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import math

target_detected = False
recognition_enabled = True  # Recognition switch (enabled by default)
TARGET_LABEL = '8'
bridge = CvBridge()
img = None
cloud_data = None
box_pub = None
img_pub = None
label_pub = None
jackal_data = None
reader = easyocr.Reader(['en'], gpu=True)

# Camera intrinsic matrix
K = np.array([[554.2547, 0.0, 320.5],
              [0.0, 554.2547, 256.5],
              [0.0, 0.0, 1.0]])

# LiDAR to camera transformation
translation = np.array([0, -0.105, -0.242])
quat = [0.500, -0.500, 0.500, 0.500]
rotation_matrix = R.from_quat(quat).as_matrix()

box_history = defaultdict(list)
DIST_THRESHOLD = 2  # meters

def image_callback(msg):
    global img
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

def lidar_callback(msg):
    global cloud_data
    cloud_data = msg

def get_yaw_from_quaternion(q):
    """Calculate yaw (rotation around Z-axis) from quaternion"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def local_to_world(x_rel, y_rel, x_jackal, y_jackal, yaw):
    """Convert local coordinate to world frame"""
    x_world = x_jackal + x_rel * math.cos(yaw) - y_rel * math.sin(yaw)
    y_world = y_jackal + x_rel * math.sin(yaw) + y_rel * math.cos(yaw)
    return x_world, y_world

def lidar2world_callback(msg):
    global jackal_data
    try:
        idx = msg.name.index("jackal")
        pos = msg.pose[idx].position
        ori = msg.pose[idx].orientation
        x_jackal = pos.x
        y_jackal = pos.y
        yaw = get_yaw_from_quaternion(ori)
        jackal_data = [x_jackal, y_jackal, yaw]
    except Exception as e:
        rospy.logwarn(f"Failed to parse model_states: {e}")

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

def detect_boxes(image, min_confidence=0.5):
    boxes = []
    seen = set()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    result = reader.readtext(binary)

    for (coords, text, confi) in result:
        if not text:
            continue
        text = text.strip()

        if not text.isdigit() or not (1 <= int(text) <= 9) or confi < min_confidence:
            continue

        polygon = np.array(coords, dtype=np.int32)
        x_min, y_min = np.min(polygon, axis=0)
        x_max, y_max = np.max(polygon, axis=0)
        w, h = x_max - x_min, y_max - y_min
        center = ((x_min + x_max) // 2, (y_min + y_max) // 2)

        key = (text, center)
        if key in seen:
            continue
        seen.add(key)

        boxes.append({
            "polygon": polygon,
            "label": text,
            "center": center,
            "bbox": (x_min, y_min, w, h)
        })
    return boxes

def euclidean(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def transform_xy_to_odom_from_lidar(xy_lidar, tf_buffer):
    point_stamped = geometry_msgs.msg.PointStamped()
    point_stamped.header.frame_id = "velodyne"
    point_stamped.header.stamp = rospy.Time.now()

    point_stamped.point.x = xy_lidar[0]
    point_stamped.point.y = xy_lidar[1]
    point_stamped.point.z = 0.0

    try:
        transform = tf_buffer.lookup_transform("odom", point_stamped.header.frame_id,
                                               rospy.Time.now(), rospy.Duration(0.5))
        transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        return np.array([transformed_point.point.x, transformed_point.point.y])
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
        rospy.logwarn(f"⚠️ Transform from {point_stamped.header.frame_id} to odom failed: {e}")
        return None

def transform_xy_to_world_from_lidar(position):
    global jackal_data
    x_world, y_world = local_to_world(position[0], position[1], jackal_data[0], jackal_data[1], jackal_data[2])
    return np.array([x_world, y_world])

def estimate_block_center(points: np.ndarray, block_size: float = 0.8):
    center = np.mean(points, axis=0)
    points_centered = points - center
    cov = np.cov(points_centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    normal = eigvecs[:, np.argmin(eigvals)]
    to_robot = -center
    if np.dot(to_robot, normal) < 0:
        normal = -normal
    center_point = center + normal * (block_size / 2)
    return center_point[0:2]

def visualize_box_history(box_history, scale=20.0, canvas_size=800, save_path=None, show_origin=True):
    image = np.ones((canvas_size, canvas_size, 3), dtype=np.uint8) * 255
    rng = np.random.default_rng(seed=42)

    for label, positions in box_history.items():
        color = tuple(map(int, rng.integers(0, 255, size=3)))
        for pos in positions:
            x, y = pos
            cx = int(x * scale)
            cy = canvas_size - int(y * scale)
            cv2.circle(image, (cx, cy), 6, color, -1)
            cv2.putText(image, f"{label}", (cx + 6, cy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    if show_origin:
        cv2.circle(image, (0, canvas_size), 6, (0, 0, 255), -1)
        cv2.putText(image, "Origin", (10, canvas_size - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    if save_path:
        cv2.imwrite(save_path, image)

    return image

def periodic_match(event):
    global img, cloud_data, tf_buffer, recognition_enabled, target_detected
    global box_pub, label_pub, img_pub, jackal_data

    if not recognition_enabled:
        rospy.loginfo("🛑 Recognition disabled - timer skipped")
        return
    if img is None or cloud_data is None:
        rospy.logwarn("Image or point cloud not available yet")
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
        xy_avg = estimate_block_center(arr[:])
        odom_xy = transform_xy_to_world_from_lidar(xy_avg)
        if odom_xy is None:
            continue

        existing = box_history[label]
        should_add = all(euclidean(odom_xy, prev) > DIST_THRESHOLD for prev in existing)

        if should_add and xy_avg[0]**2 + xy_avg[1]**2 < 100:
            box_history[label].append(odom_xy)
            new_labels.append(label)
            new_label_coords[label] = odom_xy
            rospy.loginfo(f"📦 Box {label} ✅ New world position (X,Y) = ({odom_xy[0]:.2f}, {odom_xy[1]:.2f})")

    for i, label in enumerate(new_labels):
        cv2.putText(image_vis, f"New Box: {label}", (10, 30 + 25 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    print('box_history:', box_history)
    image_vis = visualize_box_history(box_history)
    msg = bridge.cv2_to_imgmsg(image_vis, encoding="bgr8")
    img_pub.publish(msg)

    t_draw_end = time.time()
    t_total = time.time()
    rospy.loginfo(f"⏱️ Time | OCR: {t_ocr_end - t_ocr_start:.2f}s | PointCloud: {t_pc_end - t_pc_start:.2f}s | Drawing: {t_draw_end - t_draw_start:.2f}s | Total: {t_total - t0:.2f}s")

def run_node():
    global tf_buffer, tf_listener, recognition_timer, recognition_enabled
    global box_pub, label_pub, img_pub

    rospy.Subscriber("/front/image_raw", Image, image_callback)
    rospy.Subscriber("/mid/points", PointCloud2, lidar_callback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, lidar2world_callback)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    box_pub = rospy.Publisher("/recognized_boxes", PointStamped, queue_size=10)
    img_pub = rospy.Publisher("/result/image", Image, queue_size=1)

    recognition_timer = rospy.Timer(rospy.Duration(1), periodic_match)
    rospy.loginfo("🟢 lidar_camera_easyocr recognition thread started")

__all__ = ['box_history', 'recognition_enabled', 'target_detected', 'TARGET_LABEL']

if __name__ == "__main__":
    rospy.init_node("jackal_rotate_and_detect")
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(1.0)
    run_node()
    rospy.sleep(2000.0)
