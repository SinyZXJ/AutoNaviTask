
#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import yaml
import tf
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped


class Lidar2DLocalizer:
    def __init__(self):
        rospy.init_node("lidar_localizer_2d")

        # 获取参数
        pgm_path = rospy.get_param("~pgm_path", "./costmap.pgm")
        yaml_path = rospy.get_param("~yaml_path", "./costmap.yaml")

        self.map_pcd = self.load_map_as_pointcloud(pgm_path, yaml_path)
        self.last_pose = np.eye(4)

        self.br = tf.TransformBroadcaster()
        self.pose_pub = rospy.Publisher("/localization_pose", PoseStamped, queue_size=1)
        rospy.Subscriber("/mid/points", PointCloud2, self.lidar_callback)

        rospy.loginfo("✅ Lidar 2D Localizer started.")
        rospy.spin()

    def load_map_as_pointcloud(self, pgm_path, yaml_path):
        # 加载 .pgm 地图图像
        map_img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        map_img = cv2.flip(map_img, 0)  # ROS 坐标系为左下角原点

        # 读取 .yaml 元信息
        with open(yaml_path, 'r') as f:
            info = yaml.safe_load(f)
        resolution = info["resolution"]
        origin = info["origin"]  # [x, y, yaw]
        height, width = map_img.shape

        # 提取黑色障碍物像素点
        occupied = np.argwhere(map_img < 50)
        points = []
        for y_pix, x_pix in occupied:
            x = x_pix * resolution + origin[0]
            y = y_pix * resolution + origin[1]
            points.append([x, y, 0.0])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        pcd = pcd.voxel_down_sample(voxel_size=0.2)
        rospy.loginfo(f"地图点数: {len(pcd.points)}")
        return pcd

    def lidar_callback(self, msg):
        # 提取 ground 2D 点
        lidar_pts = np.array([
            [p[0], p[1], 0.0]
            for p in pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
            if abs(p[2]) > 0.5  # 可改参数
        ])
        if len(lidar_pts) < 100:
            rospy.logwarn("点数太少，跳过帧")
            return

        scan = o3d.geometry.PointCloud()
        scan.points = o3d.utility.Vector3dVector(lidar_pts)
        scan = scan.voxel_down_sample(voxel_size=0.2)

        # ICP 匹配
        reg = o3d.pipelines.registration.registration_icp(
            scan, self.map_pcd, max_correspondence_distance=1.0,
            init=self.last_pose,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        self.last_pose = reg.transformation
        self.publish_pose(msg.header.stamp, reg.transformation)

    def publish_pose(self, stamp, transform):
        x = transform[0, 3]
        y = transform[1, 3]
        yaw = np.arctan2(transform[1, 0], transform[0, 0])
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)

        self.br.sendTransform((x, y, 0), q, stamp, "base_link", "map")

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.pose_pub.publish(pose)


if __name__ == "__main__":
    try:
        Lidar2DLocalizer()
    except rospy.ROSInterruptException:
        pass
