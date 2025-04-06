#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse
import ros_numpy
import open3d as o3d
import numpy as np
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
import copy
import time


# ----------------------- 点云处理及配准函数 -----------------------

def filter(pcd, max_radius, z_min, z_max):
    """
    过滤点云，保留距离原点小于或等于 max_radius，
    且 z 坐标在 [z_min, z_max] 范围内的点。
    """
    points = np.asarray(pcd.points).copy()  # 显式复制以确保可写
    mask = (np.linalg.norm(points, axis=1) <= max_radius) & \
           (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    indices = np.where(mask)[0]
    return pcd.select_by_index(indices)


def extract_sector(pcd, center_angle, sector_width):
    """
    从全景点云 pcd 中提取以 center_angle 为中心、
    sector_width 为总宽度的候选扇区（单位：弧度）。
    """
    points = np.asarray(pcd.points).copy()  # 显式复制以确保可写
    angles = np.arctan2(points[:, 1], points[:, 0])
    half_width = sector_width / 2.0
    mask = (angles >= (center_angle - half_width)) & (angles <= (center_angle + half_width))
    indices = np.where(mask)[0]
    return pcd.select_by_index(indices)


def estimate_2d_transform(source, target):
    """
    估计源点云和目标点云之间的二维刚性变换，
    仅考虑 x,y 平面（旋转仅绕 z 轴，平移仅在 x,y 上）。
    输入：
      source, target：均为 Nx3 numpy 数组（仅用前两列）
    返回：
      T：4x4 的变换矩阵
      theta：估计得到的旋转角（弧度）
    """
    s_xy = np.array(source[:, :2])
    t_xy = np.array(target[:, :2])
    s_centroid = np.mean(s_xy, axis=0)
    t_centroid = np.mean(t_xy, axis=0)
    s_centered = s_xy - s_centroid
    t_centered = t_xy - t_centroid
    numerator = np.sum(s_centered[:, 0] * t_centered[:, 1] - s_centered[:, 1] * t_centered[:, 0])
    denominator = np.sum(s_centered[:, 0] * t_centered[:, 0] + s_centered[:, 1] * t_centered[:, 1])
    theta = np.arctan2(numerator, denominator)
    R2d = np.array([[np.cos(theta), -np.sin(theta)],
                    [np.sin(theta), np.cos(theta)]])
    t_xy_est = t_centroid - R2d @ s_centroid
    T = np.eye(4)
    T[:2, :2] = R2d
    T[0, 3] = t_xy_est[0]
    T[1, 3] = t_xy_est[1]
    return T, theta


def constrained_icp(source_points, target_points, max_iter, tolerance,
                    max_z_rotation, min_z_rotation):
    """
    自定义 ICP，只在 XY 平面内求解变换（仅允许绕 z 轴旋转和 x,y 平移）。
    在每次迭代中，对计算得到的绕 z 轴旋转角 theta 进行夹紧，
    确保其在 [min_z_rotation, max_z_rotation] 内。
    输入：
      source_points, target_points：Nx3 numpy 数组
    返回累计的 4x4 变换矩阵 T_total。
    """
    source = np.asarray(source_points).copy()  # 显式复制以确保可写
    T_total = np.eye(4)
    for i in range(max_iter):
        tree = KDTree(target_points)
        distances, indices = tree.query(source)
        indices = np.minimum(indices, target_points.shape[0] - 1)
        target_corr = np.array(target_points[indices])
        T_iter, theta = estimate_2d_transform(source, target_corr)

        if theta > max_z_rotation:
            theta = max_z_rotation
        elif theta < min_z_rotation:
            theta = min_z_rotation

        t_xy_est = T_iter[:2, 3]
        R2d_clamped = np.array([[np.cos(theta), -np.sin(theta)],
                                [np.sin(theta), np.cos(theta)]])
        T_iter = np.eye(4)
        T_iter[:2, :2] = R2d_clamped
        T_iter[0, 3] = t_xy_est[0]
        T_iter[1, 3] = t_xy_est[1]

        source_hom = np.hstack((source, np.ones((source.shape[0], 1))))
        source_new = (T_iter @ source_hom.T).T[:, :3]
        T_total = T_iter @ T_total
        if np.linalg.norm(source_new - source) < tolerance:
            source = source_new
            break
        source = source_new
    return T_total


def clamp_euler(T, max_roll_deg, max_pitch_deg, max_yaw_deg):
    """
    限制变换矩阵 T 中绕 x, y, z 轴的旋转角度，
    不超过 max_roll_deg, max_pitch_deg, max_yaw_deg 度。
    T 是 4x4 的变换矩阵，返回一个新的 4x4 变换矩阵，
    平移部分保持不变。
    """
    T = np.array(T, copy=True)
    R_global = np.array(T[:3, :3], copy=True)  # 显式复制，使其可写
    r = R.from_matrix(R_global)
    euler_angles = r.as_euler('xyz', degrees=False)
    roll, pitch, yaw = euler_angles

    max_roll = np.deg2rad(max_roll_deg)
    max_pitch = np.deg2rad(max_pitch_deg)
    max_yaw = np.deg2rad(max_yaw_deg)

    if np.abs(roll) > max_roll:
        roll = np.sign(roll) * max_roll
    if np.abs(pitch) > max_pitch:
        pitch = np.sign(pitch) * max_pitch
    if np.abs(yaw) > max_yaw:
        yaw = np.sign(yaw) * max_yaw

    new_R = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
    T_new = np.eye(4)
    T_new[:3, :3] = new_R
    T_new[:3, 3] = T[:3, 3]
    return T_new


# ----------------------- ROS 节点封装 -----------------------

class PointCloudRegistrationNode(object):
    def __init__(self):
        rospy.init_node('pointcloud_registration_node', anonymous=True)
        # 订阅全景点云，目标扇区点云从本地加载
        self.full_cloud_sub = rospy.Subscriber('/mid/points', PointCloud2, self.full_cloud_cb, queue_size=1)
        # 发布变换参数：x, y 为平移，z 为绕 z 轴旋转角（弧度）
        self.transform_pub = rospy.Publisher('/registration_transform', Vector3, queue_size=1)
        # 发布平均误差
        self.error_pub = rospy.Publisher('/registration_error', Float32, queue_size=1)
        # 创建一个服务，只有在调用服务时才执行配准
        self.reg_service = rospy.Service('run_registration', Trigger, self.handle_registration)

        self.full_cloud = None

        # 从参数服务器获取本地扇区点云的文件路径，若无则使用默认路径
        sector_file = rospy.get_param("~sector_file", "/home/sam/ME5413_Final_Project/sector_extracted.pcd")
        rospy.loginfo("Loading local sector point cloud: {}".format(sector_file))
        self.sector_cloud = o3d.io.read_point_cloud(sector_file)

        # 配准参数（可根据需要调整或从参数服务器获取）
        self.voxel_size = 0.05
        self.distance_threshold = self.voxel_size * 1.5
        self.sector_width = np.deg2rad(90)
        self.angle_range = np.deg2rad(np.arange(30, 90, 15))  # 45° 到 90°（步长15°）

        rospy.loginfo("Point cloud registration node started... Waiting for trigger...")

    def pointcloud2_to_o3d(self, pc2_msg):
        """
        将 sensor_msgs/PointCloud2 消息转换为 Open3D 点云对象。
        """
        pc = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
        xyz = np.zeros((pc.shape[0], 3), dtype=np.float32)
        xyz[:, 0] = np.copy(pc['x']).astype(np.float32)
        xyz[:, 1] = np.copy(pc['y']).astype(np.float32)
        xyz[:, 2] = np.copy(pc['z']).astype(np.float32)
        xyz = xyz.copy()  # 确保数组可写
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd

    def full_cloud_cb(self, msg):
        # 只保存全景点云数据，不自动触发配准
        self.full_cloud = self.pointcloud2_to_o3d(msg)
        rospy.loginfo("Received new full point cloud, registration not triggered.")

    def handle_registration(self, req):
        """
        通过 ROS service 触发配准流程，同时保存全景点云和最佳候选扇区配准后的点云到文件。
        """
        if self.full_cloud is None or self.sector_cloud is None:
            return TriggerResponse(success=False, message="Insufficient point cloud data for registration.")

        best_T, best_fitness, best_candidate = self.try_registration()
        if best_T is not None and best_candidate is not None:
            translation = best_T[:3, 3]
            R_mat = best_T[:3, :3]
            yaw = np.arctan2(R_mat[1, 0], R_mat[0, 0])
            rospy.loginfo("Best transformation matrix T:\n{}".format(best_T))
            rospy.loginfo("Translation vector: {}".format(translation))
            rospy.loginfo("Rotation about z-axis (radians): {:.4f}, (degrees): {:.1f}".format(yaw, np.degrees(yaw)))
            rospy.loginfo("Best candidate average error: {:.4f}".format(best_fitness))

            # 保存全景点云
            filename_full = "full_point_cloud_{}.pcd".format(time.strftime("%Y%m%d_%H%M%S"))
            o3d.io.write_point_cloud(filename_full, self.full_cloud)
            rospy.loginfo("Saved full point cloud to: {}".format(filename_full))

            # 保存最佳候选扇区配准后的点云
            filename_candidate = "aligned_candidate_{}.pcd".format(time.strftime("%Y%m%d_%H%M%S"))
            o3d.io.write_point_cloud(filename_candidate, best_candidate)
            rospy.loginfo("Saved best candidate point cloud to: {}".format(filename_candidate))

            # 发布变换参数和平均误差
            transform_msg = Vector3()
            transform_msg.x = translation[0]
            transform_msg.y = translation[1]
            transform_msg.z = yaw
            self.transform_pub.publish(transform_msg)

            error_msg = Float32()
            error_msg.data = best_fitness
            self.error_pub.publish(error_msg)

            return TriggerResponse(success=True, message="Registration completed and point clouds saved.")
        else:
            transform_msg = Vector3()
            transform_msg.x = 99
            transform_msg.y = 99
            transform_msg.z = 99
            self.error_pub.publish(99)
            self.transform_pub.publish(transform_msg)
            return TriggerResponse(success=False, message="No suitable candidate sector found.")

    def try_registration(self):
        """
        运行配准流程，返回最佳变换矩阵 T、匹配误差及最佳候选扇区配准后的点云。
        """
        # 对全景点云与本地扇区点云分别进行滤波
        pcd_full = filter(self.full_cloud, max_radius=8, z_min=-0.25, z_max=0.5)
        pcd_sector = filter(self.sector_cloud, max_radius=5, z_min=-0.25, z_max=0.5)

        # 对目标扇区点云下采样、估计法线并计算 FPFH 特征
        pcd_sector_down = pcd_sector.voxel_down_sample(self.voxel_size)
        pcd_sector_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2, max_nn=30))
        pcd_sector_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_sector_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 5, max_nn=100))

        best_fitness = float('inf')
        best_T = None
        best_candidate = None

        # 滑动窗口遍历全景点云中的候选扇区
        for center_angle in self.angle_range:
            candidate = extract_sector(pcd_full, center_angle, self.sector_width)
            if len(candidate.points) < 200:
                continue

            candidate_down = candidate.voxel_down_sample(self.voxel_size)
            candidate_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2, max_nn=30))
            candidate_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                candidate_down,
                o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 5, max_nn=100))

            # 基于 FPFH 特征的 RANSAC 全局配准
            result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                candidate_down, pcd_sector_down,
                candidate_fpfh, pcd_sector_fpfh, True,
                self.distance_threshold,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
                4,
                [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                 o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(self.distance_threshold)],
                o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))

            T_global = result_ransac.transformation
            T_limited = clamp_euler(T_global, max_roll_deg=30, max_pitch_deg=30, max_yaw_deg=160)
            T_global = T_limited

            candidate_aligned_global = copy.deepcopy(candidate_down)
            candidate_aligned_global.transform(T_global)

            source_np = np.asarray(candidate_aligned_global.points).copy()
            target_np = np.asarray(pcd_sector_down.points).copy()

            # 使用自定义 constrained_icp 在 XY 平面内精细对齐
            T_icp = constrained_icp(source_np, target_np, max_iter=60, tolerance=1e-4, max_z_rotation=180,
                                    min_z_rotation=-180)
            T_total = T_icp @ T_global

            # 计算均值最近邻距离作为匹配误差
            source_hom = np.hstack((np.asarray(candidate_down.points).copy(),
                                    np.ones((np.asarray(candidate_down.points).shape[0], 1))))
            source_transformed = (T_total @ source_hom.T).T[:, :3]
            tree = KDTree(target_np)
            distances, _ = tree.query(source_transformed)
            fitness = np.mean(distances)
            rospy.loginfo(
                "Candidate sector center angle {:.1f}° average error: {:.4f}".format(np.rad2deg(center_angle), fitness))

            if fitness < best_fitness:
                best_fitness = fitness
                best_T = T_total
                # 保存最佳候选扇区配准后的点云
                best_candidate = copy.deepcopy(candidate_down)
                best_candidate.transform(T_total)

        return best_T, best_fitness, best_candidate


if __name__ == '__main__':
    try:
        node = PointCloudRegistrationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
