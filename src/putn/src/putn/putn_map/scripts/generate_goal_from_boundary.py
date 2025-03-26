#!/usr/bin/env python
import rospy
import tf
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

# 参数设置
local_x_l, local_x_u = -10.0, 10.0  # x 方向范围
local_y_l, local_y_u = -10.0, 10.0  # y 方向范围
local_z_l, local_z_u = -1.0, 2.0    # z 方向范围
resolution = 0.1                    # 栅格大小

# 发布者定义
goal_pub = None
listener = None

# 发布导航目标
def send_goal(x, y, z, yaw):
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "world"  # 使用地图坐标系

    # 设置目标位置
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z

    # 设置目标朝向 (四元数)
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = quat[0]
    goal.pose.orientation.y = quat[1]
    goal.pose.orientation.z = quat[2]
    goal.pose.orientation.w = quat[3]

    rospy.loginfo(f"Goal sent: x={x}, y={y}, z={z}, yaw={yaw}")
    goal_pub.publish(goal)


def process_pointcloud(data, robot_position=[0.0, 0.0], grid_size=1.0, density_threshold=10, radius=1.0, modify = -1.2):
    rospy.loginfo("Processing point cloud to find valid boundary regions...")
    # 现在的点云以雷达为基准
    # 将点云转换为 numpy 数组
    points = []
    for p in point_cloud2.read_points(data, skip_nans=True):
        points.append([p[0], p[1], p[2]])

    points = np.array(points)

    # 投影到二维平面
    points_2d = points[:, :2]

    # 网格化点云
    x_min, x_max = np.min(points_2d[:, 0]), np.max(points_2d[:, 0])
    y_min, y_max = np.min(points_2d[:, 1]), np.max(points_2d[:, 1])

    x_bins = np.arange(x_min, x_max, grid_size)
    y_bins = np.arange(y_min, y_max, grid_size)

    # 统计每个网格内点的数量
    density, _, _ = np.histogram2d(points_2d[:, 0], points_2d[:, 1], bins=[x_bins, y_bins])
    explored_mask = density >= density_threshold  # 已探索网格为 True，未知网格为 False

    # 找到最近的有效边界点
    nearest_point = None
    min_distance = float('inf')

    for i in range(1, explored_mask.shape[0] - 1):
        for j in range(1, explored_mask.shape[1] - 1):
            if explored_mask[i, j]:
                # 检查当前网格的 8 邻域是否包含未知区域
                neighbors = explored_mask[i - 1:i + 2, j - 1:j + 2]
                if not np.all(neighbors):  # 如果邻域内存在未知区域
                    center_x = (x_bins[i] + x_bins[i + 1]) / 2.0
                    center_y = (y_bins[j] + y_bins[j + 1]) / 2.0

                    # 检查该网格是否包含障碍物点
                    grid_points = points[
                        (points[:, 0] >= x_bins[i]) & (points[:, 0] < x_bins[i + 1]) &
                        (points[:, 1] >= y_bins[j]) & (points[:, 1] < y_bins[j + 1])
                    ]
                    # -1.40

                    if np.any((grid_points[:, 2] >= ( modify))):  # 如果网格内为障碍物
                        continue  # 跳过该网格

                    # 检查 center_x 和 center_y 对应的点是否为障碍物
                    center_point = points[
                        (np.abs(points[:, 0] - center_x) < grid_size / 2) &
                        (np.abs(points[:, 1] - center_y) < grid_size / 2)
                    ]
                    # print(grid_points[:, 2])
                    # print(center_point[:, 2])
                    # return
                    if center_point.size > 0 and np.any((center_point[:, 2] >= ( modify))):
                        continue  # 如果该点本身是障碍物点，跳过

                    # 如果满足条件，计算到机器人的距离
                    distance = np.sqrt((center_x - robot_position[0]) ** 2 + (center_y - robot_position[1]) ** 2)
                    if distance < min_distance:
                        nearest_point = (center_x, center_y, 0.0)  # z 设为 0
                        min_distance = distance

    # 如果找到有效边界点，则发布目标
    if nearest_point:
        rospy.loginfo(f"Nearest valid boundary point: {nearest_point}, Distance: {min_distance}")
        send_goal(nearest_point[0], nearest_point[1], nearest_point[2], 0.0)  # 假设默认朝向为 0 度
    else:
        rospy.logwarn("No valid boundary point found.")




# 主函数
def main():
    global goal_pub, obs_pub, obs_array_pub, listener

    # 初始化 ROS 节点
    rospy.init_node('generate_goal_from_boundary', anonymous=True)

    # 创建发布者
    goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=10)

    # 创建 TF 监听器
    listener = tf.TransformListener()

    # 订阅点云数据
    rospy.Subscriber('laser_cloud_map', PointCloud2, process_pointcloud)

    rospy.loginfo("Goal Generator Node Started.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
