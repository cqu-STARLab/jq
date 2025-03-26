#!/usr/bin/env python3
# coding=utf-8
import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
import numpy as np
from numpy.linalg import svd, norm
import tf
from MPC import MPC
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker,MarkerArray
from std_srvs.srv import SetBool

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import cKDTree

class Local_Planner():
    def __init__(self):
        # ~ 
        # self.replan_period = rospy.get_param('/local_planner/replan_period', 0.01)
        self.ns = rospy.get_param('~namespace',"")
        if self.ns and not self.ns.startswith("/"):
            self.ns = "/" + self.ns
        if self.ns and not self.ns.endswith("/"):
            self.ns = self.ns + "/"
        self.replan_period = rospy.get_param('/local_planner/replan_period', 0.1)
        # 获取参数
        self.curr_state = np.zeros(5)
        self.z = 0
        self.N = 10
        self.goal_state = np.zeros([self.N,4])
        self.ref_path_close_set = False
        self.target_state = np.array([-1,4,np.pi/2])
        self.target_state_close = np.zeros(3)
        self.desired_global_path = [ np.zeros([300,4]) , 0]
        self.have_plan = False
        self.is_close = False
        self.is_get = False
        self.is_grasp = False
        self.is_all_task_down = False
        self.robot_state_set = False
        self.ref_path_set = False
        self.ob=[]
        self.is_end=0
        self.ob_total = []
        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)
        self.__sub_curr_state = rospy.Subscriber(self.ns+'curr_state', Float32MultiArray, self.__curr_pose_cb, queue_size=10)
        self.__sub_obs = rospy.Subscriber(self.ns+'obs', Float32MultiArray, self.__obs_cb, queue_size=10)
        self.__sub_goal_state = rospy.Subscriber(self.ns+'surf_predict_pub', Float32MultiArray, self._global_path_callback2, queue_size=10)
        self.__pub_local_path = rospy.Publisher(self.ns+'local_path', Path, queue_size=10)
        self.__pub_local_plan = rospy.Publisher(self.ns+'local_plan', Float32MultiArray, queue_size=10)
        self.control_cmd = Twist()
        self.listener = tf.TransformListener()
        self.times = 0
        self.obstacle_markerarray = MarkerArray()
        self.ob_pub = rospy.Publisher(self.ns+'ob_draw', MarkerArray, queue_size=10)

        self.full_pc = None
        self.kdtree = None
        # 局部点云的拟合半径
        self.radius = rospy.get_param('~radius', 0.5)
        # 邻居节点的移动距离
        self.step_size = rospy.get_param('~step_size', 0.15)
        # 点云监听
        # 这里只能是输入建好的地图，不能是雷达的局部点云
        # 因为导航需要地面点，但是局部点云往往没有密集的地面点
        self.pc_topic = rospy.get_param('~pc_topic','/navigation_map')
        # 局部点云的分辨率降采样尺寸
        self.grid_size = rospy.get_param('~grid_size',0.1)
        self.ratio_min = rospy.get_param('~ratio_min',0.02)
        self.ratio_max = rospy.get_param('~ratio_max',0.1)
        self.conv_thre = rospy.get_param('~conv_thre',0.1)
        self.w_flatness = rospy.get_param('~w_flatness',2000.0)
        self.w_slope = rospy.get_param('~w_slope',0.03)
        self.w_sparsity = rospy.get_param('~w_sparsity',0.2)
        self.world_frame = rospy.get_param('~world_frame', "/world")

        self.__sub_pc = rospy.Subscriber(self.pc_topic, PointCloud2, self.__pc_callback)
    
    def __pc_callback(self, msg):
        points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        self.full_pc = np.array(points, dtype=np.float32).reshape(-1, 3)
        # print(self.full_pc.shape)
        

    def distance_sqaure(self,c1,c2):
        distance = (c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1])
        return distance
    
    # 这个玩意弃用了已经
    def draw_ob(self):
        self.obstacle_markerarray.markers=[]
        num = 0
        for i in range(len(self.ob)):
            t_ob = Marker()
            t_ob.header.frame_id = self.world_frame
            t_ob.id = num
            t_ob.type = t_ob.CYLINDER
            t_ob.action = t_ob.ADD
            t_ob.pose.position.x = self.ob[i][0]
            t_ob.pose.position.y = self.ob[i][1]
            t_ob.pose.position.z=0.2
            t_ob.scale.x = 0.1
            t_ob.scale.y = 0.1
            t_ob.scale.z = 0.4
            t_ob.color.a= 1
            t_ob.color.r = 0
            t_ob.color.g = 1
            t_ob.color.b = 0
            self.obstacle_markerarray.markers.append(t_ob)
            num = num +1
        self.ob_pub.publish(self.obstacle_markerarray)

    def _scan_callback(self, data):
        self.ob = []
        phi = data.angle_min
        point_last = np.array([100, 100])
        for r in data.ranges:
            point = np.array([self.curr_state[0]+r*np.cos(phi+self.curr_state[2]),self.curr_state[1]+r*np.sin(phi+self.curr_state[2])])
            if (r >= data.range_min and r <= data.range_max and r<=1.0 and self.distance_sqaure(point,point_last) > 0.04 ):
                self.ob.append( point )
                point_last = point
            phi += data.angle_increment
        self.draw_ob()

    def __obs_cb(self, data):
        self.ob = []
        if(len(data.data)!=0):

            size = int(len(data.data) / 3)
            for i in range(size):
                self.ob.append(( (data.data[3*i]//0.3)*0.3, (data.data[3*i+1]//0.3)*0.3) )
            dic = list(set([tuple(t) for t in self.ob]))
            self.ob = [list(v) for v in dic]
            self.draw_ob()

    def __replan_cb(self, event):
        if self.robot_state_set and self.ref_path_set:
            target = []
            self.choose_goal_state()        ##  gobal planning
            dist = 1
            goal = np.array([self.target_state[0], self.target_state[1], self.target_state[2]])
            start_time = rospy.Time.now()
            # 将点转换为动作
            states_sol, input_sol = MPC(np.expand_dims(self.curr_state, axis=0),self.goal_state,self.ob) ##  gobal planning
            end_time = rospy.Time.now()
            rospy.loginfo('[pHRI Planner] phri solved in {} sec'.format((end_time-start_time).to_sec()))

            if(self.is_end == 0):
                self.__publish_local_plan(input_sol,states_sol)
            self.have_plan = True
        elif self.robot_state_set==False and self.ref_path_set==True:
            print("no pose")
        elif self.robot_state_set==True and self.ref_path_set==False:
            print("no path")
            pass
        else:
            print("no path and no pose")
        

    def __publish_local_plan(self,input_sol,state_sol):
        local_path = Path()
        local_plan = Float32MultiArray()
        sequ = 0
        local_path.header.stamp = rospy.Time.now()
        local_path.header.frame_id = self.world_frame

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i,0]
            this_pose_stamped.pose.position.y = state_sol[i,1]
            this_pose_stamped.pose.position.z = self.z+0.5 #self.desired_global_path[0][0,2]
            this_pose_stamped.header.seq = sequ
            sequ += 1
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id=self.world_frame
            local_path.poses.append(this_pose_stamped)
            
            for j in range(2):
                local_plan.data.append(input_sol[i][j])

        self.__pub_local_path.publish(local_path)
        self.__pub_local_plan.publish(local_plan)

    def distance_global(self,c1,c2):
        distance = np.sqrt((c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1]))
        return distance
    
    # 根据x和y分割点云
    def get_xy_pillar_points(self, center_xy, half_size=0.5, full_pc = None):
        if full_pc is None:
            full_pc = self.full_pc  # 默认使用当前点云 但是无法应对异步问题
        center = np.array(center_xy)
        mask = (
            (full_pc[:, 0] >= center[0] - half_size) & (full_pc[:, 0] <= center[0] + half_size) &
            (full_pc[:, 1] >= center[1] - half_size) & (full_pc[:, 1] <= center[1] + half_size) &
            (full_pc[:, 2] >= -1.5) & (full_pc[:, 2] <= 1.0)  # 限制 Z 范围
        )
        return full_pc[mask] 
    

    def find_min_distance(self,c1):
        number =  np.argmin( np.array([self.distance_global(c1,self.desired_global_path[0][i]) for i in range(self.desired_global_path[1])]) )
        return number
    
    def analyze_pillar_points(self, pillar_points, grid_size=0.1, 
                          ratio_min=0.02, ratio_max=0.1, conv_thre=0.1,
                          w_flatness=2000.0, w_slope=0.03, w_sparsity=0.2):
        """
        输入:
            pillar_points: shape (N, 3)，一个pillar中的点云
            grid_size: 稀疏度网格的分辨率
        输出:
            traversability: 可通行性得分 (0 ~ 1)
            flatness: 平坦度
            slope: 坡度
            sparsity: 稀疏度
            normal_vector: 拟合平面的法向量
        """
        if len(pillar_points) < 5:
            return 1.0, 1.0, 90.0, 1.0, np.array([0, 0, 1])  # 点太少时保守处理

        # 1. 拟合平面
        center = np.mean(pillar_points, axis=0)
        A = pillar_points - center
        _, _, Vh = svd(A, full_matrices=False)
        normal_vector = Vh[-1]  # 法向量

        # 2. 计算平坦度（四次方距离的平均值）
        distances = np.dot(A, normal_vector)
        flatness = np.mean(distances**4) / (1 + len(pillar_points))

        # 3. 计算坡度（与 z 轴的夹角）
        z_axis = np.array([0, 0, 1])
        slope = np.degrees(np.arccos(np.clip(np.dot(z_axis, normal_vector), -1.0, 1.0)))

        # 4. 计算稀疏度
        xy_coords = pillar_points[:, :2]
        min_xy = np.min(xy_coords, axis=0)
        max_xy = np.max(xy_coords, axis=0)
        grid_w = int(np.ceil((max_xy[0] - min_xy[0]) / grid_size)) + 1
        grid_h = int(np.ceil((max_xy[1] - min_xy[1]) / grid_size)) + 1
        occupancy = np.zeros((grid_w, grid_h), dtype=bool)

        for pt in xy_coords:
            gx = int((pt[0] - min_xy[0]) / grid_size)
            gy = int((pt[1] - min_xy[1]) / grid_size)
            occupancy[gx, gy] = True

        occ_count = np.sum(occupancy)
        ratio = occ_count / (grid_w * grid_h)

        sparsity = 0.0
        if occ_count > 0:
            indices = np.array(np.nonzero(occupancy)).T
            mean = np.mean(indices, axis=0)
            zero_mean = indices - mean
            # cov = zero_mean.T @ zero_mean / indices.shape[0]
            cov = np.dot(zero_mean.T, zero_mean) / indices.shape[0]
            # trace = np.trace(cov @ cov)
            trace = np.trace(np.dot(cov, cov))

            if ratio > ratio_max:
                sparsity = 1.0
            elif ratio_min < ratio <= ratio_max and 1.0 / trace > conv_thre:
                sparsity = (ratio - ratio_min) / (ratio_max - ratio_min)
            else:
                sparsity = 0.0

        # 5. 综合评估可通行性（越小越好）
        traversability = w_flatness * flatness + w_slope * slope + w_sparsity * sparsity
        traversability = min(traversability, 1.0)

        return traversability, flatness, slope, sparsity, normal_vector

    def choose_goal_state(self):
        num = self.find_min_distance(self.curr_state)
        scale = 1
        num_list = []
        for i in range(self.N):
            num_path = min(self.desired_global_path[1]-1,int(num+i*scale))
            num_list.append(num_path)
        if(num  >= self.desired_global_path[1]):
            self.is_end = 1
        # 备份一个全局点云防止异步问题

        for k in range(self.N):
            # 类似于NBV方法，只选择前段的树结构做探索
            # 这样也能保证在低性能设备的稳定性

            # 为了解决问题1：转角等场景中虽然规划路径但是路径过于危险->在不分析地图的基础上。
            # 添加路径优化算法
            # 1.验证坐标系: 全局坐标系
            # 2.根据验证的坐标系 
            # 这是为了解决计算量过高的问题
            # 2.1 获取对应的局部点云：直接获取全局坐标下的局部点云
            # 2.2 坐标点为中心分析半径，然后根据半径内其他点的危险程度 移动这个点
            # 我觉得只需要考虑每个waypoint附近的点云就ok
            # 相当于考虑当前点附近所有方向点的状态，然后往该方向移动这个点，
            # 选择的原则是当前点和周围点的连续性，如果当前点和周围某方向点相差过大，那么将当前点往方向最小的一边转移
            # 3.根据获取的局部点云“修正” waypoints （这里参照TEB试试）
            # 半径查询
            tmp_point = self.desired_global_path[0][num_list[k]]
            x, y, z = tmp_point[0], tmp_point[1], tmp_point[2]
            # 定义 8 个方向上的邻居点坐标
            tmp_neighbors = [
                (x + self.step_size, y, z),      # 右
                (x - self.step_size, y, z),      # 左
                (x, y + self.step_size, z),      # 上
                (x, y - self.step_size, z),      # 下
                (x, y, z), # 当前点
                (x + self.step_size, y + self.step_size, z),  # 右上
                (x + self.step_size, y - self.step_size, z),  # 右下
                (x - self.step_size, y + self.step_size, z),  # 左上
                (x - self.step_size, y - self.step_size, z),  # 左下
            ]
            # 1.危险程度倾斜点
            # 2.相似程度倾斜点
            # 既希望和当前点的连续性足够高，也希望危险性足够低
            # 根据这8个点采集一定范围内的点云
            # 分割点云 分析点云
            traversability_list = []  # 存储每个pillar的可通行性
            full_pc_snapshot = self.full_pc.copy() # 备份一个全局点云 应对异步问题
            for i in range(9):
                pillar_points = self.get_xy_pillar_points(center_xy=tmp_neighbors[i][:2], half_size=self.radius,full_pc = full_pc_snapshot)
                # 高级方案：平整度（Flatness）稀疏度（Sparsity）海拔变化（Elevation Variation）坡度（Slope）坡度变化（Slope Variation）     
                # 现在是初级方案，但是有更严格的：平坦度 flatness 坡度 slope 稀疏度 sparsity
                traversability, flatness, slope, sparsity, normal_vector = self.analyze_pillar_points(pillar_points = pillar_points, 
                                                                                                      grid_size=self.grid_size, 
                                                                                                      ratio_min=self.ratio_min, ratio_max=self.ratio_max, conv_thre=self.conv_thre,
                                                                                                      w_flatness=self.w_flatness, w_slope=self.w_slope, w_sparsity=self.w_sparsity)
                # 存入列表
                traversability_list.append(traversability)
            print(traversability_list)
            
            # 如果存在不可通行的pillar（可通行性 ≥ 1），替换当前点为最优邻居
            # direction = {
            #     0: (self.step_size, 0),
            #     1: (-self.step_size, 0),
            #     2: (0, self.step_size),
            #     3: (0, -self.step_size),
            #     4: (0, 0),
            #     5: (self.step_size, self.step_size),
            #     6: (self.step_size, -self.step_size),
            #     7: (-self.step_size, self.step_size),
            #     8: (-self.step_size, -self.step_size),
            # }
            if any(trav >= 1 for trav in traversability_list):
                min_index = np.argmin(traversability_list)
                tmp_point = tmp_neighbors[min_index]
                # min_index = np.argmin(traversability_list)
                # dx, dy = direction[min_index]
                # x, y, z = tmp_point
                # tmp_point = (x + dx, y + dy, z)

            # 问题2: 对于远距离未知区域点无法有效生成路径
            # self.goal_state[k] 的格式 [ 7.22811079  9.52861214  0.83670402  0.1638629 ]

            # self.goal_state[k] = self.desired_global_path[0][num_list[k]]
            self.goal_state[k][:3] = tmp_point[:3]
            self.goal_state[k][3] = self.desired_global_path[0][num_list[k]][3]
        # print(self.goal_state)
    

    def __curr_pose_cb(self, data):
        self.robot_state_set = True
        self.curr_state[0] = data.data[0]
        self.curr_state[1] = data.data[1]
        self.curr_state[2] = data.data[3]
        self.curr_state[3] = data.data[4]
        self.curr_state[4] = data.data[5]
 
        self.z = data.data[2]

    def _global_path_callback(self, data):
        if(len(data.data)!=0):
            self.ref_path_set = True
            size = int(len(data.data)/3)
            self.desired_global_path[1]=size
            for i in range(size):
                self.desired_global_path[0][i,0]=data.data[3*(size-i)-3]
                self.desired_global_path[0][i,1]=data.data[3*(size-i)-2]
                self.desired_global_path[0][i,2]=data.data[3*(size-i)-1]
    
    def _global_path_callback2(self, data):
        # 这里获取 全局路径的点集（3:1的点生成）
        # 可以在这里设置区域
        if(len(data.data)!=0):
            self.ref_path_set = True
            size = int(len(data.data)/5)
            self.desired_global_path[1]=size
            for i in range(size):
                self.desired_global_path[0][i,0]=data.data[5*(size-i)-5]
                self.desired_global_path[0][i,1]=data.data[5*(size-i)-4]
                self.desired_global_path[0][i,2]=data.data[5*(size-i)-2]
                self.desired_global_path[0][i,3]=data.data[5*(size-i)-1]
            
    def cmd(self, data):
        
        self.control_cmd.linear.x = data[0]
        self.control_cmd.angular.z = data[1]
        self.__pub_rtc_cmd.publish(self.control_cmd)



if __name__ == '__main__':
    rospy.init_node("local_planner")
    phri_planner = Local_Planner()

    rospy.spin()