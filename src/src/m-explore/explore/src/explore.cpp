/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{

void Explore::navigationCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    // 启动导航
    // 这里给的是真实坐标 需要和分辨率作计算 才能使用
    navigation_x_ = msg->x;
    navigation_y_ = msg->y;
    is_navigation = true;
    // is_constraint = false;
    // last_progress_ = last_progress_ - progress_timeout_;
    last_progress_ = ros::Time::now() - ros::Duration(progress_timeout_);
    // std::cout<<"navigation_x_:"<<std::endl;
}

void Explore::regionConstraintCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    if (msg->poses.size() < 2) {
        ROS_WARN("Region constraint message should contain at least 2 poses.");
        return;
    }
    // is_navigation = false;
        
    exploration_bottom_left_ = msg->poses[0].position;
    exploration_top_right_ = msg->poses[1].position;
    last_progress_ = ros::Time::now() - ros::Duration(progress_timeout_);

    ROS_WARN("Received region constraint: Bottom Left (%.2f, %.2f), Top Right (%.2f, %.2f)",
            exploration_bottom_left_.x, exploration_bottom_left_.y,
            exploration_top_right_.x, exploration_top_right_.y);

  // double min_cr_x_,min_cr_y_;//左下角
  // double max_cr_x_,max_cr_y_;//右上角
    // 获取探索区域的 x, y 范围
    min_cr_x_ = std::min(exploration_bottom_left_.x, exploration_top_right_.x) - size_expand;
    max_cr_x_ = std::max(exploration_bottom_left_.x, exploration_top_right_.x) + size_expand;
    min_cr_y_ = std::min(exploration_bottom_left_.y, exploration_top_right_.y) - size_expand;
    max_cr_y_ = std::max(exploration_bottom_left_.y, exploration_top_right_.y) + size_expand;

    is_constraint = true;
}

std::vector<geometry_msgs::PoseStamped> Explore::makePlan_movebase(
    ros::ServiceClient& plan_client,
    const std::pair<double, double>& start_xy,
    const std::pair<double, double>& goal_xy)
{
    geometry_msgs::PoseStamped start, goal;
    start.header.frame_id = map_frame;
    goal.header.frame_id = map_frame;

    ros::Time now = ros::Time::now();
    start.header.stamp = now;
    goal.header.stamp = now;

    start.pose.position.x = start_xy.first;
    start.pose.position.y = start_xy.second;
    start.pose.orientation.w = 1.0;

    goal.pose.position.x = goal_xy.first;
    goal.pose.position.y = goal_xy.second;
    goal.pose.orientation.w = 1.0;

    // 调用路径规划服务
    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.tolerance = 0.0;

    if (plan_client.call(srv) && !srv.response.plan.poses.empty()) {
        ROS_INFO("Path planned successfully with %lu poses:", srv.response.plan.poses.size());
        for (size_t i = 0; i < srv.response.plan.poses.size(); ++i) {
            const auto& pose = srv.response.plan.poses[i];
            ROS_INFO("  [%lu] x=%.2f, y=%.2f, yaw=%.2f",
                     i,
                     pose.pose.position.x,
                     pose.pose.position.y,
                     tf::getYaw(pose.pose.orientation));
        }
        return srv.response.plan.poses;
    } else {
        ROS_WARN("Failed to call make_plan or plan is empty.");
        return {};
    }
}


Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("max_time", max_time, 6.0);
  private_nh_.param("visualize", visualize_, true);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("clearance_scale", clearance_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
  private_nh_.param("is_3d_navigation", is_3d_navigation, true);
  private_nh_.param("cmd_topic", cmd_topic, std::string("/cmd_vel"));
  private_nh_.param("goal_3d_topic",goal_3d_topic_, std::string("/goal"));
  private_nh_.param("map_frame",map_frame, std::string("/map"));
  private_nh_.param("map_radius",map_radius, 5.0);
  private_nh_.param("size_expand",size_expand, size_expand);
  private_nh_.param("size_expand_max",size_expand_max, size_expand_max);
  private_nh_.param("if_check_path",if_check_path, if_check_path);
  // 这里获取的话题均以当前命名空间为基础
  // 获取服务话题
  private_nh_.param("plan_service", plan_service, plan_service);

  plan_client = private_nh_.serviceClient<nav_msgs::GetPlan>(plan_service);
  // 等待服务可用
  plan_client.waitForExistence();

  navigation_sub_ = private_nh_.subscribe(navigation_topic_, 100, &Explore::navigationCallback,this);
  region_constraint_sub_ = private_nh_.subscribe(region_constraint_topic_, 100, &Explore::regionConstraintCallback,this);

  goal_3d_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>(goal_3d_topic_, 10);
  cmd_pub = private_nh_.advertise<geometry_msgs::Twist>(cmd_topic, 10);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_, clearance_scale_,
                                                 min_frontier_size);

  

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });
  // ros::spin();

}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  ROS_INFO("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_INFO("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    stop();
    return;
  }

  // publish frontiers as visualization markers
  // if (visualize_) {
  //   visualizeFrontiers(frontiers);
  // }

  if (is_navigation)
  {
    // 如果触发了导航，那么
    // 1.判断是否到达目标点 (无需判断？)
    // 1.1. 判断目标点是否存在于地图中
    costmap_2d::Costmap2D* costmap2d_1 = costmap_client_.getCostmap();
    unsigned int mx, my;

    // publish frontiers as visualization markers
    if (visualize_) {
      visualizeFrontiers(frontiers);
    }

    // 先判断距离
    double distance = std::hypot(pose.position.x - navigation_x_,
                          pose.position.y - navigation_y_);
    if ((distance <= navigation_radius_)){
      // 如果在可以接受的距离内 
      // 停止导航 进入下一次规划
      ROS_INFO("navigaiton over");
      is_navigation = false;
      return;
    }
// (!costmap2d_1->worldToMap(navigation_x_, navigation_y_, mx, my))
    if (((distance>=map_radius))) {

      ROS_INFO("we cant see the navigation target!");
      // 1.2. 执行判断，选择距离 navigation_x_, navigation_y_ 最近的边界点作为目标
      double min_dist = std::numeric_limits<double>::max();
      geometry_msgs::Point closest_centroid;
      bool found_reachable = false;
      if (if_check_path){
        for (const auto& frontier : frontiers) {
          double dist = std::hypot(frontier.centroid.x - navigation_x_, 
                              frontier.centroid.y - navigation_y_);
          // 判断安全性
          auto plan_move_base = makePlan_movebase(
            plan_client,
            {pose.position.x, pose.position.y},            // 起点
            {frontier.centroid.x, frontier.centroid.y}                 // 终点
          );
          if(!plan_move_base.empty()){
            // 如果路径生成成功，那真是皆大欢喜
            if (dist < min_dist) {
              min_dist = dist;
              closest_centroid = frontier.centroid;
              found_reachable = true;
            }
          }
        }
      }

      if (!found_reachable)
      {
        // 如果没有找到点，就找个不太安全的点也行 move_base会有保护策略
        for (const auto& frontier : frontiers) {
          double dist = std::hypot(frontier.centroid.x - navigation_x_, 
                            frontier.centroid.y - navigation_y_);
          // 如果路径生成成功，那真是皆大欢喜
          if (dist < min_dist) {
            min_dist = dist;
            closest_centroid = frontier.centroid;
            found_reachable = true;
          }
        }
      }


      geometry_msgs::Point target_position;
      // 选择最近的前沿点作为目标
      target_position = closest_centroid;

      // if (is_3d_navigation){
      //   // 如果执行3D导航
      //   // 发送 geometry_msgs.msg.PoseStamped
      //   geometry_msgs::PoseStamped goal_msg;
      //   // 2. 设置 header（时间戳 & 坐标系）
      //   goal_msg.header.stamp = ros::Time::now();
      //   goal_msg.header.frame_id = map_frame;  // 目标点在 "map" 坐标系下
      //   // 3. 设定目标点坐标（单位：米）
      //   goal_msg.pose.position.x = closest_centroid.x;
      //   goal_msg.pose.position.y = closest_centroid.y;
      //   goal_msg.pose.position.z = 0.0;  // 地面目标，Z 设为 0

      //   // 4. 设定目标点方向（四元数）
      //   goal_msg.pose.orientation.x = 0.0;
      //   goal_msg.pose.orientation.y = 0.0;
      //   goal_msg.pose.orientation.z = 0.0;
      //   goal_msg.pose.orientation.w = 1.0;  // 朝向不变

      //   goal_3d_pub_.publish(goal_msg);
      //   return;
      // }

      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.pose.position.x = closest_centroid.x;
      goal.target_pose.pose.position.y = closest_centroid.y;
      goal.target_pose.pose.orientation.w = 1.;
      goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
      goal.target_pose.header.stamp = ros::Time::now();
      // move_base_client_.cancelAllGoals();
      // ros::Duration(0.1).sleep(); // 地性能 需要等待一会儿
      move_base_client_.sendGoal(
        goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
      return;

    }else{
      // 先判断距离
      // double distance = std::hypot(pose.position.x - navigation_x_,
      //                        pose.position.y - navigation_y_);
      // if ((distance <= navigation_radius_)||(frontiers.empty())){
      //   // 如果在可以接受的距离内 
      //   // 停止导航 进入下一次规划
      //   ROS_INFO("navigaiton over");
      //   is_navigation = false;
      //   if (is_3d_navigation){
      //     //cmd_vel 输入停止指令
      //     geometry_msgs::Twist stop_cmd;
      //     stop_cmd.linear.x = 0.0;
      //     stop_cmd.linear.y = 0.0;
      //     stop_cmd.linear.z = 0.0;
      //     stop_cmd.angular.x = 0.0;
      //     stop_cmd.angular.y = 0.0;
      //     stop_cmd.angular.z = 0.0;
      //     cmd_pub.publish(stop_cmd);
      //   }
      //   return;
      // }

      // if (is_3d_navigation){
      //   // 如果执行3D导航
      //   // 发送 geometry_msgs.msg.PoseStamped
      //   geometry_msgs::PoseStamped goal_msg;
      //   // 2. 设置 header（时间戳 & 坐标系）
      //   goal_msg.header.stamp = ros::Time::now();
      //   goal_msg.header.frame_id = map_frame;  // 目标点在 "map" 坐标系下
      //   // 3. 设定目标点坐标（单位：米）
      //   goal_msg.pose.position.x = navigation_x_;
      //   goal_msg.pose.position.y = navigation_y_;
      //   goal_msg.pose.position.z = 0.0;  // 地面目标，Z 设为 0

      //   // 4. 设定目标点方向（四元数）
      //   goal_msg.pose.orientation.x = 0.0;
      //   goal_msg.pose.orientation.y = 0.0;
      //   goal_msg.pose.orientation.z = 0.0;
      //   goal_msg.pose.orientation.w = 1.0;  // 朝向不变

      //   goal_3d_pub_.publish(goal_msg);
      //   return;
      // }

      // 可以看到目标点 那么直接驱动小车到达目标点
      geometry_msgs::Point target_position;
      // 设置目标位置的 x 和 y 坐标
      target_position.x = navigation_x_;
      target_position.y = navigation_y_;
      target_position.z = 0.0;  // 如果是二维空间，z 可以设置为 0

      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.pose.position.x = navigation_x_;
      goal.target_pose.pose.position.y = navigation_y_;
      goal.target_pose.pose.orientation.w = 1.;
      goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
      goal.target_pose.header.stamp = ros::Time::now();
      // move_base_client_.cancelAllGoals();
      // ros::Duration(0.1).sleep(); // 地性能 需要等待一会儿
      move_base_client_.sendGoal(
        goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
      return;
    }
  }

  //判定是否存在区域限制
  // std::cout<<min_cr_x_<<std::endl;
  // std::cout<<max_cr_x_<<std::endl;
  // std::cout<<min_cr_y_<<std::endl;
  // std::cout<<max_cr_y_<<std::endl;
  // for (const auto& f : frontiers) {
  //       ROS_WARN_STREAM("  Frontier centroid: (" << f.centroid.x << ", " << f.centroid.y << ")");
  // }
  if(is_constraint){
    frontiers.erase(std::remove_if(frontiers.begin(), frontiers.end(),
                               [&](const frontier_exploration::Frontier& f) {
                                   return (f.centroid.x < min_cr_x_ || f.centroid.x > max_cr_x_) ||
                                          (f.centroid.y < min_cr_y_ || f.centroid.y > max_cr_y_);
                               }),
                frontiers.end());
  // }
  }else{
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    ROS_WARN("see the frontiers");
    visualizeFrontiers(frontiers);
  }


  // find non blacklisted frontier
  // auto frontier =
  //     std::find_if_not(frontiers.begin(), frontiers.end(),
  //                      [this](const frontier_exploration::Frontier& f) {
  //                        return goalOnBlacklist(f.centroid);
  //                      });

  // bool found_reachable = false;
  //     if (if_check_path){
  //       for (const auto& frontier : frontiers) {
  //         double dist = std::hypot(frontier.centroid.x - navigation_x_, 
  //                             frontier.centroid.y - navigation_y_);
  //         // 判断安全性
          // auto plan_move_base = makePlan_movebase(
          //   plan_client,
          //   {pose.position.x, pose.position.y},            // 起点
          //   {frontier.centroid.x, frontier.centroid.y}                 // 终点
          // );
  //         if(!plan_move_base.empty()){
  //           // 如果路径生成成功，那真是皆大欢喜
  //           if (dist < min_dist) {
  //             min_dist = dist;
  //             closest_centroid = frontier.centroid;
  //             found_reachable = true;
  //           }
  //         }
  //       }
  //     }
  std::vector<frontier_exploration::Frontier>::iterator frontier;
  if (if_check_path){
    frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this, &pose](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid) || (makePlan_movebase(this->plan_client, {pose.position.x, pose.position.y}, {f.centroid.x, f.centroid.y})).empty();
                       });
  }else{
    frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid); //|| !f.can_achieve;
                       });
  }
  // 如果所有点都不可达 那么重新采点
  if (frontier == frontiers.end()) {
    frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  }
  if (frontier == frontiers.end()) {
    // 当前区域所有的点都被放入黑名单了 这是很有可能的
    // 当前区域已经探索完毕了吗？
    // 如果没有探索完毕 那么选择黑名单中最接近目标区域的点继续探索
    // 所以直接释放黑名单就好
    // 如果黑名单中的点都探索完毕了，那么下一轮还是会走到这里来，如果没有探索完毕，那么就继续探索
    if (!frontier_blacklist_.empty()) {
      frontier_blacklist_.clear();
      // 重置时间
      last_progress_ = ros::Time::now();
      return;
    }

    // 在判断目标区域的4个角点是不是都探索完毕了
    if (is_constraint)
    {
      if (size_expand<=size_expand_max)
      {
        last_progress_ = ros::Time::now();
        // 如果探索完毕 那么就扩大探索区域继续侦查
        // 如果探索没有完毕 那么也扩大探索区域继续侦查
        size_expand = size_expand + 4;
        min_cr_x_ = min_cr_x_ - 4;
        max_cr_x_ = max_cr_x_ + 4;
        min_cr_y_ = min_cr_y_ - 4;
        max_cr_y_ = max_cr_y_ + 4;
        return;
      }

    }


    stop();
    // 探索结束
    // 如果是3D导航那么需要发送停止指令 pose.position.x pose.position.y 这是机器人的坐标 
    // 直接以当前坐标作为目标
    if (is_3d_navigation){
      geometry_msgs::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.linear.y = 0.0;
      stop_cmd.linear.z = 0.0;
      stop_cmd.angular.x = 0.0;
      stop_cmd.angular.y = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_pub.publish(stop_cmd);
    }

    is_constraint = false;
    return;
  }
  geometry_msgs::Point target_position = frontier->centroid;
  std::cout<<"target:"<<std::endl;
  std::cout<<target_position<<std::endl;

  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now();
    prev_distance_ = frontier->min_distance;
    double distance_factor = std::min(frontier->min_distance, 30.0);
    double timeout_sec = (distance_factor / 30.0) * (progress_timeout_.toSec() * (max_time-1));
    ros::Duration dynamic_timeout(timeout_sec);
    last_progress_ += dynamic_timeout;  // 正确的写法
  }
  // black list if we've made no progress for a long time
  if (ros::Time::now() - last_progress_ > progress_timeout_) {
    // 超时也会导致加入黑名单
    frontier_blacklist_.push_back(target_position);
    ROS_DEBUG("Adding current goal to black list");
    makePlan();
    return;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return;
  }

  if (is_3d_navigation){
    // 如果执行3D导航
    // 发送 geometry_msgs.msg.PoseStamped
    geometry_msgs::PoseStamped goal_msg;
    // 2. 设置 header（时间戳 & 坐标系）
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = map_frame;  // 目标点在 "map" 坐标系下
    // 3. 设定目标点坐标（单位：米）
    goal_msg.pose.position.x = target_position.x;
    goal_msg.pose.position.y = target_position.y;
    goal_msg.pose.position.z = 0.0;  // 地面目标，Z 设为 0

    // 4. 设定目标点方向（四元数）
    goal_msg.pose.orientation.x = 0.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = 0.0;
    goal_msg.pose.orientation.w = 1.0;  // 朝向不变

    goal_3d_pub_.publish(goal_msg);
    return;
  }

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  // move_base_client_.cancelAllGoals();
  // ros::Duration(0.1).sleep(); // 地性能 需要等待一会儿
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 4; // before 5 
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if ((status == actionlib::SimpleClientGoalState::ABORTED)&&(!is_navigation)) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::stop()
{
  move_base_client_.cancelAllGoals();
  // exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explore::Explore explore;
  ros::spin();

  return 0;
}
