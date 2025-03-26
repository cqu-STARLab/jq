#include "backward.hpp"
#include "PUTN_planner.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace std_msgs;
using namespace Eigen;
using namespace PUTN;
using namespace PUTN::visualization;
using namespace PUTN::planner;

namespace backward
{
backward::SignalHandling sh;
}

// ros related
ros::Subscriber map_sub, wp_sub;

ros::Publisher nav_map_pub; // 导航点云生成
ros::Publisher grid_map_vis_pub;
ros::Publisher path_vis_pub;
ros::Publisher goal_vis_pub;
ros::Publisher surf_vis_pub;
ros::Publisher tree_vis_pub;
ros::Publisher path_interpolation_pub;
ros::Publisher tree_tra_pub;

// indicate whether the robot has a moving goal
bool has_goal = false;

// simulation param from launch file
double resolution;
double goal_thre;
double step_size;
double h_surf_car;
double max_initial_time;
double radius_fit_plane;
FitPlaneArg fit_plane_arg;
double neighbor_radius;

std::string ns;
std::string map_topic;
std::string navigation_map_topic;
std::string waypoints_topic;
std::string world_frame;
std::string robot_frame;

// useful global variables
Vector3d start_pt;
Vector3d target_pt;
World* world = NULL;
PFRRTStar* pf_rrt_star = NULL;

// 在函数外部定义全局变量
pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud(new pcl::PointCloud<pcl::PointXYZ>());
// nh 生成后初始化
tf::TransformListener* listener_ptr = nullptr;
double radius = 10.0; // 点云更新的保留半径


// function declaration
void rcvWaypointsCallback(const nav_msgs::Path& wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map);
void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* _path_interpolation_pub);
void findSolution();
void callPlanner();

/**
 *@brief receive goal from rviz
 */
void rcvWaypointsCallback(const nav_msgs::Path& wp)
{
  if (!world->has_map_)
    return;
  has_goal = true;
  //直接获取第一个点作为导航目标
  target_pt = Vector3d(wp.poses[0].pose.position.x, wp.poses[0].pose.position.y, wp.poses[0].pose.position.z);
  ROS_INFO("Receive the planning target");
}

/**
 *@brief receive point cloud to build the grid map
 */
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(pointcloud_map, cloud);
  // 点云过滤 先获取robot的坐标点
  tf::StampedTransform transform;
  try {
    listener_ptr->lookupTransform(world_frame, robot_frame, ros::Time(0), transform);
    // 使用 transform
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  // 机器人的x和y坐标
  double x = transform.getOrigin().x();
  double y = transform.getOrigin().y();

  // 使用CropBox裁剪globalCloud
  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(globalCloud);

  Eigen::Vector4f min_pt(x - radius, y - radius, -std::numeric_limits<float>::max(), 1.0);
  Eigen::Vector4f max_pt(x + radius, y + radius, -0.6, 1.0); // 保留-0.6 之下的点 就是保留地面点及凹陷点

  crop.setMin(min_pt);
  crop.setMax(max_pt);

  pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZ>());
  crop.filter(*croppedCloud);

  // 替换全局点云为裁剪后的点云
  *globalCloud = *croppedCloud;

  // 将当前帧点云加入全局点云
  *globalCloud += cloud;

  // 进行降采样
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(globalCloud);
  voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); // 自定义分辨率
  voxel_filter.filter(*globalCloud); // 更新为降采样后的点云

  world->initGridMap(*globalCloud);

  // 再将点云 globalCloud 发布出去
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*globalCloud, cloud_msg);
  cloud_msg.header.stamp = ros::Time::now(); 
  cloud_msg.header.frame_id = world_frame;
  nav_map_pub.publish(cloud_msg);

  for (const auto& pt : *globalCloud)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    world->setObs(obstacle);
  }
  visWorld(world, &grid_map_vis_pub);
}

/**
 *@brief Linearly interpolate the generated path to meet the needs of local planning
 */
void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* path_interpolation_pub)
{
  if (path_interpolation_pub == NULL)
    return;
  Float32MultiArray msg;
  for (size_t i = 0; i < solution.size(); i++)
  {
    if (i == solution.size() - 1)
    {
      msg.data.push_back(solution[i]->position_(0));
      msg.data.push_back(solution[i]->position_(1));
      msg.data.push_back(solution[i]->position_(2));
    }
    else
    {
      // 两个点之间根据0.1的长度预测插入多个值，但是多个值的可经过性通过高斯过程回归达成
      // 存在问题，高斯过程回归的拟合对于规则较为简单的场景还好，
      // 但是复杂的无规则场景可能导致预测值出现偏差
      // 1.保证路径细粒度
      // 2.节约计算量->全局分析优化为路径分析->再将路径粗粒度分割
      size_t interpolation_num = (size_t)(EuclideanDistance(solution[i + 1], solution[i]) / 0.1);
      Vector3d diff_pt = solution[i + 1]->position_ - solution[i]->position_;
      for (size_t j = 0; j < interpolation_num; j++)
      {
        Vector3d interpt = solution[i]->position_ + diff_pt * (float)j / interpolation_num;
        msg.data.push_back(interpt(0));
        msg.data.push_back(interpt(1));
        msg.data.push_back(interpt(2));
      }
    }
  }
  path_interpolation_pub->publish(msg);
}

/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void findSolution()
{
  // 这里生成路径 调整一下生成路径的偏激程度
  printf("=========================================================================\n");
  ROS_INFO("Start calling PF-RRT*");
  Path solution = Path();

  pf_rrt_star->initWithGoal(start_pt, target_pt);

  // Case1: The PF-RRT* can't work at when the origin can't be project to surface
  if (pf_rrt_star->state() == Invalid)
  {
    ROS_WARN("The start point can't be projected.Unable to start PF-RRT* algorithm!!!");
  }
  // Case2: If both the origin and the target can be projected,the PF-RRT* will execute
  //       global planning and try to generate a path
  else if (pf_rrt_star->state() == Global)
  {
    ROS_INFO("Starting PF-RRT* algorithm at the state of global planning");
    int max_iter = 5000;
    double max_time = 100.0;

    while (solution.type_ == Path::Empty && max_time < max_initial_time)
    {
      solution = pf_rrt_star->planner(max_iter, max_time);
      max_time += 100.0;
    }

    if (!solution.nodes_.empty())
      ROS_INFO("Get a global path!");
    else
      ROS_WARN("No solution found!");
  }
  // Case3: If the origin can be projected while the target can not,the PF-RRT*
  //       will try to find a temporary target for transitions.
  // 这里就是地图外的探索逻辑
  else
  {
    ROS_INFO("Starting PF-RRT* algorithm at the state of rolling planning");
    int max_iter = 1500;
    double max_time = 100.0;

    solution = pf_rrt_star->planner(max_iter, max_time);

    if (!solution.nodes_.empty())
      ROS_INFO("Get a sub path!");
    else
      ROS_WARN("No solution found!");
  }
  ROS_INFO("End calling PF-RRT*");
  printf("=========================================================================\n");

  pubInterpolatedPath(solution.nodes_, &path_interpolation_pub);
  visPath(solution.nodes_, &path_vis_pub);
  visSurf(solution.nodes_, &surf_vis_pub);

  // When the PF-RRT* generates a short enough global path,it's considered that the robot has
  // reached the goal region.
  if (solution.type_ == Path::Global && EuclideanDistance(pf_rrt_star->origin(), pf_rrt_star->target()) < goal_thre)
  {
    has_goal = false;
    visOriginAndGoal({}, &goal_vis_pub);  // Passing an empty set to delete the previous display
    visPath({}, &path_vis_pub);
    ROS_INFO("The Robot has achieved the goal!!!");
  }

  if (solution.type_ == Path::Empty)
    visPath({}, &path_vis_pub);
}

/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void callPlanner()
{
  static double init_time_cost = 0.0;
  if (!world->has_map_)
    return;

  // The tree will expand at a certain frequency to explore the space more fully
  if (!has_goal && init_time_cost < 1000)
  {
    timeval start;
    gettimeofday(&start, NULL);
    pf_rrt_star->initWithoutGoal(start_pt);
    timeval end;
    gettimeofday(&end, NULL);
    init_time_cost = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    if (pf_rrt_star->state() == WithoutGoal)
    {
      int max_iter = 550;
      double max_time = 100.0;
      pf_rrt_star->planner(max_iter, max_time);
      ROS_INFO("Current size of tree: %d", (int)(pf_rrt_star->tree().size()));
    }
    else
      ROS_WARN("The start point can't be projected,unable to execute PF-RRT* algorithm");
  }
  // If there is a specified moving target,call PF-RRT* to find a solution
  else if (has_goal)
  {
    findSolution();
    init_time_cost = 0.0;
  }
  // The expansion of tree will stop after the process of initialization takes more than 1s
  else
    ROS_INFO("The tree is large enough.Stop expansion!Current size: %d", (int)(pf_rrt_star->tree().size()));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planning_node");
  ros::NodeHandle nh("~");
  tf::TransformListener listener;
  listener_ptr = &listener;

  nh.param<std::string>("namespace", ns, "");
  nh.param<std::string>("map_topic", map_topic, "/cloud_registered_body");
  nh.param<std::string>("navigation_map_topic", navigation_map_topic, "/navigation_map");
  nh.param<std::string>("waypoints_topic", waypoints_topic, "/waypoint_generator/waypoints");
  nh.param<std::string>("world_frame", world_frame, "world");
  nh.param<std::string>("robot_frame", robot_frame, "base_link");
  nh.param<double>("radius", radius, 20.0);

  if (!ns.empty()) {
    if (ns.front() != '/')
      ns = "/" + ns;
    if (ns.back() != '/')
      ns += "/";
  }

  map_sub = nh.subscribe(map_topic, 1, rcvPointCloudCallBack);
  wp_sub = nh.subscribe(waypoints_topic, 1, rcvWaypointsCallback);
  // 发布降采样后的点云 用于局部导航
  nav_map_pub = nh.advertise<sensor_msgs::PointCloud2>(navigation_map_topic, 3);

  grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>(ns + "grid_map_vis", 1);
  path_vis_pub = nh.advertise<visualization_msgs::Marker>(ns + "path_vis", 20);
  goal_vis_pub = nh.advertise<visualization_msgs::Marker>(ns + "goal_vis", 1);
  surf_vis_pub = nh.advertise<sensor_msgs::PointCloud2>(ns + "surf_vis", 100);
  tree_vis_pub = nh.advertise<visualization_msgs::Marker>(ns + "tree_vis", 1);
  tree_tra_pub = nh.advertise<std_msgs::Float32MultiArray>(ns + "tree_tra", 1);
  path_interpolation_pub = nh.advertise<std_msgs::Float32MultiArray>(ns + "global_path", 1000);

  nh.param("map/resolution", resolution, 0.1);

  nh.param("planning/goal_thre", goal_thre, 1.0);
  nh.param("planning/step_size", step_size, 0.2);
  nh.param("planning/h_surf_car", h_surf_car, 0.4);
  nh.param("planning/neighbor_radius", neighbor_radius, 1.0);

  nh.param("planning/w_fit_plane", fit_plane_arg.w_total_, 0.4);
  nh.param("planning/w_flatness", fit_plane_arg.w_flatness_, 4000.0);
  nh.param("planning/w_slope", fit_plane_arg.w_slope_, 0.4);
  nh.param("planning/w_sparsity", fit_plane_arg.w_sparsity_, 0.4);
  nh.param("planning/ratio_min", fit_plane_arg.ratio_min_, 0.25);
  nh.param("planning/ratio_max", fit_plane_arg.ratio_max_, 0.4);
  nh.param("planning/conv_thre", fit_plane_arg.conv_thre_, 0.1152);

  nh.param("planning/radius_fit_plane", radius_fit_plane, 1.0);

  nh.param("planning/max_initial_time", max_initial_time, 1000.0);

  // Initialization
  world = new World(resolution);
  pf_rrt_star = new PFRRTStar(h_surf_car, world);

  // Set argument of PF-RRT*
  pf_rrt_star->setGoalThre(goal_thre);
  pf_rrt_star->setStepSize(step_size);
  pf_rrt_star->setFitPlaneArg(fit_plane_arg);
  pf_rrt_star->setFitPlaneRadius(radius_fit_plane);
  pf_rrt_star->setNeighborRadius(neighbor_radius);

  pf_rrt_star->goal_vis_pub_ = &goal_vis_pub;
  pf_rrt_star->tree_vis_pub_ = &tree_vis_pub;
  pf_rrt_star->tree_tra_pub_ = &tree_tra_pub;

  while (ros::ok())
  {
    timeval start;
    gettimeofday(&start, NULL);

    // Update the position of the origin
    tf::StampedTransform transform;
    while (true && ros::ok())
    {
      try
      {
        listener.lookupTransform(world_frame, robot_frame, ros::Time(0), transform);  //查询变换
        break;
      }
      catch (tf::TransformException& ex)
      {
        continue;
      }
    }
    start_pt << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();

    // Execute the callback functions to update the grid map and check if there's a new goal
    ros::spinOnce();
    // Call the PF-RRT* to work
    callPlanner();
    double ms;
    do
    {
      timeval end;
      gettimeofday(&end, NULL);
      ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    } while (ms < 100);  // Cycle in 100ms
  }
  return 0;
}
