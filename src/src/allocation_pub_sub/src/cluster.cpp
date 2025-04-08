// DBSCAN.cpp
#include "cluster.h"
#include <cmath>
#include <limits>
#include <algorithm>

// Point 结构体构造函数实现
Point::Point(double x_, double y_,int demanda_,int demandb_, double area_)
    : x(x_), y(y_),demanda(demanda_),demandb(demandb_) ,area(area_), clusterID(-1) {}

// DBSCAN 构造函数实现
DBSCAN::DBSCAN(const std::vector<int>& city_x, 
               const std::vector<int>& city_y, 
               const std::vector<int>& city_p0,  
               const std::vector<int>& city_p1, 
               const std::vector<int>& city_s,
               double eps, int minPts)
    : eps(eps), minPts(minPts) {
    // 构建点的列表
    for (size_t i = 0; i < city_x.size(); ++i) {
        points.emplace_back(city_x[i], city_y[i], city_p0[i],city_p1[i],city_s[i]);
    }
}

// 计算两点之间的欧氏距离
double DBSCAN::distance(const Point& a, const Point& b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// 寻找点 p 的邻域点
std::vector<int> DBSCAN::regionQuery(const Point& p) {
    std::vector<int> neighbors;
    for (int i = 0; i < points.size(); ++i) {
        if (distance(p, points[i]) <= eps) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

// 扩展簇
void DBSCAN::expandCluster(int pointIdx, std::vector<int>& neighbors, int clusterID) {
    points[pointIdx].clusterID = clusterID; // 将点标记为属于簇

    // 遍历邻域中的每个点
    for (size_t i = 0; i < neighbors.size(); ++i) {
        int neighborIdx = neighbors[i];

        // 如果是未访问的点
        if (points[neighborIdx].clusterID == -1) {
            points[neighborIdx].clusterID = clusterID; // 标记为属于当前簇
            std::vector<int> neighborNeighbors = regionQuery(points[neighborIdx]);

            // 如果邻域点数量大于等于 minPts，将其加入队列
            if (neighborNeighbors.size() >= minPts) {
                neighbors.insert(neighbors.end(), neighborNeighbors.begin(), neighborNeighbors.end());
            }
        } else if (points[neighborIdx].clusterID == 0) {
            points[neighborIdx].clusterID = clusterID; // 将边界点加入簇
        }
    }
}

// 寻找最近的非噪声点簇ID
int DBSCAN::findNearestCluster(int pointIdx) {
    double minDist = std::numeric_limits<double>::max();
    int nearestClusterID = 0;

    for (const auto& point : points) {
        if (point.clusterID > 0) {  // 只考虑非噪声点
            double dist = distance(points[pointIdx], point);
            if (dist < minDist) {
                minDist = dist;
                nearestClusterID = point.clusterID;
            }
        }
    }
    return nearestClusterID;
}

// 执行 DBSCAN 算法
void DBSCAN::run() {
    int clusterID = 0;

    for (int i = 0; i < points.size(); ++i) {
        // 如果点已经被访问过，跳过
        if (points[i].clusterID != -1) {
            continue;
        }

        std::vector<int> neighbors = regionQuery(points[i]);

        // 点不满足核心点条件，标记为噪声
        if (neighbors.size() < minPts) {
            points[i].clusterID = 0; // 暂时标记为噪声
        } else {
            // 创建新簇
            clusterID++;
            expandCluster(i, neighbors, clusterID);
        }
    }

    // 将所有噪声点归类到最近的非噪声点簇
    for (int i = 0; i < points.size(); ++i) {
        if (points[i].clusterID == 0) { // 如果是噪声点
            int nearestClusterID = findNearestCluster(i);
            if (nearestClusterID > 0) {
                points[i].clusterID = nearestClusterID;
            }
        }
    }
}

// 输出聚类结果
void DBSCAN::printClusters() const {
    for (const auto& point : points) {
        std::cout << "Point (" << point.x << ", " << point.y << ") - Area: " << point.area 
                  << " - ClusterID: " << point.clusterID << std::endl;
    }
}
std::vector<int> DBSCAN::getClusterIDs() {
        std::vector<int> clusterIDs;
        for (const auto& point : points) {
        
            clusterIDs.push_back(point.clusterID);
        }
        return clusterIDs;
    }

void DBSCAN::calculateClusterStats(){
    std::vector<int> clusterIDs = getClusterIDs();
    // 获取最大簇ID，确保统计的簇数量正确
    int maxClusterID = *std::max_element(clusterIDs.begin(), clusterIDs.end());

    // 初始化统计数据
    avgX.resize(maxClusterID + 1, 0.0);
    avgY.resize(maxClusterID + 1, 0.0);
    sumP0.resize(maxClusterID + 1, 0);
    sumP1.resize(maxClusterID + 1, 0);
    sumS.resize(maxClusterID + 1, 0);
    std::vector<int> count(maxClusterID + 1, 0);

    // 遍历每个点，按簇ID累加统计数据
    for (const auto& point : points) {
        if (point.clusterID > 0) {  // 只统计非噪声点
            int clusterID = point.clusterID;
            avgX[clusterID] += point.x;
            avgY[clusterID] += point.y;
            count[clusterID]++;
            sumS[clusterID] += point.area;
            sumP0[clusterID] += point.demanda;  // 需求1的累加
            sumP1[clusterID] += point.demandb;  // 需求2的累加
        }
    }

    // 计算每个簇的平均值
    for (int clusterID = 1; clusterID <= maxClusterID; ++clusterID) {
        if (count[clusterID] > 0) {
            avgX[clusterID] /= count[clusterID];
            avgY[clusterID] /= count[clusterID];
        }
    }
    }

void DBSCAN::getClusterStats(std::vector<int>& city_avgx, 
                            std::vector<int>& city_avgy, 
                            std::vector<int>& city_sump0, 
                            std::vector<int>& city_sump1, 
                            std::vector<int>& city_sums, 
                            std::vector<int>& city_ID) {

    for (size_t i = 1; i < avgX.size(); ++i) { // 忽略簇ID为0的噪声点
        city_avgx.push_back(static_cast<int>(avgX[i]));      // 平均 x 坐标
        city_avgy.push_back(static_cast<int>(avgY[i]));      // 平均 y 坐标
        city_sump0.push_back(sumP0[i]);                      // 第一种能力的需求数量
        city_sump1.push_back(sumP1[i]);                      // 第二种能力的需求数量
        city_sums.push_back(sumS[i]);                        // 面积总和
        city_ID.push_back(static_cast<int>(i));           // 簇 ID
    }
}
std::vector<int> DBSCAN::getClusterIDs() const {
    std::vector<int> clusterIDs;
    // 遍历所有点
    for (const auto& point : points) {
        // 将当前点的聚类ID添加到 vector 中
        clusterIDs.push_back(point.clusterID);
    }
    return clusterIDs;
}