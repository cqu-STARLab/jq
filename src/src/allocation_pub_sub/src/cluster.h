#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>
#include <iostream>

// 定义点的结构体
struct Point {
    double x, y;     // 坐标
    double area;     // 区域大小
    int demanda;     // 对a类的需求
    int demandb;
    int clusterID;   // 簇的ID（-1表示未分配，0表示噪声，正值表示簇ID）

    // 构造函数
    Point(double x_, double y_,int demanda_,int demandb_, double area_);
};

// DBSCAN算法类
class DBSCAN {
private:
    std::vector<Point> points; // 点的列表
    double eps;                // 邻域半径
    int minPts;                // 核心点的最小邻域点数
    std::vector<double> avgX;
    std::vector<double> avgY;
    std::vector<int> sumP0;
    std::vector<int> sumP1;
    std::vector<int> sumS;

    // 计算两点之间的欧氏距离
    double distance(const Point& a, const Point& b);

    // 寻找点的邻域点
    std::vector<int> regionQuery(const Point& p);

    // 扩展簇
    void expandCluster(int pointIdx, std::vector<int>& neighbors, int clusterID);

    // 寻找最近的非噪声点的簇ID
    int findNearestCluster(int pointIdx);

public:
    // 构造函数
    DBSCAN(const std::vector<int>& city_x, 
               const std::vector<int>& city_y, 
               const std::vector<int>& city_p0,  
               const std::vector<int>& city_p1, 
               const std::vector<int>& city_s,
               double eps, int minPts);
    // 执行 DBSCAN 算法
    void run();

    // 输出聚类结果
    void printClusters() const;

    std::vector<int> getClusterIDs();

    void calculateClusterStats();

    void getClusterStats(std::vector<int>& city_avgx, 
                             std::vector<int>& city_avgy, 
                             std::vector<int>& city_sump0, 
                             std::vector<int>& city_sump1, 
                             std::vector<int>& city_sums, 
                             std::vector<int>& city_ID) ;
    std::vector<int> getClusterIDs() const;                          



};

#endif // CLUSTER_H
