// // Assignment.h
// #ifndef ASSIGNMENT_H
// #define ASSIGNMENT_H

// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <limits>

// // 定义小车的结构体
// struct Point_ugv {
//     double x, y;     // 坐标
//     int ability1;    // 第一种能力（0或1）
//     int ability2;    // 第二种能力（0或1）
// };

// // 定义分区的结构体
// struct Region {
//     double x, y;       // 中心坐标
//     double area;       // 面积
//     int demand1;       // 对第一种能力的需求数量
//     int demand2;       // 对第二种能力的需求数量
// };

// // 城市参数的结构体，用于初始化分区
// struct CityParameters {
//     std::vector<int> x;       // 分区的x坐标
//     std::vector<int> y;       // 分区的y坐标
//     std::vector<int> area;    // 分区的面积
//     std::vector<int> demand1; // 第一种能力的需求数量
//     std::vector<int> demand2; // 第二种能力的需求数量
// };

// // 小车参数的结构体，用于初始化点
// struct CarParameters {
//     std::vector<int> x;   // 小车的x坐标
//     std::vector<int> y;   // 小车的y坐标
//     std::vector<int> p;   // 第一种能力
//     std::vector<int> q;   // 第二种能力
// };

// // Assignment类用于实现分配算法
// class Assignment {
// private:
//     std::vector<Region> regions;              // 分区列表
//     std::vector<Point_ugv> points;            // 小车点列表
//     std::vector<std::vector<int>> assignments; // 记录每个分区分配的点索引
//     CityParameters cityParams;                // 城市参数
//     CarParameters carParams;                  // 小车参数

//     // 计算两点之间的欧氏距离
//     double euclideanDistance(const Point_ugv& a, const Region& b);

//     // 根据点的能力和距离，计算分配优先级（可以自定义评分函数）
//     double priorityScore(const Point_ugv& point, const Region& region);

//     // 初始化分区
//     void initializeRegions();

//     // 初始化小车点
//     void initializePoints();

// public:
//     // 构造函数：初始化城市和小车参数
//     Assignment(const CityParameters& cityParams, const CarParameters& carParams);

//     // 分配点到分区
//     void distributePoints();

//     // 输出分配结果
//     void printResults() const;
// };

// #endif // ASSIGNMENT_H


// Assignment.h
#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

#include <vector>
#include <cmath>
#include <limits>
#include <iostream>

struct CityParameters {
    std::vector<int> x;
    std::vector<int> y;
    std::vector<int> area;
    std::vector<int> demand1;
    std::vector<int> demand2;
};

struct CarParameters {
    std::vector<int> x;
    std::vector<int> y;
    std::vector<int> ability; // 0代表一种能力，1代表另一种能力
    std::vector<int> power;   // 电量0-100
};

struct Point_ugv {
    double x, y;
    int ability; // 0或1，表示小车的能力类型
    int power;   // 小车的电量
};

struct Region {
    double x, y;
    double area;
    int demand1; // 对第一种能力的需求
    int demand2; // 对第二种能力的需求
};

class Assignment {
public:
    Assignment(const CityParameters& cityParams, const CarParameters& carParams);
    void distributePoints();
    void printResults() const;
    std::vector<int> getPointRegionIDs() const;

private:
    double euclideanDistance(const Point_ugv& a, const Region& b);
    double priorityScore(const Point_ugv& point, const Region& region);
    void initializeRegions();
    void initializePoints();

    CityParameters cityParams;
    CarParameters carParams;
    std::vector<Region> regions;
    std::vector<Point_ugv> points;
    std::vector<std::vector<int>> assignments; // 每个分区分配的小车点索引
};

#endif
