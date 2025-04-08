// // Assignment.cpp
// #include "allance.h"

// // 计算两点之间的欧氏距离
// double Assignment::euclideanDistance(const Point_ugv& a, const Region& b) {
//     return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
// }

// // 根据点的能力和距离，计算分配优先级（可以自定义评分函数）
// double Assignment::priorityScore(const Point_ugv& point, const Region& region) {
//     double distance = euclideanDistance(point, region);
//     return 1.0 / (distance + 1e-9);  // 简单评分机制，避免除零
// }

// // 初始化分区
// void Assignment::initializeRegions() {
//     for (size_t i = 0; i < cityParams.x.size(); ++i) {
//         regions.push_back({static_cast<double>(cityParams.x[i]),
//                            static_cast<double>(cityParams.y[i]),
//                            static_cast<double>(cityParams.area[i]),
//                            cityParams.demand1[i],
//                            cityParams.demand2[i]});
//     }
// }

// // 初始化小车点
// void Assignment::initializePoints() {
//     for (size_t i = 0; i < carParams.x.size(); ++i) {
//         points.push_back({static_cast<double>(carParams.x[i]),
//                           static_cast<double>(carParams.y[i]),
//                           carParams.p[i],
//                           carParams.q[i]});
//     }
// }

// // 构造函数：初始化城市和小车参数
// Assignment::Assignment(const CityParameters& cityParams, const CarParameters& carParams)
//     : cityParams(cityParams), carParams(carParams) {
//     initializeRegions();  // 初始化分区
//     initializePoints();   // 初始化小车点
//     assignments.resize(regions.size());  // 初始化分配记录
// }

// // 分配点到分区
// void Assignment::distributePoints() {
//     for (size_t i = 0; i < points.size(); ++i) {
//         auto& point = points[i];
//         double bestScore = -std::numeric_limits<double>::infinity();
//         int bestRegionIndex = -1;

//         for (size_t j = 0; j < regions.size(); ++j) {
//             auto& region = regions[j];
//             // 检查是否满足两种能力的需求
//             if ((region.demand1 > 0 && point.ability1 == 1) ||
//                 (region.demand2 > 0 && point.ability2 == 1)) {
//                 double score = priorityScore(point, region);

//                 if (score > bestScore) {
//                     bestScore = score;
//                     bestRegionIndex = j;
//                 }
//             }
//         }

//         // 分配点到最佳分区，并减少对应能力的需求
//         if (bestRegionIndex != -1) {
//             assignments[bestRegionIndex].push_back(i);
//             if (point.ability1 == 1) --regions[bestRegionIndex].demand1;
//             if (point.ability2 == 1) --regions[bestRegionIndex].demand2;
//         } else {
//             std::cerr << "No suitable region found for point (" << point.x << ", " << point.y << ")\n";
//         }
//     }
// }

// // 输出分配结果
// void Assignment::printResults() const {
//     for (size_t i = 0; i < regions.size(); ++i) {
//         const auto& region = regions[i];
//         std::cout << "Region center (" << region.x << ", " << region.y << "):\n";
//         for (int index : assignments[i]) {
//             const auto& point = points[index];
//             std::cout << "  Point (" << point.x << ", " << point.y << ") Ability1: " 
//                       << point.ability1 << " Ability2: " << point.ability2 << "\n";
//         }
//     }
// }


// Assignment.cpp
#include "allance.h"

// 计算两点之间的欧氏距离
double Assignment::euclideanDistance(const Point_ugv& a, const Region& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

// 根据点的能力、电量和距离，计算分配优先级
double Assignment::priorityScore(const Point_ugv& point, const Region& region) {
    double distance = euclideanDistance(point, region);
    double areaFactor = region.area;  // 面积因子，面积越大优先级越高
    double powerFactor = point.power; // 电量因子，电量越高优先级越高
    return (powerFactor / (distance + 1e-9)) * areaFactor; // 计算优先级
}

// 初始化分区
void Assignment::initializeRegions() {
    for (size_t i = 0; i < cityParams.x.size(); ++i) {
        regions.push_back({
            static_cast<double>(cityParams.x[i]),
            static_cast<double>(cityParams.y[i]),
            static_cast<double>(cityParams.area[i]),
            cityParams.demand1[i],
            cityParams.demand2[i]
        });
    }
}

// 初始化小车点
void Assignment::initializePoints() {
    for (size_t i = 0; i < carParams.x.size(); ++i) {
        points.push_back({
            static_cast<double>(carParams.x[i]),
            static_cast<double>(carParams.y[i]),
            carParams.ability[i],
            carParams.power[i]
        });
    }
}

// 构造函数：初始化城市和小车参数
Assignment::Assignment(const CityParameters& cityParams, const CarParameters& carParams)
    : cityParams(cityParams), carParams(carParams) {
    initializeRegions();  // 初始化分区
    initializePoints();   // 初始化小车点
    assignments.resize(regions.size());  // 初始化分配记录
}

// 分配点到分区
void Assignment::distributePoints() {
    std::vector<bool> assigned(points.size(), false); // 记录每个点是否已分配

    // 第一步：确保每个分区至少分配一个小车
    for (size_t j = 0; j < regions.size(); ++j) {
        auto& region = regions[j];
        double bestScore = -std::numeric_limits<double>::infinity();
        int bestPointIndex = -1;

        for (size_t i = 0; i < points.size(); ++i) {
            if (assigned[i]) continue; // 如果点已分配，跳过
            auto& point = points[i];
            double score = priorityScore(point, region);

            // 优先满足区域的能力需求
            if ((region.demand1 > 0 && point.ability == 0) ||
                (region.demand2 > 0 && point.ability == 1)) {
                if (score > bestScore) {
                    bestScore = score;
                    bestPointIndex = i;
                }
            }
        }

        // 如果找到了最佳小车，进行分配
        if (bestPointIndex != -1) {
            assignments[j].push_back(bestPointIndex);
            assigned[bestPointIndex] = true;

            // 减少需求
            if (points[bestPointIndex].ability == 0) --region.demand1;
            else --region.demand2;
        }
    }

    // 第二步：使用剩余的小车进行最优分配
    for (size_t i = 0; i < points.size(); ++i) {
        if (!assigned[i]) {
            auto& point = points[i];
            double bestScore = -std::numeric_limits<double>::infinity();
            int bestRegionIndex = -1;

            for (size_t j = 0; j < regions.size(); ++j) {
                auto& region = regions[j];
                double score = priorityScore(point, region);

                // 优先分配需求较大的区域
                if (score > bestScore) {
                    bestScore = score;
                    bestRegionIndex = j;
                }
            }

            // 分配未被分配的小车
            if (bestRegionIndex != -1) {
                assignments[bestRegionIndex].push_back(i);
                assigned[i] = true;

                // 根据能力类型减少分区需求
                if (points[i].ability == 0 && regions[bestRegionIndex].demand1 > 0) {
                    --regions[bestRegionIndex].demand1;
                } else if (points[i].ability == 1 && regions[bestRegionIndex].demand2 > 0) {
                    --regions[bestRegionIndex].demand2;
                }
            }
        }
    }
}

// 输出分配结果
void Assignment::printResults() const {
    for (size_t i = 0; i < regions.size(); ++i) {
        const auto& region = regions[i];
        std::cout << "Region center (" << region.x << ", " << region.y << "):\n";
        for (int index : assignments[i]) {
            const auto& point = points[index];
            std::cout << "  Point (" << point.x << ", " << point.y << ") Ability: " 
                      << point.ability << " Power: " << point.power << "\n";
        }
    }
}


// 返回每个点的分区 ID
std::vector<int> Assignment::getPointRegionIDs() const {
    std::vector<int> pointRegionIDs(points.size(), -1); // 初始化为-1，表示尚未分配
    
    // 遍历所有分区
    for (size_t regionIndex = 0; regionIndex < assignments.size(); ++regionIndex) {
        // 遍历每个分区中的点
        for (int pointIndex : assignments[regionIndex]) {
            pointRegionIDs[pointIndex] = regionIndex+1; // 记录每个点的分区 ID
        }
    }

    return pointRegionIDs;
}