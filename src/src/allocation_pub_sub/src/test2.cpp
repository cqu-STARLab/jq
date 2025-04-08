// // main.cpp
// #include "allance.h"

// int main() {
//     // 城市参数初始化
//     CityParameters cityParams = {
//         {0, 5},           // 分区的x坐标
//         {0, 5},           // 分区的y坐标
//         {100, 100},       // 分区的面积
//         {2, 1},           // 第一种能力的需求数量
//         {1, 2}            // 第二种能力的需求数量
//     };

//     // 小车参数初始化
//     CarParameters carParams = {
//         {1, 2, 3},        // 小车的x坐标
//         {2, 3, 4},        // 小车的y坐标
//         {1, 0, 1},        // 第一种能力
//         {0, 1, 1}         // 第二种能力
//     };

//     // 创建Assignment对象并执行分配
//     Assignment assignment(cityParams, carParams);
//     assignment.distributePoints(); // 执行分配
//     assignment.printResults();     // 输出分配结果

//     return 0;
// }
// main.cpp
// main.cpp
// main.cpp
#include "allance.h"

int main() {
    std::vector<int> city_id;
    // 初始化城市分区参数
    CityParameters cityParams = {
        {0, 5, 10},      // 分区的x坐标
        {0, 5, 10},      // 分区的y坐标
        {100, 150, 200}, // 分区的面积
        {2, 1, 2},       // 第一种能力的需求数量
        {1, 2, 1}        // 第二种能力的需求数量
    };

    // 初始化小车参数，数量大于分区数量的两倍
    CarParameters carParams = {
        {1, 2, 3, 7, 8, 9, 11},    // 小车的x坐标
        {2, 3, 4, 8, 9, 10, 12},   // 小车的y坐标
        {0, 1, 0, 0, 1, 1, 0},     // 小车的能力类型
        {50, 60, 70, 80, 90, 100, 30} // 小车的电量
    };

    // 创建Assignment对象并执行分配
    Assignment assignment(cityParams, carParams);
    assignment.distributePoints(); // 执行分配
    assignment.printResults();     // 输出分配结果
    city_id = assignment.getPointRegionIDs();
    for (const int& value : city_id) {
        std::cout <<"id"<< value << std::endl;
    }

    return 0;
}
