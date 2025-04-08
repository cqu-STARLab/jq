#include <iostream>
#include "compute.h"

int main() {
    // 城市参数初始化
    std::vector<int> city_x = {0, 2, 4, 6, 8, 1, 3, 5, 7, 9, 10, 12, 14, 16, 18};
    std::vector<int> city_y = {0, 2, 4, 6, 8, 1, 3, 5, 7, 9, 10, 12, 14, 16, 18};
    std::vector<int> city_p0 = {5, 3, 8, 6, 7, 4, 2, 9, 5, 3, 8, 6, 7, 4, 2};
    std::vector<int> city_p1 = {2, 4, 1, 3, 5, 7, 9, 6, 8, 2, 4, 1, 3, 5, 7};
    std::vector<int> city_s = {20, 15, 25, 10, 30, 10, 15, 20, 25, 30, 10, 20, 25, 30, 10};
    std::vector<int> city_ID = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8};

    // 小车参数初始化
    std::vector<int> car_x = {1, 3, 5, 7, 9, 11, 13, 15, 17, 19};
    std::vector<int> car_y = {1, 3, 5, 7, 9, 11, 13, 15, 17, 19};
    std::vector<int> car_p = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0}; // 0和1表示不同能力
    std::vector<int> car_q = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1}; // 0和1表示不同能力
    std::vector<int> car_ID = {1, 2, 3, 4, 5, 1, 2, 3, 4, 5}; // 对应分区ID

    // 创建Compute对象并执行分配
    Compute compute(city_x, city_y, city_p0, city_p1, city_s, city_ID,
                    car_x, car_y, car_p, car_q, car_ID);
    std::vector<int> assignments = compute.computeAssignments();

    // 输出分配结果
    for (size_t i = 0; i < assignments.size(); ++i) {
        std::cout << "Car " << i << " assigned to city " << assignments[i] << std::endl;
    }

    return 0;
}
