#include "compute.h"
#include <cmath>
Compute::Compute(const std::vector<int>& city_x, const std::vector<int>& city_y,
                 const std::vector<int>& city_p0, const std::vector<int>& city_p1,
                 const std::vector<int>& city_s, const std::vector<int>& city_ID,
                 const std::vector<int>& car_x, const std::vector<int>& car_y,
                 const std::vector<int>& car_p, const std::vector<int>& car_q,
                 const std::vector<int>& car_ID)
    : city_x(city_x), city_y(city_y), city_p0(city_p0), city_p1(city_p1),
      city_s(city_s), city_ID(city_ID), car_x(car_x), car_y(car_y),
      car_p(car_p), car_q(car_q), car_ID(car_ID) {}

double Compute::euclideanDistance(int x1, int y1, int x2, int y2) const {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

std::vector<int> Compute::computeAssignments() const {
    std::vector<int> assignments(car_x.size(), -1);  // -1表示未分配
    std::map<int, std::vector<int>> cityMap;          // 城市ID到城市点索引的映射
    std::map<int, std::vector<int>> carMap;           // 小车ID到小车点索引的映射

    // 构建城市点和小车点的映射
    for (size_t i = 0; i < city_ID.size(); ++i) {
        cityMap[city_ID[i]].push_back(i);
    }
    for (size_t i = 0; i < car_ID.size(); ++i) {
        carMap[car_ID[i]].push_back(i);
    }
    // 输出 cityMap 的值
    std::cout << "cityMap:" << std::endl;
    for (const auto& pair : cityMap) {
        std::cout << "City ID: " << pair.first << ", Indices: ";
        for (int index : pair.second) {
            std::cout << index << " ";
        }
        std::cout << std::endl;
    }

    // 输出 carMap 的值
    std::cout << "carMap:" << std::endl;
    for (const auto& pair : carMap) {
        std::cout << "Car ID: " << pair.first << ", Indices: ";
        for (int index : pair.second) {
            std::cout << index << " ";
        }
        std::cout << std::endl;
    }

    // 记录已分配的小车
    std::vector<bool> carAssigned(car_x.size(), false);
    std::vector<bool> cityAssigned(city_x.size(), false);
    for(auto it = carMap.begin();it!=carMap.end();++it){
        int carID = it->first;
        const std::vector<int> carIndex = it->second;
        //找到对应id的城市索引
        if(cityMap.find(carID)!=cityMap.end()){
            const std::vector<int> cityIndex = cityMap[carID];
            int len_carIndex = carIndex.size();
            std::cout<<"len_carIndex:"<<len_carIndex<<std::endl;
            int len_cityIndex = cityIndex.size();
            std::cout<<"len_cityIndex:"<<len_cityIndex<<std::endl;
            int length = len_carIndex;
            int len_c = len_cityIndex;
            for (int i = 0; i < length; i++) {
                if(len_cityIndex==0&&len_carIndex!=0)
                {
                    assignments[carIndex[i]] = i%len_c;
                    // assignments[carIndex[i]] = 100;
                    len_carIndex--;
                }
                
                if(len_cityIndex!=0 && len_carIndex!=0)
                {              
                   int m = carIndex[i];
                   std::cout<<"carIndex:"<<m<<std::endl;
                   int n = cityIndex[i]; 
                   std::cout<<"cityIndex:"<<n<<std::endl;
                   assignments[m] = n ;
                    len_cityIndex--;
                    len_carIndex--;
                }

            }
        }
    }

    return assignments;
}
