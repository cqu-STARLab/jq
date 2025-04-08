#ifndef COMPUTE_H
#define COMPUTE_H

#include <vector>
#include <map>
#include <algorithm>
#include <limits>
#include <tuple>
#include <iostream>
class Compute {
public:
    Compute(const std::vector<int>& city_x, const std::vector<int>& city_y,
            const std::vector<int>& city_p0, const std::vector<int>& city_p1,
            const std::vector<int>& city_s, const std::vector<int>& city_ID,
            const std::vector<int>& car_x, const std::vector<int>& car_y,
            const std::vector<int>& car_p, const std::vector<int>& car_q,
            const std::vector<int>& car_ID);

    std::vector<int> computeAssignments() const;

private:
    std::vector<int> city_x, city_y, city_p0, city_p1, city_s, city_ID;
    std::vector<int> car_x, car_y, car_p, car_q, car_ID;

    double euclideanDistance(int x1, int y1, int x2, int y2) const;
};

#endif // COMPUTE_H
