#include "ros/ros.h"
struct Point2D {
    float x;
    float y;
};
int main(int argc, char **argv)
{
    // 设置编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "Param_initial_node");
    // 实例化ROS句柄
    ros::NodeHandle nh;

    std::vector<int> city_x;
    std::vector<int> city_y;
    std::vector<int> city_p0;
    std::vector<int> city_p1;
    std::vector<int> city_s;

    std::vector<int> car_x;
    std::vector<int> car_y;
    std::vector<int> car_p;
    std::vector<int> car_q;
    std::vector<int> car_q_real;


    // 初始化任务序列点
    std::vector<int> task_point;
    task_point={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

    city_x = {-46,-18,-32,-28,-15,8,4,42,16,28,-31,-32,-40,-39,-14,-2,12,34,34,40};

    city_y = {17,32,46,17,45,18,46,19,5,30,-15,-35,-45,-18,-42,-37,-37,-49,-20,-13};

    // 设置城市子区域
    city_p0 = {1,2,2,1,0,0,2,0,1,0,1,1,1,1,1,1,1,1,1,1};

    // 设置城市子区域
    city_p1 = {2,1,1,1,1,1,0,1,1,1,2,1,0,2,1,0,1,2,2,2};   

    // 设置城市子区域的面积
    city_s  = {150,142,147,130,153,145,138,129,133,128,117,134,133,145,139,128,141,156,139,148};

    nh.setParam("city_x",city_x);
    nh.setParam("city_y",city_y);
    nh.setParam("city_p0",city_p0);
    nh.setParam("city_p1",city_p1);
    nh.setParam("city_s",city_s);

    // 设置小车的横坐标
    car_x = {0,0,0,0,-2,-2,-2,-2,2,2,2,2};
    // 设置小车的纵坐标
    car_y = {-5,0,5,10,-5,0,5,10,-5,0,5,10};
    // 设置小车的种类,1为轮式，0为履带式，第一二次测试
    car_p = {0,0,0,0,0,0,1,1,1,1,1,1};
    // 设置小车的电量，，第一二次测试
    car_q = {98,88,93,95,92,89,98,88,93,95,92,89};

    car_q_real = {0,0,0,0,0,0,0,0,0,0,0,0}; 

    nh.setParam("car_x",car_x);
    nh.setParam("car_y",car_y);
    nh.setParam("car_q",car_q);

    nh.setParam("car_q_real",car_q_real);

    nh.setParam("task_point",task_point);
    
    nh.setParam("car_p",car_p);

    ros::spin();
    return 0;



}
