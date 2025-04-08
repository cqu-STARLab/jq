#include <ros/ros.h>
// 初始化任务目标点
std::vector<int> task_point;
std::vector<int> city_x;
std::vector<int> city_y;

int target = -1;
int x = -1;
int y = -1;
int main(int argc, char *argv[])
{
    // 设置编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "ugv1_node");
    // 实例化ROS句柄
    ros::NodeHandle nh;
    //通过参数服务器接收目标点
    nh.getParam("task_point",task_point); 
    target = task_point[0];
    nh.getParam("city_x",city_x);
    nh.getParam("city_y",city_y);
    x = city_x[target];
    y = city_y[target];

    std::cout<<"无人车节点1的目标任务点为"<<target<<std::endl;
    std::cout<<"无人车节点1的目标点x坐标"<<x<<std::endl;
    std::cout<<"无人车节点1的目标点y坐标"<<y<<std::endl;

    //xxxxx
    /* code */
    return 0;
}
