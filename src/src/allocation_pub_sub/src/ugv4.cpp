#include <ros/ros.h>
// 初始化任务目标点
std::vector<int> task_point;
int target = -1;
int main(int argc, char *argv[])
{
    // 设置编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "ugv4_node");
    // 实例化ROS句柄
    ros::NodeHandle nh;
    //通过参数服务器接收目标点
    nh.getParam("task_point",task_point); 
    target = task_point[3];
    std::cout<<"无人车节点4的目标任务点为"<<target<<std::endl;
    //xxxxx
    /* code */
    return 0;
}

