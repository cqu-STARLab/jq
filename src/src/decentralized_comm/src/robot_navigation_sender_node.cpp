#include <ros/ros.h>
#include <zmq.hpp>
#include "robot_message.pb.h"
#include "string.h"

std::string server_ip, connection_type, zmq_socket_type;
int server_port;
int timeout_ms = 10000;  // 每次最多等 10 秒
int max_retries = 3;     // 最大重传次数
bool success = false;
std::vector<int> task_point={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
struct Point2D {
    float x;
    float y;
};
std::vector<std::vector<Point2D>> city_vertex=
{{{0,0},{3,5}},{{1,2},{4,7}},{{2,1},{5,6}},{{3,3},{6,8}},
 {{4,0},{7,5}},{{5,2},{8,7}},{{6,1},{9,6}},{{7,3},{10,8}},
 {{8,0},{11,5}},{{9,2},{13,7}},{{10,1},{13,5}},{{11,3},{13,5}},
 {{12,0},{15,5}},{{13,2},{16,7}},{{14,1},{17,6}},{{15,3},{18,8}},
 {{16,0},{19,5}},{{17,2},{20,7}},{{18,1},{20,5}},{{19,3},{22,5}},
};
std::vector<int> city_x = {-46,-18,-32,-28,-15,8,4,42,16,28,-31,-32,-40,-39,-14,-2,12,34,34,40};
std::vector<int> city_y = {17,32,46,17,45,18,46,19,5,30,-15,-35,-45,-18,-42,-37,-37,-49,-20,-13};


int main(int argc, char** argv) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    ros::init(argc, argv, "navigation_sender_node");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    ROS_INFO("robot_sender_node started!");

    // 参数读取
    private_nh.param<std::string>("server_ip", server_ip, "127.0.0.1");
    private_nh.param<int>("server_port", server_port, 5555);
    private_nh.param<std::string>("connection_type", connection_type, "tcp");
    private_nh.param<int>("timeout_ms", timeout_ms, 10000);
    private_nh.param<int>("max_retries", max_retries, 3);
    private_nh.param<std::string>("zmq_socket_type", zmq_socket_type, "dealer");

    std::string zmq_address = connection_type + "://" + server_ip + ":" + std::to_string(server_port);
    ROS_INFO_STREAM("Connecting to " << zmq_address);
    while(task_point[0]==-1 && ros::ok()){
        nh.getParam("task_point",task_point);
    }
    int id = task_point[0];


    // 构造 Protobuf 消息
    decentralized::NavigationPoint point;
    point.set_robot_id(0);
    point.set_center_x1(city_x[0]);
    point.set_center_y1(city_y[0]);
    point.set_left_x2(city_vertex[id][0].x);
    point.set_left_y2(city_vertex[id][0].y);
    point.set_right_x3(city_vertex[id][1].x);
    point.set_right_y3(city_vertex[id][1].y);

    std::string serialized_data;
    if (!point.SerializeToString(&serialized_data)) {
        ROS_ERROR("Failed to serialize RobotStatus.");
        return 1;
    }

    zmq::context_t context(1);

    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        zmq::socket_t socket(context, ZMQ_REQ);
        socket.connect(zmq_address);

        // 设置接收超时
        socket.setsockopt(ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));

        // 发送请求
        zmq::message_t request(serialized_data.begin(), serialized_data.end());
        socket.send(request, zmq::send_flags::none);
        ROS_WARN_STREAM("send time: " << attempt);

        // 阻塞等待服务端回复（或超时）
        zmq::message_t reply;
        if (socket.recv(reply)) {
            decentralized::ServerAck ack;
            if (ack.ParseFromArray(reply.data(), reply.size())) {
                ROS_INFO_STREAM("Reply: code=" << ack.code() << ", msg=" << ack.message());
                success = true;
                break;
            } else {
                ROS_WARN("can't parse to ServerAck");
            }
        } else {
            ROS_WARN("recv failed, retrying...");
        }
    }

    if (!success) {
        ROS_ERROR("failed");
    }

    return 0;
}
