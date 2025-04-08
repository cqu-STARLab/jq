#include <ros/ros.h>
#include <zmq.hpp>
#include "robot_message.pb.h"
#include "string.h"

std::string server_ip, connection_type, zmq_socket_type;
int server_port;
int timeout_ms = 10000;  // 每次最多等 10 秒
int max_retries = 3;     // 最大重传次数
bool success = false;

int main(int argc, char** argv) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    ros::init(argc, argv, "robot_sender_node");
    ros::NodeHandle private_nh("~");

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

    // 构造 Protobuf 消息
    decentralized::RobotStatus status;
    status.set_robot_id(1);
    status.set_x(1.23);
    status.set_y(4.56);
    status.set_z(0.0);
    status.set_energy(87);  // 电量百分比

    std::string serialized_data;
    if (!status.SerializeToString(&serialized_data)) {
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
