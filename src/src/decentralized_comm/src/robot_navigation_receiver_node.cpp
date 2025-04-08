#include <ros/ros.h>
#include <zmq.hpp>
#include "robot_message.pb.h"

std::string bind_ip, connection_type;
int bind_port;
int num_thread = 3;

int main(int argc, char** argv) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    ros::init(argc, argv, "navigation_receiver_node");
    ros::NodeHandle private_nh("~");

    // 参数读取
    private_nh.param<std::string>("bind_ip", bind_ip, "0.0.0.0");
    private_nh.param<int>("bind_port", bind_port, 5555);
    private_nh.param<std::string>("connection_type", connection_type, "tcp");
    private_nh.param<int>("num_thread", num_thread, 3);

    std::string bind_address = connection_type + "://" + bind_ip + ":" + std::to_string(bind_port);
    ROS_INFO_STREAM("REP server IP:" << bind_address);

    // 初始化 ZeroMQ
    zmq::context_t context(num_thread);
    zmq::socket_t socket(context, ZMQ_REP);
    socket.bind(bind_address);

    ROS_INFO("robot_receiver_node started, waiting for requests...");

    while (ros::ok()) {
        zmq::message_t request;

        // 阻塞接收来自客户端的请求
        if (!socket.recv(request)) {
            ROS_WARN("no message");
            continue;
        }

        decentralized::NavigationPoint status;
        if (status.ParseFromArray(request.data(), request.size())) {
            ROS_INFO("recv NavigationPoint: id=%d pos=(%.2f, %.2f, %.2f), energy=%d",
                     status.robot_id(), status.center_x1(), status.center_y1(), status.left_x2(), status.left_y2());

            // 构造回应消息
            decentralized::ServerAck ack;
            ack.set_code(0);
            ack.set_message("Received OK (REP)");
            ack.set_original_robot_id(status.robot_id());

            std::string ack_data;
            if (!ack.SerializeToString(&ack_data)) {
                ROS_WARN("ServerAck serl failed");
                continue;
            }

            // 发送响应
            socket.send(zmq::buffer(ack_data), zmq::send_flags::none);
            ROS_INFO("respond successfully");
        } else {
            ROS_WARN("parse RobotStatus fail");
            decentralized::ServerAck ack;
            ack.set_code(1);
            ack.set_message("Received Failed (REP)");
            ack.set_original_robot_id(status.robot_id());

            std::string ack_data;
            if (!ack.SerializeToString(&ack_data)) {
                ROS_WARN("ServerAck serl failed");
                continue;
            }
            socket.send(zmq::buffer(ack_data), zmq::send_flags::none);
        }
    }

    return 0;
}
