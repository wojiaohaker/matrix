/**
 * 测试: 向 mujoco_cmd 发送 RobotCmd，验证是否触发 mujoco_state 发布
 */
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <robot_sdk.pb.h>

#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char* argv[])
{
    eCAL::Initialize(argc, argv, "test_cmd_publisher");

    eCAL::protobuf::CPublisher<robot_sdk::pb::RobotCmd> pub("mujoco_cmd");

    std::cout << "[Test] Publishing RobotCmd to mujoco_cmd..." << std::endl;

    robot_sdk::pb::RobotCmd cmd;
    // 发送空命令即可触发状态发布

    int count = 0;
    while (eCAL::Ok() && count < 100) {
        pub.Send(cmd);
        count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz
    }

    std::cout << "[Test] Sent " << count << " commands" << std::endl;
    eCAL::Finalize();
    return 0;
}
