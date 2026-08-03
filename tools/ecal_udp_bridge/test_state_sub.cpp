/**
 * 测试: 直接用 eCAL protobuf subscriber 订阅 mujoco_state
 */
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <robot_sdk.pb.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

static std::atomic<int> g_count{0};

int main(int argc, char* argv[])
{
    eCAL::Initialize(argc, argv, "test_state_sub");

    eCAL::protobuf::CSubscriber<robot_sdk::pb::RobotState> sub("mujoco_state");

    sub.AddReceiveCallback(
        [](const char* topic_name, const robot_sdk::pb::RobotState& msg,
           const long long time, const long long clock, const long long id)
        {
            int count = ++g_count;
            if (count <= 5 || count % 100 == 0) {
                std::cout << "[Recv] #" << count
                          << " q_abad_size=" << msg.q_abad_size()
                          << " quat_size=" << msg.quat_size()
                          << " serialized_size=" << msg.ByteSizeLong()
                          << std::endl;
            }
        });

    std::cout << "[Test] Subscribed to mujoco_state, waiting..." << std::endl;

    for (int i = 0; i < 30 && eCAL::Ok(); i++) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "[Test] " << i+1 << "s elapsed, received " << g_count.load() << " msgs" << std::endl;
    }

    eCAL::Finalize();
    return 0;
}
