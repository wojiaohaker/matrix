/**
 * eCAL → UDP 桥接器
 * 
 * 功能: 订阅 eCAL topic "mujoco_state" (robot_sdk.pb.RobotState),
 *       将收到的序列化 protobuf 字节流通过 UDP 转发到指定端口。
 * 
 * 用途: CarlaUnreal 不集成 eCAL，通过 UDP 25001 接收 RobotState 数据。
 * 
 * 数据流:
 *   robot_mujoco ──eCAL [mujoco_state]──→ bridge ──UDP 25001──→ CarlaUnreal
 * 
 * 编译:
 *   g++ -O2 -o ecal_udp_bridge main.cpp -lecal_core -lpthread
 * 
 * 运行:
 *   ./ecal_udp_bridge [target_ip] [target_port]
 *   默认: 127.0.0.1 25001
 */

#include <ecal/ecal.h>
#include <ecal/ecal_subscriber.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

// 统计信息
static std::atomic<uint64_t> g_msg_count{0};
static std::atomic<uint64_t> g_last_msg_size{0};

int main(int argc, char* argv[])
{
    // 解析参数
    std::string target_ip   = "127.0.0.1";
    int         target_port = 25001;

    if (argc >= 2) target_ip   = argv[1];
    if (argc >= 3) target_port = std::stoi(argv[2]);

    std::cout << "[eCAL-UDP Bridge] 启动" << std::endl;
    std::cout << "  eCAL topic: mujoco_state" << std::endl;
    std::cout << "  UDP target: " << target_ip << ":" << target_port << std::endl;

    // 初始化 eCAL
    eCAL::Initialize(argc, argv, "ecal_udp_bridge");
    std::cout << "[eCAL-UDP Bridge] eCAL 初始化完成" << std::endl;

    // 创建 UDP socket
    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        std::cerr << "[ERROR] 创建 UDP socket 失败: " << strerror(errno) << std::endl;
        eCAL::Finalize();
        return 1;
    }

    struct sockaddr_in dest_addr{};
    dest_addr.sin_family      = AF_INET;
    dest_addr.sin_port        = htons(static_cast<uint16_t>(target_port));
    dest_addr.sin_addr.s_addr = inet_addr(target_ip.c_str());

    // 创建 eCAL 原始订阅器 (接收序列化后的 protobuf 字节)
    eCAL::CSubscriber subscriber("mujoco_state");

    // 注册接收回调 - 零拷贝转发
    subscriber.AddReceiveCallback(
        [&](const char* /*topic_name*/, const struct eCAL::SReceiveCallbackData* data)
        {
            if (data == nullptr || data->buf == nullptr || data->size <= 0) {
                return;
            }

            // 直接转发原始字节到 UDP
            ssize_t sent = sendto(
                udp_sock,
                data->buf,
                data->size,
                0,
                reinterpret_cast<struct sockaddr*>(&dest_addr),
                sizeof(dest_addr));

            if (sent > 0) {
                g_msg_count++;
                g_last_msg_size = static_cast<uint64_t>(data->size);
            }
        });

    std::cout << "[eCAL-UDP Bridge] 订阅器已创建，等待数据..." << std::endl;

    // 主循环 - 打印统计信息
    uint64_t last_count = 0;
    while (eCAL::Ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(2));

        uint64_t current_count = g_msg_count.load();
        uint64_t rate          = (current_count - last_count) / 2;  // 2秒间隔
        last_count             = current_count;

        if (current_count > 0) {
            std::cout << "[Bridge] 已转发: " << current_count << " 帧"
                      << " | 速率: " << rate << " Hz"
                      << " | 最后帧大小: " << g_last_msg_size.load() << " bytes"
                      << std::endl;
        }
    }

    std::cout << "[eCAL-UDP Bridge] 退出" << std::endl;

    close(udp_sock);
    eCAL::Finalize();
    return 0;
}
