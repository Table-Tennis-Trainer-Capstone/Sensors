#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <sstream>
#include <iomanip>

// ---------------------------------------------------------------------------
// Packet layout (little-endian, packed)
//
//  Byte(s)  Field
//  0-1      Magic:       0xAA 0x55
//  2        Version:     0x01
//  3        Sequence:    rolling 0-255
//  4-7      Timestamp:   milliseconds since node start (uint32)
//  8        Num joints:  number of valid joint entries (uint8, max 8)
//  9-40     Positions:   8 x float32 (radians)
//  41-72    Velocities:  8 x float32 (rad/s)
//  73-74    CRC16:       CCITT over bytes 0-72
//  75       End:         0xFF
//
//  Total: 76 bytes
// ---------------------------------------------------------------------------

#pragma pack(push, 1)
struct HardwarePacket {
    uint8_t  magic[2]       = {0xAA, 0x55};
    uint8_t  version        = 0x01;
    uint8_t  seq            = 0;
    uint32_t timestamp_ms   = 0;
    uint8_t  num_joints     = 0;
    float    positions[8]   = {};
    float    velocities[8]  = {};
    uint16_t crc            = 0;
    uint8_t  end            = 0xFF;
};
#pragma pack(pop)

static uint16_t crc16_ccitt(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

class HardwareNode : public rclcpp::Node
{
public:
    HardwareNode() : Node("hardware_node"), seq_(0)
    {
        this->declare_parameter("stm_ip",   "192.168.1.50");
        this->declare_parameter("stm_port", 5000);
        this->declare_parameter("joint_topic", "/joint_states");

        stm_ip_   = this->get_parameter("stm_ip").as_string();
        stm_port_ = this->get_parameter("stm_port").as_int();
        std::string topic = this->get_parameter("joint_topic").as_string();

        start_time_ = this->now();

        // Open UDP socket
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket");
            throw std::runtime_error("UDP socket creation failed");
        }

        memset(&stm_addr_, 0, sizeof(stm_addr_));
        stm_addr_.sin_family      = AF_INET;
        stm_addr_.sin_port        = htons(stm_port_);
        stm_addr_.sin_addr.s_addr = inet_addr(stm_ip_.c_str());

        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            topic, 10,
            std::bind(&HardwareNode::jointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Hardware node ready — sending to %s:%d",
                    stm_ip_.c_str(), stm_port_);
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Packet size: %zu bytes", sizeof(HardwarePacket));
    }

    ~HardwareNode()
    {
        if (sock_ >= 0) close(sock_);
    }

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        HardwarePacket pkt;

        // Timestamp (ms since node start)
        auto elapsed = this->now() - start_time_;
        pkt.timestamp_ms = static_cast<uint32_t>(elapsed.nanoseconds() / 1'000'000);

        pkt.seq = seq_++;

        uint8_t n = static_cast<uint8_t>(std::min(msg->name.size(), size_t(8)));
        pkt.num_joints = n;

        for (uint8_t i = 0; i < n; i++) {
            if (i < msg->position.size())
                pkt.positions[i]  = static_cast<float>(msg->position[i]);
            if (i < msg->velocity.size())
                pkt.velocities[i] = static_cast<float>(msg->velocity[i]);
        }

        // CRC over everything except the crc and end fields
        size_t crc_len = offsetof(HardwarePacket, crc);
        pkt.crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pkt), crc_len);

        ssize_t sent = sendto(sock_, &pkt, sizeof(pkt), 0,
                              reinterpret_cast<sockaddr*>(&stm_addr_),
                              sizeof(stm_addr_));

        if (sent < 0) {
            RCLCPP_WARN(this->get_logger(), "UDP send failed");
        } else {
            std::ostringstream ss;
            for (uint8_t i = 0; i < n; i++) {
                ss << std::fixed << std::setprecision(3) << pkt.positions[i] << " ";
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Sent UDP [seq=%3d] joints: [ %s]", pkt.seq, ss.str().c_str());
        }
    }

    int sock_;
    sockaddr_in stm_addr_;
    std::string stm_ip_;
    int stm_port_;
    uint8_t seq_;
    rclcpp::Time start_time_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareNode>());
    rclcpp::shutdown();
    return 0;
}
