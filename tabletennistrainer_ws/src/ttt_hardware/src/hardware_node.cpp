#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <array>
#include <string>
#include <unordered_map>
#include <algorithm>

// ---------------------------------------------------------------------------
// Joint configuration — Applies URDF-to-STM32 physical offsets
// ---------------------------------------------------------------------------
struct JointConfig {
    float offset_deg;     // Physical encoder offset
    float scale;          // +1.0 or -1.0 to reverse direction
    float min_deg;        // Safe minimum for the STM motor
    float max_deg;        // Safe maximum for the STM motor
};

static const JointConfig JOINT_CFG[5] = {
    {   0.0f,  1.0f, -120.0f, 120.0f }, // M1: Base         (ros_deg)
    { 105.0f, -1.0f,    0.0f, 170.0f }, // M2: Shoulder     (105 - ros_deg)
    {  90.0f,  1.0f,    0.0f, 150.0f }, // M3: Elbow        (90 + ros_deg)
    {   0.0f,  1.0f,  -90.0f,  90.0f }, // M4: Wrist pitch  (ros_deg)
    {   0.0f,  1.0f,  -90.0f,  90.0f }  // M5: Wrist roll   (ros_deg)
};

static const std::array<std::string, 5> JOINT_ORDER = {
    "BaseRotate_0", "UpperArmRotate_0", "ForeArmRotate_0", "WristRotate_0", "PaddleRotate_0",
};

class HardwareNode : public rclcpp::Node
{
public:
    HardwareNode() : Node("hardware_node")
    {
        this->declare_parameter("stm_ip",      "192.168.1.100");
        this->declare_parameter("stm_port",    7777);
        this->declare_parameter("joint_topic", "/joint_states");

        stm_ip_   = this->get_parameter("stm_ip").as_string();
        stm_port_ = this->get_parameter("stm_port").as_int();
        std::string topic = this->get_parameter("joint_topic").as_string();

        // Initialize tracking array to impossible values so the first packet always sends
        for(int i=0; i<5; i++) last_sent_positions_[i] = -999.0f;

        // UDP socket
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket: %s", strerror(errno));
            throw std::runtime_error("UDP socket creation failed");
        }

        struct timeval tv{1, 0};
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        memset(&stm_addr_, 0, sizeof(stm_addr_));
        stm_addr_.sin_family      = AF_INET;
        stm_addr_.sin_port        = htons(static_cast<uint16_t>(stm_port_));
        stm_addr_.sin_addr.s_addr = inet_addr(stm_ip_.c_str());

        // Wake up motors on boot
        RCLCPP_INFO(this->get_logger(), "Sending initial Home command (H)...");
        sendUdp("H\n");

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            topic, 10,
            std::bind(&HardwareNode::jointCallback, this, std::placeholders::_1));

        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/stm_cmd", 10,
            std::bind(&HardwareNode::cmdCallback, this, std::placeholders::_1));

        reply_running_ = true;
        reply_thread_  = std::thread(&HardwareNode::replyListener, this);

        RCLCPP_INFO(this->get_logger(), "Hardware node ready — Filtering duplicate packets.");
    }

    ~HardwareNode()
    {
        reply_running_ = false;
        if (sock_ >= 0) shutdown(sock_, SHUT_RDWR);
        if (reply_thread_.joinable()) reply_thread_.join();
        if (sock_ >= 0) close(sock_);
    }

private:
    void sendUdp(const std::string & cmd)
    {
        ssize_t sent = sendto(sock_, cmd.c_str(), cmd.size(), 0,
                              reinterpret_cast<sockaddr *>(&stm_addr_),
                              sizeof(stm_addr_));
        if (sent < 0) {
            RCLCPP_WARN(this->get_logger(), "UDP send failed: %s", strerror(errno));
        }
    }

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        constexpr double RAD_TO_DEG = 180.0 / M_PI;
        constexpr float DEADBAND_DEG = 0.1f; // Ignore changes smaller than 0.1 degrees

        std::unordered_map<std::string, double> pos_map;
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (i < msg->position.size()) pos_map[msg->name[i]] = msg->position[i];
        }

        float new_positions[5] = {0};
        bool valid_joint[5] = {false};
        bool should_send = false;

        // 1. Calculate new target positions and check if they moved
        for (size_t i = 0; i < JOINT_ORDER.size(); i++) {
            auto it = pos_map.find(JOINT_ORDER[i]);
            if (it != pos_map.end()) {
                double ros_deg = it->second * RAD_TO_DEG;
                
                // Apply offsets and clamps
                double stm_val = JOINT_CFG[i].offset_deg + (JOINT_CFG[i].scale * ros_deg);
                new_positions[i] = std::max(static_cast<double>(JOINT_CFG[i].min_deg), 
                                   std::min(static_cast<double>(JOINT_CFG[i].max_deg), stm_val));
                
                valid_joint[i] = true;

                // Check against the deadband threshold
                if (std::abs(new_positions[i] - last_sent_positions_[i]) > DEADBAND_DEG) {
                    should_send = true;
                }
            }
        }

        // 2. If nothing moved significantly, drop the packet and exit silently!
        if (!should_send) {
            return; 
        }

        // 3. Movement detected! Construct the string, update state, and send.
        std::ostringstream ss;
        ss << "M";
        for (size_t i = 0; i < 5; i++) {
            if (valid_joint[i]) {
                ss << " " << std::fixed << std::setprecision(2) << new_positions[i];
                last_sent_positions_[i] = new_positions[i]; // Remember this position
            } else {
                ss << " _";
            }
        }
        ss << "\n";

        std::string cmd = ss.str();
        sendUdp(cmd);

        // Print to console only when we actually send a packet
        RCLCPP_INFO(this->get_logger(), "Sent: %s", cmd.c_str());
    }

    void cmdCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string & data = msg->data;

        if (data == "home") {
            sendUdp("H\n");
            RCLCPP_INFO(this->get_logger(), "Sent: H (home all motors)");
        } else if (data == "estop") {
            sendUdp("E\n");
            RCLCPP_INFO(this->get_logger(), "Sent: E (emergency stop)");
        } else if (data.size() >= 3 && data[0] == 'H' && data[1] == ' ') {
            sendUdp(data + "\n");
            RCLCPP_INFO(this->get_logger(), "Sent: %s", data.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown /stm_cmd: '%s'", data.c_str());
        }
    }

    void replyListener()
    {
        char buf[256];
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);

        while (reply_running_) {
            ssize_t n = recvfrom(sock_, buf, sizeof(buf) - 1, 0,
                                 reinterpret_cast<sockaddr *>(&from), &from_len);
            if (n > 0) {
                buf[n] = '\0';
                while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) buf[--n] = '\0';
                RCLCPP_INFO(this->get_logger(), "STM reply: %s", buf);
            }
        }
    }

    int          sock_{-1};
    sockaddr_in  stm_addr_{};
    std::string  stm_ip_;
    int          stm_port_;
    
    // Tracks the last sent positions to prevent network spam
    float        last_sent_positions_[5];

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         cmd_sub_;

    std::thread        reply_thread_;
    std::atomic<bool>  reply_running_{false};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareNode>());
    rclcpp::shutdown();
    return 0;
}