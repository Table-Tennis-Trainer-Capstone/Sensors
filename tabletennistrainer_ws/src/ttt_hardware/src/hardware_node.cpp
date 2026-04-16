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

// Joint name → M-command motor slot (m1..m5) mapping.
// Order must match the STM motor numbering:
//   m1 = Base   (Motor 1, AS5600 I2C1)
//   m2 = Shoulder (Motor 2, AS5600 I2C2)
//   m3 = Elbow  (Motor 3, AS5600 I2C3)
//   m4 = Wrist  (Motor 4, MG995 servo TIM3 CH1)
//   m5 = Wrist roll / Paddle (Motor 5, MG995 servo TIM3 CH2)
static const std::array<std::string, 5> JOINT_ORDER = {
    "BaseRotate_0",
    "UpperArmRotate_0",
    "ForeArmRotate_0",
    "WristRotate_0",
    "PaddleRotate_0",
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

        // UDP socket — also used for receiving STM replies (OS assigns source port
        // on first sendto, and the STM replies to that same source IP:port).
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket: %s", strerror(errno));
            throw std::runtime_error("UDP socket creation failed");
        }

        // 1-second receive timeout so the reply thread can exit cleanly.
        struct timeval tv{1, 0};
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        memset(&stm_addr_, 0, sizeof(stm_addr_));
        stm_addr_.sin_family      = AF_INET;
        stm_addr_.sin_port        = htons(static_cast<uint16_t>(stm_port_));
        stm_addr_.sin_addr.s_addr = inet_addr(stm_ip_.c_str());

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            topic, 10,
            std::bind(&HardwareNode::jointCallback, this, std::placeholders::_1));

        // /stm_cmd accepts "home", "estop", or "H <n>" to control the STM directly.
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/stm_cmd", 10,
            std::bind(&HardwareNode::cmdCallback, this, std::placeholders::_1));

        reply_running_ = true;
        reply_thread_  = std::thread(&HardwareNode::replyListener, this);

        RCLCPP_INFO(this->get_logger(), "Hardware node ready — sending to %s:%d",
                    stm_ip_.c_str(), stm_port_);
        RCLCPP_INFO(this->get_logger(), "Subscribed to joint topic: %s", topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribed to /stm_cmd for home/estop commands");
    }

    ~HardwareNode()
    {
        reply_running_ = false;
        if (sock_ >= 0) {
            // Unblock recvfrom by shutting down the socket.
            shutdown(sock_, SHUT_RDWR);
        }
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
        // Build name → position map (values are in radians from ROS/MoveIt).
        std::unordered_map<std::string, double> pos_map;
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (i < msg->position.size()) {
                pos_map[msg->name[i]] = msg->position[i];
            }
        }

        // Build: M <m1_deg> <m2_deg> <m3_deg> <m4_deg> <m5_deg>
        // Use "_" for any joint not present so the STM skips it.
        std::ostringstream ss;
        ss << "M";
        for (const auto & jname : JOINT_ORDER) {
            auto it = pos_map.find(jname);
            if (it != pos_map.end()) {
                double deg = it->second * (180.0 / M_PI);
                ss << " " << std::fixed << std::setprecision(2) << deg;
            } else {
                ss << " _";
            }
        }
        ss << "\n";

        std::string cmd = ss.str();
        sendUdp(cmd);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Sent: %s", cmd.c_str());
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
            // "H <n>" — home a specific motor
            sendUdp(data + "\n");
            RCLCPP_INFO(this->get_logger(), "Sent: %s", data.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown /stm_cmd: '%s'", data.c_str());
        }
    }

    // Logs replies from the STM.  The STM sends plain ASCII back to whichever
    // IP:port last sent it a packet, so we can recvfrom on the same socket.
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
                // Strip trailing CR/LF
                while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
                    buf[--n] = '\0';
                }
                RCLCPP_INFO(this->get_logger(), "STM reply: %s", buf);
            }
            // On timeout (EAGAIN/EWOULDBLOCK) or shutdown, loop back and check flag.
        }
    }

    int          sock_{-1};
    sockaddr_in  stm_addr_{};
    std::string  stm_ip_;
    int          stm_port_;

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
