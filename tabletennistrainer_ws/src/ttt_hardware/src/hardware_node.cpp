#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>

class HardwareNode : public rclcpp::Node
{
public:
    HardwareNode() : Node("hardware_node")
    {
        this->declare_parameter("stm_ip",      "192.168.1.100");
        this->declare_parameter("stm_port",    7777);

        stm_ip_   = this->get_parameter("stm_ip").as_string();
        stm_port_ = this->get_parameter("stm_port").as_int();

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

        RCLCPP_INFO(this->get_logger(), "Sending initial Home command (H)...");
        sendUdp("H\n");

        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/stm_cmd", 10,
            std::bind(&HardwareNode::cmdCallback, this, std::placeholders::_1));

        reply_running_ = true;
        reply_thread_  = std::thread(&HardwareNode::replyListener, this);

        RCLCPP_INFO(this->get_logger(), "Hardware node ready — Awaiting single-fire targets on /stm_cmd.");
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

    void cmdCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string & data = msg->data;

        if (data == "home") {
            sendUdp("H\n");
        } else if (data == "estop") {
            sendUdp("E\n");
        } else if (data.size() >= 2 && data[0] == 'M' && data[1] == ' ') {
            // forward M string from control_node to the STM32
            sendUdp(data + "\n");
            RCLCPP_INFO(this->get_logger(), "Sent: %s", data.c_str());
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
                // S1-S5 pos/tgt/err status lines
                if (buf[0] == 'S' && buf[1] >= '1' && buf[1] <= '9')
                    RCLCPP_DEBUG(this->get_logger(), "STM: %s", buf);
                else
                    RCLCPP_INFO(this->get_logger(), "STM reply: %s", buf);
            }
        }
    }

    int          sock_{-1};
    sockaddr_in  stm_addr_{};
    std::string  stm_ip_;
    int          stm_port_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
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