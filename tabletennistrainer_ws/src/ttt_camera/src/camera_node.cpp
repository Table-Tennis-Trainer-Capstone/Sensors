#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class JetsonCameraNode : public rclcpp::Node {
public:
    JetsonCameraNode() : Node("camera_node") {
        this->declare_parameter("sensor_id", 0);
        this->declare_parameter("camera_id", "left");
        this->declare_parameter("width", 1280);
        this->declare_parameter("height", 720);
        this->declare_parameter("fps", 120);

        int sensor_id = this->get_parameter("sensor_id").as_int();
        std::string camera_id = this->get_parameter("camera_id").as_string();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();

        // THE FIX: Direct kernel access via v4l2src (Bypasses NVIDIA ISP)
        std::string pipeline = 
            "v4l2src device=/dev/video" + std::to_string(sensor_id) + " ! "
            "video/x-raw, format=GRAY8, width=" + std::to_string(width) + 
            ", height=" + std::to_string(height) + ", framerate=" + 
            std::to_string(fps) + "/1 ! appsink drop=true sync=false";

        RCLCPP_INFO(this->get_logger(), "Initializing V4L2 Camera: %s", camera_id.c_str());
        RCLCPP_INFO(this->get_logger(), "Pipeline: %s", pipeline.c_str());

        cap_.open(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open camera via GStreamer V4L2!");
            throw std::runtime_error("Camera initialization failed");
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/" + camera_id + "/image_raw", 10);

        int timer_ms = (1000 / fps) - 1; 
        if (timer_ms < 1) timer_ms = 1;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_ms), 
            std::bind(&JetsonCameraNode::capture_frame, this));

        RCLCPP_INFO(this->get_logger(), "Camera %s publishing at %d FPS.", camera_id.c_str(), fps);
    }

private:
    void capture_frame() {
        cv::Mat frame;
        cap_.read(frame);

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Dropped frame! Camera returned empty matrix.");
            return; 
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_" + this->get_parameter("camera_id").as_string() + "_link";
        
        publisher_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<JetsonCameraNode>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_node"), "Fatal Exception: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
