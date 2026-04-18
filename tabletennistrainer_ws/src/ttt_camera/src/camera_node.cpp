#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node"), frame_count_(0) {
        this->declare_parameter("device", "/dev/video0");
        this->declare_parameter("camera_id", "left");
        this->declare_parameter("width", 640);
        this->declare_parameter("height", 400);
        this->declare_parameter("fps", 240);
        this->declare_parameter("exposure", 7000);
        this->declare_parameter("analogue_gain", 1000);

        std::string device = this->get_parameter("device").as_string();
        camera_id_ = this->get_parameter("camera_id").as_string();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();
        int exposure = this->get_parameter("exposure").as_int();
        int analogue_gain = this->get_parameter("analogue_gain").as_int();

        // GStreamer Pipeline:
        // 1. v4l2src pulls MJPEG from the OV9281
        // 2. nvv4l2decoder decodes MJPEG on the Jetson Hardware Decoder
        // 3. nvvidconv converts to GRAY8 (mono8)

        std::string pipeline =
            "v4l2src device=" + device + " io-mode=2 "
            "extra-controls=\"s,exposure=" + std::to_string(exposure) +
            ",analogue_gain=" + std::to_string(analogue_gain) + "\" ! "
            "video/x-raw, format=GRAY8, width=" + std::to_string(width) +
            ", height=" + std::to_string(height) +
            ", framerate=" + std::to_string(fps) + "/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw, format=GRAY8 ! appsink drop=true sync=false";

        RCLCPP_INFO(this->get_logger(), "Launching Pipeline: %s", pipeline.c_str());
        cap_.open(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera at %d FPS.", fps);
            throw std::runtime_error("Camera Init Failed");
        }

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/" + camera_id_ + "/image_raw", rclcpp::SensorDataQoS());
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/" + camera_id_ + "/camera_info", 10);

        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/" + camera_id_ + "/compressed", rclcpp::SensorDataQoS());

        // 240 FPS = ~4.17ms per frame
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(4167),
            std::bind(&CameraNode::captureFrame, this));
        fps_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CameraNode::reportFPS, this));
    }

private:
    void captureFrame() {
        cv::Mat frame;
        if (!cap_.read(frame)) return;

        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = camera_id_ + "_optical_frame";
        msg->height = frame.rows;
        msg->width = frame.cols;
        msg->encoding = "mono8";
        msg->step = frame.cols;
        msg->data.assign(frame.data, frame.data + (frame.rows * frame.cols));

        image_pub_->publish(*msg);

        auto info_msg = sensor_msgs::msg::CameraInfo();
        info_msg.header = msg->header;
        camera_info_pub_->publish(info_msg);

        // THROTTLE GUI FEED: JPEG encoding 240 times a second starves the CPU!
        // Only encode and send the GUI stream at 30 FPS (every 8th frame)
        if (frame_count_ % 8 == 0) {
            auto comp_msg = sensor_msgs::msg::CompressedImage();
            comp_msg.header.stamp = this->now();
            comp_msg.header.frame_id = camera_id_ + "_optical_frame";
            comp_msg.format = "jpeg";
            cv::imencode(".jpg", frame, comp_msg.data, { cv::IMWRITE_JPEG_QUALITY, 50 });
            compressed_pub_->publish(comp_msg);
        }

        frame_count_++;
    }

    void reportFPS() {
        frame_count_ = 0;
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::TimerBase::SharedPtr timer_, fps_timer_;
    std::string camera_id_;
    size_t frame_count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
