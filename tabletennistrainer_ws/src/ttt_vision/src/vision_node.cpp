#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ttt_msgs/msg/ball_detection.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <chrono>

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node") {
        this->declare_parameter("camera_id", "left");
        this->declare_parameter("min_brightness", 20);
        this->declare_parameter("min_radius", 3);
        this->declare_parameter("max_radius", 50);
        this->declare_parameter("show_window", false);

        camera_id_ = this->get_parameter("camera_id").as_string();
        min_brightness_ = this->get_parameter("min_brightness").as_int();
        min_radius_ = this->get_parameter("min_radius").as_int();
        max_radius_ = this->get_parameter("max_radius").as_int();
        show_window_ = this->get_parameter("show_window").as_bool();

        // Pre-allocate CUDA Gaussian Filter to avoid re-creating it every frame
        gaussian_filter_ = cv::cuda::createGaussianFilter(
            CV_8UC1, CV_8UC1, cv::Size(5, 5), 0);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/" + camera_id_ + "/image_raw", 10,
            std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

        detection_pub_ = this->create_publisher<ttt_msgs::msg::BallDetection>(
            "/ball_detection/" + camera_id_, 10);

        if (show_window_) {
            cv::namedWindow("Vision - " + camera_id_, cv::WINDOW_AUTOSIZE);
        }
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto start = std::chrono::high_resolution_clock::now();

        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "mono8");

        // 1. Upload to GPU
        gpu_frame_.upload(cv_ptr->image);

        // 2. GPU Gaussian Blur
        gaussian_filter_->apply(gpu_frame_, gpu_blurred_);

        // 3. GPU Hough Circles
        // Parameters: dp=1, minDist=rows/8, canny_thresh=100, votes=20, minR, maxR
        cv::Ptr<cv::cuda::HoughCirclesDetector> detector =
            cv::cuda::createHoughCirclesDetector(
                1.0f, (float)gpu_blurred_.rows / 8.0f,
                100, 20, min_radius_, max_radius_);

        cv::cuda::GpuMat d_circles;
        detector->detect(gpu_blurred_, d_circles);

        // 4. Download Results to CPU
        std::vector<cv::Vec3f> circles;
        if (!d_circles.empty()) {
            circles.resize(d_circles.cols);
            d_circles.download(cv::Mat(circles).reshape(3, 1));
        }

        // 5. Brightness Check (ROI) on CPU — pick the brightest candidate
        auto detect_msg = ttt_msgs::msg::BallDetection();
        detect_msg.header = msg->header;
        detect_msg.x = -1.0;
        detect_msg.y = -1.0;

        float max_bright = 0;
        for (const auto& c : circles) {
            cv::Rect roi(
                cvRound(c[0] - c[2]), cvRound(c[1] - c[2]),
                cvRound(c[2] * 2), cvRound(c[2] * 2));

            // Boundary check
            if (roi.x >= 0 && roi.y >= 0 &&
                roi.x + roi.width <= cv_ptr->image.cols &&
                roi.y + roi.height <= cv_ptr->image.rows)
            {
                cv::Scalar avg = cv::mean(cv_ptr->image(roi));
                if (avg[0] > max_bright && avg[0] >= min_brightness_) {
                    max_bright = avg[0];
                    detect_msg.x = c[0];
                    detect_msg.y = c[1];
                    detect_msg.radius = c[2];
                    detect_msg.confidence = max_bright / 255.0f;
                }
            }
        }

        detection_pub_->publish(detect_msg);

        // 6. Optional debug window (CPU only — no need to touch GPU for display)
        if (show_window_) {
            cv::Mat display;
            cv::cvtColor(cv_ptr->image, display, cv::COLOR_GRAY2BGR);

            if (detect_msg.x >= 0) {
                cv::Point center(cvRound(detect_msg.x), cvRound(detect_msg.y));
                int radius = cvRound(detect_msg.radius);
                // Outer circle (green)
                cv::circle(display, center, radius, { 0, 255, 0 }, 2);
                // Center dot (red)
                cv::circle(display, center, 2, { 0, 0, 255 }, -1);
                // Confidence label
                std::string label = cv::format("conf: %.2f", detect_msg.confidence);
                cv::putText(display, label,
                    { center.x - radius, center.y - radius - 5 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, { 0, 255, 0 }, 1);
            }

            cv::imshow("Vision - " + camera_id_, display);
            cv::waitKey(1); // Must be called to pump the GUI event loop
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto lat = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[%s] GPU Pipeline Latency: %.2f ms | Circles found: %zu",
            camera_id_.c_str(), lat / 1000.0, circles.size());
    }

    std::string camera_id_;
    int min_brightness_, min_radius_, max_radius_;
    bool show_window_;

    cv::cuda::GpuMat gpu_frame_, gpu_blurred_;
    cv::Ptr<cv::cuda::Filter> gaussian_filter_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<ttt_msgs::msg::BallDetection>::SharedPtr detection_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}