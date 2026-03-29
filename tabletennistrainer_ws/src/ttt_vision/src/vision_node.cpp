#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <ttt_msgs/msg/ball_detection.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node") {
        this->declare_parameter("camera_id", "left");
        this->declare_parameter("min_circularity", 0.65);
        this->declare_parameter("min_radius", 4);
        this->declare_parameter("max_radius", 25);
        this->declare_parameter("edge_margin", 25);
        this->declare_parameter("motion_threshold", 30);  // pixel diff to flag as motion
        this->declare_parameter("min_brightness",   200); // original frame brightness at blob centre
        this->declare_parameter("dilate_iters",       2); // fill ball blob after diff
        // Manual 4-corner ROI [x0,y0,…,x3,y3].  Leave empty → auto-detect.
        this->declare_parameter("table_roi", std::vector<int64_t>{});

        camera_id_        = this->get_parameter("camera_id").as_string();
        min_circularity_  = this->get_parameter("min_circularity").as_double();
        min_radius_       = this->get_parameter("min_radius").as_int();
        max_radius_       = this->get_parameter("max_radius").as_int();
        edge_margin_      = this->get_parameter("edge_margin").as_int();
        motion_threshold_ = this->get_parameter("motion_threshold").as_int();
        min_brightness_   = this->get_parameter("min_brightness").as_int();
        dilate_iters_     = this->get_parameter("dilate_iters").as_int();

        auto roi_vec = this->get_parameter("table_roi").as_integer_array();
        if (roi_vec.size() == 8) {
            for (int i = 0; i < 4; i++)
                table_roi_.emplace_back((int)roi_vec[i*2], (int)roi_vec[i*2+1]);
            auto_detect_ = false;
            RCLCPP_INFO(this->get_logger(),
                "[%s] Manual table ROI: (%ld,%ld) (%ld,%ld) (%ld,%ld) (%ld,%ld)",
                camera_id_.c_str(),
                roi_vec[0],roi_vec[1], roi_vec[2],roi_vec[3],
                roi_vec[4],roi_vec[5], roi_vec[6],roi_vec[7]);
        } else {
            auto_detect_ = true;
            RCLCPP_INFO(this->get_logger(),
                "[%s] Auto table-detect ON", camera_id_.c_str());
        }

        gaussian_filter_ = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(5, 5), 0);
        dilate_kernel_   = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        open_kernel_     = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        detect_kernel_   = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));

        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera/" + camera_id_ + "/compressed", 10,
            std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

        detection_pub_ = this->create_publisher<ttt_msgs::msg::BallDetection>(
            "/ball_detection/" + camera_id_, 10);

        RCLCPP_INFO(this->get_logger(),
            "[%s] Vision node online | motion>%d AND brightness>%d",
            camera_id_.c_str(), motion_threshold_, min_brightness_);
    }

private:
    // ── Auto-detect table from dark surface ──────────────────────────────────
    void autoDetectTable(const cv::Mat& gray) {
        cv::Mat dark;
        cv::threshold(gray, dark, 60, 255, cv::THRESH_BINARY_INV);
        cv::morphologyEx(dark, dark, cv::MORPH_CLOSE, detect_kernel_);
        cv::morphologyEx(dark, dark, cv::MORPH_OPEN,  detect_kernel_);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dark, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return;

        std::sort(contours.begin(), contours.end(),
            [](const auto& a, const auto& b){ return cv::contourArea(a) > cv::contourArea(b); });

        double min_area = gray.cols * gray.rows * 0.12;
        for (const auto& c : contours) {
            if (cv::contourArea(c) < min_area) break;
            std::vector<cv::Point> approx;
            cv::approxPolyDP(c, approx, 0.03 * cv::arcLength(c, true), true);
            std::vector<cv::Point> hull;
            cv::convexHull(approx, hull);
            if (hull.size() < 4 || hull.size() > 6) continue;
            table_roi_ = hull;
            RCLCPP_INFO(this->get_logger(),
                "[%s] Table auto-detected: %zu corners, %.0f%% of frame — ROI locked",
                camera_id_.c_str(), hull.size(),
                100.0 * cv::contourArea(hull) / (gray.cols * gray.rows));
            return;
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[%s] Auto-detect: no table yet (attempt %d)", camera_id_.c_str(), detect_attempt_);
    }

    // ── Main callback ─────────────────────────────────────────────────────────
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
        if (img.empty()) return;

        if (auto_detect_ && table_roi_.empty()) {
            if (++detect_attempt_ % 10 == 1)
                autoDetectTable(img);
        }

        // ── Build the ROI mask once per unique ROI ────────────────────────────
        if (!table_roi_.empty() && (roi_mask_.empty() ||
            roi_mask_.size() != img.size())) {
            roi_mask_ = cv::Mat::zeros(img.size(), CV_8UC1);
            cv::fillPoly(roi_mask_,
                std::vector<std::vector<cv::Point>>{table_roi_}, 255);
        }

        // ── GPU pipeline ──────────────────────────────────────────────────────
        gpu_frame_.upload(img);

        // Blur for noise-robust diff
        gaussian_filter_->apply(gpu_frame_, gpu_blurred_);

        // Brightness mask: white ball on dark table
        cv::cuda::threshold(gpu_frame_, gpu_bright_, min_brightness_, 255, cv::THRESH_BINARY);

        if (gpu_prev_blurred_.empty()) {
            gpu_blurred_.copyTo(gpu_prev_blurred_);
            return;
        }

        // Motion mask: pixels that changed significantly since last frame
        cv::cuda::absdiff(gpu_blurred_, gpu_prev_blurred_, gpu_diff_);
        cv::cuda::threshold(gpu_diff_, gpu_motion_, motion_threshold_, 255, cv::THRESH_BINARY);
        gpu_blurred_.copyTo(gpu_prev_blurred_);

        // AND gate: must be BOTH moving AND bright white
        cv::cuda::bitwise_and(gpu_motion_, gpu_bright_, gpu_combined_);

        cv::Mat mask;
        gpu_combined_.download(mask);

        // ── CPU morphology ────────────────────────────────────────────────────
        // Dilate fills the hollow ring the diff creates around the ball edges
        for (int i = 0; i < dilate_iters_; i++)
            cv::dilate(mask, mask, dilate_kernel_);
        // Open removes thin residual noise
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel_);

        // ── Apply table ROI ───────────────────────────────────────────────────
        if (!roi_mask_.empty())
            cv::bitwise_and(mask, roi_mask_, mask);

        // ── Find ball ────────────────────────────────────────────────────────
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        auto det_msg = ttt_msgs::msg::BallDetection();
        det_msg.header = msg->header;
        det_msg.x = -1.0; det_msg.y = -1.0;
        float best_score = 0.0;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < 5) continue;

            double perimeter = cv::arcLength(contour, true);
            if (perimeter < 5) continue;

            double circularity = (4.0 * M_PI * area) / (perimeter * perimeter);
            if (circularity <= min_circularity_) continue;

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);

            if (radius < min_radius_ || radius > max_radius_) continue;

            if (center.x < edge_margin_ || center.x > img.cols - edge_margin_) continue;
            if (center.y < edge_margin_ || center.y > img.rows - edge_margin_) continue;

            if (circularity > best_score) {
                best_score = circularity;
                det_msg.x = center.x;
                det_msg.y = center.y;
                det_msg.radius = radius;
                det_msg.confidence = circularity;
            }
        }
        detection_pub_->publish(det_msg);
    }

    std::string camera_id_;
    int min_radius_, max_radius_, edge_margin_, motion_threshold_, min_brightness_, dilate_iters_;
    double min_circularity_;
    bool auto_detect_ = false;
    int detect_attempt_ = 0;
    std::vector<cv::Point> table_roi_;
    cv::Mat roi_mask_, dilate_kernel_, open_kernel_, detect_kernel_;
    cv::cuda::GpuMat gpu_frame_, gpu_blurred_, gpu_prev_blurred_;
    cv::cuda::GpuMat gpu_diff_, gpu_motion_, gpu_bright_, gpu_combined_;
    cv::Ptr<cv::cuda::Filter> gaussian_filter_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Publisher<ttt_msgs::msg::BallDetection>::SharedPtr detection_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}
