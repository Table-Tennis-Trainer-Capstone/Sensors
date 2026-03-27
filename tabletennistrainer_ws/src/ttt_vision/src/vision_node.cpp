#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ttt_msgs/msg/ball_detection.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <chrono>

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node") {
        this->declare_parameter("camera_id",        "left");
        this->declare_parameter("min_radius",        4);
        this->declare_parameter("max_radius",       40);
        this->declare_parameter("min_brightness",   10);
        this->declare_parameter("min_contrast",      6);
        this->declare_parameter("min_circularity",  0.35);
        this->declare_parameter("max_aspect_ratio",  3.5);
        this->declare_parameter("diff_threshold",   15);   // pixel diff to count as motion
        this->declare_parameter("show_window",      false);
        this->declare_parameter("noise_cell_size",       8);
        this->declare_parameter("noise_suppress_thresh", 4.0);
        this->declare_parameter("noise_decay_per_sec",   0.3);
        this->declare_parameter("edge_margin",          30);

        camera_id_        = this->get_parameter("camera_id").as_string();
        min_radius_       = this->get_parameter("min_radius").as_int();
        max_radius_       = this->get_parameter("max_radius").as_int();
        min_brightness_   = this->get_parameter("min_brightness").as_int();
        min_contrast_     = this->get_parameter("min_contrast").as_int();
        min_circularity_  = this->get_parameter("min_circularity").as_double();
        max_aspect_ratio_ = this->get_parameter("max_aspect_ratio").as_double();
        diff_threshold_   = this->get_parameter("diff_threshold").as_int();
        show_window_      = this->get_parameter("show_window").as_bool();
        noise_cell_size_       = this->get_parameter("noise_cell_size").as_int();
        noise_suppress_thresh_ = this->get_parameter("noise_suppress_thresh").as_double();
        noise_decay_per_sec_   = this->get_parameter("noise_decay_per_sec").as_double();
        edge_margin_           = this->get_parameter("edge_margin").as_int();

        gaussian_filter_ = cv::cuda::createGaussianFilter(
            CV_8UC1, CV_8UC1, cv::Size(5, 5), 0);

        cv::Mat dk = cv::getStructuringElement(cv::MORPH_ELLIPSE, {9, 9});
        dilate_filter_ = cv::cuda::createMorphologyFilter(
            cv::MORPH_DILATE, CV_8UC1, dk);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/" + camera_id_ + "/image_raw", 10,
            std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

        detection_pub_ = this->create_publisher<ttt_msgs::msg::BallDetection>(
            "/ball_detection/" + camera_id_, 10);

        last_decay_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(),
            "[%s] frame-diff thresh=%d | r=%d-%d circ=%.2f | noise cell=%d thresh=%.1f | edge=%dpx",
            camera_id_.c_str(), diff_threshold_,
            min_radius_, max_radius_, min_circularity_,
            noise_cell_size_, noise_suppress_thresh_, edge_margin_);

        if (show_window_) {
            cv::namedWindow("Vision - "  + camera_id_, cv::WINDOW_AUTOSIZE);
            cv::namedWindow("Diff - "    + camera_id_, cv::WINDOW_AUTOSIZE);
        }
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto t0 = std::chrono::high_resolution_clock::now();

        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "mono8");
        const cv::Mat& img = cv_ptr->image;
        const int W = img.cols, H = img.rows;

        // Init noise map on first frame
        int map_cols = (W + noise_cell_size_ - 1) / noise_cell_size_;
        int map_rows = (H + noise_cell_size_ - 1) / noise_cell_size_;
        if (noise_map_.empty()) {
            noise_map_ = cv::Mat::zeros(map_rows, map_cols, CV_32F);
        }

        // 1. Upload + blur
        gpu_frame_.upload(img);
        gaussian_filter_->apply(gpu_frame_, gpu_blurred_);

        // 2. Need two frames for differencing
        if (gpu_prev_.empty()) {
            gpu_blurred_.copyTo(gpu_prev_);
            return;
        }

        // 3. Absolute frame difference
        cv::cuda::absdiff(gpu_blurred_, gpu_prev_, gpu_diff_);
        gpu_blurred_.copyTo(gpu_prev_);

        // 4. Threshold to binary motion mask
        cv::cuda::threshold(gpu_diff_, gpu_thresh_, diff_threshold_, 255, cv::THRESH_BINARY);

        // 5. Dilate to connect ball pixels
        dilate_filter_->apply(gpu_thresh_, gpu_dilated_);

        // 6. Download motion mask
        cv::Mat fg_mask;
        gpu_dilated_.download(fg_mask);

        // 7. Time-based noise map decay
        auto now = std::chrono::steady_clock::now();
        double dt_sec = std::chrono::duration<double>(now - last_decay_time_).count();
        last_decay_time_ = now;
        noise_map_ *= static_cast<float>(std::pow(noise_decay_per_sec_, dt_sec));

        // 8. Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        const double min_area = M_PI * min_radius_ * min_radius_;
        const double max_area = M_PI * max_radius_ * max_radius_;

        auto detect_msg = ttt_msgs::msg::BallDetection();
        detect_msg.header = msg->header;
        detect_msg.x = -1.0;
        detect_msg.y = -1.0;

        float best_score = -1.0f;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < min_area || area > max_area) continue;

            double perimeter = cv::arcLength(contour, true);
            if (perimeter < 1.0) continue;
            double circularity = 4.0 * M_PI * area / (perimeter * perimeter);

            cv::Point2f center;
            float radius;

            if (circularity >= min_circularity_) {
                cv::minEnclosingCircle(contour, center, radius);
            } else {
                // Streak fallback for motion-blurred ball
                cv::RotatedRect rect = cv::minAreaRect(contour);
                float major = std::max(rect.size.width, rect.size.height) / 2.0f;
                float minor = std::min(rect.size.width, rect.size.height) / 2.0f;
                if (minor < 1.0f) continue;
                if (major / minor > max_aspect_ratio_) continue;
                if (minor < min_radius_ || minor > max_radius_) continue;
                center = rect.center;
                radius = minor;
                circularity *= 0.5f;
            }

            int cx = cvRound(center.x);
            int cy = cvRound(center.y);
            int r  = cvRound(radius);
            int r2 = r + std::max(3, r / 2);
            if (cx < 0 || cx >= W || cy < 0 || cy >= H) continue;

            // Reject detections near frame edge
            if (cx < edge_margin_ || cx >= W - edge_margin_ ||
                cy < edge_margin_ || cy >= H - edge_margin_) continue;

            // Noise suppression: reject cells that fire too often
            int ncx = std::min(cx / noise_cell_size_, map_cols - 1);
            int ncy = std::min(cy / noise_cell_size_, map_rows - 1);
            noise_map_.at<float>(ncy, ncx) += 1.0f;
            if (noise_map_.at<float>(ncy, ncx) >= noise_suppress_thresh_) continue;

            // Brightness + contrast check on original image
            auto make_roi = [&](int half) -> cv::Rect {
                return cv::Rect(
                    std::max(0, cx - half), std::max(0, cy - half),
                    std::min(W, cx + half) - std::max(0, cx - half),
                    std::min(H, cy + half) - std::max(0, cy - half));
            };
            cv::Rect inner_roi = make_roi(r);
            cv::Rect outer_roi = make_roi(r2);
            if (inner_roi.area() < 4) continue;

            double inner_mean = cv::mean(img(inner_roi))[0];
            if (inner_mean < min_brightness_) continue;

            int ring_area = outer_roi.area() - inner_roi.area();
            if (ring_area >= 4) {
                double outer_mean =
                    (cv::mean(img(outer_roi))[0] * outer_roi.area()
                     - inner_mean * inner_roi.area()) / ring_area;
                if (inner_mean - outer_mean < min_contrast_) continue;
            }

            float score = static_cast<float>(
                0.5 * circularity +
                0.3 * (inner_mean / 255.0) +
                0.2 * std::min(1.0, area / (M_PI * max_radius_ * max_radius_)));

            if (score > best_score) {
                best_score            = score;
                detect_msg.x          = center.x;
                detect_msg.y          = center.y;
                detect_msg.radius     = radius;
                detect_msg.confidence = score;
            }
        }

        detection_pub_->publish(detect_msg);

        if (show_window_) {
            cv::Mat display;
            cv::cvtColor(img, display, cv::COLOR_GRAY2BGR);
            if (detect_msg.x >= 0) {
                cv::Point c(cvRound(detect_msg.x), cvRound(detect_msg.y));
                cv::circle(display, c, cvRound(detect_msg.radius), {0, 255, 0}, 2);
                cv::circle(display, c, 2, {0, 0, 255}, -1);
            }
            cv::imshow("Vision - " + camera_id_, display);
            cv::Mat fg_display;
            cv::cvtColor(fg_mask, fg_display, cv::COLOR_GRAY2BGR);
            cv::imshow("Diff - " + camera_id_, fg_display);
            cv::waitKey(1);
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[%s] %.2f ms | contours=%zu | score=%.2f",
            camera_id_.c_str(),
            std::chrono::duration<double, std::milli>(t1 - t0).count(),
            contours.size(), best_score);
    }

    std::string camera_id_;
    int    min_radius_, max_radius_, min_brightness_, min_contrast_, diff_threshold_;
    double min_circularity_, max_aspect_ratio_;
    bool   show_window_;
    int    noise_cell_size_, edge_margin_;
    double noise_suppress_thresh_, noise_decay_per_sec_;
    cv::Mat noise_map_;
    std::chrono::steady_clock::time_point last_decay_time_;

    cv::cuda::GpuMat gpu_frame_, gpu_blurred_, gpu_prev_;
    cv::cuda::GpuMat gpu_diff_, gpu_thresh_, gpu_dilated_;
    cv::Ptr<cv::cuda::Filter> gaussian_filter_;
    cv::Ptr<cv::cuda::Filter> dilate_filter_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr   image_sub_;
    rclcpp::Publisher<ttt_msgs::msg::BallDetection>::SharedPtr detection_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}
