#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ttt_msgs/msg/ball_detection.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>
#include <algorithm>
#include <deque>

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node") {
        this->declare_parameter("camera_id", "left");
        this->declare_parameter("min_area", 8);
        this->declare_parameter("max_area", 150);
        this->declare_parameter("edge_margin", 10);
        this->declare_parameter("motion_threshold", 15);
        this->declare_parameter("min_contrast", 25);
        this->declare_parameter("dilate_iters", 1);
        this->declare_parameter("consistency_min", 2);
        this->declare_parameter("kf_gate_px", 250.0);
        this->declare_parameter("kf_reset_misses", 30);
        this->declare_parameter("kf_process_noise", 0.05);
        this->declare_parameter("hysteresis_r", 12);
        this->declare_parameter("hysteresis_thresh", 140);
        this->declare_parameter("table_roi", std::vector<int64_t>{});

        // NEW: Static mode for calibration
        this->declare_parameter("static_mode", false);

        camera_id_ = this->get_parameter("camera_id").as_string();
        min_area_ = this->get_parameter("min_area").as_int();
        max_area_ = this->get_parameter("max_area").as_int();
        edge_margin_ = this->get_parameter("edge_margin").as_int();
        motion_threshold_ = this->get_parameter("motion_threshold").as_int();
        min_contrast_ = this->get_parameter("min_contrast").as_int();
        dilate_iters_ = this->get_parameter("dilate_iters").as_int();
        consistency_min_ = this->get_parameter("consistency_min").as_int();
        kf_gate_px_sq_ = std::pow(this->get_parameter("kf_gate_px").as_double(), 2.0);
        kf_reset_misses_ = this->get_parameter("kf_reset_misses").as_int();
        kf_process_noise_ = (float)this->get_parameter("kf_process_noise").as_double();
        hysteresis_r_ = this->get_parameter("hysteresis_r").as_int();
        hysteresis_thresh_ = this->get_parameter("hysteresis_thresh").as_int();
        static_mode_ = this->get_parameter("static_mode").as_bool();

        auto roi_vec = this->get_parameter("table_roi").as_integer_array();
        if (roi_vec.size() == 8) {
            for (int i = 0; i < 4; i++)
                table_roi_.emplace_back((int)roi_vec[i * 2], (int)roi_vec[i * 2 + 1]);
            auto_detect_ = false;
        }
        else {
            auto_detect_ = true;
        }

        gaussian_filter_ = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(5, 5), 0);
        dilate_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        open_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        detect_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));

        gpu_open_filter_ = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, CV_8UC1, open_kernel_);
        gpu_close_filter_ = cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, CV_8UC1, dilate_kernel_);

        kf_ = cv::KalmanFilter(4, 2, 0, CV_32F);
        kf_.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
        cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(kf_process_noise_));
        cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(0.1f));
        cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0f));

        history_ring_.resize(frame_delay_ + 1);

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                rcl_interfaces::msg::SetParametersResult res;
                res.successful = true;
                for (const auto& p : params) {
                    if (p.get_name() == "static_mode") static_mode_ = p.as_bool();
                    if (p.get_name() == "min_contrast") min_contrast_ = p.as_int();
                    if (p.get_name() == "edge_margin") {
                        edge_margin_ = p.as_int();
                        roi_mask_ = cv::Mat(); // Force rebuild to apply new margin
                        gpu_roi_mask_ = cv::cuda::GpuMat();
                    }
                    if (p.get_name() == "table_roi") {
                        auto vals = p.as_integer_array();
                        if (vals.empty()) {
                            table_roi_.clear();
                            roi_mask_ = cv::Mat();
                            gpu_roi_mask_ = cv::cuda::GpuMat();
                            auto_detect_ = true;
                            detect_attempt_ = 0;
                        }
                        else if (vals.size() >= 8 && vals.size() % 2 == 0) {
                            table_roi_.clear();
                            for (size_t i = 0; i < vals.size(); i += 2)
                                table_roi_.emplace_back((int)vals[i], (int)vals[i + 1]);
                            roi_mask_ = cv::Mat();
                            gpu_roi_mask_ = cv::cuda::GpuMat();
                            auto_detect_ = false;
                        }
                    }
                }
                return res;
            });

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/" + camera_id_ + "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

        detection_pub_ = this->create_publisher<ttt_msgs::msg::BallDetection>("/ball_detection/" + camera_id_, 10);
    }

private:
    void autoDetectTable(const cv::Mat& gray) {
        cv::Mat dark;
        cv::threshold(gray, dark, 35, 255, cv::THRESH_BINARY_INV);
        cv::erode(dark, dark, detect_kernel_);
        cv::morphologyEx(dark, dark, cv::MORPH_CLOSE, detect_kernel_);
        cv::morphologyEx(dark, dark, cv::MORPH_OPEN, detect_kernel_);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dark, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return;

        std::sort(contours.begin(), contours.end(), [](const auto& a, const auto& b) { return cv::contourArea(a) > cv::contourArea(b); });

        double frame_area = gray.cols * gray.rows;
        for (const auto& c : contours) {
            double ca = cv::contourArea(c);
            if (ca < frame_area * 0.10 || ca > frame_area * 0.92) continue;

            std::vector<cv::Point> approx;
            cv::approxPolyDP(c, approx, 0.03 * cv::arcLength(c, true), true);
            std::vector<cv::Point> hull;
            cv::convexHull(approx, hull);
            if (hull.size() < 4 || hull.size() > 6) continue;

            table_roi_ = hull;
            std::vector<int64_t> roi_param;
            for (const auto& pt : hull) { roi_param.push_back(pt.x); roi_param.push_back(pt.y); }
            this->set_parameter(rclcpp::Parameter("table_roi", roi_param));
            RCLCPP_INFO(this->get_logger(),
                "[%s] Table auto-detected: %zu corners, %.0f%% of frame — ROI locked",
                camera_id_.c_str(), hull.size(), 100.0 * ca / frame_area);
            return;
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat img;
        try { img = cv_bridge::toCvShare(msg, "mono8")->image; }
        catch (const cv_bridge::Exception& e) { return; }
        if (img.empty()) return;

        if (auto_detect_ && table_roi_.empty()) {
            if (++detect_attempt_ % 10 == 1) autoDetectTable(img);
        }

        if (!table_roi_.empty() && (roi_mask_.empty() || roi_mask_.size() != img.size())) {
            roi_mask_ = cv::Mat::zeros(img.size(), CV_8UC1);
            // Extend the table polygon to the full image height so the ball is
            // tracked at any elevation (in the air above or below the table surface)
            // while still staying within the table's left-right (X) extent.
            std::vector<cv::Point> extended_roi = table_roi_;
            for (const auto& pt : table_roi_) {
                extended_roi.push_back(cv::Point(pt.x, 0));             // top of image
                extended_roi.push_back(cv::Point(pt.x, img.rows - 1)); // bottom of image
            }
            std::vector<cv::Point> extended_hull;
            cv::convexHull(extended_roi, extended_hull);
            cv::fillPoly(roi_mask_, std::vector<std::vector<cv::Point>>{extended_hull}, 255);
            
            // DILATE the ROI mask to expand the tracking zone OUTSIDE the white lines.
            // This ensures we catch fast balls that fly slightly wide of the table edges!
            int k_size = 21;
            cv::Mat roi_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k_size, k_size));
            cv::dilate(roi_mask_, roi_mask_, roi_dilate);

            gpu_roi_mask_.upload(roi_mask_);
        }

        gpu_frame_.upload(img);
        gaussian_filter_->apply(gpu_frame_, gpu_blurred_);

        if (static_mode_) {
            cv::cuda::threshold(gpu_blurred_, gpu_motion_, min_contrast_, 255, cv::THRESH_BINARY);
            kf_initialized_ = false; // Prevent lockout on static targets like flashlights
        } else {
            if (history_ring_[history_idx_].empty()) {
                gpu_blurred_.copyTo(history_ring_[history_idx_]);
                history_idx_ = (history_idx_ + 1) % (frame_delay_ + 1);
                return; // Fill buffer phase
            }
            
            // Motion Differencing
            cv::cuda::subtract(gpu_blurred_, history_ring_[history_idx_], gpu_diff_);
            cv::cuda::threshold(gpu_diff_, gpu_motion_, motion_threshold_, 255, cv::THRESH_BINARY);
            // Brightness Masking (Only allow moving objects that are ALSO currently bright)
            cv::cuda::threshold(gpu_blurred_, gpu_mask_current_, min_contrast_, 255, cv::THRESH_BINARY);
            cv::cuda::bitwise_and(gpu_motion_, gpu_mask_current_, gpu_motion_);

            gpu_blurred_.copyTo(history_ring_[history_idx_]); // Overwrite oldest
            history_idx_ = (history_idx_ + 1) % (frame_delay_ + 1);
        }

        // Execute all morphological noise reduction and ROI masking strictly on the GPU
        gpu_open_filter_->apply(gpu_motion_, gpu_motion_);
        for (int i = 0; i < dilate_iters_; i++) gpu_close_filter_->apply(gpu_motion_, gpu_motion_);
        if (!gpu_roi_mask_.empty()) cv::cuda::bitwise_and(gpu_motion_, gpu_roi_mask_, gpu_motion_);

        cv::Mat motion_mask;
        gpu_motion_.download(motion_mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(motion_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        float best_x = -1.0f, best_y = -1.0f, best_r = 0.0f;
        float best_score = -1.0f;
        float best_brightness = 0.0f;
        bool found = false;

        for (const auto& contour : contours) {
            cv::Rect bbox = cv::boundingRect(contour);
            double bbox_area = bbox.width * bbox.height;
            if (bbox_area < min_area_ || bbox_area > max_area_) continue;

            // Reject highly elongated shapes (like sweeping arms and paddles)
            float aspect_ratio = static_cast<float>(bbox.width) / bbox.height;
            if (aspect_ratio < 0.4f || aspect_ratio > 2.5f) continue; // Highly strict: blurred balls rarely exceed 1.5:1

            // Circularity check: Ping pong balls are round, sweeping paddle edges are jagged lines
            double contour_area = cv::contourArea(contour);
            double perimeter = cv::arcLength(contour, true);
            double circularity = 1.0;
            // Only enforce circularity on blobs large enough to have a measurable shape.
            // Tiny distant balls (<16px area) are just pixelated squares and will fail math checks!
            if (perimeter > 0.0 && bbox_area >= 16.0) {
                circularity = 4.0 * CV_PI * contour_area / (perimeter * perimeter);
                if (circularity < 0.55) continue; // Strict: blurred balls are ~0.8, jagged lines are <0.4
            }

            cv::Point2f center(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
            if (center.x < edge_margin_ || center.x > img.cols - edge_margin_) continue;
            if (center.y < edge_margin_ || center.y > img.rows - edge_margin_) continue;

            // CPU OPTIMIZATION: The GPU bitwise_and already enforced our min_contrast_ threshold.
            // Skipping the CPU-bound cv::mean slice calculation saves critical microseconds per contour!
            double brightness = 255.0;

            // KF PROXIMITY SCORING:
            // If we have a track, prioritize the bright blob closest to the prediction!
            // If we don't have a track, prioritize the most circular blob.
            float score = 0.0f;
            if (kf_initialized_) {
                float pred_x = kf_.statePre.at<float>(0);
                float pred_y = kf_.statePre.at<float>(1);
                float dist_sq = std::pow(center.x - pred_x, 2) + std::pow(center.y - pred_y, 2);
                if (dist_sq > kf_gate_px_sq_) continue; // Ignore white lines far away from the ball
                score = 1000.0f / (std::sqrt(dist_sq) + 1.0f); // Closer = Higher Score
            } else {
                score = static_cast<float>(circularity * bbox_area);
            }

            if (score > best_score) {
                best_score = score;
                best_x = center.x;
                best_y = center.y;
                best_r = std::max(bbox.width, bbox.height) / 2.0f;
                best_brightness = static_cast<float>(brightness);
                found = true;
            }
        }

        consistency_buf_.push_back(found);
        if ((int)consistency_buf_.size() > 3) consistency_buf_.pop_front();
        int hit_count = std::count(consistency_buf_.begin(), consistency_buf_.end(), true);
        bool consistent = (hit_count >= consistency_min_);

        double t_now = rclcpp::Time(msg->header.stamp).seconds();

        if (kf_initialized_) {
            float dt = std::clamp((float)(t_now - last_stamp_), 0.001f, 0.1f);
            last_stamp_ = t_now;
            kf_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1);
            kf_.predict();

            if (found) {
                float dx = best_x - kf_.statePre.at<float>(0), dy = best_y - kf_.statePre.at<float>(1);
                if (dx * dx + dy * dy <= kf_gate_px_sq_) {
                    cv::Mat meas = (cv::Mat_<float>(2, 1) << best_x, best_y);
                    kf_.correct(meas);
                    miss_count_ = 0;
                }
                else {
                    kf_.statePre.copyTo(kf_.statePost);
                    miss_count_++;
                }
            }
            else {
                kf_.statePre.copyTo(kf_.statePost);
                miss_count_++;
            }

            if (miss_count_ >= kf_reset_misses_) {
                kf_initialized_ = false; miss_count_ = 0; consistency_buf_.clear();
            }
        }
        else if (consistent && found) {
            kf_.statePost = (cv::Mat_<float>(4, 1) << best_x, best_y, 0.0f, 0.0f);
            cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(100.0f));
            kf_initialized_ = true; last_stamp_ = t_now; miss_count_ = 0;
        }

        auto det_msg = ttt_msgs::msg::BallDetection();
        det_msg.header = msg->header;
        det_msg.x = -1.0f; det_msg.y = -1.0f;

        if (kf_initialized_) {
            det_msg.x = kf_.statePost.at<float>(0); det_msg.y = kf_.statePost.at<float>(1);
            det_msg.radius = found ? best_r : 0.0f; det_msg.confidence = found ? best_brightness : 0.0f;
        }
        detection_pub_->publish(det_msg);
    }

    std::string camera_id_;
    int min_area_, max_area_, edge_margin_, motion_threshold_, min_contrast_, dilate_iters_;
    bool auto_detect_ = false, static_mode_ = false;
    int detect_attempt_ = 0;
    std::vector<cv::Point> table_roi_;
    cv::Mat roi_mask_, dilate_kernel_, open_kernel_, detect_kernel_;

    cv::cuda::GpuMat gpu_frame_, gpu_blurred_, gpu_diff_, gpu_mask_current_, gpu_motion_, gpu_roi_mask_;
    cv::Ptr<cv::cuda::Filter> gaussian_filter_;
    cv::Ptr<cv::cuda::Filter> gpu_open_filter_, gpu_close_filter_;

    std::vector<cv::cuda::GpuMat> history_ring_;
    int history_idx_ = 0;
    int frame_delay_ = 10;

    int consistency_min_; std::deque<bool> consistency_buf_;
    cv::KalmanFilter kf_;
    bool kf_initialized_ = false; int miss_count_ = 0; double last_stamp_ = 0.0;
    float kf_gate_px_sq_, kf_process_noise_;
    int kf_reset_misses_, hysteresis_r_, hysteresis_thresh_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<ttt_msgs::msg::BallDetection>::SharedPtr detection_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}