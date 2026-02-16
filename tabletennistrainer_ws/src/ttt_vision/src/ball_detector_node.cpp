#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ttt_msgs/msg/ball_detection.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

class OpenCVBallDetectorNode : public rclcpp::Node
{
public:
    OpenCVBallDetectorNode() : Node("ball_detector")
    {
        this->declare_parameter("min_brightness", 150); // Lowered to be more forgiving in normal lighting
        this->declare_parameter("min_radius", 3);
        this->declare_parameter("max_radius", 30);
        this->declare_parameter("blur_size", 3);
        this->declare_parameter("camera_id", "left");

        min_brightness_ = this->get_parameter("min_brightness").as_int();
        min_radius_ = this->get_parameter("min_radius").as_int();
        max_radius_ = this->get_parameter("max_radius").as_int();
        blur_size_ = this->get_parameter("blur_size").as_int();
        camera_id_ = this->get_parameter("camera_id").as_string();

        if (blur_size_ % 2 == 0) blur_size_++; 

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/" + camera_id_ + "/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&OpenCVBallDetectorNode::imageCallback, this, std::placeholders::_1)
        );

        detection_pub_ = this->create_publisher<ttt_msgs::msg::BallDetection>(
            "/ball/detection_2d/" + camera_id_, 10
        );

        RCLCPP_INFO(this->get_logger(), "OpenCV Ball detector started for camera: %s", camera_id_.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        cv::Mat blurred;

        cv::GaussianBlur(frame, blurred, cv::Size(blur_size_, blur_size_), 0, 0);

        std::vector<cv::Vec3f> circles;
        // HoughCircles Sensitivity tuned down from 30 to 20 to catch more circles
        cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1,
                         frame.rows / 8,
                         100, 20, 
                         min_radius_, max_radius_);

        auto detection = ttt_msgs::msg::BallDetection();
        detection.header = msg->header;

        if (!circles.empty()) {
            int brightest_idx = 0;
            int max_brightness = 0;

            for (size_t i = 0; i < circles.size(); i++) {
                cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);

                cv::Rect roi(std::max(0, center.x - radius),
                             std::max(0, center.y - radius),
                             std::min(2 * radius, frame.cols - center.x + radius),
                             std::min(2 * radius, frame.rows - center.y + radius));

                if (roi.x >= 0 && roi.y >= 0 &&
                    roi.x + roi.width <= frame.cols &&
                    roi.y + roi.height <= frame.rows) {
                    cv::Scalar avg = cv::mean(frame(roi));
                    if (avg[0] > max_brightness) {
                        max_brightness = avg[0];
                        brightest_idx = i;
                    }
                }
            }

            detection.x = circles[brightest_idx][0];
            detection.y = circles[brightest_idx][1];
            detection.radius = circles[brightest_idx][2];
            detection.confidence = max_brightness / 255.0;
        } else {
            // No ball found, output default "empty" values
            detection.x = -1.0;
            detection.y = -1.0;
            detection.radius = 0.0;
            detection.confidence = 0.0;
        }

        detection_pub_->publish(detection);

        frame_count_++;
        if (frame_count_ % 120 == 0) { 
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
            
            if (detection.confidence > 0) {
                RCLCPP_INFO(this->get_logger(), "Tracking Ball at X:%.1f Y:%.1f (Lat: %.2f ms)", 
                            detection.x, detection.y, duration / 1000.0);
            } else {
                RCLCPP_INFO(this->get_logger(), "No ball detected... (Lat: %.2f ms)", duration / 1000.0);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<ttt_msgs::msg::BallDetection>::SharedPtr detection_pub_;

    int min_brightness_;
    int min_radius_;
    int max_radius_;
    int blur_size_;
    std::string camera_id_;
    int frame_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<OpenCVBallDetectorNode>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ball_detector"), "Exception: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
