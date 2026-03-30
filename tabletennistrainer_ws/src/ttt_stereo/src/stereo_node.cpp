#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <ttt_msgs/msg/ball_detection.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <chrono>

using BallDetection = ttt_msgs::msg::BallDetection;
using ApproxPolicy = message_filters::sync_policies::ApproximateTime<BallDetection, BallDetection>;

class StereoNode : public rclcpp::Node {
public:
    StereoNode() : Node("stereo_node") {

        // ── Camera Intrinsics (replace with real calibration later) ──────────
        // fx/fy: focal length in pixels.
        //   Estimate: fx ≈ width / (2 * tan(FOV/2))
        //   For 640px wide, ~90° FOV → fx ≈ 320
        this->declare_parameter("fx", 448.2);
        this->declare_parameter("fy", 400.0);
        // cx/cy: principal point — default to image center
        this->declare_parameter("cx", 640.0);
        this->declare_parameter("cy", 400.0);
        // baseline: physical distance between the two cameras in meters
        this->declare_parameter("baseline_m", 0.3);
        // max age: reject detection pairs further apart than this (ms)
        this->declare_parameter("max_sync_age_ms", 50);

        fx_         = this->get_parameter("fx").as_double();
        fy_         = this->get_parameter("fy").as_double();
        cx_         = this->get_parameter("cx").as_double();
        cy_         = this->get_parameter("cy").as_double();
        baseline_   = this->get_parameter("baseline_m").as_double();
        max_sync_age_ms_ = this->get_parameter("max_sync_age_ms").as_int();

        RCLCPP_INFO(this->get_logger(),
            "Stereo params | fx=%.1f fy=%.1f cx=%.1f cy=%.1f baseline=%.3fm",
            fx_, fy_, cx_, cy_, baseline_);

        // ── Subscriptions with ApproximateTime sync ───────────────────────────
        // Queue size 10, will pair left+right detections within max_sync_age_ms
        left_sub_.subscribe(this, "/ball_detection/left");
        right_sub_.subscribe(this, "/ball_detection/right");

        sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy>>(
            ApproxPolicy(10), left_sub_, right_sub_);
        sync_->registerCallback(
            std::bind(&StereoNode::syncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        // ── Publisher ─────────────────────────────────────────────────────────
        position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/ball_position_3d", 10);

        RCLCPP_INFO(this->get_logger(), "Stereo node ready.");
    }

private:
    void syncCallback(
        const BallDetection::ConstSharedPtr& left,
        const BallDetection::ConstSharedPtr& right)
    {
        // ── Reject if either camera has no detection ──────────────────────────
        if (left->x < 0 || right->x < 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for detection on both cameras...");
            return;
        }

        // ── Reject if timestamps are too far apart ────────────────────────────
        auto dt_ns = std::abs(
            (rclcpp::Time(left->header.stamp) - rclcpp::Time(right->header.stamp)).nanoseconds());
        if (dt_ns > (int64_t)max_sync_age_ms_ * 1'000'000) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Detection timestamps too far apart: %.1f ms — skipping",
                dt_ns / 1e6);
            return;
        }

        // ── Triangulation ─────────────────────────────────────────────────────
        // disparity: with cameras facing each other (one on each side of the table)
        // the right camera's image is horizontally mirrored relative to the left,
        // so use right->x - left->x to get a positive disparity for real objects.
        double disparity = right->x - left->x;

        if (std::abs(disparity) < 1.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Disparity too small (%.2f px) — ball may be too far or detections mismatched",
                disparity);
            return;
        }

        // Z (depth) = (fx * baseline) / disparity
        double Z = (fx_ * baseline_) / disparity;

        // X = (u_left - cx) * Z / fx
        double X = (left->x - cx_) * Z / fx_;

        // Y = (v_left - cy) * Z / fy  (average left+right for robustness)
        double v_avg = (left->y + right->y) / 2.0;
        double Y = (v_avg - cy_) * Z / fy_;

        // ── Publish ───────────────────────────────────────────────────────────
        auto out = geometry_msgs::msg::PointStamped();
        out.header.stamp = left->header.stamp;
        out.header.frame_id = "camera_left_optical_frame";
        out.point.x = X;
        out.point.y = Y;
        out.point.z = Z;

        position_pub_->publish(out);

        RCLCPP_INFO(this->get_logger(),
            "Ball 3D | X: %+.3f m  Y: %+.3f m  Z: %.3f m  | disparity: %.1f px  dt: %.1f ms",
            X, Y, Z, disparity, dt_ns / 1e6);
    }

    // Intrinsics
    double fx_, fy_, cx_, cy_, baseline_;
    int max_sync_age_ms_;

    // Sync
    message_filters::Subscriber<BallDetection> left_sub_, right_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproxPolicy>> sync_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoNode>());
    rclcpp::shutdown();
    return 0;
}