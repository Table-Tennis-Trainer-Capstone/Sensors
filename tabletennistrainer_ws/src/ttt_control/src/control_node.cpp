#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <mutex>
#include <atomic>

// ─────────────────────────────────────────────────────────────────────────────
// Subscribes to /ball_trajectory/predicted, transforms the target point into
// robot_base frame, then sends a pose goal to MoveIt's move_group (arm group,
// Paddle end-effector). MoveIt runs KDL IK and drives the arm_controller,
// which publishes /joint_states → ttt_hardware → UDP → STM32.
// ─────────────────────────────────────────────────────────────────────────────

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("ttt_control_node"), new_target_(false)
    {
        this->declare_parameter("update_rate_hz",  10.0);
        this->declare_parameter("planning_time_s",  0.05);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/ball_trajectory/predicted", 10,
            std::bind(&ControlNode::ballCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "TTT Control node ready — waiting for ball trajectory");
    }

    // Called from main after the executor is already spinning so that
    // MoveGroupInterface can reach the move_group action server.
    void run_moveit()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");

        move_group_->setEndEffectorLink("Paddle");
        move_group_->setPlanningTime(this->get_parameter("planning_time_s").as_double());
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);

        RCLCPP_INFO(this->get_logger(),
            "MoveIt ready | group: arm | end-effector: Paddle | planning time: %.3f s",
            this->get_parameter("planning_time_s").as_double());

        rclcpp::Rate rate(this->get_parameter("update_rate_hz").as_double());

        while (rclcpp::ok()) {
            if (new_target_.exchange(false)) {
                geometry_msgs::msg::Pose target;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    target = latest_target_;
                }

                move_group_->setPoseTarget(target);
                moveit::planning_interface::MoveGroupInterface::Plan plan;

                auto result = move_group_->plan(plan);
                if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                    move_group_->execute(plan);
                    RCLCPP_DEBUG(this->get_logger(),
                        "Executed plan → (%.3f, %.3f, %.3f)",
                        target.position.x, target.position.y, target.position.z);
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "IK failed for target (%.3f, %.3f, %.3f)",
                        target.position.x, target.position.y, target.position.z);
                }
            }
            rate.sleep();
        }
    }

private:
    void ballCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Transform predicted ball position into robot_base frame
        geometry_msgs::msg::PointStamped in_base;
        try {
            tf_buffer_->transform(*msg, in_base, "robot_base",
                tf2::durationFromSec(0.05));
        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "TF transform failed: %s", e.what());
            return;
        }

        // Reject targets with clearly invalid coordinates (e.g. trajectory fit blowup)
        if (std::abs(in_base.point.x) > 5.0 ||
            std::abs(in_base.point.y) > 5.0 ||
            std::abs(in_base.point.z) > 5.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Rejecting out-of-range target (%.1f, %.1f, %.1f) — trajectory fit may be invalid",
                in_base.point.x, in_base.point.y, in_base.point.z);
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        latest_target_.position.x  = in_base.point.x;
        latest_target_.position.y  = in_base.point.y;
        latest_target_.position.z  = in_base.point.z;
        latest_target_.orientation.w = 1.0;  // TODO: compute intercept orientation
        new_target_ = true;
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>    move_group_;
    std::shared_ptr<tf2_ros::Buffer>           tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex mutex_;
    geometry_msgs::msg::Pose latest_target_;
    std::atomic<bool> new_target_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ControlNode>();

    // Spin in background so MoveGroupInterface can connect to move_group
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // MoveIt planning loop — blocks until shutdown
    node->run_moveit();

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
