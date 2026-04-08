#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>

// Subscribes to /ball_trajectory/predicted, transforms the target point into
// world frame, then sends a pose goal to MoveIt's move_group (armgroup,
// paddle_tcp end-effector). MoveIt runs KDL IK and drives the armgroup_controller,
// which publishes /joint_states to ttt_hardware

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("ttt_control_node"), new_target_(false)
    {
        this->declare_parameter("update_rate_hz",  10.0);
        this->declare_parameter("planning_time_s",  0.1);  // 100ms planning budget
        this->declare_parameter("return_delay_ms",  100);  // delay before returning home after swing
        this->declare_parameter("speed_multiplier", 4.0);  // Overdrive multiplier to bypass slow URDF limits

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/ball_trajectory/predicted", 10,
            std::bind(&ControlNode::ballCallback, this, std::placeholders::_1));

        arm_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/arm_named_target", 10,
            std::bind(&ControlNode::armCmdCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "TTT Control node ready — waiting for ball trajectory");
    }

    // Called from main after the executor is already spinning so that
    // MoveGroupInterface can reach the move_group action server.
    void run_moveit()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "armgroup");

        move_group_->setEndEffectorLink("paddle_tcp");
        // OMPL RRTConnect for ball tracking (handles position-only Cartesian goals).
        move_group_->setPlannerId("RRTConnect");
        move_group_->setPlanningTime(this->get_parameter("planning_time_s").as_double());
        move_group_->setNumPlanningAttempts(1);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);

        RCLCPP_INFO(this->get_logger(),
            "MoveIt ready | group: armgroup | end-effector: paddle_tcp | planning time: %.3f s",
            this->get_parameter("planning_time_s").as_double());

        rclcpp::Rate rate(this->get_parameter("update_rate_hz").as_double());

        double speed_mult = this->get_parameter("speed_multiplier").as_double();
        auto overdrive_trajectory = [speed_mult](moveit::planning_interface::MoveGroupInterface::Plan& p) {
            if (speed_mult <= 1.0) return;
            for (auto& pt : p.trajectory_.joint_trajectory.points) {
                double sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
                sec /= speed_mult;
                pt.time_from_start.sec = static_cast<int32_t>(std::floor(sec));
                pt.time_from_start.nanosec = static_cast<uint32_t>((sec - pt.time_from_start.sec) * 1e9);
                for (auto& v : pt.velocities) v *= speed_mult;
                for (auto& a : pt.accelerations) a *= (speed_mult * speed_mult);
            }
        };

        while (rclcpp::ok()) {
            // Named target commands 
            std::string named_target;
            bool do_named = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (new_named_target_) {
                    named_target = pending_named_target_;
                    new_named_target_ = false;
                    new_target_ = false;  // discard any queued ball target
                    do_named = true;
                }
            }
            if (do_named) {
                RCLCPP_INFO(this->get_logger(), "Moving to named target: '%s'", named_target.c_str());
                // Named targets are joint-space goals — use OMPL RRTConnect (reliable,
                // no IK required since joint values come directly from the SRDF).
                move_group_->setPlannerId("RRTConnect");
                move_group_->setNamedTarget(named_target);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    overdrive_trajectory(plan);
                    move_group_->execute(plan);
                    RCLCPP_INFO(this->get_logger(), "Named target '%s' executed", named_target.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Plan failed for named target '%s'", named_target.c_str());
                }
                rate.sleep();
                continue;
            }

            if (new_target_.exchange(false)) {
                geometry_msgs::msg::Pose target;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    target = latest_target_;
                }

                move_group_->setPositionTarget(
                    target.position.x, target.position.y, target.position.z, "paddle_tcp");
                move_group_->setGoalPositionTolerance(0.08);  // 8cm — ball interception tolerance

                moveit::planning_interface::MoveGroupInterface::Plan plan;

                RCLCPP_INFO(this->get_logger(),
                    "Planning to (%.3f, %.3f, %.3f) [root frame]",
                    target.position.x, target.position.y, target.position.z);
                auto result = move_group_->plan(plan);
                if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                    overdrive_trajectory(plan);
                    move_group_->execute(plan);
                    RCLCPP_INFO(this->get_logger(), "Executed plan → table (%.3f, %.3f, %.3f)",
                        target.position.x, target.position.y, target.position.z);

                    // Return to ready position after swing
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(
                            static_cast<int>(this->get_parameter("return_delay_ms").as_int())));
                    move_group_->setPlannerId("RRTConnect");
                    move_group_->setNamedTarget("ready");
                    moveit::planning_interface::MoveGroupInterface::Plan ready_plan;
                    if (move_group_->plan(ready_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                        overdrive_trajectory(ready_plan);
                        move_group_->execute(ready_plan);
                        RCLCPP_INFO(this->get_logger(), "Returned to ready");
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Planning failed (code %d) for target (%.3f, %.3f, %.3f)",
                        result.val, target.position.x, target.position.y, target.position.z);
                }
            }
            rate.sleep();
        }
    }

private:
    void armCmdCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_named_target_ = msg->data;
        new_named_target_ = true;
    }

    void ballCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Adapter for vision space to ROS space
        // Vision uses: X = Right, Y = Up, Z = Deep
        //ROS uses: X = Forward, Y = Left, Z = Up

        geometry_msgs::msg::PointStamped rep103_msg = *msg;
        rep103_msg.point.x = -msg->point.z;
        rep103_msg.point.y = -msg->point.x;
        rep103_msg.point.z = msg->point.y;
        
        // Transform predicted ball position into world frame
        geometry_msgs::msg::PointStamped in_base;
        try {
            // Transform into 'root' frame (the URDF base link, which equals MoveIt's
            // planning frame 'world' via the fixed virtual joint — identity transform).
            // 'world' is not published to TF by move_group for fixed virtual joints,
            // so we use 'root' which is always present in the TF tree.
            tf_buffer_->transform(*msg, in_base, "root",
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
        latest_target_.orientation.w = 1.0;
        new_target_ = true;
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             arm_cmd_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>    move_group_;
    std::shared_ptr<tf2_ros::Buffer>           tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex mutex_;
    geometry_msgs::msg::Pose latest_target_;
    std::atomic<bool> new_target_;
    bool new_named_target_{false};
    std::string pending_named_target_;
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
