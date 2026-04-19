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
#include <algorithm>

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("ttt_control_node"), new_target_(false)
    {
        this->declare_parameter("update_rate_hz",  240.0);
        this->declare_parameter("planning_time_s",  0.01);
        this->declare_parameter("speed_multiplier", 5.0);
        this->declare_parameter("intercept_x_offset", -0.25);
        this->declare_parameter("return_delay_ms", 50);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/ball_trajectory/predicted", 10,
            std::bind(&ControlNode::ballCallback, this, std::placeholders::_1));

        arm_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/arm_named_target", 10,
            std::bind(&ControlNode::armCmdCallback, this, std::placeholders::_1));

        stm_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/stm_cmd", 10);

        RCLCPP_INFO(this->get_logger(), "TTT Control node ready — waiting for ball trajectory");
    }

    void run_moveit()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "armgroup");

        move_group_->setEndEffectorLink("paddle_tcp");
        move_group_->setPlannerId("RRTConnect");
        move_group_->setPlanningTime(this->get_parameter("planning_time_s").as_double());
        move_group_->setNumPlanningAttempts(10);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setPoseReferenceFrame("root");

        // Drive arm to extended home position on startup (STM32 H-command leaves arm collapsed)
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        move_group_->setNamedTarget("home");
        moveit::planning_interface::MoveGroupInterface::Plan startup_plan;
        if (move_group_->plan(startup_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            send_goal_to_stm(startup_plan);
            move_group_->execute(startup_plan);
            RCLCPP_INFO(this->get_logger(), "Arm moved to home position");
        }

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

        // Extract final waypoint, apply STM32 offsets, and fire once
        auto send_goal_to_stm = [this](moveit::planning_interface::MoveGroupInterface::Plan& p) {
            if (p.trajectory_.joint_trajectory.points.empty()) return;

            auto final_point = p.trajectory_.joint_trajectory.points.back();
            auto joint_names = p.trajectory_.joint_trajectory.joint_names;
            
            std::vector<std::string> order = {"BaseRotate_0", "UpperArmRotate_0", "ForeArmRotate_0", "WristRotate_0", "PaddleRotate_0"};
            
            // STM32 hardware zero-offsets
            float offsets[5] = {0.0f, 15.0f, 25.0f, 0.0f, 0.0f}; 
            float degs[5] = {0};

            for (size_t i = 0; i < 5; i++) {
                auto it = std::find(joint_names.begin(), joint_names.end(), order[i]);
                if (it != joint_names.end()) {
                    int idx = std::distance(joint_names.begin(), it);
                    degs[i] = (final_point.positions[idx] * (180.0 / M_PI)) + offsets[i];
                }
            }

            // Only fire a new UDP packet if the goal has changed by > 0.1 degrees
            bool significant_change = false;
            for (size_t i = 0; i < 5; i++) {
                if (std::abs(degs[i] - last_degs_[i]) > 0.1f) {
                    significant_change = true;
                    break;
                }
            }
            if (!significant_change) return; // Skip redundant packets
            
            for (size_t i = 0; i < 5; i++) last_degs_[i] = degs[i];

            std::ostringstream ss;
            ss << "M " << std::fixed << std::setprecision(2) 
               << degs[0] << " " << degs[1] << " " << degs[2] << " " << degs[3] << " " << degs[4];
               
            std_msgs::msg::String msg;
            msg.data = ss.str();
            stm_cmd_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Fired Goal to STM32: %s", msg.data.c_str());
        };

        while (rclcpp::ok()) {
            std::string named_target;
            bool do_named = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (new_named_target_) {
                    named_target = pending_named_target_;
                    new_named_target_ = false;
                    new_target_ = false;
                    do_named = true;
                }
            }
            if (do_named) {
                move_group_->setPlannerId("RRTConnect");
                move_group_->setNamedTarget(named_target);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    overdrive_trajectory(plan);
                    send_goal_to_stm(plan);
                    move_group_->execute(plan);
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

                move_group_->setPositionTarget(target.position.x, target.position.y, target.position.z, "paddle_tcp");
                move_group_->setGoalPositionTolerance(0.08);   

                move_group_->clearPathConstraints();

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                auto result = move_group_->plan(plan);
                if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                    overdrive_trajectory(plan);
                    send_goal_to_stm(plan);
                    move_group_->execute(plan);
                    
                    // Auto-return to ready state
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(this->get_parameter("return_delay_ms").as_int()));
                    
                    move_group_->setNamedTarget("ready");
                    moveit::planning_interface::MoveGroupInterface::Plan return_plan;
                    if (move_group_->plan(return_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                        overdrive_trajectory(return_plan);
                        send_goal_to_stm(return_plan);
                        move_group_->execute(return_plan);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "IK Planning failed for Root Frame -> X: %.3f, Y: %.3f, Z: %.3f", target.position.x, target.position.y, target.position.z);
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
        geometry_msgs::msg::PointStamped in_base;
        in_base.header.stamp = msg->header.stamp;
        in_base.header.frame_id = "root";

        // Transform from "table" frame to MoveIt "root" frame:
        // Root X (Forward) = Table Z (Depth). Robot is at Z = -1.4732 relative to the net (Z=0).
        // intercept_x_offset pulls the hit point toward the robot's end of the table (negative = closer).
        double x_offset = this->get_parameter("intercept_x_offset").as_double();
        in_base.point.x = msg->point.z + 1.4732 + x_offset;
        // Root Y (Left) = Table X.
        in_base.point.y = msg->point.x;
        // Root Z (Up) = Table Y (Height).
        in_base.point.z = msg->point.y;

        RCLCPP_INFO(this->get_logger(), "Received Table Frame  -> X: %.3f, Y: %.3f, Z: %.3f", msg->point.x, msg->point.y, msg->point.z);
        RCLCPP_INFO(this->get_logger(), "Converted Root Frame  -> X: %.3f, Y: %.3f, Z: %.3f", in_base.point.x, in_base.point.y, in_base.point.z);

        std::lock_guard<std::mutex> lock(mutex_);
        latest_target_.position.x  = in_base.point.x;
        latest_target_.position.y  = in_base.point.y;
        latest_target_.position.z  = in_base.point.z;
        latest_target_.orientation.w = 1.0;
        new_target_ = true;
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             arm_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                stm_cmd_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>    move_group_;
    std::shared_ptr<tf2_ros::Buffer>           tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex mutex_;
    geometry_msgs::msg::Pose latest_target_;
    std::atomic<bool> new_target_;
    bool new_named_target_{false};
    std::string pending_named_target_;
    float last_degs_[5] = {-999.0f, -999.0f, -999.0f, -999.0f, -999.0f};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });
    node->run_moveit();
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}