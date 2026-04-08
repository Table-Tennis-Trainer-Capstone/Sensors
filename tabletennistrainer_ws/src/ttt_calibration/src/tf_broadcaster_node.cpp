#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class TFBroadcasterNode : public rclcpp::Node
{
public:
    TFBroadcasterNode() : Node("tf_broadcaster")
    {
        // Load parameters from config
        this->declare_parameter("left_camera.x", -0.75);
        this->declare_parameter("left_camera.y", 0.0);
        this->declare_parameter("left_camera.z", 1.0);
        this->declare_parameter("left_camera.roll", 0.0);
        this->declare_parameter("left_camera.pitch", -0.785);
        this->declare_parameter("left_camera.yaw", 0.0);
        
        this->declare_parameter("right_camera.x", 0.75);
        this->declare_parameter("right_camera.y", 0.0);
        this->declare_parameter("right_camera.z", 1.0);
        this->declare_parameter("right_camera.roll", 0.0);
        this->declare_parameter("right_camera.pitch", -0.785);
        this->declare_parameter("right_camera.yaw", 0.0);
        
        this->declare_parameter("robot_base.x", -1.4732);
        this->declare_parameter("robot_base.y", 0.0);
        this->declare_parameter("robot_base.z", 0.0);
        this->declare_parameter("robot_base.roll", 0.0);
        this->declare_parameter("robot_base.pitch", 0.0);
        this->declare_parameter("robot_base.yaw", 0.0);
        
        // Create static transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // Publish transforms
        publishTransforms();
        
        RCLCPP_INFO(this->get_logger(), "TF broadcaster started - publishing static transforms");
    }

private:
    void publishTransforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        
        // Table frame (origin)
        auto world_to_table = createTransform("world", "table", 0, 0, 0, 0, 0, 0);
        transforms.push_back(world_to_table);
        
        // Left camera
        auto table_to_left_cam = createTransform(
            "table", "camera_left_optical_frame",
            this->get_parameter("left_camera.x").as_double(),
            this->get_parameter("left_camera.y").as_double(),
            this->get_parameter("left_camera.z").as_double(),
            this->get_parameter("left_camera.roll").as_double(),
            this->get_parameter("left_camera.pitch").as_double(),
            this->get_parameter("left_camera.yaw").as_double()
        );
        transforms.push_back(table_to_left_cam);
        
        // Right camera
        auto table_to_right_cam = createTransform(
            "table", "camera_right_optical_frame",
            this->get_parameter("right_camera.x").as_double(),
            this->get_parameter("right_camera.y").as_double(),
            this->get_parameter("right_camera.z").as_double(),
            this->get_parameter("right_camera.roll").as_double(),
            this->get_parameter("right_camera.pitch").as_double(),
            this->get_parameter("right_camera.yaw").as_double()
        );
        transforms.push_back(table_to_right_cam);
        
        // Robot base
        auto table_to_robot = createTransform(
            "table", "robot_base",
            this->get_parameter("robot_base.x").as_double(),
            this->get_parameter("robot_base.y").as_double(),
            this->get_parameter("robot_base.z").as_double(),
            this->get_parameter("robot_base.roll").as_double(),
            this->get_parameter("robot_base.pitch").as_double(),
            this->get_parameter("robot_base.yaw").as_double()
        );
        transforms.push_back(table_to_robot);
        
        // Broadcast all transforms
        tf_broadcaster_->sendTransform(transforms);
    }
    
    geometry_msgs::msg::TransformStamped createTransform(
        const std::string& parent, const std::string& child,
        double x, double y, double z,
        double roll, double pitch, double yaw)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = parent;
        t.child_frame_id = child;
        
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;
        
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        return t;
    }
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFBroadcasterNode>());
    rclcpp::shutdown();
    return 0;
}
