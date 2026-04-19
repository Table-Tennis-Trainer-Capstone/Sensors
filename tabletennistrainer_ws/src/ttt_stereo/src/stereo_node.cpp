#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <ttt_msgs/msg/ball_detection.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <chrono>
#include <cmath>

using BallDetection = ttt_msgs::msg::BallDetection;
using ApproxPolicy = message_filters::sync_policies::ApproximateTime<BallDetection, BallDetection>;

class StereoNode : public rclcpp::Node {
public:
    StereoNode() : Node("stereo_node") {
        // Lens Params
        this->declare_parameter("fx", 448.2);
        this->declare_parameter("fy", 400.0);
        this->declare_parameter("cx", 320.0);
        this->declare_parameter("cy", 200.0);
        this->declare_parameter("baseline_m", 1.525);
        this->declare_parameter("max_sync_age_ms", 50);

        // Alignment Params
        this->declare_parameter("pan_left_deg", 15.0);
        this->declare_parameter("pan_right_deg", 15.0);
        this->declare_parameter("roll_left_deg", 0.0);
        this->declare_parameter("roll_right_deg", 0.0);
        this->declare_parameter("tilt_left_deg", 45.0);
        this->declare_parameter("tilt_right_deg", 45.0);
        
        // Origin Shift Params
        this->declare_parameter("height_m", 1.25);   // Avg height for Y-zero
        this->declare_parameter("net_dist_m", 0.52); // Dist to net for Z-zero

        // 3D Workspace Limits (Table Frame)
        this->declare_parameter("limit_x_m", 1.5);         // Max distance left/right of center
        this->declare_parameter("limit_y_top_m", 2.0);     // Max height above table
        this->declare_parameter("limit_y_bottom_m", -0.2); // Max depth below table surface
        this->declare_parameter("limit_z_m", 2.5);         // Max distance forward/back from net

        load_params();

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) {
                for (const auto &p : params) {
                    if (p.get_name() == "pan_left_deg") pan_l_rad_ = p.as_double() * M_PI / 180.0;
                    if (p.get_name() == "pan_right_deg") pan_r_rad_ = p.as_double() * M_PI / 180.0;
                    if (p.get_name() == "tilt_left_deg") tilt_l_rad_ = p.as_double() * M_PI / 180.0;
                    if (p.get_name() == "tilt_right_deg") tilt_r_rad_ = p.as_double() * M_PI / 180.0;
                    if (p.get_name() == "roll_left_deg") roll_l_rad_ = p.as_double() * M_PI / 180.0;
                    if (p.get_name() == "roll_right_deg") roll_r_rad_ = p.as_double() * M_PI / 180.0;
                    if (p.get_name() == "baseline_m") baseline_ = p.as_double();
                }
                rcl_interfaces::msg::SetParametersResult res;
                res.successful = true;
                return res;
            });

        left_sub_.subscribe(this, "/ball_detection/left");
        right_sub_.subscribe(this, "/ball_detection/right");
        sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy>>(ApproxPolicy(10), left_sub_, right_sub_);
        sync_->registerCallback(std::bind(&StereoNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/ball_position_3d", 10);
    }

private:
    void load_params() {
        fx_ = this->get_parameter("fx").as_double();
        fy_ = this->get_parameter("fy").as_double();
        cx_ = this->get_parameter("cx").as_double();
        cy_ = this->get_parameter("cy").as_double();
        baseline_ = this->get_parameter("baseline_m").as_double();
        max_sync_age_ms_ = this->get_parameter("max_sync_age_ms").as_int();
        pan_l_rad_ = this->get_parameter("pan_left_deg").as_double() * M_PI / 180.0;
        pan_r_rad_ = this->get_parameter("pan_right_deg").as_double() * M_PI / 180.0;
        tilt_l_rad_ = this->get_parameter("tilt_left_deg").as_double() * M_PI / 180.0;
        tilt_r_rad_ = this->get_parameter("tilt_right_deg").as_double() * M_PI / 180.0;
        roll_l_rad_ = this->get_parameter("roll_left_deg").as_double() * M_PI / 180.0;
        roll_r_rad_ = this->get_parameter("roll_right_deg").as_double() * M_PI / 180.0;
    }

    void syncCallback(const BallDetection::ConstSharedPtr& left, const BallDetection::ConstSharedPtr& right) {
        if (left->x < 0 || right->x < 0) return;

        double ul = (left->x - cx_) / fx_;
        double vl = (left->y - cy_) / fy_;
        double ur = (right->x - cx_) / fx_;
        double vr = (right->y - cy_) / fy_;

        // Left Ray Construction
        double xl1 = ul * cos(roll_l_rad_) + vl * sin(roll_l_rad_);
        double yl1 = -ul * sin(roll_l_rad_) + vl * cos(roll_l_rad_);
        double zl1 = 1.0;
        double xl2 = xl1;
        double yl2 = yl1 * cos(tilt_l_rad_) + zl1 * sin(tilt_l_rad_);
        double zl2 = -yl1 * sin(tilt_l_rad_) + zl1 * cos(tilt_l_rad_);
        double dl_x = xl2 * cos(pan_l_rad_) + zl2 * sin(pan_l_rad_);
        double dl_y = yl2;
        double dl_z = -xl2 * sin(pan_l_rad_) + zl2 * cos(pan_l_rad_);

        // Right Ray Construction
        double xr1 = ur * cos(roll_r_rad_) + vr * sin(roll_r_rad_);
        double yr1 = -ur * sin(roll_r_rad_) + vr * cos(roll_r_rad_);
        double zr1 = 1.0;
        double xr2 = xr1;
        double yr2 = yr1 * cos(tilt_r_rad_) + zr1 * sin(tilt_r_rad_);
        double zr2 = -yr1 * sin(tilt_r_rad_) + zr1 * cos(tilt_r_rad_);
        double dr_x = xr2 * cos(-pan_r_rad_) + zr2 * sin(-pan_r_rad_);
        double dr_y = yr2;
        double dr_z = -xr2 * sin(-pan_r_rad_) + zr2 * cos(-pan_r_rad_);

        // Intersection Math
        double w0_x = -baseline_; double w0_y = 0.0; double w0_z = 0.0;
        double a = dl_x*dl_x + dl_y*dl_y + dl_z*dl_z;
        double b = dl_x*dr_x + dl_y*dr_y + dl_z*dr_z;
        double c = dr_x*dr_x + dr_y*dr_y + dr_z*dr_z;
        double d = w0_x*dl_x + w0_y*dl_y + w0_z*dl_z;
        double e = w0_x*dr_x + w0_y*dr_y + w0_z*dr_z;

        double denom = a*c - b*b;
        if (std::abs(denom) < 1e-6) return;

        double s = (b*e - c*d) / denom;
        double t = (a*e - b*d) / denom;

        // Raw 3D Point in Camera Space
        double raw_x = (s * dl_x + baseline_ + t * dr_x) / 2.0;
        double raw_y = (s * dl_y + t * dr_y) / 2.0;
        double raw_z = (s * dl_z + t * dr_z) / 2.0;

        // GHOST POINT FILTER (Ray Miss Distance)
        // If the cameras detected two different background objects, their rays won't physically intersect
        double pL_x = s * dl_x;
        double pL_y = s * dl_y;
        double pL_z = s * dl_z;
        
        double pR_x = baseline_ + t * dr_x;
        double pR_y = t * dr_y;
        double pR_z = t * dr_z;
        
        double ray_miss_dist = std::sqrt(std::pow(pL_x - pR_x, 2) + std::pow(pL_y - pR_y, 2) + std::pow(pL_z - pR_z, 2));
        
        // If rays miss each other by more than 25cm, they are looking at different things (Ghost point)
        if (ray_miss_dist > 0.25) {
            return;
        }

        // TRIPLE ORIGIN SHIFT
        // Center X (Table Center = 0)
        double out_x = raw_x - (baseline_ / 2.0);

        // Center Y (Table Surface = 0)
        // Camera Y is positive down. Subtracting from cam height makes 'up' positive.
        double cam_h = this->get_parameter("height_m").as_double();
        double out_y = cam_h - raw_y;

        // Center Z (Net = 0)
        double net_z = this->get_parameter("net_dist_m").as_double();
        double out_z = raw_z - net_z;

        // BOUNDING BOX
        // Reject background lights and noise that triangulate outside the play area
        double lim_x   = this->get_parameter("limit_x_m").as_double();
        double lim_y_t = this->get_parameter("limit_y_top_m").as_double();
        double lim_y_b = this->get_parameter("limit_y_bottom_m").as_double();
        double lim_z   = this->get_parameter("limit_z_m").as_double();

        if (std::abs(out_x) > lim_x || out_y > lim_y_t || out_y < lim_y_b || std::abs(out_z) > lim_z) {
            return; // Out of bounds, ignore entirely
        }

        auto out = geometry_msgs::msg::PointStamped();
        out.header.stamp = left->header.stamp;
        out.header.frame_id = "table";
        out.point.x = out_x;
        out.point.y = out_y;
        out.point.z = out_z;
        position_pub_->publish(out);
    }

    double fx_, fy_, cx_, cy_, baseline_;
    double pan_l_rad_, pan_r_rad_, tilt_l_rad_, tilt_r_rad_, roll_l_rad_, roll_r_rad_;
    int max_sync_age_ms_;
    message_filters::Subscriber<BallDetection> left_sub_, right_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproxPolicy>> sync_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoNode>());
    rclcpp::shutdown();
    return 0;
}