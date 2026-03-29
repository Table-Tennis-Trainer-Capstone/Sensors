#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <deque>
#include <vector>
#include <cmath>
#include <numeric>

// ── Stamped 3D sample ────────────────────────────────────────────────────────
struct Sample {
    double x, y, z;
    double t; // seconds, relative to first sample in buffer
};

class TrajectoryNode : public rclcpp::Node {
public:
    TrajectoryNode() : Node("trajectory_node") {

        // ── Parameters ───────────────────────────────────────────────────────
        this->declare_parameter("lookahead_ms",     200);
        this->declare_parameter("min_samples",       4);
        this->declare_parameter("max_samples",      10);
        this->declare_parameter("gravity",          9.81);
        this->declare_parameter("table_y",          0.0);   // Y of table in camera frame — measure after mounting
        this->declare_parameter("restitution",      0.85);
        this->declare_parameter("camera_tilt_deg",  0.0);   // positive = pitched down
        // net_z: Z depth of the net in camera frame (metres). Ball detections with
        // Z > net_z are on the opponent's side — buffer is cleared and tracking waits.
        // Set to 0.0 (default) to disable gating and track everywhere.
        this->declare_parameter("net_z",            0.0);
        //   0°  → camera level    → gravity only in Y
        //   45° → 45° down tilt  → gravity split equally between Y and Z
        //   90° → straight down  → gravity only in Z

        lookahead_ms_  = this->get_parameter("lookahead_ms").as_int();
        min_samples_   = this->get_parameter("min_samples").as_int();
        max_samples_   = this->get_parameter("max_samples").as_int();
        gravity_       = this->get_parameter("gravity").as_double();
        table_y_       = this->get_parameter("table_y").as_double();
        restitution_   = this->get_parameter("restitution").as_double();
        net_z_         = this->get_parameter("net_z").as_double();

        double tilt_rad = this->get_parameter("camera_tilt_deg").as_double() * M_PI / 180.0;
        gravity_y_ =  gravity_ * std::cos(tilt_rad);  // component along camera Y (down in image)
        gravity_z_ =  gravity_ * std::sin(tilt_rad);  // component along camera Z (into scene)

        RCLCPP_INFO(this->get_logger(),
            "Trajectory node | lookahead=%dms samples=%d-%d "
            "gravity=%.2f tilt=%.1f° → gy=%.2f gz=%.2f table_y=%.2f restitution=%.2f",
            lookahead_ms_, min_samples_, max_samples_,
            gravity_, this->get_parameter("camera_tilt_deg").as_double(),
            gravity_y_, gravity_z_, table_y_, restitution_);

        position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/ball_position_3d", 10,
            std::bind(&TrajectoryNode::positionCallback, this, std::placeholders::_1));

        predicted_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/ball_trajectory/predicted", 10);

        landing_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/ball_trajectory/landing", 10);
    }

private:
    void positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {

        // Net-Z gate: if enabled, ball on opponent's side (Z > net_z) resets tracking
        if (net_z_ > 0.0 && msg->point.z > net_z_) {
            if (!buffer_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Ball crossed back over net (Z=%.3f > %.3f) — resetting", msg->point.z, net_z_);
                buffer_.clear();
            }
            return;
        }

        double t_abs = rclcpp::Time(msg->header.stamp).seconds();

        if (buffer_.empty()) t0_ = t_abs;

        Sample s;
        s.x = msg->point.x;
        s.y = msg->point.y;
        s.z = msg->point.z;
        s.t = t_abs - t0_;
        buffer_.push_back(s);

        if ((int)buffer_.size() > max_samples_) buffer_.pop_front();
        if ((int)buffer_.size() < min_samples_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Collecting samples... (%zu/%d)", buffer_.size(), min_samples_);
            return;
        }

        // Normalize timestamps so oldest = 0
        std::vector<double> ts, xs, ys, zs;
        double t_offset = buffer_.front().t;
        for (const auto& b : buffer_) {
            ts.push_back(b.t - t_offset);
            xs.push_back(b.x);
            ys.push_back(b.y);
            zs.push_back(b.z);
        }

        // X is always linear (horizontal, no gravity component)
        double vx, x0;
        fitLinear(ts, xs, x0, vx);

        // Y: parabolic with gravity_y component
        //    y(t) = y0 + vy*t - 0.5*gravity_y*t²
        double vy, y0;
        fitParabolic(ts, ys, gravity_y_, y0, vy);

        // Z: parabolic with gravity_z component (non-zero when camera is tilted down)
        //    z(t) = z0 + vz*t + 0.5*gravity_z*t²
        //    (+ because tilted camera Z axis points forward-and-down, same direction as gravity)
        double vz, z0;
        fitParabolic(ts, zs, -gravity_z_, z0, vz);  // negative: subtract to get linear remainder

        double t_now   = s.t - t_offset;
        double t_ahead = t_now + (lookahead_ms_ / 1000.0);

        auto [px, py, pz, bounced] = predictWithBounce(x0, vx, y0, vy, z0, vz, t_now, t_ahead);

        auto predicted_msg = geometry_msgs::msg::PointStamped();
        predicted_msg.header.stamp    = msg->header.stamp;
        predicted_msg.header.frame_id = "camera_left_optical_frame";
        predicted_msg.point.x = px;
        predicted_msg.point.y = py;
        predicted_msg.point.z = pz;
        predicted_pub_->publish(predicted_msg);

        auto landing = findLanding(x0, vx, y0, vy, z0, vz, t_now);
        if (landing.has_value()) {
            auto landing_msg = geometry_msgs::msg::PointStamped();
            landing_msg.header.stamp    = msg->header.stamp;
            landing_msg.header.frame_id = "camera_left_optical_frame";
            landing_msg.point.x = landing->x;
            landing_msg.point.y = table_y_;
            landing_msg.point.z = landing->z;
            landing_pub_->publish(landing_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                "Predicted | X:%+.3f Y:%+.3f Z:%.3f (in %dms)%s | Landing X:%+.3f Z:%.3f in %.0fms",
                px, py, pz, lookahead_ms_, bounced ? " [post-bounce]" : "",
                landing->x, landing->z, landing->t_land * 1000.0);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                "Predicted | X:%+.3f Y:%+.3f Z:%.3f (in %dms)%s | No landing in range",
                px, py, pz, lookahead_ms_, bounced ? " [post-bounce]" : "");
        }
    }

    // ── Linear least squares: pos = p0 + v*t ─────────────────────────────────
    void fitLinear(const std::vector<double>& t, const std::vector<double>& p,
                   double& p0, double& v)
    {
        int n = t.size();
        double sum_t  = std::accumulate(t.begin(), t.end(), 0.0);
        double sum_p  = std::accumulate(p.begin(), p.end(), 0.0);
        double sum_tt = 0.0, sum_tp = 0.0;
        for (int i = 0; i < n; i++) { sum_tt += t[i]*t[i]; sum_tp += t[i]*p[i]; }
        double denom = n * sum_tt - sum_t * sum_t;
        if (std::abs(denom) < 1e-9) { p0 = sum_p / n; v = 0.0; return; }
        v  = (n * sum_tp - sum_t * sum_p) / denom;
        p0 = (sum_p - v * sum_t) / n;
    }

    // ── Parabolic fit: pos = p0 + v*t - 0.5*gcomp*t² ─────────────────────────
    // Pass gcomp = gravity_y_ for Y, or -gravity_z_ for Z (see sign note in caller)
    void fitParabolic(const std::vector<double>& t, const std::vector<double>& p,
                      double gcomp, double& p0, double& v)
    {
        std::vector<double> corrected(p.size());
        for (size_t i = 0; i < p.size(); i++)
            corrected[i] = p[i] + 0.5 * gcomp * t[i] * t[i];
        fitLinear(t, corrected, p0, v);
    }

    // ── Predict position, handling one bounce ─────────────────────────────────
    struct PredResult { double x, y, z; bool bounced; };

    PredResult predictWithBounce(
        double x0, double vx, double y0, double vy,
        double z0, double vz, double t_now, double t_pred)
    {
        double dt = t_pred - t_now;

        // Bounce detection: solve y0 + vy*dt_b - 0.5*gravity_y*dt_b² = table_y
        double a = 0.5 * gravity_y_;
        double b = -vy;
        double c = table_y_ - y0;
        double disc = b*b - 4*a*c;

        if (disc >= 0) {
            double sqrt_disc = std::sqrt(disc);
            double t1 = (-b - sqrt_disc) / (2*a);
            double t2 = (-b + sqrt_disc) / (2*a);
            double t_bounce = -1.0;
            if (t1 > 1e-3 && t1 < dt) t_bounce = t1;
            if (t2 > 1e-3 && t2 < dt && (t_bounce < 0 || t2 < t_bounce)) t_bounce = t2;

            if (t_bounce > 0) {
                double bx = x0 + vx * t_bounce;
                double bz = z0 + vz * t_bounce - 0.5 * gravity_z_ * t_bounce * t_bounce;
                double bvy = -(vy - gravity_y_ * t_bounce) * restitution_;
                double bvz =   vz - gravity_z_ * t_bounce;   // Z vel unchanged at bounce
                double dt_rem = dt - t_bounce;
                double px = bx + vx  * dt_rem;
                double py = table_y_ + bvy * dt_rem - 0.5 * gravity_y_ * dt_rem * dt_rem;
                double pz = bz + bvz * dt_rem - 0.5 * gravity_z_ * dt_rem * dt_rem;
                return {px, py, pz, true};
            }
        }

        double px = x0 + vx * dt;
        double py = y0 + vy * dt - 0.5 * gravity_y_ * dt * dt;
        double pz = z0 + vz * dt - 0.5 * gravity_z_ * dt * dt;
        return {px, py, pz, false};
    }

    // ── Find next table landing point ─────────────────────────────────────────
    struct LandingResult { double x, z, t_land; };

    std::optional<LandingResult> findLanding(
        double x0, double vx, double y0, double vy,
        double z0, double vz, double t_now)
    {
        double a = 0.5 * gravity_y_;
        double b = -vy;
        double c = table_y_ - y0;
        double disc = b*b - 4*a*c;
        if (disc < 0) return std::nullopt;

        double sqrt_disc = std::sqrt(disc);
        double t1 = (-b - sqrt_disc) / (2*a);
        double t2 = (-b + sqrt_disc) / (2*a);

        double t_land = -1.0;
        if (t1 > 1e-3) t_land = t1;
        if (t2 > 1e-3 && (t_land < 0 || t2 < t_land)) t_land = t2;
        if (t_land < 0) return std::nullopt;

        return LandingResult{
            x0 + vx * t_land,
            z0 + vz * t_land - 0.5 * gravity_z_ * t_land * t_land,
            t_land
        };
    }

    // ── Members ───────────────────────────────────────────────────────────────
    int lookahead_ms_, min_samples_, max_samples_;
    double gravity_, gravity_y_, gravity_z_;
    double table_y_, restitution_, net_z_;
    double t0_ = 0.0;

    std::deque<Sample> buffer_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr predicted_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr landing_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}
