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
        this->declare_parameter("lookahead_ms",   250);   // how far ahead to predict
        this->declare_parameter("min_samples",    5);     // minimum samples before predicting
        this->declare_parameter("max_samples",    20);    // rolling buffer size
        this->declare_parameter("gravity",        9.81);  // m/s²
        this->declare_parameter("table_y",        0.0);   // Y coord of table surface (meters)
        this->declare_parameter("restitution",    0.85);  // bounce energy retention (0-1)

        lookahead_ms_  = this->get_parameter("lookahead_ms").as_int();
        min_samples_   = this->get_parameter("min_samples").as_int();
        max_samples_   = this->get_parameter("max_samples").as_int();
        gravity_       = this->get_parameter("gravity").as_double();
        table_y_       = this->get_parameter("table_y").as_double();
        restitution_   = this->get_parameter("restitution").as_double();

        RCLCPP_INFO(this->get_logger(),
            "Trajectory node | lookahead=%dms samples=%d-%d gravity=%.2f table_y=%.2f restitution=%.2f",
            lookahead_ms_, min_samples_, max_samples_, gravity_, table_y_, restitution_);

        // ── Subscriptions / Publishers ───────────────────────────────────────
        position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/ball_position_3d", 10,
            std::bind(&TrajectoryNode::positionCallback, this, std::placeholders::_1));

        // Predicted position at lookahead_ms in the future
        predicted_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/ball_trajectory/predicted", 10);

        // Bounce landing point (where ball will next hit the table)
        landing_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/ball_trajectory/landing", 10);
    }

private:
    void positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {

        double t_abs = rclcpp::Time(msg->header.stamp).seconds();

        // ── Maintain rolling buffer ───────────────────────────────────────────
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

        // ── Fit physics model to buffer ───────────────────────────────────────
        // Separate into time vector and position vectors
        std::vector<double> ts, xs, ys, zs;
        for (const auto& b : buffer_) {
            ts.push_back(b.t);
            xs.push_back(b.x);
            ys.push_back(b.y);
            zs.push_back(b.z);
        }

        // X and Z are linear:  pos = p0 + v*t
        double vx, x0, vz, z0;
        fitLinear(ts, xs, x0, vx);
        fitLinear(ts, zs, z0, vz);

        // Y is parabolic:  pos = y0 + vy*t - 0.5*g*t²
        double vy, y0;
        fitParabolic(ts, ys, y0, vy);

        // ── Predict position at lookahead ─────────────────────────────────────
        double t_now     = s.t;
        double t_ahead   = t_now + (lookahead_ms_ / 1000.0);

        auto [px, py, pz, bounced] = predictWithBounce(x0, vx, y0, vy, z0, vz, t_now, t_ahead);

        auto predicted_msg = geometry_msgs::msg::PointStamped();
        predicted_msg.header.stamp = msg->header.stamp;
        predicted_msg.header.frame_id = "stereo_camera_frame";
        predicted_msg.point.x = px;
        predicted_msg.point.y = py;
        predicted_msg.point.z = pz;
        predicted_pub_->publish(predicted_msg);

        // ── Find next table landing point ─────────────────────────────────────
        auto landing = findLanding(x0, vx, y0, vy, z0, vz, t_now);
        if (landing.has_value()) {
            auto landing_msg = geometry_msgs::msg::PointStamped();
            landing_msg.header.stamp = msg->header.stamp;
            landing_msg.header.frame_id = "stereo_camera_frame";
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

    // ── Linear least squares fit: pos = p0 + v*t ─────────────────────────────
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

    // ── Parabolic fit: pos = y0 + vy*t - 0.5*g*t²  ───────────────────────────
    // Gravity is known, so we subtract it and fit the remainder linearly
    void fitParabolic(const std::vector<double>& t, const std::vector<double>& y,
                      double& y0, double& vy)
    {
        // Remove known gravity component then fit linear
        std::vector<double> y_corrected(y.size());
        for (size_t i = 0; i < y.size(); i++) {
            y_corrected[i] = y[i] + 0.5 * gravity_ * t[i] * t[i];
        }
        fitLinear(t, y_corrected, y0, vy);
    }

    // ── Predict position, handling one bounce off the table ───────────────────
    struct PredResult { double x, y, z; bool bounced; };

    PredResult predictWithBounce(
        double x0, double vx, double y0, double vy,
        double z0, double vz, double t_now, double t_pred)
    {
        double dt = t_pred - t_now;

        // Check if ball hits table before t_pred
        // Solve: y0 + vy*dt_b - 0.5*g*dt_b² = table_y
        // → 0.5*g*dt_b² - vy*dt_b + (table_y - y0) = 0
        double a = 0.5 * gravity_;
        double b = -vy;
        double c = table_y_ - y0;
        double disc = b*b - 4*a*c;

        if (disc >= 0) {
            double sqrt_disc = std::sqrt(disc);
            double t1 = (-b - sqrt_disc) / (2*a);
            double t2 = (-b + sqrt_disc) / (2*a);
            // Pick smallest positive root
            double t_bounce = -1.0;
            if (t1 > 1e-3 && t1 < dt) t_bounce = t1;
            if (t2 > 1e-3 && t2 < dt && (t_bounce < 0 || t2 < t_bounce)) t_bounce = t2;

            if (t_bounce > 0) {
                // Position at bounce
                double bx = x0 + vx * t_bounce;
                double bz = z0 + vz * t_bounce;
                // Velocity at bounce — flip Y with restitution
                double bvy = -(vy - gravity_ * t_bounce) * restitution_;
                // Remaining time after bounce
                double dt_rem = dt - t_bounce;
                double px = bx + vx * dt_rem;
                double py = table_y_ + bvy * dt_rem - 0.5 * gravity_ * dt_rem * dt_rem;
                double pz = bz + vz * dt_rem;
                return {px, py, pz, true};
            }
        }

        // No bounce — straight projectile
        double px = x0 + vx * dt;
        double py = y0 + vy * dt - 0.5 * gravity_ * dt * dt;
        double pz = z0 + vz * dt;
        return {px, py, pz, false};
    }

    // ── Find next table landing point ─────────────────────────────────────────
    struct LandingResult { double x, z, t_land; };

    std::optional<LandingResult> findLanding(
        double x0, double vx, double y0, double vy,
        double z0, double vz, double t_now)
    {
        double a = 0.5 * gravity_;
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
            z0 + vz * t_land,
            t_land
        };
    }

    // ── Members ───────────────────────────────────────────────────────────────
    int lookahead_ms_, min_samples_, max_samples_;
    double gravity_, table_y_, restitution_;
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