#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <deque>
#include <vector>
#include <cmath>
#include <numeric>

// Stamped 3D sample
struct Sample {
    double x, y, z;
    double t;      // seconds, relative to first sample in buffer
    double t_abs;  // absolute timestamp for gap checking
};

class TrajectoryNode : public rclcpp::Node {
public:
    TrajectoryNode() : Node("trajectory_node") {


        this->declare_parameter("lookahead_ms",       200);
        this->declare_parameter("min_samples",          4);
        this->declare_parameter("max_samples",         10);
        this->declare_parameter("gravity",             9.81);
        this->declare_parameter("table_y",             0.0);   
        this->declare_parameter("restitution",         0.85);
        this->declare_parameter("camera_tilt_deg",     0.0);   
        
        // Minimum speed towards MARTY (-Z direction) to trigger a prediction
        this->declare_parameter("min_incoming_speed",  0.5);   
        // How close to the net (Z=0) the ball must have been to be considered an opponent hit
        this->declare_parameter("net_margin_z",       -0.2);   
        // Maximum Z depth to track. Ignores noise beyond the table end.
        this->declare_parameter("max_track_z",         1.15);  
        // Maximum physical velocity (m/s). Rejects noise jumps.
        this->declare_parameter("max_velocity",        25.0);  

        lookahead_ms_  = this->get_parameter("lookahead_ms").as_int();
        min_samples_   = this->get_parameter("min_samples").as_int();
        max_samples_   = this->get_parameter("max_samples").as_int();
        gravity_       = this->get_parameter("gravity").as_double();
        table_y_       = this->get_parameter("table_y").as_double();
        restitution_   = this->get_parameter("restitution").as_double();
        
        min_speed_     = this->get_parameter("min_incoming_speed").as_double();
        net_margin_    = this->get_parameter("net_margin_z").as_double();
        max_track_z_   = this->get_parameter("max_track_z").as_double();
        max_velocity_  = this->get_parameter("max_velocity").as_double();

        // The prediction math uses: py = y0 + vy*t - 0.5 * gravity_y_ * t^2
        // Since Y is UP and we want gravity to pull DOWN (negative), 
        // gravity_y_ must be POSITIVE (e.g. +9.81) so the subtracted term is negative.
        gravity_y_ = gravity_;
        gravity_z_ = 0.0;

        RCLCPP_INFO(this->get_logger(),
            "Trajectory node (Robust) | lookahead=%dms samples=%d-%d | min_speed=%.1fm/s net_margin=%.1fm",
            lookahead_ms_, min_samples_, max_samples_, min_speed_, net_margin_);

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
        double t_abs = rclcpp::Time(msg->header.stamp).seconds();

        // SPATIAL PRE-FILTER: Ignore anything physically beyond the tracking limit 
        if (msg->point.z > max_track_z_) {
            return;
        }

        // CONTINUITY CHECK: If tracking drops for >200ms, assume it's a new stroke and wipe the buffer.
        if (!buffer_.empty() && (t_abs - buffer_.back().t_abs > 0.2)) {
            RCLCPP_INFO(this->get_logger(), "Tracking gap > 200ms. Resetting trajectory buffer.");
            buffer_.clear();
            has_prev_pred_ = false;
            has_prev_land_ = false;
            originated_across_net_ = false;
            despike_strikes_ = 0;
        }

        // Keep track of origin state persistently across bounces
        if (msg->point.z > net_margin_) {
            originated_across_net_ = true;
        }

        // BOUNCE CLEAR: If the ball bounces ANYWHERE, clear the buffer.
        // This prevents fitting a single parabola across the V-shape of a bounce.
        // (Table Y is 0.0, ball radius + jitter margin = 0.05m)
        if (msg->point.y < table_y_ + 0.05) {
            buffer_.clear();
            has_prev_pred_ = false;
            has_prev_land_ = false;
            despike_strikes_ = 0;
        }

        // DESPIKE PRE-FILTER: Prevent impossible tracking jumps from corrupting the buffer.
        // Protects the historical arc if the vision system accidentally flashes onto background noise.
        if (!buffer_.empty()) {
            double dt = t_abs - buffer_.back().t_abs;
            if (dt > 0.0) {
                double dist = std::sqrt(std::pow(msg->point.x - buffer_.back().x, 2) +
                                        std::pow(msg->point.y - buffer_.back().y, 2) +
                                        std::pow(msg->point.z - buffer_.back().z, 2));
                if ((dist / dt) > max_velocity_) {
                    despike_strikes_++;
                    if (despike_strikes_ > 3) {
                        // If it teleports and STAYS there, it's a new legitimate target. Restart!
                        RCLCPP_WARN(this->get_logger(), "Tracking teleported! Resetting buffer to new target.");
                        buffer_.clear();
                        has_prev_pred_ = false;
                        has_prev_land_ = false;
                        originated_across_net_ = false;
                        despike_strikes_ = 0;
                    } else {
                        return; // Drop the 1-to-3 frame noise glitch, keep the valid buffer history!
                    }
                } else {
                    despike_strikes_ = 0; // Good point, reset strikes
                }
            }
        }

        if (buffer_.empty()) t0_ = t_abs;

        Sample s;
        s.x = msg->point.x;
        s.y = msg->point.y;
        s.z = msg->point.z;
        s.t_abs = t_abs;
        s.t = t_abs - t0_;
        buffer_.push_back(s);

        if ((int)buffer_.size() > max_samples_) buffer_.pop_front();
        if ((int)buffer_.size() < min_samples_) return;

        // Normalize timestamps so oldest = 0
        std::vector<double> ts, xs, ys, zs;
        double t_offset = buffer_.front().t;
        for (const auto& b : buffer_) {
            ts.push_back(b.t - t_offset);
            xs.push_back(b.x);
            ys.push_back(b.y);
            zs.push_back(b.z);
        }

        // Fit Z first so we can check the velocity before doing the heavy lifting
        double vz, z0;
        fitParabolic(ts, zs, -gravity_z_, z0, vz); 

        // DIRECTION CHECK: Is the ball moving towards MARTY fast enough?
        // (vz must be negative, and the magnitude must be > min_speed_)
        if (vz > -min_speed_) {
            // Ball is either moving away (outgoing shot) or rolling too slowly.
            return;
        }

        // MAX VELOCITY CHECK: Reject teleporting noise blobs
        if (std::abs(vz) > max_velocity_) {
            return;
        }

        // STRICT DISPLACEMENT & TIME CHECK: Filter out Z-noise and slow dribbles.
        // The ball must have dropped a certain amount of Z in a certain time.
        double z_drop = buffer_.front().z - buffer_.back().z;
        double dt_total = buffer_.back().t - buffer_.front().t;
        
        if (z_drop < 0.04) {  // Require 4cm of movement to overcome stereo jitter
            return;
        }
        if (dt_total > 0.0 && (z_drop / dt_total) < min_speed_) {
            return;
        }

        // ORIGIN CHECK: Did this shot come from across the net?
        if (!originated_across_net_) {
            // Ball is moving towards MARTY, but it started deep on MARTY's side 
            // (e.g., dribbling backward or rolling). Ignore it.
            return;
        }

        // Fit X and Y
        double vx, x0;
        fitLinear(ts, xs, x0, vx);

        double vy, y0;
        fitParabolic(ts, ys, gravity_y_, y0, vy);

        // SANITY CHECK: Are the fitted lateral/vertical speeds physically possible?
        if (std::abs(vx) > max_velocity_ || std::abs(vy) > max_velocity_) {
            return;
        }

        double t_now   = s.t - t_offset;

        auto landing = findLanding(x0, vx, y0, vy, z0, vz, t_now);
        if (landing.has_value()) {
            
            // LANDING BOUNDS CHECK: Reject trajectories that land in another timezone
            // (Table width is +/-0.76m. Robot base is at Z=-1.4732m. If it lands behind the robot or way off side, ignore it)
            if (std::abs(landing->x) > 1.5 || landing->z < -1.48 || landing->z > 2.0) {
                return;
            }

            // ONLY ALLOW LANDING SPOTS ON MARTY'S SIDE OF THE NET (Z < 0)
            if (landing->z < 0.0) {
                
                // POST-BOUNCE INTERCEPT: Target the ball X milliseconds AFTER it bounces on Marty's side!
                double t_intercept = landing->t_land + (lookahead_ms_ / 1000.0);
                auto [px, py, pz, bounced] = predictWithBounce(x0, vx, y0, vy, z0, vz, t_now, t_intercept);

                // If it's a new prediction OR the jump is huge (ghost), snap to it immediately
                if (!has_prev_pred_ || std::abs(px - smooth_px_) > 1.0) {
                    smooth_px_ = px; smooth_py_ = py; smooth_pz_ = pz;
                    has_prev_pred_ = true;
                } else {
                    double a = 0.4; // Increased from 0.15 so it tracks faster and stops lagging behind
                    smooth_px_ = a * px + (1.0 - a) * smooth_px_;
                    smooth_py_ = a * py + (1.0 - a) * smooth_py_;
                    smooth_pz_ = a * pz + (1.0 - a) * smooth_pz_;
                }

                auto predicted_msg = geometry_msgs::msg::PointStamped();
                predicted_msg.header.stamp    = msg->header.stamp;
                predicted_msg.header.frame_id = "table";
                predicted_msg.point.x = smooth_px_;
                predicted_msg.point.y = smooth_py_;
                predicted_msg.point.z = smooth_pz_;
                predicted_pub_->publish(predicted_msg);

                if (!has_prev_land_) {
                    smooth_lx_ = landing->x; smooth_lz_ = landing->z;
                    has_prev_land_ = true;
                } else {
                    double a = 0.15;
                    smooth_lx_ = a * landing->x + (1.0 - a) * smooth_lx_;
                    smooth_lz_ = a * landing->z + (1.0 - a) * smooth_lz_;
                }

                auto landing_msg = geometry_msgs::msg::PointStamped();
                landing_msg.header.stamp    = msg->header.stamp;
                landing_msg.header.frame_id = "table";
                landing_msg.point.x = smooth_lx_;
                landing_msg.point.y = table_y_;
                landing_msg.point.z = smooth_lz_;
                landing_pub_->publish(landing_msg);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                    "INCOMING! | Vz:%.2f m/s | Landing X:%+.3f Z:%.3f in %.0fms",
                    vz, smooth_lx_, smooth_lz_, landing->t_land * 1000.0);
            }
        }
    }

    // Linear least squares: pos = p0 + v*t 
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

    // Parabolic fit: pos = p0 + v*t - 0.5 * gcomp * t^2
    void fitParabolic(const std::vector<double>& t, const std::vector<double>& p,
                      double gcomp, double& p0, double& v)
    {
        std::vector<double> corrected(p.size());
        for (size_t i = 0; i < p.size(); i++)
            corrected[i] = p[i] + 0.5 * gcomp * t[i] * t[i];
        fitLinear(t, corrected, p0, v);
    }

    // Predict position, handling one bounce
    struct PredResult { double x, y, z; bool bounced; };

    PredResult predictWithBounce(
        double x0, double vx, double y0, double vy,
        double z0, double vz, double t_now, double t_pred)
    {
        double a = 0.5 * gravity_y_;
        double b = -vy;
        double c = table_y_ - y0;
        double disc = b*b - 4*a*c;

        if (disc >= 0) {
            double sqrt_disc = std::sqrt(disc);
            double t1 = (-b - sqrt_disc) / (2*a);
            double t2 = (-b + sqrt_disc) / (2*a);
            double t_bounce = -1.0;
                if (t1 > t_now && t1 < t_pred) t_bounce = t1;
                if (t2 > t_now && t2 < t_pred && (t_bounce < 0 || t2 < t_bounce)) t_bounce = t2;

            if (t_bounce > 0) {
                double bx = x0 + vx * t_bounce;
                double bz = z0 + vz * t_bounce - 0.5 * gravity_z_ * t_bounce * t_bounce;
                double bvy = -(vy - gravity_y_ * t_bounce) * restitution_;
                double bvz =   vz - gravity_z_ * t_bounce; 
                    double dt_rem = t_pred - t_bounce;
                    double px = bx + vx * dt_rem;
                double py = table_y_ + bvy * dt_rem - 0.5 * gravity_y_ * dt_rem * dt_rem;
                double pz = bz + bvz * dt_rem - 0.5 * gravity_z_ * dt_rem * dt_rem;
                return {px, py, pz, true};
            }
        }

            double px = x0 + vx * t_pred;
            double py = y0 + vy * t_pred - 0.5 * gravity_y_ * t_pred * t_pred;
            double pz = z0 + vz * t_pred - 0.5 * gravity_z_ * t_pred * t_pred;
        return {px, py, pz, false};
    }

    // Find next table landing point
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
            if (t1 > t_now) t_land = t1;
            if (t2 > t_now && (t_land < 0 || t2 < t_land)) t_land = t2;
        if (t_land < 0) return std::nullopt;

        return LandingResult{
            x0 + vx * t_land,
            z0 + vz * t_land - 0.5 * gravity_z_ * t_land * t_land,
            t_land
        };
    }
    
    int lookahead_ms_, min_samples_, max_samples_;
    double gravity_, gravity_y_, gravity_z_;
    double table_y_, restitution_, net_z_;
    double min_speed_, net_margin_;
    double max_track_z_;
    double max_velocity_;
    double t0_ = 0.0;
    int despike_strikes_ = 0;

    // EMA Smoothing State
    bool has_prev_pred_ = false;
    bool has_prev_land_ = false;
    double smooth_px_ = 0.0, smooth_py_ = 0.0, smooth_pz_ = 0.0;
    double smooth_lx_ = 0.0, smooth_lz_ = 0.0;
    bool originated_across_net_ = false;

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