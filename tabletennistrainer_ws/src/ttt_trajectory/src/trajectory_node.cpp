#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <deque>
#include <vector>
#include <cmath>
#include <numeric>
#include <optional>

// ── Prediction stage ──────────────────────────────────────────────────────────
// IDLE  : no active tracking
// STAGE1: 2+ samples on opponent's side  → rough side/X direction, robot faces the right side
// STAGE2: 4+ samples on opponent's side  → coarse landing estimate, robot moves to intercept zone
// STAGE3: ball has crossed the net (Z<0) → final precise prediction, robot refines to return spot
enum class PredStage : int { IDLE = 0, STAGE1 = 1, STAGE2 = 2, STAGE3 = 3 };

struct Sample {
    double x, y, z, t, t_abs;
};

class TrajectoryNode : public rclcpp::Node {
public:
    TrajectoryNode() : Node("trajectory_node") {
        // ── Parameters ────────────────────────────────────────────────────────
        this->declare_parameter("lookahead_ms",      200);
        this->declare_parameter("max_samples",        15);
        this->declare_parameter("gravity",           9.81);
        this->declare_parameter("table_y",            0.0);
        this->declare_parameter("restitution",        0.85);
        this->declare_parameter("min_incoming_speed", 0.5);
        this->declare_parameter("net_margin_z",      -0.2);
        this->declare_parameter("max_track_z",        1.15);
        this->declare_parameter("max_velocity",       25.0);
        // Per-stage minimum samples before publishing
        this->declare_parameter("stage1_min_samples",  2);
        this->declare_parameter("stage2_min_samples",  4);
        this->declare_parameter("stage3_min_samples",  2);  // after crossing net

        lookahead_ms_  = this->get_parameter("lookahead_ms").as_int();
        max_samples_   = this->get_parameter("max_samples").as_int();
        gravity_y_     = this->get_parameter("gravity").as_double();
        table_y_       = this->get_parameter("table_y").as_double();
        restitution_   = this->get_parameter("restitution").as_double();
        min_speed_     = this->get_parameter("min_incoming_speed").as_double();
        net_margin_    = this->get_parameter("net_margin_z").as_double();
        max_track_z_   = this->get_parameter("max_track_z").as_double();
        max_velocity_  = this->get_parameter("max_velocity").as_double();
        s1_min_        = this->get_parameter("stage1_min_samples").as_int();
        s2_min_        = this->get_parameter("stage2_min_samples").as_int();
        s3_min_        = this->get_parameter("stage3_min_samples").as_int();

        RCLCPP_INFO(this->get_logger(),
            "Trajectory node | lookahead=%dms | stages min_samples=[%d,%d,%d] | "
            "min_speed=%.1fm/s net_margin=%.2fm",
            lookahead_ms_, s1_min_, s2_min_, s3_min_, min_speed_, net_margin_);

        position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/ball_position_3d", 10,
            std::bind(&TrajectoryNode::cb, this, std::placeholders::_1));

        predicted_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/ball_trajectory/predicted", 10);
        landing_pub_   = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/ball_trajectory/landing",   10);
        phase_pub_     = this->create_publisher<std_msgs::msg::Int32>(
            "/ball_trajectory/phase",     10);
    }

private:
    // ── Helpers ───────────────────────────────────────────────────────────────
    void fitLinear(const std::vector<double>& t, const std::vector<double>& p,
                   double& p0, double& v) {
        int n = t.size();
        double st=0, sp=0, stt=0, stp=0;
        for (int i=0;i<n;i++){st+=t[i];sp+=p[i];stt+=t[i]*t[i];stp+=t[i]*p[i];}
        double d = n*stt - st*st;
        if (std::abs(d)<1e-9){p0=sp/n;v=0;return;}
        v  = (n*stp - st*sp)/d;
        p0 = (sp - v*st)/n;
    }

    void fitParabolic(const std::vector<double>& t, const std::vector<double>& p,
                      double gcomp, double& p0, double& v) {
        std::vector<double> c(p.size());
        for (size_t i=0;i<p.size();i++) c[i]=p[i]+0.5*gcomp*t[i]*t[i];
        fitLinear(t, c, p0, v);
    }

    struct Landing { double x, z, t_land; };

    std::optional<Landing> findLanding(double x0,double vx,double y0,double vy,
                                       double z0,double vz,double t_now) {
        double a=0.5*gravity_y_, b=-vy, c=table_y_-y0;
        double disc=b*b-4*a*c;
        if(disc<0) return std::nullopt;
        double sq=std::sqrt(disc);
        double t1=(-b-sq)/(2*a), t2=(-b+sq)/(2*a);
        double tl=-1;
        if(t1>t_now) tl=t1;
        if(t2>t_now&&(tl<0||t2<tl)) tl=t2;
        if(tl<0) return std::nullopt;
        return Landing{x0+vx*tl, z0+vz*tl, tl};
    }

    struct PredPt { double x,y,z; bool bounced; };

    PredPt predictWithBounce(double x0,double vx,double y0,double vy,
                             double z0,double vz,double t_now,double t_pred) {
        double a=0.5*gravity_y_, b=-vy, c=table_y_-y0;
        double disc=b*b-4*a*c;
        if(disc>=0){
            double sq=std::sqrt(disc);
            double t1=(-b-sq)/(2*a), t2=(-b+sq)/(2*a);
            double tb=-1;
            if(t1>t_now&&t1<t_pred) tb=t1;
            if(t2>t_now&&t2<t_pred&&(tb<0||t2<tb)) tb=t2;
            if(tb>0){
                double bvy=-(vy-gravity_y_*tb)*restitution_;
                double dt=t_pred-tb;
                return {x0+vx*t_pred,
                        table_y_+bvy*dt-0.5*gravity_y_*dt*dt,
                        z0+vz*t_pred, true};
            }
        }
        return {x0+vx*t_pred, y0+vy*t_pred-0.5*gravity_y_*t_pred*t_pred,
                z0+vz*t_pred, false};
    }

    void resetBuffer(const char* reason) {
        RCLCPP_INFO(this->get_logger(), "Buffer reset: %s", reason);
        buffer_.clear();
        stage_ = PredStage::IDLE;
        originated_across_net_ = false;
        ball_crossed_net_ = false;
        despike_strikes_ = 0;
        has_pred_ = false; has_land_ = false;
        pred_locked_ = false; land_locked_ = false;
        publishPhase();
    }

    void publishPhase() {
        std_msgs::msg::Int32 m;
        m.data = static_cast<int>(stage_);
        phase_pub_->publish(m);
    }

    // ── Per-stage smoothing alpha ─────────────────────────────────────────────
    // Stage 1: snap fast to first estimate (robot needs to start moving early)
    // Stage 2: track well but don't jitter
    // Stage 3: mostly stable (ball almost at bounce point, don't overshoot)
    double predAlpha()  { return stage_==PredStage::STAGE1 ? 0.85
                               : stage_==PredStage::STAGE2 ? 0.55 : 0.30; }
    double landAlpha()  { return stage_==PredStage::STAGE1 ? 0.75
                               : stage_==PredStage::STAGE2 ? 0.40 : 0.20; }

    // ── Main callback ─────────────────────────────────────────────────────────
    void cb(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        double t_abs = rclcpp::Time(msg->header.stamp).seconds();
        double bx=msg->point.x, by=msg->point.y, bz=msg->point.z;

        // Ignore points physically beyond table end
        if (bz > max_track_z_) return;

        // Gap > 200 ms ⇒ new rally
        if (!buffer_.empty() && (t_abs - buffer_.back().t_abs > 0.2))
            resetBuffer("tracking gap >200ms");

        // Ball touched table surface ⇒ bounce, new arc
        if (by < table_y_ + 0.05) {
            resetBuffer("bounce detected");
            return;
        }

        // Track whether ball originated on opponent's side
        if (bz > std::abs(net_margin_)) originated_across_net_ = true;

        // Track net crossing (opponent's side → MARTY's side)
        if (!ball_crossed_net_ && originated_across_net_ && bz < 0.0) {
            ball_crossed_net_ = true;
            // Snap EMA so Stage 3 locks onto first crossing prediction immediately
            has_pred_ = false; has_land_ = false;
            pred_locked_ = false; land_locked_ = false;
            RCLCPP_INFO(this->get_logger(), "Ball crossed net → STAGE 3");
        }

        // Despike
        if (!buffer_.empty()) {
            double dt = t_abs - buffer_.back().t_abs;
            if (dt > 0.0) {
                double dist = std::hypot(std::hypot(bx-buffer_.back().x, by-buffer_.back().y),
                                         bz-buffer_.back().z);
                if ((dist/dt) > max_velocity_) {
                    if (++despike_strikes_ > 3)
                        resetBuffer("teleport detected");
                    return;
                }
                despike_strikes_ = 0;
            }
        }

        if (buffer_.empty()) t0_ = t_abs;
        buffer_.push_back({bx, by, bz, t_abs-t0_, t_abs});
        if ((int)buffer_.size() > max_samples_) buffer_.pop_front();

        // ── Build fit vectors ─────────────────────────────────────────────────
        std::vector<double> ts, xs, ys, zs;
        double t_off = buffer_.front().t;
        for (const auto& s : buffer_) {
            ts.push_back(s.t - t_off);
            xs.push_back(s.x); ys.push_back(s.y); zs.push_back(s.z);
        }

        // ── Fit Z, check velocity direction ───────────────────────────────────
        double vz, z0; fitParabolic(ts, zs, 0.0, z0, vz);
        if (vz > -min_speed_) return;                // not coming toward MARTY
        if (std::abs(vz) > max_velocity_) return;    // noise

        double z_drop = buffer_.front().z - buffer_.back().z;
        double dt_span = buffer_.back().t - buffer_.front().t;
        if (z_drop < 0.015) return;
        if (dt_span > 0.0 && (z_drop/dt_span) < min_speed_) return;
        if (!originated_across_net_) return;

        // ── Determine stage ───────────────────────────────────────────────────
        int n = (int)buffer_.size();
        PredStage new_stage;
        if (ball_crossed_net_ && n >= s3_min_)
            new_stage = PredStage::STAGE3;
        else if (!ball_crossed_net_ && n >= s2_min_)
            new_stage = PredStage::STAGE2;
        else if (!ball_crossed_net_ && n >= s1_min_)
            new_stage = PredStage::STAGE1;
        else
            return; // not enough samples yet

        if (new_stage != stage_) {
            stage_ = new_stage;
            // Snap EMA to first value at each stage transition
            has_pred_ = false; has_land_ = false;
            pred_locked_ = false; land_locked_ = false;
            RCLCPP_INFO(this->get_logger(), "→ STAGE %d (n=%d, Vz=%.2f m/s)",
                        static_cast<int>(stage_), n, vz);
            publishPhase();
        }

        // ── Fit X, Y ─────────────────────────────────────────────────────────
        double vx, x0; fitLinear(ts, xs, x0, vx);
        double vy, y0; fitParabolic(ts, ys, gravity_y_, y0, vy);
        if (std::abs(vx) > max_velocity_ || std::abs(vy) > max_velocity_) return;

        double t_now = buffer_.back().t - t_off;
        auto land = findLanding(x0,vx,y0,vy,z0,vz,t_now);
        if (!land) return;

        // Must land on MARTY's side
        if (land->z >= 0.0) return;
        // Sanity bounds
        if (std::abs(land->x) > 1.5 || land->z < -1.5) return;

        // ── Landing EMA ───────────────────────────────────────────────────────
        double la = landAlpha();
        if (!has_land_) {
            sl_x_=land->x; sl_z_=land->z; has_land_=true;
            if (stage_ == PredStage::STAGE3) land_locked_ = true;
        } else if (!land_locked_) {
            sl_x_ = la*land->x + (1-la)*sl_x_;
            sl_z_ = la*land->z + (1-la)*sl_z_;
        }

        auto lmsg = geometry_msgs::msg::PointStamped();
        lmsg.header = msg->header;
        lmsg.header.frame_id = "table";
        lmsg.point.x=sl_x_; lmsg.point.y=table_y_; lmsg.point.z=sl_z_;
        landing_pub_->publish(lmsg);

        // ── Post-bounce intercept EMA ─────────────────────────────────────────
        double t_int = land->t_land + lookahead_ms_/1000.0;
        auto [px,py,pz,bounced] = predictWithBounce(x0,vx,y0,vy,z0,vz,t_now,t_int);

        double pa = predAlpha();
        if (!has_pred_) {
            sp_x_=px; sp_y_=py; sp_z_=pz; has_pred_=true;
            if (stage_ == PredStage::STAGE3) {
                pred_locked_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "🔒 STAGE3 prediction LOCKED → X:%+.3f Z:%.3f", sp_x_, sp_z_);
            }
        } else if (!pred_locked_) {
            sp_x_ = pa*px + (1-pa)*sp_x_;
            sp_y_ = pa*py + (1-pa)*sp_y_;
            sp_z_ = pa*pz + (1-pa)*sp_z_;
        }

        auto pmsg = geometry_msgs::msg::PointStamped();
        pmsg.header = msg->header;
        pmsg.header.frame_id = "table";
        pmsg.point.x=sp_x_; pmsg.point.y=sp_y_; pmsg.point.z=sp_z_;
        predicted_pub_->publish(pmsg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 80,
            "[S%d] Vz:%.2f | Land X:%+.3f Z:%.3f | Int X:%+.3f Z:%.3f in %.0fms",
            static_cast<int>(stage_), vz,
            sl_x_, sl_z_, sp_x_, sp_z_, land->t_land*1000.0);
    }

    // ── State ─────────────────────────────────────────────────────────────────
    int lookahead_ms_, max_samples_, s1_min_, s2_min_, s3_min_;
    double gravity_y_, table_y_, restitution_;
    double min_speed_, net_margin_, max_track_z_, max_velocity_;
    double t0_ = 0.0;
    int despike_strikes_ = 0;

    PredStage stage_ = PredStage::IDLE;
    bool originated_across_net_ = false;
    bool ball_crossed_net_      = false;

    bool has_pred_ = false, has_land_ = false;
    bool pred_locked_ = false, land_locked_ = false;
    double sp_x_=0, sp_y_=0, sp_z_=0;
    double sl_x_=0, sl_z_=0;

    std::deque<Sample> buffer_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr predicted_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr landing_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr phase_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}
