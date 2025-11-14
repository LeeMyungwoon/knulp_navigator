#include <memory>
#include <mutex>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdint>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

#include "local_planner/types.hpp"
#include "local_planner/trajectory_utils.hpp"
#include "local_planner/local_mpc.hpp"
#include "local_planner/costmap_handler.hpp"

using local_planner::LinearModelSequence;
using local_planner::MPCResult;
using local_planner::PathSample;
using local_planner::MpcHorizon;
using local_planner::VehicleGeom;
using local_planner::ReferenceSequence;
using local_planner::State;
using local_planner::Control;
using local_planner::MpcProblem;
using local_planner::SolverOptions;
using local_planner::ROI;
using local_planner::GlobalSnapshotCostmap;
using local_planner::CostmapParams;
using local_planner::LateralCorridorSequence;

// ---------------------------------------------------------
// Local Planner 노드
// ---------------------------------------------------------
class LocalPlannerNode : public rclcpp::Node {
public:
    LocalPlannerNode() : Node("local_planner") {
        using std::placeholders::_1;

        // MPC 기본 파라미터 설정
        hor_.N  = 20;
        hor_.dt = 0.1;

        prob_.hor        = hor_;
        prob_.model.mode = local_planner::ModelMode::Kinematic;
        prob_.model.geom = geom_;

        // 코스트맵 튜닝 파라미터
        costmap_params_.scan_max         = 4.0;
        costmap_params_.scan_step        = 0.05;
        costmap_params_.inflation_y      = 0.20;
        costmap_params_.occ_thres        = 50;
        costmap_params_.unknown_as_occ   = true;
        costmap_params_.out_of_map_block = true;

        // 구독자/퍼블리셔 설정
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/global_path", 1,
            std::bind(&LocalPlannerNode::pathCallback, this, _1));

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 1,
            std::bind(&LocalPlannerNode::mapCallback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vins_estimator/odometry", 20,
            std::bind(&LocalPlannerNode::odomCallback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        ref_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/local_ref_path", 10);
        pred_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/mpc_pred_path", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),   // 20 Hz
            std::bind(&LocalPlannerNode::onTimer, this));

        RCLCPP_INFO(this->get_logger(), "local_planner_node initialized.");
    }

private:
    // --------------------
    // 콜백들
    // --------------------
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        global_path_samples_.clear();

        const auto &poses = msg->poses;
        if (poses.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Received global path with <2 poses.");
            have_path_ = false;
            return;
        }

        double accum_s = 0.0;
        global_path_samples_.reserve(poses.size());

        for (std::size_t i = 0; i < poses.size(); ++i) {
            const auto &p = poses[i].pose;

            PathSample ps;
            ps.x   = p.position.x;
            ps.y   = p.position.y;

            const auto &q = p.orientation;
            tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
            ps.yaw = tf2::getYaw(tf_q);

            if (i > 0) {
                double dx = ps.x - global_path_samples_.back().x;
                double dy = ps.y - global_path_samples_.back().y;
                accum_s += std::hypot(dx, dy);
            }
            ps.s = accum_s;
            global_path_samples_.push_back(ps);
        }

        last_s_on_path_ = 0.0;
        have_path_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Updated global path: %zu points, length ~ %.2f m",
                    global_path_samples_.size(), accum_s);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        const auto &info = msg->info;
        const int width  = static_cast<int>(info.width);
        const int height = static_cast<int>(info.height);
        const double res = info.resolution;
        const double ox  = info.origin.position.x;
        const double oy  = info.origin.position.y;

        // GlobalSnapshotCostmap 스냅샷으로 저장
        global_costmap_.width      = width;
        global_costmap_.height     = height;
        global_costmap_.resolution = res;
        global_costmap_.origin_x   = ox;
        global_costmap_.origin_y   = oy;
        global_costmap_.data       = msg->data;  // int8_t 복사

        have_map_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;

        const auto &q = msg->pose.pose.orientation;
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        odom_yaw_ = tf2::getYaw(tf_q);

        odom_vx_ = msg->twist.twist.linear.x;
        odom_wz_ = msg->twist.twist.angular.z;

        have_odom_ = true;
    }

    // --------------------
    // 타이머 콜백 (MPC 한 주기 실행)
    // --------------------
    void onTimer()
    {
        std::vector<PathSample> path;
        double x, y, yaw, vx, wz;
        bool have_path, have_map, have_odom;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            path      = global_path_samples_;
            x         = odom_x_;
            y         = odom_y_;
            yaw       = odom_yaw_;
            vx        = odom_vx_;
            wz        = odom_wz_;
            have_path = have_path_;
            have_map  = have_map_;
            have_odom = have_odom_;
        }

        if (!have_path || !have_map || !have_odom) {
            return;
        }
        if (path.size() < 2) {
            return;
        }

        MpcProblem prob_local = prob_;  // 템플릿에서 복사 후 수정
        std::vector<double> x_ref, y_ref;
        State x_curr{};

        if (!buildProblemFromCurrentState(path, x, y, yaw, vx, wz,
                                          prob_local, x_ref, y_ref, x_curr)) {
            return;
        }

        // MPC 풀이
        MPCResult res = local_planner::solve_local_mpc(
            prob_local, x_curr, &last_u_seq_, solver_opt_);

        if (!res.success) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "MPC solver failed.");
            return;
        }

        // warm start 업데이트
        last_u_seq_ = res.u_pred;

        // 제어 명령 출력
        if (!res.u_pred.empty()) {
            const Control &u0 = res.u_pred.front();
            publishCmd(u0, vx);
        }

        // 디버그용 path publish
        publishDebugPaths(prob_local.ref, res, x_ref, y_ref);
    }

    // ---------------------------------------------------
    // MPC 문제 생성: Reference + Corridor + LTV 모델 구성
    // ---------------------------------------------------
    bool buildProblemFromCurrentState(const std::vector<PathSample> &path,
                                      double rx, double ry, double ryaw,
                                      double rvx, double rwz,
                                      MpcProblem &prob_out,
                                      std::vector<double> &x_ref,
                                      std::vector<double> &y_ref,
                                      State &x_curr)
    {
        const std::size_t N = hor_.N;
        if (path.size() < 2 || N == 0) {
            return false;
        }

        // 1) 현재 위치에서 가장 가까운 경로 인덱스 찾기
        std::size_t nearest_idx = 0;
        double best_d2 = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < path.size(); ++i) {
            double dx = rx - path[i].x;
            double dy = ry - path[i].y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                nearest_idx = i;
            }
        }

        double s0 = path[nearest_idx].s;
        // 진행방향으로만 가도록, 이전보다 뒤로 가지 않게
        if (s0 < last_s_on_path_) {
            s0 = last_s_on_path_;
        }
        last_s_on_path_ = s0;

        // 2) ReferenceSequence 생성 (yaw, curvature, 속도 레퍼런스)
        double vx_ref_default = std::max(0.5, std::abs(rvx));
        ReferenceSequence ref =
            local_planner::traj::buildReferenceSequenceTimeParam(path, hor_, s0, vx_ref_default);

        if (ref.s_ref.size() < N) {
            RCLCPP_WARN(this->get_logger(),
                        "ReferenceSequence shorter than horizon.");
            return false;
        }

        // 3) ref.s_ref 에 맞춰 x_ref / y_ref 보간
        x_ref.resize(N);
        y_ref.resize(N);
        for (std::size_t k = 0; k < N; ++k) {
            double sk = ref.s_ref[k];

            // 경로 상에서 sk에 가장 가까운 segment 찾기
            std::size_t i = 0;
            while (i + 1 < path.size() && path[i + 1].s < sk) {
                ++i;
            }
            const PathSample &p0 = path[i];
            const PathSample &p1 = (i + 1 < path.size()) ? path[i + 1] : path.back();

            double s0_seg = p0.s;
            double s1_seg = p1.s;
            double denom  = s1_seg - s0_seg;
            double t = 0.0;
            if (denom > 1e-6) {
                t = (sk - s0_seg) / denom;
                if (t < 0.0) t = 0.0;
                if (t > 1.0) t = 1.0;
            }

            x_ref[k] = p0.x + (p1.x - p0.x) * t;
            y_ref[k] = p0.y + (p1.y - p0.y) * t;
        }

        // 4) ROI 설정 (로봇 주변 일부 영역만 corridor 계산)
        ROI roi;
        roi.cx = rx;
        roi.cy = ry;
        roi.width  = 20.0;
        roi.height = 20.0;

        // 5) 코리도 생성 (occupancy grid 스냅샷 기반)
        LateralCorridorSequence corridor =
            local_planner::buildCorridorFromCostmap(
                global_costmap_, roi, ref, x_ref, y_ref, geom_, costmap_params_);

        // 6) LTV 모델 구성 (키네마틱 에러 모델 선형화)
        LinearModelSequence ltv = buildLtv(ref);

        // 7) MpcProblem 세팅
        prob_out.hor        = hor_;
        prob_out.model.geom = geom_;
        prob_out.ref        = ref;
        prob_out.corridor   = corridor;
        prob_out.ltv        = ltv;
        // friction_poly / safety / slack / weights / limits 는 기존값 사용

        // 8) 현재 상태를 에러 좌표계로 변환
        const double yaw_ref0 = ref.yaw_ref[0];
        const double x_c0 = x_ref[0];
        const double y_c0 = y_ref[0];

        const double nx = -std::sin(yaw_ref0);
        const double ny =  std::cos(yaw_ref0);

        const double dx0 = rx - x_c0;
        const double dy0 = ry - y_c0;

        const double ey   = dx0 * nx + dy0 * ny;
        const double epsi = radNormalize(ryaw - yaw_ref0);

        x_curr.ey   = ey;
        x_curr.epsi = epsi;
        x_curr.s    = 0.0;      // 현재 시점에서 s-error는 0
        x_curr.vx   = rvx;
        x_curr.wz   = rwz;

        return true;
    }

    // Continuous-time error 모델을 간단히 선형화 한 LTV 생성
    LinearModelSequence buildLtv(const ReferenceSequence &ref) const
    {
        const std::size_t N  = hor_.N;
        const std::size_t nx = 5; // [ey, epsi, s, vx, wz]
        const std::size_t nu = 2; // [ax, delta]

        LinearModelSequence seq;
        seq.nx = nx;
        seq.nu = nu;
        seq.models.resize(N);

        const double dt = hor_.dt;
        const double L  = geom_.wheelbase;

        for (std::size_t k = 0; k < N; ++k) {
            double v_ref = (k < ref.vx_ref.size())    ? ref.vx_ref[k]    : ref.vx_ref.back();
            double kappa = (k < ref.kappa_ref.size()) ? ref.kappa_ref[k] : ref.kappa_ref.back();

            if (v_ref < 0.1) v_ref = 0.1;

            Eigen::MatrixXd Ac = Eigen::MatrixXd::Zero(nx, nx);
            Eigen::MatrixXd Bc = Eigen::MatrixXd::Zero(nx, nu);
            Eigen::VectorXd cc = Eigen::VectorXd::Zero(nx);

            // ey_dot = v_ref * epsi
            Ac(0, 1) = v_ref;

            // epsi_dot = wz - kappa * vx
            Ac(1, 3) = -kappa;
            Ac(1, 4) =  1.0;

            // s_dot ≈ v_ref (오차항은 무시)
            cc(2) = v_ref;

            // vx_dot = ax
            Bc(3, 0) = 1.0;

            // wz_dot = (v_ref / L) * delta
            if (std::abs(L) > 1e-6) {
                Bc(4, 1) = v_ref / L;
            }

            // 간단한 전진 오일러 적분으로 discrete-time 근사
            Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(nx, nx) + dt * Ac;
            Eigen::MatrixXd Bd = dt * Bc;
            Eigen::VectorXd cd = dt * cc;

            seq.models[k].A = Ad;
            seq.models[k].B = Bd;
            seq.models[k].c = cd;
        }

        return seq;
    }

    // 제어 명령 publish (ax, delta -> v, yaw_rate)
    void publishCmd(const Control &u0, double vx_curr)
    {
        const double dt = hor_.dt;
        const double L  = geom_.wheelbase;

        double v_cmd = vx_curr + u0.ax * dt;
        if (v_cmd < 0.0) {
            v_cmd = 0.0;
        }

        double yaw_rate_cmd = 0.0;
        if (std::abs(L) > 1e-6) {
            yaw_rate_cmd = v_cmd * std::tan(u0.delta) / L;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = v_cmd;
        cmd.angular.z = yaw_rate_cmd;

        cmd_pub_->publish(cmd);
    }

    // 디버그용 레퍼런스/예측 경로 publish
    void publishDebugPaths(const ReferenceSequence &ref,
                           const MPCResult &res,
                           const std::vector<double> &x_ref,
                           const std::vector<double> &y_ref)
    {
        const std::size_t N = hor_.N;

        // Reference path (center line)
        nav_msgs::msg::Path ref_path;
        ref_path.header.frame_id = "map";
        ref_path.header.stamp    = this->now();

        for (std::size_t k = 0; k < N; ++k) {
            if (k >= x_ref.size() || k >= y_ref.size()) {
                break;
            }
            geometry_msgs::msg::PoseStamped p;
            p.header = ref_path.header;
            p.pose.position.x = x_ref[k];
            p.pose.position.y = y_ref[k];

            double yaw = (k < ref.yaw_ref.size()) ? ref.yaw_ref[k] : ref.yaw_ref.back();
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            p.pose.orientation.w = q.w();

            ref_path.poses.push_back(p);
        }
        ref_pub_->publish(ref_path);

        // Predicted MPC states
        if (res.x_pred.size() < N + 1 || ref.s_ref.size() < N) {
            return;
        }

        nav_msgs::msg::Path pred_path;
        pred_path.header = ref_path.header;

        for (std::size_t k = 0; k < N; ++k) {
            const State &xk = res.x_pred[k];
            double ey   = xk.ey;
            double yawc = (k < ref.yaw_ref.size()) ? ref.yaw_ref[k] : ref.yaw_ref.back();

            double nx = -std::sin(yawc);
            double ny =  std::cos(yawc);

            if (k >= x_ref.size() || k >= y_ref.size()) {
                break;
            }
            double xc = x_ref[k];
            double yc = y_ref[k];

            double xp = xc + nx * ey;
            double yp = yc + ny * ey;

            geometry_msgs::msg::PoseStamped p;
            p.header = pred_path.header;
            p.pose.position.x = xp;
            p.pose.position.y = yp;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yawc);
            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            p.pose.orientation.w = q.w();

            pred_path.poses.push_back(p);
        }
        pred_pub_->publish(pred_path);
    }

    static double radNormalize(double a)
    {
        return std::atan2(std::sin(a), std::cos(a));
    }

private:
    // MPC 관련 기본 파라미터
    MpcHorizon hor_;
    VehicleGeom geom_;
    MpcProblem prob_;
    SolverOptions solver_opt_;

    // Costmap 설정
    GlobalSnapshotCostmap global_costmap_;
    CostmapParams        costmap_params_;

    // 마지막 MPC 해 (warm start 용)
    std::vector<Control> last_u_seq_;

    // 전역 경로 (PathSample)
    std::vector<PathSample> global_path_samples_;
    double last_s_on_path_ = 0.0;

    // 최신 odom
    double odom_x_ = 0.0;
    double odom_y_ = 0.0;
    double odom_yaw_ = 0.0;
    double odom_vx_ = 0.0;
    double odom_wz_ = 0.0;

    // 상태 플래그
    bool have_path_ = false;
    bool have_map_ = false;
    bool have_odom_ = false;

    // ROS 인터페이스
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pred_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mutex_;
};

// ---------------------------------------------------------
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
