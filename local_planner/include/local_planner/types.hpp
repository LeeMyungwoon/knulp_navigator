// types.hpp
#pragma once

#include <vector>
#include <array>
#include <cstddef>
#include <Eigen/Dense>

namespace local_planner {

// 차량정보
struct VehicleGeom {
    double wheelbase = 2.7;         // [m] 축간거리 L
    double half_width = 0.4;        // [m] 반폭 (인플레이션 전에 차량 자체 반폭)
    double front_overhang = 0.9;    // [m] 전방 오버행 (선택)
    double rear_overhang  = 0.9;    // [m] 후방 오버행 (선택)
};

// 레퍼런스 (길, 방향, 속도, 곡률)
struct ReferenceSequence {
    std::vector<double> s_ref;      // size N
    std::vector<double> yaw_ref;    // size N
    std::vector<double> vx_ref;     // size N
    std::vector<double> kappa_ref;  // size N
};

// MPC 상태변수
struct State {
    double ey = 0.0;    // [m]
    double epsi = 0.0;  // [rad]
    double s = 0.0;     // [m]
    double vx = 0.0;    // [m/s]
    double wz = 0.0;    // [rad/s] (Dynamic 모델/선형화에 사용할 경우)
};

// MPC 입력변수
struct Control {
    double ax = 0.0;    // [m/s^2]
    double delta = 0.0; // [rad]
};

// 경로 샘플
struct PathSample {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;   // [rad]
    double s = 0.0;     // [m]
};

// MPC 지평
struct MpcHorizon {
    std::size_t N = 20;   // 예측 지평 길이
    double dt = 0.1;      // [s] 샘플 간격 N 스탭을 dt간격으로
};

// 코스트 가중치
struct Weights {
    // 상태 오차 벌점
    double q_ey = 4.0;
    double q_epsi = 2.0;
    double q_vx = 0.5;
    double q_s = 0.0;  

    // 입력/입력 변화 벌점
    double r_ax = 0.05;
    double r_delta = 0.05;
    double rd_ax = 0.02;    // 가속도 변화율
    double rd_delta = 0.02; // 각속도 변화율

    // 슬랙/안전도 벌점 (소프트 제약)
    double r_slack_obs = 1000.0;
    double r_slack_friction = 1000.0;
    double r_safety_gamma = 10.0;  // 안전도 스칼라 벌점
};

// ROI localmap
struct ROI {
    double cx = 0.0;     // 중심 x [m]
    double cy = 0.0;     // 중심 y [m]
    double width = 10.0; // 가로 길이 [m]
    double height = 10.0;// 세로 길이 [m]
};

// 모델 종류 스위치 -> 저속: Kinematic, 고속: Dynamic
enum class ModelMode { Kinematic, Dynamic };

// 모델선택
struct ModelParams {
    ModelMode mode = ModelMode::Kinematic;
    double kin_switch_speed = 3.0; // [m/s] 이 속도 미만이면 키네마틱 사용
    VehicleGeom geom{};
};

// Local cost map
struct LocalCostmap {
    int width = 0;
    int height = 0;
    double resolution = 0.1;            // [m/cell]
    double origin_x = 0.0;              // [m]
    double origin_y = 0.0;              // [m]
    std::vector<float> distance_field;  // 장애물까지 거리(확장 반영)
};

// Global Snapshot map (스냅샷)
struct GlobalSnapshotCostmap {
    int width = 0;
    int height = 0;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    std::vector<int8_t> data;   // -1, 0..100
};

// 코스트맵 관련 튜닝 파라미터
struct CostmapParams {
    double scan_max = 4.0;
    double scan_step = 0.05;
    double inflation_y = 0.20;
    bool out_of_map_block = true;
    int8_t occ_thres = 50;
    bool unknown_as_occ = true;
};

// 황방향 통로 제약 
struct LateralBounds {
    // e_y bounds for Rear / Middle / Front points
    double ey_min_R = -1e9, ey_max_R = 1e9;
    double ey_min_M = -1e9, ey_max_M = 1e9;
    double ey_min_F = -1e9, ey_max_F = 1e9;
};

// 횡방향 통로 제약 시퀀스
struct LateralCorridorSequence {
    std::vector<LateralBounds> bounds; // size N
};

// 선형모델
struct LinearModel {
    Eigen::MatrixXd A;   // (nx x nx)
    Eigen::MatrixXd B;   // (nx x nu)
    Eigen::VectorXd c;   // (nx)
};

// 선형모델 시퀀스
struct LinearModelSequence {
    std::vector<LinearModel> models; // size N
    std::size_t nx = 0;              // 상태 차원
    std::size_t nu = 0;              // 입력 차원
};

// Solver Options 
struct SolverOptions {
    int max_iter = 100;
    double tol = 1e-4;
    bool verbose  = false;
};

// 하드 제약조건
struct HardLimits {
    // 입력 자체
    double ax_min    = -4.0, ax_max    = 2.0;   // [m/s^2]
    double delta_min = -0.5, delta_max = 0.5;   // [rad]

    // 입력 변화율 (rate)
    double dax_min   = -2.5, dax_max   = 2.5;   // [m/s^3] (Δax / dt 관점이면 수치 조정)
    double ddelta_min= -0.3, ddelta_max= 0.3;   // [rad/s]  (Δdelta / dt)

    // 상태 제한(선택)
    double vx_min    = 0.0,  vx_max    = 8.0;   // [m/s]
    double wz_min    = -2.0, wz_max    = 2.0;   // [rad/s]
    double ey_abs_max= 5.0;                     // [m] 절대 횡오차 상한(긴급 시)
};

// 마찰타이어 제약
struct PolytopeSequence {
    std::vector<Eigen::MatrixXd> H;     // 각 k에서의 H_k (rows x nu)
    std::vector<Eigen::VectorXd> h;     // 각 k에서의 h_k (rows)
};

// 안전 변수
struct SafetyParams {
    bool use_safety_gamma = false; // true면 corridor를 γ로 타이팅/릴랙싱
    double gamma_min = 0.0, gamma_max = 1.0;
};

// 슬랙 변수
struct SlackPolicy {
    bool use_obstacle_slack  = true;  // ey-제약 소프트닝
    bool use_friction_slack  = false; // 마찰 제약 소프트닝
    double slack_max_obs     = 0.5;   // [m]
    double slack_max_friction= 0.2;   // [unit]
};

struct MpcProblem {
    MpcHorizon hor{};
    ModelParams model{};
    Weights weights{};
    HardLimits limits{};

    // 참조/경로/코리도
    ReferenceSequence ref{};
    LateralCorridorSequence corridor{};

    // 마찰/추가 제약(선택)
    PolytopeSequence friction_poly;   // 빈 시퀀스면 미사용
    SafetyParams safety{};
    SlackPolicy slack{};

    // 선형 모델 (A,B,c)
    LinearModelSequence ltv{};

    // warm start (선택)
    std::vector<Control> u_init;   // size N
    std::vector<State> x_init;   // size N+1
};

struct MPCResult {
    bool success = false;
    double optimal_cost = 0.0;

    // 즉시 적용할 입력
    Control u0{};

    // 예측 시퀀스 (시각화/디버깅용)
    std::vector<State> x_pred;  // size N+1
    std::vector<Control> u_pred;  // size N
};

/*******************************/
/*     이건 안써도 될것 같은데?     */
/*******************************/

// Circle 형태 장애물 (MPC 제약식용)
struct CircleObstacle {
    double cx = 0.0;   // 중심 x [m]
    double cy = 0.0;   // 중심 y [m]
    double r  = 0.3;   // 반경 [m]
};

}   // namespace local_planner
