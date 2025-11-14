// mpc_utils.hpp
#pragma once

#include <cstddef>
#include <Eigen/Dense>

namespace local_planner {
namespace mpc {

// 상태/입력 인덱스 매핑
struct IndexMap {
    // State indices inside prob.ltv models
    int idx_ey   = 0; // lateral error
    int idx_epsi = 1; // heading error
    int idx_s    = 2; // along-path distance
    int idx_vx   = 3; // forward velocity
    int idx_wz   = 4; // yaw rate

    // Input indices (u = [ax, delta])
    int idx_ax    = 0; // acceleration
    int idx_delta = 1; // steering angle
};

// QP 컨테이너
struct BuiltQP {
    // Equality constraints: Aeq z = beq
    Eigen::MatrixXd Aeq;
    Eigen::VectorXd beq;

    // Inequality constraints: Aineq z <= bineq
    Eigen::MatrixXd Aineq;
    Eigen::VectorXd bineq;

    // Variable bounds: lb <= z <= ub
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;

    // bookkeeping
    std::size_t nvar = 0;
    std::size_t nx = 0, nu = 0, N = 0;
    std::size_t off_x = 0, off_u = 0, off_eps_obs = 0, off_eps_fric = 0;
};

// z = [x_0,...,x_N, u_0,...,u_{N-1}, ...] 에서 x_k 블록 시작 인덱스
inline std::size_t xOffset(std::size_t k, std::size_t nx) {
    return k * nx;
}

// 같은 z 에서 u_k 블록 시작 인덱스
inline std::size_t uOffset(std::size_t N, std::size_t nx, std::size_t nu, std::size_t k) {
    return (N + 1) * nx + k * nu;
}

}   // mpc local_planner
}   // namespace local_planner
