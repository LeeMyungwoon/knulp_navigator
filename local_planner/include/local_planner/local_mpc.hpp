// local_mpc.hpp
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <limits>
#include <cmath>
#include <cassert>
#include <iostream>
#include <algorithm>

#include "local_planner/types.hpp"
#include "local_planner/mpc_utils.hpp"

namespace local_planner {
namespace mpc {

MPCResult run_local_mpc(const MpcProblem &prob,
                        const State &x_curr,
                        const Eigen::Vector2d *u_prev_opt,  // 직전 최적 입력 (rate 비용/제약에 쓰면 부드러움)
                        const SolverOptions &opt);
                  
}   // namespace mpc

MPCResult solve_local_mpc(const MpcProblem &prob,
                          const State &x_curr,
                          const std::vector<Control> *warm_u,  // 옵션(미사용 가능)
                          const SolverOptions &opt);

}   // namespace local_planner
