// constraint_builder.hpp
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <limits>
#include <cassert>

#include "local_planner/types.hpp"
#include "local_planner/mpc_utils.hpp"

namespace local_planner {
namespace mpc{

BuiltQP buildConstraints(
    const MpcProblem &prob, 
    const Eigen::VectorXd &x0, 
    const IndexMap &idx = IndexMap{}, 
    bool use_small_angle = true, 
    const Eigen::Vector2d *u_prev_opt = nullptr);

}   // namespace mpc
}   // namespace local_planner
