// trajectory_utils.hpp
#pragma once

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <stdexcept>

#include "local_planner/types.hpp"

namespace local_planner {
namespace traj {

ReferenceSequence buildReferenceSequenceTimeParam(
    std::vector<PathSample> path,    
    const MpcHorizon &hor,
    double s0,
    const std::vector<double> *vx_profile, 
    double vx_default);

ReferenceSequence buildReferenceSequenceTimeParam(
    std::vector<PathSample> path, 
    const MpcHorizon &hor, 
    double s0, 
    double vx_default);

}   // namespace traj
}   // namespace local_planner
