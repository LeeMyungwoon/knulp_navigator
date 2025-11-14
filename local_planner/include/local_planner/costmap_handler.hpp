// costmap_handler.hpp
#pragma once

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <utility>
#include <stdexcept>
#include <functional>
#include <Eigen/Dense>

#include "local_planner/types.hpp"  

namespace local_planner {

local_planner::LateralCorridorSequence buildCorridorFromCostmap(
    const local_planner::GlobalSnapshotCostmap &map,
    const local_planner::ROI &roi,
    const local_planner::ReferenceSequence &ref,
    const std::vector<double> &x_ref,
    const std::vector<double> &y_ref,
    const local_planner::VehicleGeom &geom,
    const local_planner::CostmapParams &params); 

}   // namespace local_planner
