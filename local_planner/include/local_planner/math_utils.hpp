#pragma once

#include <algorithm>
#include <cmath>

namespace local_planner {
namespace math {

// 범위 제한
inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
}

// 각도 wrap
inline double wrapToPi(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

// 선형보간
inline double lerp(double a, double b, double t) {
    return a + t * (b - a);
}


}   // namespace math
}   // namespace local_planner
