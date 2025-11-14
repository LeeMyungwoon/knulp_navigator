// trajectory_utils.cpp

#include "local_planner/types.hpp"
#include "local_planner/math_utils.hpp"
#include "local_planner/trajectory_utils.hpp"

namespace local_planner {
namespace {

struct InterpOut {
    double yaw = 0.0;
    double kappa = 0.0;
    size_t i0 = 0;   // 아래쪽 인덱스 (디버깅용)
    double t = 0.0;  // 보간 계수
};

void computeYawAndArc(std::vector<PathSample> &path) {
    if (path.size() < 2) {
        if (!path.empty()) path[0].s = 0.0;
        return;
    }
    // s누적
    path[0].s = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        const double dx = path[i].x - path[i - 1].x;
        const double dy = path[i].y - path[i - 1].y;
        path[i].s = path[i - 1].s + std::hypot(dx, dy);
    }
    // yaw가 비어있으면 접선각으로 계산
    for (size_t i = 0; i < path.size() - 1; i++) {
        const double dx = path[i + 1].x - path[i].x;
        const double dy = path[i + 1].y - path[i].y;
        double yaw = std::atan2(dy, dx);
        if (i == 0 && std::abs(path[i].yaw) < 1e-12) path[i].yaw = yaw;
        if (std::abs(path[i + 1].yaw) < 1e-12) path[i + 1].yaw = yaw;
    }
    // unwrap (연속화)
    for (size_t i = 1; i < path.size(); i++) {
        double dyaw = path[i].yaw - path[i - 1].yaw;
        dyaw = local_planner::math::wrapToPi(dyaw);
        path[i].yaw = path[i - 1].yaw + dyaw;
    }
}

// 중앙차분 기반 곡률 근사
std::vector<double> computeCurvature(const std::vector<PathSample> &path) {
    const size_t n = path.size();
    std::vector<double> kappa(n, 0.0);
    if (n < 3) return kappa;
    // 중앙차분으로 x', y', x'', y'' 근사
    for (size_t i = 1; i + 1 < n; i++) {
        const double s0 = path[i - 1].s, s1 = path[i].s, s2 = path[i + 1].s;
        const double ds0 = std::max(1e-6, s1 - s0);
        const double ds1 = std::max(1e-6, s2 - s1);
        // 1차 미분 근사 (0->왼쪽구간, 1->오른쪽구간)
        const double dx0 = (path[i].x - path[i - 1].x) / ds0;
        const double dy0 = (path[i].y - path[i - 1].y) / ds0;
        const double dx1 = (path[i + 1].x - path[i].x) / ds1;
        const double dy1 = (path[i + 1].y - path[i].y) / ds1;
        // 1차 미분 평균
        const double dx = 0.5 * (dx0 + dx1);
        const double dy = 0.5 * (dy0 + dy1);
        // 2차 미분 근사 
        const double d2x = (dx1 - dx0) / std::max(1e-6, 0.5 * (ds0 + ds1));
        const double d2y = (dy1 - dy0) / std::max(1e-6, 0.5 * (ds0 + ds1));
        // 곡률공식
        const double num = std::abs(dx * d2y - dy * d2x);
        const double den = std::pow(std::max(1e-6, dx * dx + dy * dy), 1.5);
        kappa[i] = num / den;
        const double dyaw = local_planner::math::wrapToPi(path[i + 1].yaw - path[i - 1].yaw);
        if (dyaw < 0.0) kappa[i] = -kappa[i];
    }
    // 양끝 이웃값 복사
    kappa.front() = kappa[1];
    kappa.back() = kappa[n-2];

    return kappa;
}

static InterpOut interpolateYawKappaByS(const std::vector<PathSample> &path, const std::vector<double> &kappa, double s_query) {
    InterpOut out;
    if (path.empty()) 
        return out;
    if (s_query <= path.front().s) {
        out.yaw   = path.front().yaw;
        out.kappa = kappa.empty() ? 0.0 : kappa.front();
        out.i0 = 0; out.t = 0.0;

        return out;
    }
    if (s_query >= path.back().s) {
        out.yaw   = path.back().yaw;
        out.kappa = kappa.empty() ? 0.0 : kappa.back();
        out.i0 = path.size()-1; out.t = 0.0;

        return out;
    }
    // 하한 인덱스 탐색 -> 이진탐색
    size_t i = 0;
    size_t l = 1;
    size_t r = path.size() - 1;
    while (l <= r) {
        size_t mid = (l + r) / 2;
        if (path[mid].s <= s_query) {
            i = mid;
            l = mid + 1;
        }
        else r = mid - 1;
    }
    const size_t i0 = i;
    const size_t i1 = i0 + 1;
    
    const double s0 = path[i0].s;
    const double s1 = path[i1].s;
    const double t  = local_planner::math::clamp((s_query - s0) / std::max(1e-6, (s1 - s0)), 0.0, 1.0);

    // yaw는 unwrap 전제 하에 선형보간
    double yaw0 = path[i0].yaw;
    double yaw1 = path[i1].yaw;
    double dyaw = local_planner::math::wrapToPi(yaw1 - yaw0);
    double yaw  = local_planner::math::wrapToPi(yaw0 + t * dyaw);

    double k0 = kappa.empty() ? 0.0 : kappa[i0];
    double k1 = kappa.empty() ? 0.0 : kappa[i1];
    double k  = local_planner::math::lerp(k0, k1, t);

    out.yaw = yaw; out.kappa = k; out.i0 = i0; out.t = t;

    return out;
}

}   // namespace anonymous

namespace traj {

ReferenceSequence buildReferenceSequenceTimeParam(std::vector<PathSample> path, const MpcHorizon &hor, double s0, const std::vector<double> *vx_profile, double vx_default) {
    ReferenceSequence ref;
    ref.s_ref.resize(hor.N, 0.0);
    ref.yaw_ref.resize(hor.N, 0.0);
    ref.vx_ref.resize(hor.N, 0.0);
    ref.kappa_ref.resize(hor.N, 0.0);

    if (path.size() < 2) return ref;

    // s/yaw 정리
    computeYawAndArc(path);
    const std::vector<double> kappa = computeCurvature(path);
    const double s_max = path.back().s;

    // 시간기반으로 s를 누적: s_{k+1} = s_k + vx_k * dt
    double s = local_planner::math::clamp(s0, 0.0, s_max);
    for (size_t k = 0; k < hor.N; ++k) {
        // 속도 참조
        double vx = vx_default;
        if (vx_profile && k < vx_profile->size())
            vx = std::max(0.0, (*vx_profile)[k]);

        // 보간으로 yaw, kappa 획득
        auto itp = interpolateYawKappaByS(path, kappa, s);

        ref.s_ref[k]     = s;
        ref.yaw_ref[k]   = itp.yaw;
        ref.vx_ref[k]    = vx;
        ref.kappa_ref[k] = itp.kappa;

        // 다음 스텝 s 갱신
        s = local_planner::math::clamp(s + vx * hor.dt, 0.0, s_max);
    }
    return ref;
}

// 속도 프로파일 없이 고정속도만 주는 오버로드 함수
ReferenceSequence buildReferenceSequenceTimeParam(std::vector<PathSample> path, const MpcHorizon &hor, double s0, double vx_default) {
    return buildReferenceSequenceTimeParam(path, hor, s0, nullptr, vx_default);
}

}   // namespac traj
}   // namespac local_planner
