// costmap_handler.cpp

#include "local_planner/types.hpp"
#include "local_planner/math_utils.hpp"
#include "local_planner/costmap_handler.hpp"

namespace local_planner {
namespace {

struct GridView {
    int W = 0, H = 0;
    double res = 0.1;
    double origin_x = 0.0, origin_y = 0.0;
    const std::vector<int8_t> *data = nullptr;

    // 값 유효 체크
    bool valid() const {
        return (W > 0 && H > 0 && res > 0.0 && data != nullptr);
    };
    // 범위 체크
    bool inBounds(int gx, int gy) const {
        return (gx >= 0 && gy >= 0 && gx < W && gy < H);
    }
    // 인덱스 반환
    int idx(int gx, int gy) const {
        return gy * W + gx;
    }
    // world -> grid 변환
    bool worldToGrid(double wx, double wy, int& gx, int& gy) const {
        if (res <= 0.0) return false;
        const double rx = (wx - origin_x) / res;
        const double ry = (wy - origin_y) / res;
        gx = static_cast<int>(std::floor(rx));
        gy = static_cast<int>(std::floor(ry));
        return inBounds(gx, gy);
    }
    // grid -> world 변환
    void gridToWorld(int gx, int gy, double& wx, double& wy) const {
        wx = origin_x + (static_cast<double>(gx) + 0.5) * res;
        wy = origin_y + (static_cast<double>(gy) + 0.5) * res;
    }
    // 해당 값 반환
    int8_t at(int gx, int gy) const {
        if (!inBounds(gx, gy) || data == nullptr) return -1;
        return (*data)[idx(gx, gy)];
    }
};

struct RoiIndex { 
    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0; 
};

// ROI (world 단위) -> grid index 범위로 변환
RoiIndex computeRoiIndex(const GridView &g, const ROI &roi) {
    RoiIndex r;
    if (!g.valid()) return r;

    const double half_w = 0.5 * roi.width;
    const double half_h = 0.5 * roi.height;

    const double wx_min = roi.cx - half_w;
    const double wx_max = roi.cx + half_w;
    const double wy_min = roi.cy - half_h;
    const double wy_max = roi.cy + half_h;

    // 연속 좌표를 grid index로 투영
    int gx_min = static_cast<int>(std::floor((wx_min - g.origin_x) / g.res));
    int gx_max = static_cast<int>(std::ceil ((wx_max - g.origin_x) / g.res));
    int gy_min = static_cast<int>(std::floor((wy_min - g.origin_y) / g.res));
    int gy_max = static_cast<int>(std::ceil ((wy_max - g.origin_y) / g.res));

    // 클램핑
    gx_min = std::max(0, gx_min);
    gy_min = std::max(0, gy_min);
    gx_max = std::min(g.W, gx_max);
    gy_max = std::min(g.H, gy_max);

    if (gx_max <= gx_min || gy_max <= gy_min) {
        // ROI가 맵 바깥에 거의 전부 있는 경우 -> 비어있는 ROI
        r.x0 = r.y0 = 0;
        r.x1 = r.y1 = 0;
        return r;
    }

    r.x0 = gx_min;
    r.y0 = gy_min;
    r.x1 = gx_max;
    r.y1 = gy_max;

    return r;
}

// distance_field 계산
LocalCostmap buildLocalDistanceField(const GridView &g, const CostmapParams &P, const ROI &roi) {
    LocalCostmap local;
    if (!g.valid()) return local;

    const RoiIndex ri = computeRoiIndex(g, roi);
    const int W = ri.x1 - ri.x0;
    const int H = ri.y1 - ri.y0;

    if (W <= 0 || H <= 0) return local;

    local.width = W;
    local.height = H;
    local.resolution = g.res;
    local.origin_x = g.origin_x + static_cast<double>(ri.x0) * g.res;
    local.origin_y = g.origin_y + static_cast<double>(ri.y0) * g.res;
    local.distance_field.assign(static_cast<std::size_t>(W * H), std::numeric_limits<float>::infinity());

    auto isOcc = [&](int gx, int gy) -> bool {
        if (!g.inBounds(gx, gy)) {
            return P.out_of_map_block;  // 맵 바깥을 장애물로 볼지 여부
        }
        const int8_t v = g.at(gx, gy);
        if (v < 0) {
            return P.unknown_as_occ;    // unknown 처리 정책
        }
        return v >= P.occ_thres;
    };

    using Node = std::pair<float, int>; // (dist, idx)
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    // 초기화: ROI 내 점유셀을 distance=0 소스로 push
    for (int ly = 0; ly < H; ly++) {
        for (int lx = 0; lx < W; lx++) {
            const int gx = ri.x0 + lx;
            const int gy = ri.y0 + ly;
            if (isOcc(gx, gy)) {
                const int idx = ly * W + lx;
                local.distance_field[static_cast<std::size_t>(idx)] = 0.0f;
                pq.emplace(0.0f, idx);
            }
        }
    }

    // 이웃 (8-연결)
    constexpr int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    constexpr int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    const double sqr = std::sqrt(2.0);
    constexpr double step_cost[8] = {1, sqr, 1, sqr, 1, sqr, 1, sqr};

    const float step = static_cast<float>(g.res);

    // Dijkstra
    while (pq.size()) {
        const float here_cost = pq.top().first;
        const int u = pq.top().second;
        pq.pop();

        if (here_cost > local.distance_field[static_cast<std::size_t>(u)]) {
            continue; // 느긋한 삭제
        }

        const int lx = u % W;
        const int ly = u / W;

        for (int dir = 0; dir < 8; dir++) {
            const int nx = lx + dx[dir];
            const int ny = ly + dy[dir];
            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;

            const int v = ny * W + nx;
            const float weight = step * static_cast<float>(step_cost[dir]);
            const float new_cost = here_cost + weight;      

            if (new_cost < local.distance_field[static_cast<std::size_t>(v)]) {
                local.distance_field[static_cast<std::size_t>(v)] = new_cost;
                pq.emplace(new_cost, v);
            }
        }
    }

    // scan_max가 양수이면, 그 이상은 자른다
    if (P.scan_max > 0.0) {
        const float cap = static_cast<float>(P.scan_max);
        for (float& n : local.distance_field) {
            if (n > cap) n = cap;
        }
    }

    return local;
}

bool sampleDist(const LocalCostmap &local, double wx, double wy, float &dist_out) {
    if (local.width <= 0 || local.height <= 0 || local.resolution <= 0.0) return false;

    const double rx = (wx - local.origin_x) / local.resolution;
    const double ry = (wy - local.origin_y) / local.resolution;
    const int gx = static_cast<int>(std::floor(rx));
    const int gy = static_cast<int>(std::floor(ry));

    if (gx < 0 || gy < 0 || gx >= local.width || gy >= local.height) return false;

    const int idx = gy * local.width + gx;
    dist_out = local.distance_field[static_cast<std::size_t>(idx)];

    return true;
}

// 차량 중심좌표 반환 (앞, 중간, 뒤)
void footprintsAt(const VehicleGeom &geom, double xc, double yc, double yaw, Eigen::Vector2d &rear, Eigen::Vector2d &mid, Eigen::Vector2d &front) {
    const double L_full = geom.front_overhang + geom.wheelbase + geom.rear_overhang;    // 차량 전체 길이
    const double halfL  = 0.5 * L_full;
    const double hx = std::cos(yaw);    // 진행방향 단위벡터
    const double hy = std::sin(yaw);

    mid = Eigen::Vector2d(xc, yc);
    front = Eigen::Vector2d(xc + halfL * hx, yc + halfL * hy);
    rear = Eigen::Vector2d(xc - halfL * hx, yc - halfL * hy);
}

// ey_min(오른쪽 한계), ey_max(왼쪽 한계) 계산
void computeEYBoundsAtPoint(const LocalCostmap &local, const CostmapParams &P, double xc, double yc, double yaw, double &ey_min, double &ey_max) {
    // 좌표계: 전방 = heading, 좌측 = +ey 방향
    const double nx = -std::sin(yaw); // left normal x
    const double ny =  std::cos(yaw); // left normal y

    ey_min = 0.0;
    ey_max = 0.0;

    auto scan_dir = [&](double dir_sign) -> double {
        double last_ok = 0.0;
        bool any_ok = false;

        // 0에서 시작해서 scan_max까지 진행
        for (double s = 0.0; s <= P.scan_max; s += P.scan_step) {
            const double ey = dir_sign * s;
            const double wx = xc + ey * nx;
            const double wy = yc + ey * ny;

            float d = 0.0f;
            bool valid = sampleDist(local, wx, wy, d);
            if (!valid) {
                if (P.out_of_map_block) {
                    // 맵 밖을 벽으로 보면 더 진행 불가
                    break;
                } else {
                    // 맵 밖을 허용(장애물 없음)으로 보면 그냥 통과
                    last_ok = ey;
                    any_ok = true;
                    continue;
                }
            }
            // distance < inflation_y 이면 장애물에 너무 가까운 것
            if (d < static_cast<float>(P.inflation_y)) {
                break;
            }
            last_ok = ey;
            any_ok = true;
        }

        if (!any_ok) {
            // 한 번도 안전 영역을 찾지 못했다면 0으로 취급
            return 0.0;
        }
        return last_ok;
    };

    const double ey_left  = scan_dir(+1.0);  // 왼쪽(+)
    const double ey_right = scan_dir(-1.0);  // 오른쪽(-)

    ey_min = ey_right; // 음수 또는 0
    ey_max = ey_left;  // 양수 또는 0
}

// 황방향 제약 sequence 묶음 반환
LateralCorridorSequence buildCorridorFromDistance(const LocalCostmap &local, const ReferenceSequence &ref, const std::vector<double> &x_ref, const std::vector<double> &y_ref, const VehicleGeom &geom, const CostmapParams &P) {
    LateralCorridorSequence seq;

    const std::size_t N = ref.yaw_ref.size();
    if (x_ref.size() < N || y_ref.size() < N) {
        // ref 길이와 xy 길이가 맞지 않으면 빈 시퀀스
        return seq;
    }

    seq.bounds.resize(N);

    for (std::size_t k = 0; k < N; k++) {
        const double xc  = x_ref[k];
        const double yc  = y_ref[k];
        const double yaw = ref.yaw_ref[k];

        Eigen::Vector2d rear, mid, front;
        footprintsAt(geom, xc, yc, yaw, rear, mid, front);

        double ey_min_R = 0.0, ey_max_R = 0.0;
        double ey_min_M = 0.0, ey_max_M = 0.0;
        double ey_min_F = 0.0, ey_max_F = 0.0;

        computeEYBoundsAtPoint(local, P, rear.x(),  rear.y(),  yaw, ey_min_R, ey_max_R);
        computeEYBoundsAtPoint(local, P, mid.x(),   mid.y(),   yaw, ey_min_M, ey_max_M);
        computeEYBoundsAtPoint(local, P, front.x(), front.y(), yaw, ey_min_F, ey_max_F);

        auto& b = seq.bounds[k];
        b.ey_min_R = ey_min_R;
        b.ey_max_R = ey_max_R;
        b.ey_min_M = ey_min_M;
        b.ey_max_M = ey_max_M;
        b.ey_min_F = ey_min_F;
        b.ey_max_F = ey_max_F;
    }

    return seq;
}

}   // namespace anonymous

LateralCorridorSequence buildCorridorFromCostmap(const GlobalSnapshotCostmap &map, const ROI &roi, const ReferenceSequence &ref, const std::vector<double>& x_ref, const std::vector<double>& y_ref, const VehicleGeom &geom, const CostmapParams &params) {
    // 1) GlobalSnapshotCostmap -> GridView 래핑
    GridView g;
    g.W = map.width;
    g.H = map.height;
    g.res = map.resolution;
    g.origin_x = map.origin_x;
    g.origin_y = map.origin_y;
    g.data = &map.data;

    if (!g.valid()) return LateralCorridorSequence{};

    // 2) ROI 기반 distance field 생성
    LocalCostmap local = buildLocalDistanceField(g, params, roi);

    // 3) distance field에서 corridor bounds 추출
    return buildCorridorFromDistance(local, ref, x_ref, y_ref, geom, params);
}

}   // namespace local_planner
