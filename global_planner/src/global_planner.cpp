#include "global_planner.hpp"

bool GridMap::inBounds(int gx, int gy) const {
    return (gx >= 0 && gx < width && gy >= 0 && gy < height);
}

bool GridMap::worldToGrid(double xw, double yw, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((xw - origin_x) / resolution));
    gy = static_cast<int>(std::floor((yw - origin_y) / resolution));

    return inBounds(gx, gy);
}

void GridMap::gridToWorld(int gx, int gy, double& xw, double& yw) const {
    xw = origin_x + (gx + 0.5) * resolution;
    yw = origin_y + (gy + 0.5) * resolution;
}

int GridMap::idx(int gx, int gy) const { return gy * width + gx; }

bool GridMap::isFreeCell(int gx, int gy) const {
    if (!inBounds(gx, gy)) return false;
    int8_t v = data[idx(gx, gy)];
    if (v == 0) return true;    // free
    if (v == 100) return false; // occupied

    return unknown_is_free;
}

// Bresenham Algorithm
bool GridMap::isSegmentFree(int gx0, int gy0, int gx1, int gy1) const {
    int x = gx0, y = gy0;
    int dx = std::abs(gx1 - gx0), sx = (gx0 < gx1) ? 1 : -1;
    int dy = -std::abs(gy1 - gy0), sy = (gy0 < gy1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (!inBounds(x, y)) return false;
        if (!isFreeCell(x, y)) return false;

        if (x == gx1 && y == gy1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }

    return true;
}

HybridAStarRoutePlanner::PathXYZT HybridAStarRoutePlanner::search_route(const Pose& start_w, const Pose& goal_w) {
    // start/goal grid
    int sgx, sgy, ggx, ggy;
    if (!map_.worldToGrid(start_w.x, start_w.y, sgx, sgy)) return {{}, {}};
    if (!map_.worldToGrid(goal_w.x,  goal_w.y,  ggx, ggy)) return {{}, {}};

    goal_pose_ = goal_w;

    using QItem = std::pair<double, int>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;

    std::unordered_map<int, Node> best_node; 
    std::unordered_map<int, double> dist;   
    std::unordered_map<int, int> parent;  

    Node start_n(start_w, sgx, sgy, 0.0, 0.0, -1);
    int start_id = id(start_n.gx, start_n.gy);
    best_node[start_id] = start_n;
    dist[start_id] = 0.0;
    parent[start_id] = -1;

    pq.push({heuristic(start_n), start_id});

    const double GOAL_THRESH = 1.0; // [m]

    while (pq.size()) {
        double here_cost = pq.top().first; 
        int u = pq.top().second;
        pq.pop();

        // 느긋한 삭제(lazy deletion)
        double expected_f = dist[u] + heuristic(best_node[u]);
        if (std::fabs(expected_f - here_cost) > 1e-9) continue;

        Node cur = best_node[u];

        // 도착 체크
        if (distance_to_goal(cur.pose) <= GOAL_THRESH) {
            return reconstruct(parent, best_node, u);
        }

        for (double steer : steering_inputs_) {
            for (double chord : chord_lengths_) {
                Node nxt;
                if (!next_node(cur, u, chord, steer, nxt)) continue;

                int v = id(nxt.gx, nxt.gy);
                double weight = chord;                 // 이동 거리
                double new_cost = dist[u] + weight;

                if (dist.find(v) == dist.end() || new_cost < dist[v]) {
                    dist[v] = new_cost;
                    nxt.g_cost = new_cost;
                    nxt.parent = u;
                    best_node[v] = nxt;
                    parent[v] = u;
                    pq.push({new_cost + heuristic(nxt), v});
                }
            }
        }
    }

    return {{}, {}};
}

int HybridAStarRoutePlanner::id(int gx, int gy) const { 
    return gy * map_.width + gx; 
}

bool HybridAStarRoutePlanner::next_node(const Node &cur, int cur_id, double chord, double steer, Node &nxt) const {
    // bicycle model
    double theta = rad_normalize(cur.pose.theta + chord * std::tan(steer) / wheelbase_);
    double x = cur.pose.x + chord * std::cos(theta);
    double y = cur.pose.y + chord * std::sin(theta);

    int ngx, ngy;
    if (!map_.worldToGrid(x, y, ngx, ngy)) return false; // 바운더리 밖
    if (!map_.isSegmentFree(cur.gx, cur.gy, ngx, ngy)) return false; // 충돌

    nxt = Node(Pose{x, y, theta}, ngx, ngy, cur.g_cost + chord, steer, -1);

    return true;
}

// 휴리스틱함수
double HybridAStarRoutePlanner::heuristic(const Node& n) const {
    double dist  = distance_to_goal(n.pose);
    double ang   = std::fabs(rad_normalize(n.pose.theta - goal_pose_.theta)) * 0.1;
    double steer = std::fabs(n.steering) * 10.0;

    return dist + ang + steer;
}

// 유클리드거리
double HybridAStarRoutePlanner::distance_to_goal(const Pose& p) const {
    double dx = p.x - goal_pose_.x;
    double dy = p.y - goal_pose_.y;
    
    return std::hypot(dx, dy);
    // return std::sqrt(dx*dx + dy*dy);
}

HybridAStarRoutePlanner::PathXYZT HybridAStarRoutePlanner::reconstruct(const std::unordered_map<int,int> &parent, const std::unordered_map<int,Node> &best_node, int goal_id) const {
    PathXYZT path;
    int cur = goal_id;
    while (cur != -1) {
        const auto& nd = best_node.at(cur);
        path.x.push_back(nd.pose.x);
        path.y.push_back(nd.pose.y);
        path.theta.push_back(nd.pose.theta);
        auto it = parent.find(cur);
        cur = (it == parent.end()) ? -1 : it->second;
    }
    std::reverse(path.x.begin(), path.x.end());
    std::reverse(path.y.begin(), path.y.end());
    std::reverse(path.theta.begin(), path.theta.end());

    return path;
}
