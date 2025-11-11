#pragma once
#include <functional>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <algorithm>
#include <limits>
#include <string>

inline double rad_normalize(double a) {
    return std::atan2(std::sin(a), std::cos(a)); // [-pi,pi]
}

struct Pose {
    double x = 0.0, y = 0.0, theta = 0.0; // world [m], [rad]
    Pose() = default;
    Pose(double X, double Y, double T) : x(X), y(Y), theta(T) {}
};

struct Node {
    Pose pose;          
    int gx = 0, gy = 0;
    double g_cost = 0.0;
    double steering = 0.0;
    int parent = -1;
    Node() = default;
    Node(const Pose& p, int ix, int iy, double g, double st, int par) : pose(p), gx(ix), gy(iy), g_cost(g), steering(st), parent(par) {}
};

class GridMap {
public:
    GridMap() = default;

    GridMap(int w, int h, double res, double ox, double oy, int8_t unknown = -1) : resolution(res), width(w), height(h), origin_x(ox), origin_y(oy) {
        data.assign(width * height, unknown);
    }

    bool inBounds(int gx, int gy) const;
    bool worldToGrid(double xw, double yw, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& xw, double& yw) const;
    int idx(int gx, int gy) const;
    bool isFreeCell(int gx, int gy) const;
    bool isSegmentFree(int gx0, int gy0, int gx1, int gy1) const;

    double resolution = 0.1; // [m/cell]
    int width = 1000;        // cells
    int height = 1000;       // cells
    double origin_x = -50.0; // world [m], grid (0,0) 의 world 좌표
    double origin_y = -50.0;
    bool unknown_is_free = false;
    std::vector<int8_t> data;
};

class HybridAStarRoutePlanner {
public:
    struct PathXYZT {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> theta;  // [rad]
    };

    HybridAStarRoutePlanner(const GridMap& m) : map_(m) {
        wheelbase_ = 2.7;
        const int degs[] = {-40, -20, -10, 0, 10, 20, 40};
        for (int d : degs) steering_inputs_.push_back(deg2rad(d));
        chord_lengths_ = {2.0, 1.0};
    }

    PathXYZT search_route(const Pose& start_w, const Pose& goal_w);

private:
    inline double deg2rad(double d) { 
        return d * M_PI / 180.0;    
    }
    int id(int gx, int gy) const;
    bool next_node(const Node &cur, int cur_id, double chord, double steer, Node &nxt) const;
    double heuristic(const Node& n) const;
    double distance_to_goal(const Pose& p) const;
    PathXYZT reconstruct(const std::unordered_map<int,int> &parent, const std::unordered_map<int,Node> &best_node, int goal_id) const;
    
    const GridMap& map_;
    double wheelbase_ = 2.7;
    std::vector<double> steering_inputs_;
    std::vector<double> chord_lengths_;
    Pose goal_pose_;
};
