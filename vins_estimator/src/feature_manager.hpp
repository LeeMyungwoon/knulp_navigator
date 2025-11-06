#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
#include <rcutils/logging_macros.h>

#include <eigen3/Eigen/Dense>

#include "parameters.hpp"

class FeaturePerFrame {
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td) {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
    }
    double cur_td;
    Eigen::Vector3d point;
    Eigen::Vector2d uv;
    Eigen::Vector2d velocity;
    double z = 0.0;
    bool is_used = false;
    double parallax = 0.0;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    double dep_gradient = 0.0;
};

class FeaturePerId {
  public:
    const int feature_id = -1;      // 이 특징점의 ID
    int start_frame = 0;            // 처음 관측된 프레임 번호(슬라이딩 윈도우 내 상대 인덱스)
                                    // 실제등장프레임번호 - 현재윈도우에서 가장 오래된프레임번호
    std::vector<FeaturePerFrame> feature_per_frame;

    int used_num = 0;           // 최적화/삼각측량에 실제 활용된 관측 수
    bool is_outlier = false;    // 아웃라이어로 판정되었는지
    bool is_margin = false;     // 마지널라이즈 과정에서 경계/삭제 후보인지
    double estimated_depth = -1.0;
    int solve_flag = 0; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Eigen::Vector3d gt_p = Eigen::Vector3d::Zero();

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

class FeatureManager {
  public:
    FeatureManager(Eigen::Matrix3d _Rs[]);

    void setRic(Eigen::Matrix3d _ric[]);

    void clearState();

    int getFeatureCount();

    bool addFeatureCheckParallax(int frame_count, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
    void debugShow();
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    //void updateDepth(const VectorXd &x);
    void setDepth(const Eigen::VectorXd &x);
    void removeFailures();
    void clearDepth(const Eigen::VectorXd &x);
    Eigen::VectorXd getDepthVector();
    void triangulate(Eigen::Vector3d Ps[], Eigen::Vector3d tic[], Eigen::Matrix3d ric[]);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier();
    std::list<FeaturePerId> feature;
    int last_track_num = 0;

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Eigen::Matrix3d *Rs;
    Eigen::Matrix3d ric[NUM_OF_CAM];
};

#endif
