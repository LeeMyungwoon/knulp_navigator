#include "feature_manager.hpp"

int FeaturePerId::endFrame() {
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Eigen::Matrix3d _Rs[]) : Rs(_Rs) {
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Eigen::Matrix3d _ric[]) {
    for (int i = 0; i < NUM_OF_CAM; i++) {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState() {
    feature.clear();
}

// “현재 최적화 윈도우 내에서 유효하게 사용 가능한 특징점 개수”를 반환
int FeatureManager::getFeatureCount() {
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2) {
            cnt++;
        }
    }
    return cnt;
}

// 이번 프레임에서 들어온 피쳐들 반영, 키프레임 반영확인 (시차사용)
bool FeatureManager::addFeatureCheckParallax(int frame_count, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td) {
    RCUTILS_LOG_DEBUG("input feature: %d", (int)image.size());
    RCUTILS_LOG_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image) {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end()) {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id) {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature) {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1) {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0) {
        return true;
    }
    else {
        RCUTILS_LOG_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        RCUTILS_LOG_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

void FeatureManager::debugShow() {
    RCUTILS_LOG_DEBUG("debug show");
    for (auto &it : feature) {
        assert(it.feature_per_frame.size() != 0);
        assert(it.start_frame >= 0);
        assert(it.used_num >= 0);

        RCUTILS_LOG_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame) {
            RCUTILS_LOG_DEBUG("%d,", int(j.is_used));
            sum += j.is_used;
            RCUTILS_LOG_INFO("(%lf,%lf) ",j.point(0), j.point(1));
        }
        assert(it.used_num == sum);
    }
}

// 닮은 피쳐 쌍 반환 (두 프레임 비교)
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r) {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
    for (auto &it : feature) {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r) {
            Eigen::Vector3d a = Eigen::Vector3d::Zero(), b = Eigen::Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(std::make_pair(a, b));
        }
    }
    return corres;
}

// BA이후 계산된 값 depth 적용
void FeatureManager::setDepth(const Eigen::VectorXd &x) {   // x = 최적화 결과 벡터
    int feature_index = -1;
    for (auto &it_per_id : feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //RCUTILS_LOG_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0) {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

// 깊이추정 실패한 feature 제거
void FeatureManager::removeFailures() {
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next) {
        it_next++;
        if (it->solve_flag == 2)    // 삼각측량 실패 음수 depth 인 경우
            feature.erase(it);
    }
}

// depth 재초기화
void FeatureManager::clearDepth(const Eigen::VectorXd &x) {
    int feature_index = -1;
    for (auto &it_per_id : feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

// setDepth() 의 반대 개념
// 현재 상태를 벡터로 꺼내서 최적화 입력으로 전달 (VectorXd)
Eigen::VectorXd FeatureManager::getDepthVector() {
    Eigen::VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        // 역깊이(inverse depth = 1 / depth) 
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

// 삼각측량 -> 선형최적화
void FeatureManager::triangulate(Eigen::Vector3d Ps[], Eigen::Vector3d tic[], Eigen::Matrix3d ric[]) {
    for (auto &it_per_id : feature) {
        // 해당 특징점이 몇 개의 프레임에서 관측되었는지 카운트
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // 최소 2프레임 이상 봐야 삼각측량 가능
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        // 이미 깊이가 양수로 잘 잡혀 있으면 재삼각측량 생략
        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        assert(NUM_OF_CAM == 1);

        // 선형 시스템 AX = 0 의 계수 행렬 A를 만든다.
        // 한 프레임당 두 줄(제약 2개)을 쌓으므로 행 개수 = 2 × 관측 수(프레임 등장 횟수)
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        // t0, R0는 i번째 카메라의 월드 기준 포즈.
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);             // 월드 좌표에서의 두 카메라 포즈로 좌표계 변환
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;                              // 카메라j의 투영행렬
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);   // 서로 평행하니 외적 했을때 0
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        assert(svd_idx == svd_A.rows());
        // SVD로 제약식 행렬쪼개서 V구해서 특이값 가장 작게 곱해지는 V의 마지막 열 가져오기
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        // 동차좌표(X, Y, Z, W) 를 depth로 변환
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1) {
            it_per_id.estimated_depth = INIT_DEPTH;
        }
    }
}

// 이상치 제거
void FeatureManager::removeOutlier() {
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next) {
        it_next++;
        i += it->used_num != 0;
        // Outlier 판정
        if (it->used_num != 0 && it->is_outlier == true) {
            feature.erase(it);
        }
    }
}

// marg_R, marg_P 마지널라이즈되는(버려지는) 프레임의 포즈
// new_R, new_P 새 윈도우의 기준(첫 프레임) 포즈
void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P) {
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next) {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2) {
                feature.erase(it);
                continue;
            }
            else {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                // 양수면 정상 깊이로 갱신. 음수면 기본 깊이로 초기화
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}

// 뒤에서 지우기 (윈도우)
void FeatureManager::removeBack() {
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next) {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

// 앞서 지우기 (윈도우)
void FeatureManager::removeFront(int frame_count) {
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next) {
        it_next++;

        if (it->start_frame == frame_count) {
            it->start_frame--;
        }
        else {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

// 피쳐 시차계산 (이전 프레임과의 시차 (최근 2 프레임))
double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count) {
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Eigen::Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Eigen::Vector3d p_i = frame_i.point;
    Eigen::Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = std::max(ans, sqrt(std::min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}
