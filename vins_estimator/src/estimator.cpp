#include "estimator.hpp"

Estimator::Estimator(): f_manager{Rs} {
    RCUTILS_LOG_INFO("init begins");
    clearState();
}

void Estimator::setParameter() {
    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
    td = TD;
}

// 상태초기화
void Estimator::clearState() {
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        Rs[i].setIdentity();    // 회전 비우기
        Ps[i].setZero();        // 위치 비우기
        Vs[i].setZero();        // 속도 비우기
        Bas[i].setZero();       // 가속도 바이어스 비우기
        Bgs[i].setZero();       // 자이로 바이어스 비우기
        dt_buf[i].clear();      // 시간버퍼 비우기
        linear_acceleration_buf[i].clear(); // 가속도 버퍼 비우기
        angular_velocity_buf[i].clear();    // 각속도 버퍼 비우기

        // 슬라이딩 윈도우 내 프레임 간 연결
        // IMU 프리인테그레이션 객체 삭제
        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    // tic, ric 초기화 -> 같은 위치,자세라고 가정
    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = Eigen::Vector3d::Zero();
        ric[i] = Eigen::Matrix3d::Identity();
    }

    // 초기화 단계 IMU 데이터 누적 프리인테그레이션 객체 삭제
    // tmp_pre_integration 를 하나씩 누적해놓은 pre_integration
    for (auto &it : all_image_frame) {
        if (it.second.pre_integration != nullptr) {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Eigen::Matrix3d::Identity();
    drift_correct_t = Eigen::Vector3d::Zero();
}

// IMU 데이터 처리
void Estimator::processIMU(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity) {
    // 첫 번째 IMU 데이터 저장
    if (!first_imu) {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count]) {
        // 키프레임 사이의 알파, 베타, 감마 저장 인테그레이션
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0) {
        // 이번 IMU 샘플을 프리인테그레이션 객체에 추가해서 Δp/Δv/Δq와 야코비안/공분산을 업데이트
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)

        // "이전 이미지 -> 현재 이미지” 구간 IMU 적분, 키프레임이 아님
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        // 스트랩다운(실시간) 예측 전파
        int j = frame_count;         
        Eigen::Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Eigen::Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

// 이미지 처리
void Estimator::processImage(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::msg::Header &header) {
    RCUTILS_LOG_DEBUG("new image coming ------------------------------------------");
    RCUTILS_LOG_DEBUG("Adding feature points %lu", image.size());
    // 시차기준 키프레임 판별
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;  // MARGIN_OLD: 가장 오래된 프레임을 제거(창 폭 유지)
    else
        marginalization_flag = MARGIN_SECOND_NEW; // MARGIN_SECOND_NEW: 두 번째로 새로운 프레임을 제거

    RCUTILS_LOG_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    RCUTILS_LOG_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    RCUTILS_LOG_DEBUG("Solving %d", frame_count);
    RCUTILS_LOG_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;
    
    // 이번에 들어온 카메라 프레임을 ImageFrame 객체로 하나 만든다
    ImageFrame imageframe(image, header.stamp.sec + header.stamp.nanosec * (1e-9));
    
    // 이전 프레임 이후부터 지금까지의 IMU 적분 결과를 연결
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(std::make_pair(header.stamp.sec + header.stamp.nanosec * (1e-9), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if (ESTIMATE_EXTRINSIC == 2) {
        RCUTILS_LOG_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0) {
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Eigen::Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric)) {
                RCUTILS_LOG_WARN("initial extrinsic rotation calib success");
                std::cout << "initial extrinsic rotation: " << std::endl << calib_ric;
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    // 초기화 단계
    if (solver_flag == INITIAL) {
        if (frame_count == WINDOW_SIZE) {
            bool result = false;
            if (ESTIMATE_EXTRINSIC != 2 && (header.stamp.sec + header.stamp.nanosec * (1e-9) - initial_timestamp) > 0.1) {
               result = initialStructure(); // 초기화 수행
               initial_timestamp = header.stamp.sec + header.stamp.nanosec*(1e-9);
            }
            if(result) {
                solver_flag = NON_LINEAR;
                solveOdometry();            // 첫 비선형 최적화(solveOdometry())
                slideWindow();              // 슬라이딩 윈도우
                f_manager.removeFailures(); // 실패 피쳐 제거
                RCUTILS_LOG_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
                
            }
            else
                slideWindow();  // 실패 시 그냥 슬라이딩 윈도우 실행
        }
        else
            frame_count++;  // 아직 윈도우가 안 찼으면 새 프레임 인덱스 증가 (윈도우 마지막 인덱스)
    }
    else {  // 초기화 단계 아닐경우
        TicToc t_solve;
        solveOdometry();    // 비선형 최적화 

        RCUTILS_LOG_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection()) {
            RCUTILS_LOG_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            RCUTILS_LOG_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();

        f_manager.removeFailures();
        RCUTILS_LOG_DEBUG("marginalization costs: %fms", t_margin.toc());
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}

// IMU가 초기화하기에 충분히 움직였는가 판단
bool Estimator::initialStructure() {
    TicToc t_sfm;
    //check imu observibility
    {
        std::map<double, ImageFrame>::iterator frame_it;
        Eigen::Vector3d sum_g;
        // 각 프레임 구간에서 평균 가속도 계산하고 합산
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
            double dt = frame_it->second.pre_integration->sum_dt;
            Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Eigen::Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
            double dt = frame_it->second.pre_integration->sum_dt;
            Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25) {    // 임계값 으로 판단
            RCUTILS_LOG_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // 시각만으로 전 프레임의 전역 포즈와 3D 포인트를 복원 SFM 수행 (삼각측량)
    // global sfm
    // Eigen::Quaterniond Q[frame_count + 1];  // 카메라 이미지의 자세 , SFM 결과를 담을 출력
    // Eigen::Vector3d T[frame_count + 1];
    Eigen::Quaterniond Q[34];  // 여유를 두고 초기화
    Eigen::Vector3d T[34];

    std::map<int, Eigen::Vector3d> sfm_tracked_points;  // SFM으로 복원된 3D 포인트 (id → 3D)
    std::vector<SFMFeature> sfm_f;  // SFM 입력: 피처별 다프레임 관측

    for (auto &it_per_id : f_manager.feature) {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            Eigen::Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(std::make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Eigen::Matrix3d relative_R;
    Eigen::Vector3d relative_T;
    int l;

    if (!relativePose(relative_R, relative_T, l)) {
        RCUTILS_LOG_INFO("Not enough features or parallax; Move device around");
        return false;
    }

    // 시각적으로만 전 프레임의 자세(Q, T)와 3D 포인트(X)를 복원
    GlobalSFM sfm;
    if (!sfm.construct(frame_count + 1, Q, T, l, relative_R, relative_T, sfm_f, sfm_tracked_points)) {
        RCUTILS_LOG_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    // SFM 로 안푼것들 PnP 구하기
    // solve pnp for all frame 
    std::map<double, ImageFrame>::iterator frame_it;
    std::map<int, Eigen::Vector3d>::iterator it;
    frame_it = all_image_frame.begin();

    for (int i = 0; frame_it != all_image_frame.end(); frame_it++) {
        // provide initial guess
        // solvePnP에 들어갈 회전/이동(rvec,t) 및 중간 변환 버퍼
        cv::Mat r, rvec, t, D, tmp_r;
        if ((frame_it->first) == Headers[i].stamp.sec + Headers[i].stamp.nanosec * (1e-9)) {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if ((frame_it->first) > (Headers[i].stamp.sec + Headers[i].stamp.nanosec * (1e-9))) {
            i++;
        }
        Eigen::Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Eigen::Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);     // rvec -> OpenCV가 사용하는 Rodrigues 회전벡터 형식 (변경)
        cv::eigen2cv(P_inital, t);      // t -> OpenCV가 요구하는 이동벡터

        frame_it->second.is_key_frame = false;  // 이 프레임은 SFM에서 직접 푼 키프레임이 아니므로 False
        std::vector<cv::Point3f> pts_3_vector; 
        std::vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points) {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second) {
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end()) {   // SFM으로 구한 피쳐만 PnP입력으로 사용 (PnP는 2d-3d 비교해야해서)
                    Eigen::Vector3d world_pts = it->second;                             // 3d 점 필요
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Eigen::Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        // 정규화 좌표를 쓰므로 내부행렬 K = I (항등행렬) 로 설정
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if (pts_3_vector.size() < 6) {
            std::cout << "pts_3_vector size " << pts_3_vector.size() << std::endl;
            RCUTILS_LOG_INFO("Not enough points for solve pnp !");
            return false;
        }

        // PnP 수행
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
            RCUTILS_LOG_DEBUG("solve pnp fail!");
            return false;
        }

        cv::Rodrigues(rvec, r);
        Eigen::MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        Eigen::MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else {
        RCUTILS_LOG_INFO("misalign visual structure with IMU");
        return false;
    }
}

// 초기 시각-IMU 정렬을 수행, 결과반환
bool Estimator::visualInitialAlign() {
    TicToc t_g;
    Eigen::VectorXd x;
    // solve scale
    // 시각 - IMU 선형최적화로 스케일, 방향바이어스 구하기
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result) {
        RCUTILS_LOG_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++) {
        Eigen::Matrix3d Ri = all_image_frame[Headers[i].stamp.sec + Headers[i].stamp.nanosec * (1e-9)].R;
        Eigen::Vector3d Pi = all_image_frame[Headers[i].stamp.sec + Headers[i].stamp.nanosec * (1e-9)].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i].stamp.sec + Headers[i].stamp.nanosec * (1e-9)].is_key_frame = true;
    }

    // 기존 피처 깊이 추정치를 모두 초기화
    // 새 포즈/스케일 기준으로 다시 삼각측량
    Eigen::VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //triangulat on cam pose , no tic
    Eigen::Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();   // 카메라 중심과 IMU 중심의 위치 차이를 일단 무시
    ric[0] = RIC[0];    // 회전 extrinsic(ric)은 적용
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);    // 스케일값 추출
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        pre_integrations[i]->repropagate(Eigen::Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    std::map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++) {
        if(frame_i->second.is_key_frame) {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);  // 속도값 최신화
        }
    }
    for (auto &it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Eigen::Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Eigen::Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++) {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    RCUTILS_LOG_INFO("g0: %f, %f, %f", g.x(), g.y(), g.z());
    RCUTILS_LOG_INFO("my R0: %f, %f, %f", Utility::R2ypr(Rs[0]).x(), Utility::R2ypr(Rs[0]).y(), Utility::R2ypr(Rs[0]).z()); 

    return true;
}

// 시각적 초기 상대 자세(R, t)를 추정 -> 윈도우 내 적합한 두 프레임 반환
bool Estimator::relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l) {
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++) {
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20) {   // 최소 피쳐 20개 이상
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++) {
                Eigen::Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Eigen::Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            
            // 5 points algorithm
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T)) {
                l = i;
                RCUTILS_LOG_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::solveOdometry() {
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR) {
        TicToc t_tri;
        f_manager.triangulate(Ps, tic, ric);
        RCUTILS_LOG_DEBUG("triangulation costs %f", t_tri.toc());
        optimization(); // Sliding Window BA (비선형 최적화) 실행
    }
}

// Ceres Solver가 사용할 수 있는 double 배열 형태로 변환
void Estimator::vector2double() {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Eigen::Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++) {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Eigen::Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    Eigen::VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

// Ceres 최적화가 끝나고 내부 상태로 값을 되돌려 넣는 역변환 
void Estimator::double2vector() {
    Eigen::Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Eigen::Vector3d origin_P0 = Ps[0];

    if (failure_occur) {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    Eigen::Vector3d origin_R00 = Utility::R2ypr(Eigen::Quaterniond(para_Pose[0][6],
                                                      para_Pose[0][3],
                                                      para_Pose[0][4],
                                                      para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Eigen::Matrix3d rot_diff = Utility::ypr2R(Eigen::Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
        RCUTILS_LOG_DEBUG("euler singular point!");
        rot_diff = Rs[0] * Eigen::Quaterniond(para_Pose[0][6],
                                       para_Pose[0][3],
                                       para_Pose[0][4],
                                       para_Pose[0][5]).toRotationMatrix().transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++) {

        Rs[i] = rot_diff * Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        
        Ps[i] = rot_diff * Eigen::Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                para_Pose[i][1] - para_Pose[0][1],
                                para_Pose[i][2] - para_Pose[0][2]) + origin_P0;

        Vs[i] = rot_diff * Eigen::Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Eigen::Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Eigen::Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = Eigen::Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Eigen::Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }

    // 깊이(depth) 복구 & td 복구
    Eigen::VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
    if (ESTIMATE_TD)
        td = para_Td[0][0];

    // 루프클로저(relocalization) 결과가 있을 때 드리프트 보정량 갱신
    // relative info between two loop frame
    if (relocalization_info) { 
        Eigen::Matrix3d relo_r;
        Eigen::Vector3d relo_t;
        relo_r = rot_diff * Eigen::Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5]).normalized().toRotationMatrix();
        relo_t = rot_diff * Eigen::Vector3d(relo_Pose[0] - para_Pose[0][0],
                                     relo_Pose[1] - para_Pose[0][1],
                                     relo_Pose[2] - para_Pose[0][2]) + origin_P0;
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Eigen::Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;   
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
        //cout << "vins relo " << endl;
        //cout << "vins relative_t " << relo_relative_t.transpose() << endl;
        //cout << "vins relative_yaw " <<relo_relative_yaw << endl;
        relocalization_info = 0;    

    }
}

// 비정상 발산 감지(실패검사)
bool Estimator::failureDetection() {
    if (f_manager.last_track_num < 2) {     // 피처 수 검사 — 시각 정보 부족 확인
        RCUTILS_LOG_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5) {    // 가속도계 바이어스 크기 체크
        RCUTILS_LOG_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0) {    // 자이로 바이어스 크기 체크
        RCUTILS_LOG_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        RCUTILS_LOG_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Eigen::Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5) {      // Translation(이동량) 검사
        RCUTILS_LOG_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1) {  // 수직 이동(z축) 검사
        RCUTILS_LOG_INFO(" big z translation");
        return true; 
    }
    Eigen::Matrix3d tmp_R = Rs[WINDOW_SIZE];// 회전량 검사 (각도 변화 너무 큰지)
    Eigen::Matrix3d delta_R = tmp_R.transpose() * last_R;
    Eigen::Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50) {                 // 비정상적인 각도 변화 검사
        RCUTILS_LOG_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}

// BA 비선형 최적화
void Estimator::optimization() {
    ceres::Problem problem;             // Ceres 최적화 문제 컨테이너 생성
    ceres::LossFunction *loss_function; // 손실함수
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        // 최적화해야 하는 변수 추가
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    for (int i = 0; i < NUM_OF_CAM; i++) {
        // 회전을 so(3) 상의 3차원 벡터로 근사하여 업데이트
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC) {
            RCUTILS_LOG_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            RCUTILS_LOG_DEBUG("estimate extinsic param");
    }
    if (ESTIMATE_TD) {
        problem.AddParameterBlock(para_Td[0], 1);
        //problem.SetParameterBlockConstant(para_Td[0]);
    }

    TicToc t_whole, t_prepare;
    vector2double();    // Ceres에 올릴 파라미터 메모리 배열로 복사

    // 슐어 보완 잔차
    if (last_marginalization_info) {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    // IMU 프리인테그레이션 Preintegration 잔차
    for (int i = 0; i < WINDOW_SIZE; i++) {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }

    // 시각 visual 잔차
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // 최소 두 프레임 이상에서 보인 feature만 사용
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            if (imu_i == imu_j) {  // 자신은 제외
                continue;
            }
            Eigen::Vector3d pts_j = it_per_frame.point;
            if (ESTIMATE_TD)
            {
                    ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                     it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                    problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                    /*
                    double **para = new double *[5];
                    para[0] = para_Pose[imu_i];
                    para[1] = para_Pose[imu_j];
                    para[2] = para_Ex_Pose[0];
                    para[3] = para_Feature[feature_index];
                    para[4] = para_Td[0];
                    f_td->check(para);
                    */
            }
            else {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            }
            f_m_cnt++;
        }
    }

    RCUTILS_LOG_DEBUG("visual measurement count: %d", f_m_cnt);
    RCUTILS_LOG_DEBUG("prepare for ceres: %f", t_prepare.toc());

    // 루프클로져
    if (relocalization_info) {
        //printf("set relocalization factor! \n");
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);
        int retrive_feature_index = 0;
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature) {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            ++feature_index;
            int start = it_per_id.start_frame;
            if (start <= relo_frame_local_index) {   
                while ((int)match_points[retrive_feature_index].z() < it_per_id.feature_id) {
                    retrive_feature_index++;
                }
                if ((int)match_points[retrive_feature_index].z() == it_per_id.feature_id) {
                    Eigen::Vector3d pts_j = Eigen::Vector3d(match_points[retrive_feature_index].x(), match_points[retrive_feature_index].y(), 1.0);
                    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                    
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    problem.AddResidualBlock(f, loss_function, para_Pose[start], relo_Pose, para_Ex_Pose[0], para_Feature[feature_index]);
                    retrive_feature_index++;
                }     
            }
        }

    }

    ceres::Solver::Options options; // Solver에 전달할 옵션 구조체 생성

    options.linear_solver_type = ceres::DENSE_SCHUR;    // 최적화 방법
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG; // 신뢰 구간 최적화 방법 선택
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);  // Ceres 최적화 실행
    //cout << summary.BriefReport() << endl;
    RCUTILS_LOG_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    RCUTILS_LOG_DEBUG("solver costs: %f", t_solver.toc());

    double2vector();    // 최적화로 갱신된 내부 파라미터 배열(para_Pose[], para_SpeedBias[] 등)을 다시 Estimator 
                        // 내부 변수(Ps[], Rs[], Vs[], Bas[], Bgs[])로 복사
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD) {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        // 이전 priors(과거 마지널라이제이션 결과) 재주입 + '누구 버릴지' 지정
        if (last_marginalization_info) {
            std::vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        
            // IMU 팩터 마지널라이제이션 추가
            if (pre_integrations[1]->sum_dt < 10.0) {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           std::vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           std::vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature) {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame) {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Eigen::Vector3d pts_j = it_per_frame.point;
                    if (ESTIMATE_TD) {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                          it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        std::vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        std::vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       std::vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                       std::vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }
        // 마지널라이제이션 수행
        // 정보 버리지 않고 prior 형태로 유지
        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        RCUTILS_LOG_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        RCUTILS_LOG_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++) {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        if (ESTIMATE_TD) {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
        std::vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info)
            {
                std::vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                    assert(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            RCUTILS_LOG_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            RCUTILS_LOG_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            RCUTILS_LOG_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            RCUTILS_LOG_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++) {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE) {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            if (ESTIMATE_TD) {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }
            
            std::vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    RCUTILS_LOG_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());
    
    RCUTILS_LOG_DEBUG("whole time for ceres: %f", t_whole.toc());
}

// 슬라이딩 윈도우
void Estimator::slideWindow() {
    TicToc t_margin;
    // 오래된 버퍼 버릴경우
    if (marginalization_flag == MARGIN_OLD) {
        double t_0 = Headers[0].stamp.sec + Headers[0].stamp.nanosec * (1e-9);
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE) {
            for (int i = 0; i < WINDOW_SIZE; i++) {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }
            // 맨 끝 슬롯(WINDOW_SIZE)은 이전 끝-1의 내용으로 채워 일단 일관된 상태 형태를 유지
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL) {
                std::map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                it_0->second.pre_integration = nullptr;
 
                for (std::map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++it) {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);

            }
            slideWindowOld();
        }
    }
    else {
        // 두 번째 최신 프레임을 버리는 경우
        if (frame_count == WINDOW_SIZE) {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++) {
                double tmp_dt = dt_buf[frame_count][i];
                Eigen::Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Eigen::Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// 가장 최신 프레임(윈도우의 맨 앞)을 제거
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew() {
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

// 가장 오래된 프레임(윈도우의 맨 뒤)을 제거
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld() {
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth) {
        Eigen::Matrix3d R0, R1;
        Eigen::Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

// 루프 클로저(relocalization) 발생 시 그 결과를 Estimator 내부 상태에 반영
void Estimator::setReloFrame(double _frame_stamp, int _frame_index, std::vector<Eigen::Vector3d> &_match_points, Eigen::Vector3d _relo_t, Eigen::Matrix3d _relo_r) {
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t = _relo_t;
    prev_relo_r = _relo_r;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        if (relo_frame_stamp == Headers[i].stamp.sec + Headers[i].stamp.nanosec * (1e-9)) {
            relo_frame_local_index = i;
            relocalization_info = 1;
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = para_Pose[i][j];
        }
    }
}
