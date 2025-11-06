#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.hpp"
#include "parameters.hpp"
#include "utility/visualization.hpp"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
std::queue<sensor_msgs::msg::PointCloud::SharedPtr> feature_buf;
std::queue<sensor_msgs::msg::PointCloud::SharedPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

// 현재의 값을 갖고 추정값 도출
void predict(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    double t = imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9);
    if (init_imu) {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};    // IMU에서 읽은 가속도

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};   // IMU에서 읽은 각속도

    // 직전 시점의 가속도 계산
    // acc_0: 직전 IMU의 가속도 값
    // tmp_Ba: 가속도 바이어스(오차)
    // tmp_Q 현재까지의 회전행렬 (IMU → world)
    // estimator.g: 중력 벡터
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    // 자이로(각속도)로 회전 업데이트
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    // 현재 시점의 가속도 계산 (갱신된 tmp_Q 이용)
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    // 평균 가속도로 위치·속도 적분
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    // 다음 스텝을 위한 저장
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

// Rs, Vs, Ps 의 결과를 담아서 predict() 준비
void update() {
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    std::queue<sensor_msgs::msg::Imu::SharedPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::msg::Imu::SharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());
}

// <IMU 데이터, 특징점들이 들어있는 하나의 이미지 데이터>
std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>> getMeasurements() {
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>> measurements;

    while (true) {
        if (imu_buf.empty() || feature_buf.empty()) // 버퍼 비어 있으면 종료
            return measurements;

        // 이미지보다 IMU가 너무 적은 경우 (대기 상태)
        if (!((imu_buf.back()->header.stamp.sec+imu_buf.back()->header.stamp.nanosec * (1e-9)) > (feature_buf.front()->header.stamp.sec+feature_buf.front()->header.stamp.nanosec * (1e-9)) + estimator.td)) {
            // imu_buf.back() → 가장 최근 IMU 시간
            // feature_buf.front() → 가장 오래된 이미지 시간
            //RCUTILS_LOG_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        // IMU가 너무 앞선 경우 (이미지 버리기)
        if (!((imu_buf.front()->header.stamp.sec+imu_buf.front()->header.stamp.nanosec * (1e-9)) < (feature_buf.front()->header.stamp.sec+feature_buf.front()->header.stamp.nanosec * (1e-9)) + estimator.td)) {
            RCUTILS_LOG_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::msg::PointCloud::SharedPtr img_msg = feature_buf.front();
        feature_buf.pop();

        // 대응되는 IMU 데이터 수집
        std::vector<sensor_msgs::msg::Imu::SharedPtr> IMUs;
        while ((imu_buf.front()->header.stamp.sec+imu_buf.front()->header.stamp.nanosec * (1e-9)) < (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec * (1e-9)) + estimator.td) {
            // 이 이미지가 찍힐 때까지의 IMU 데이터 구간”을 전부 추출한
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        
        // IMU가 하나도 없는 경우 경고
        if (IMUs.empty())
            RCUTILS_LOG_WARN("no imu between two image");   // IMU + 이미지 쌍을 measurements에 추가
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

// IMU 데이터 콜백함수 
void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    // 시간 순서 체크
    if ((imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9)) <= last_imu_t) {
        RCUTILS_LOG_WARN("imu message in disorder!");
        return;
    }

    last_imu_t = imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9);

    m_buf.lock();           // m_buf: 버퍼 보호용 mutex (다른 스레드와의 동시 접근 방지)
    imu_buf.push(imu_msg);  // imu_buf: 전역 큐(Queue)
    m_buf.unlock();
    con.notify_one();       // process() 깨우기

    last_imu_t = imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9);
    {
        // 현재 자세 예측 (IMU propagation)
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::msg::Header header = imu_msg->header;
        header.frame_id = "world";  // RViz 등에서 출력할 좌표계 지정
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

// feature 콜백함수
void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg) {
    if (!init_feature) {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();   // 컨디션 변수 깨우기
}

// Estimator 초기화
void restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg) {
    if (restart_msg->data == true) {
        RCUTILS_LOG_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

// 루프클로저 (pose_graph로 부터 토픽 받으면)
void relocalization_callback(const sensor_msgs::msg::PointCloud::SharedPtr points_msg) {
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// 멀티스레드
// thread: visual-inertial odometry
void process() {
    while (true) {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        // 조건변수 -> imu, feature callback 에 값 들어오면 깨운다
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;  // 두 데이터 모두 있는지 확인
                 });

        lk.unlock();
        m_estimator.lock(); // 단일 스레드 접근 보장 -> Estimator 내부 메모리 건드린다
        for (auto &measurement : measurements) {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;  // 현재 IMU 데이터
            for (auto &imu_msg : measurement.first) {
                double t = imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9);    // IMU가 찍힌 시각
                double img_t = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec * (1e-9) + estimator.td; // 카메라가 찍힌 시각 (보정된 시간)
                if (t <= img_t) { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;   // 이전 IMU에서 이번 IMU까지 구간
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            // set relocalization frame
            sensor_msgs::msg::PointCloud::SharedPtr relo_msg = NULL;
            while (!relo_buf.empty()) { // Pose Graph 노드(pose_graph.cpp) 에서 퍼블리시되는 데이터
                relo_msg = relo_buf.front();
                relo_buf.pop(); // 가장 최신(recent) 루프 감지 결과만 가져온다
            }
            if (relo_msg != NULL) {
                std::vector<Eigen::Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.sec+relo_msg->header.stamp.nanosec * (1e-9);
                for (unsigned int i = 0; i < relo_msg->points.size(); i++) {
                    Eigen::Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id); // 각 매칭된 포인트의 정규화 좌표와 피처 ID
                }
                // 루프 감지에서 얻은 자세(R, t) 파싱
                Eigen::Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Eigen::Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Eigen::Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                // Estimator에게 루프 정보 전달
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            RCUTILS_LOG_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.sec+img_msg->header.stamp.nanosec * (1e-9));

            TicToc t_s;
            std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++) {
                int v = img_msg->channels[0].values[i] + 0.5;   // 피쳐 ID 및 카메라 ID 추출
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                assert(z == 1); // 유효성 체크 -> 정규화 좌표이므로 z는 1 인거 확인
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::msg::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //RCUTILS_LOG_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("vins_estimator");
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    RCUTILS_LOG_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    RCUTILS_LOG_WARN("waiting for image and imu...");

    registerPub(n);

    auto sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS(rclcpp::KeepLast(2000)), imu_callback);
    auto sub_image = n->create_subscription<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", rclcpp::QoS(rclcpp::KeepLast(2000)), feature_callback);
    auto sub_restart = n->create_subscription<std_msgs::msg::Bool>("/feature_tracker/restart", rclcpp::QoS(rclcpp::KeepLast(2000)), restart_callback);
    auto sub_relo_points = n->create_subscription<sensor_msgs::msg::PointCloud>("/pose_graph/match_points", rclcpp::QoS(rclcpp::KeepLast(2000)), relocalization_callback);

    std::thread measurement_process{process};
    rclcpp::spin(n);

    return 0;
}
