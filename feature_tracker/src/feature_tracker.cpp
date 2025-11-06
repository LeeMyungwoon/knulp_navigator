#include "feature_tracker.hpp"

int FeatureTracker::n_id = 0;

// Bound 검사 -> 너무 모서리쪽 피쳐는 지운다
bool inBorder(const cv::Point2f &pt) {
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

// status = 0 인 피쳐들 지운다
void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(std::vector<int> &v, std::vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker() {
}

// 마스크 설정 -> 서로 떨어지게 선택
void FeatureTracker::setMask() {
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(std::make_pair(track_cnt[i], std::make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const std::pair<int, std::pair<cv::Point2f, int>> &a, const std::pair<int, std::pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    // 가장 오래 추적된것으로 내림차순 정렬하고, 그거 뽑고 주위 검은색 칠해서 다음거 가까운거 안뽑게
    for (auto &it : cnt_pts_id) {
        if (mask.at<uchar>(it.second.first) == 255) {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

// 피쳐 추가
void FeatureTracker::addPoints() {
    for (auto &p : n_pts) {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time) {
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        RCUTILS_LOG_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty()) {
        prev_img = cur_img = forw_img = img;
    }
    else {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0) {
        TicToc t_o;
        std::vector<uchar> status;
        std::vector<float> err;
        // 옵티컬플로우로 연결된 피쳐찾기
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        // 경계 밖 제거
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        RCUTILS_LOG_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME) {
        rejectWithF();  // Fundamental Matrix로 outlier 제거
        RCUTILS_LOG_DEBUG("set mask begins");
        TicToc t_m;
        setMask();      // 공간 분산 확보용 마스크 생성
        RCUTILS_LOG_DEBUG("set mask costs %fms", t_m.toc());

        RCUTILS_LOG_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());    // 부족한 피쳐
        if (n_max_cnt > 0) {
            if(mask.empty())
                RCUTILS_LOG_INFO("mask is empty ");
            if (mask.type() != CV_8UC1)
                RCUTILS_LOG_INFO("mask type wrong ");
            if (mask.size() != forw_img.size())
                RCUTILS_LOG_INFO("wrong size ");
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);  // 피처 신규 검출
        }
        else
            n_pts.clear();
        RCUTILS_LOG_DEBUG("detect feature costs: %fms", t_t.toc());

        RCUTILS_LOG_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();

        RCUTILS_LOG_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    prev_time = cur_time;
}

// Fundmental Matrix 사용
// Optical Flow로 추적된 Feature 쌍들 중에서 에피폴라 제약(Epipolar constraint) 을 만족하지 않는 피쳐 삭제
void FeatureTracker::rejectWithF() {
    if (forw_pts.size() >= 8) {
        RCUTILS_LOG_DEBUG("FM ransac(usac) begins");
        TicToc t_f;
        std::vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());

        for (unsigned int i = 0; i < cur_pts.size(); i++) {
            Eigen::Vector3d tmp_p;
            // 비정상 왜곡 제거 및 픽셀 좌표로 변환
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        std::vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::USAC_FM_8PTS, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        // RCUTILS_LOG_INFO("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        // RCUTILS_LOG_INFO("FM ransac costs: %fms", t_f.toc());
    }
}

// feature ID 부여
bool FeatureTracker::updateID(unsigned int i) {
    if (i < ids.size()) {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const std::string &calib_file) {
    RCUTILS_LOG_INFO("reading paramerter of camera %s", calib_file.c_str());
    // 싱글턴 패턴으로 구현된 카메라 객체 생성, 프로그램 전체에서 동일한 객체공유
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

// Undistortion 이미지 시각화
void FeatureTracker::showUndistortion(const std::string &name) {
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    std::vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++) {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++) {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600) {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

// 피쳐 왜곡이 없는 정규화 좌표계로 변환 및 속도계산
void FeatureTracker::undistortedPoints() {
    cur_un_pts.clear();
    cur_un_pts_map.clear();

    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(std::make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity -> 프레임별 속도계산
    if (!prev_un_pts_map.empty()) {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++) {
            if (ids[i] != -1) {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);  // 이전프레임과 지금프레임에서 같은 피쳐ID 찾기 
                if (it != prev_un_pts_map.end()) {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt; // 정규화 좌표 기준 속도
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else {
        for (unsigned int i = 0; i < cur_pts.size(); i++) {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
