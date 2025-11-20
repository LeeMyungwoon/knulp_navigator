# knulp Navigator

ROS2 기반 **<mark>"비전(단안카메라)·IMU SLAM"</mark>** + **<mark>"글로벌/로컬 경로 계획"</mark>** 통합 프로젝트입니다.  

<img width="353" height="296" alt="Image" src="https://github.com/user-attachments/assets/28a1292c-e6f8-481a-8811-55bdcbc063a1" />

실내 자율주행 로봇을 대상으로, **VINS-mono 기반 SLAM**으로 추정한 자세와 **Occupancy Grid 맵**을 사용해

- Hybrid A* 기반 **Global Planner**
- LTV-MPC + 안전 코리도 기반 **Local Planner**

를 한 시스템 안에서 동작시키는 것을 목표로 합니다.

---

## 1. System Overview (시스템 개요)

전체 시스템은 4개의 프로세스 그룹으로 나뉩니다.

- **SENSOR BRIDGE SERVER**
  - 실제 단안카메라 / IMU / 시뮬레이터(Isaac Sim 등)에서 데이터를 받아 ROS2 Topic으로 Publish
- **SLAM SERVER**
  - VINS-mono 기반 Visual-Inertial Odometry
  - Feature 추적, 비주얼-IMU 최적화, Pose Graph 관리
  - IMU Preintegration + Key frame + Loop Closure 최적화 결과로 odometry 데이터 ROS2 Topic으로 Publish
- **MAP BUILDER SERVER**
  - VINS keyframe / point cloud를 이용해 Occupancy Grid 맵을 생성·업데이트
  - Bresenham Algorithm 사용
- **PATH PLANNER SERVER**
  - **Global Planner** 
    - Hybrid A* Algorithm 으로 전역 경로 생성
    - 휴리스틱 Cost로 유클리드 거리 + 도착 각도 + 스티어링 앵글 입력각 사용
  - **Local Planner**
    - distance field 기반 코리도 + LTV-MPC로 로컬 회피 및 속도/조향 명령 생성
    - dijkstra Algorithm 사용

최종적으로 `ackermann_cmd` 형태의 제어 명령을 로봇(또는 시뮬레이터)에 전달합니다.

---

## 2. Software Architecture (소프트웨어 아키텍쳐)

<img width="742" height="461" alt="Image" src="https://github.com/user-attachments/assets/6b8d9b47-d8c2-4c33-a13c-83836f18f9e6" />

### 데이터 흐름 요약

- **SENSOR BRIDGE SERVER**
  - 입력:
    - 카메라
    - IMU
  - Publish:
    - 원본이미지 → `/cam0/image_raw`
    - IMU → `/imu0`
- **SLAM SERVER**
  - `Feature Tracker`  
    - Subscribe:
      - 원본이미지 → `/cam0/image_raw`
    - Publish:
      - 특징점 → `/feature_tracker/feature`
  - `Vins Estimator`
    - Subscribe: 
      - 특징점 → `/feature_tracker/feature`
      - IMU → `/imu0` Visual-Inertial Odometry 추정 및 최적화 
    - Publish:
      - 최적화 결과 자세 → `/vins_estimator/odometry`
      - 키프레임 → `/vins_estimator/keyframe_pose`
      - 키프레임 속 매칭포인트 → `/vins_estimator/keyframe_point`
  - `Pose Graph`
    - Subscribe:
      - 원본이미지 → `/cam0/image_raw`
      - 최적화 결과 자세 → `/vins_estimator/odometry`
      - 키프레임 → `/vins_estimator/keyframe_pose`
      - 키프레임 속 매칭포인트 → `/vins_estimator/keyframe_point`
    - Publish: 
      - Loop Closure, 전역 Pose Graph 최적화
- **MAP BUILDER SERVER**  
  - `Map Builder` 
    - Subscribe: 
      - 키프레임 속 매칭포인트 → `/vins_estimator/keyframe_point`
      - 최적화 결과 자세 → `/vins_estimator/odometry`
    - Publish:
      - 전역 맵 → `/map`
- **PATH PLANNER SERVER**
  - `Global Planner`
    - Subscribe:
      - 전역 맵 → `/map`
      - 자세 → `/vins_estimator/odometry`
      - rviz2 의 click goal point → `/clicked_point` 
    - Publish:
      - 전역경로 `/global_path`
  - `Local Planner`  
    - Subscribe:
      - 전역경로 → `/global_path`
      - VINS odom → `/vins_estimator/odometry`
      - 전역맵 → `/map`
    - Publish:
      - ackermann 메시지 → `/ackermann_cmd`

---

## 3. Tech Stack (기술 스택)

| Category | Technology    |
| :---: | --- |
| Development environment | **Ubuntu 22.04**, vscode |
| Middleware | **ROS2 (Humble)** |
| Language | **C++**, Python |
| Libraries | OpenCV, Eigen |
| Optimization Libraries | Ceres Solver, OSQP |
| Configuration Management | Github, Notion |



---

## 4. Demo Video (데모 영상)

### VINS-Mono
<img width="400" height="250" alt="Image" src="https://github.com/user-attachments/assets/b03c69bc-3ccf-4411-847d-2b0b7de3117f" />
<br />
VINS-mono SLAM
Euroc dataset을 활용하여 테스트

### Map builder
<img width="400" height="250" alt="Image" src="https://github.com/user-attachments/assets/f413962d-e79c-4801-93a6-e08cae1c32b8" />
<br />
Euroc dataset을 활용하여 테스트 되었습니다. 
<br />
드론 dataset이라 높이기반 장애물 필터링을 하지 못했습니다.
<br />
추후 차량 dataset으로 대체 될 예정입니다.

### Global Planner
<img width="400" height="250" alt="Image" src="https://github.com/user-attachments/assets/d6487aad-5a00-46d1-b653-5b20644a2cd5" />
<br />
Hybrid A* 를 사용하여 Global Planner 를 구축했습니다.
<br />
rivz2 의 Publish Point 를 받으면 goal 경로 및 현재 스티어링각도와 
<br />
차량 Ackermann Steering 모델에 맞게 경로를 생성합니다.

### Local Planner
<img width="400" height="250" alt="Image" src="https://github.com/user-attachments/assets/29440d1c-2175-4b03-93dd-e459e04d9bef" />
<br />
이 레포지터리의 local_planner 에 있는 Test dummy node를 사용해 태스트를 진행했습니다.
<br />
추후 차량 dataset으로 대체 될 예정입니다.

---

## 6. Acknowledgements
We use **ROS1 version of VINS-Mono**, **Ceres Solver** for non-linear optimization, **DBoW2** for loop detection, and a **generic camera model**.
We also referred to parts of the implementations from:
- [VINS-Mono]([https://github.com/HKUST-Aerial-Robotics/VINS-Mono)](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- [Ceres Solver]([http://ceres-solver.org/installation.html])(http://ceres-solver.org/installation.html)
- [DBoW2]([https://github.com/dorian3d/DBoW2])(https://github.com/dorian3d/DBoW2)
- [OSQP]([https://github.com/osqp/osqp])(https://github.com/osqp/osqp)

---

## 7. Licence
The source code is released under the **GPLv3** license.
See the full license here:
- [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html)

---
