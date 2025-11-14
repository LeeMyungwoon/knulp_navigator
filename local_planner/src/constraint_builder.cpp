// constraint_builder.cpp

#include "local_planner/types.hpp"
#include "local_planner/mpc_utils.hpp"
#include "local_planner/constraint_builder.hpp"

namespace local_planner {
namespace {

// 부등식 제약 추가함수
void appendRows(Eigen::MatrixXd &A, Eigen::VectorXd &b, const Eigen::MatrixXd &Aadd, const Eigen::VectorXd &badd) {
    if (A.rows() == 0) {
        A = Aadd;
        b = badd;
    } 
    else {
        Eigen::MatrixXd Anew(A.rows() + Aadd.rows(), A.cols());
        Eigen::VectorXd bnew(b.size() + badd.size());
        Anew << A, Aadd;    //   << 세로로 이어붙이기 Eigen 연산자
        bnew << b, badd;
        A.swap(Anew);
        b.swap(bnew);
    }
}

// 등식 제약 추가함수
void appendRowsEq(Eigen::MatrixXd &Aeq, Eigen::VectorXd &beq, const Eigen::MatrixXd &Aadd, const Eigen::VectorXd &badd) {
    if (Aeq.rows() == 0) {
        Aeq = Aadd; beq = badd;
    } 
    else {
        Eigen::MatrixXd Anew(Aeq.rows() + Aadd.rows(), Aeq.cols());
        Eigen::VectorXd bnew(beq.size() + badd.size());
        Anew << Aeq, Aadd;
        bnew << beq, badd;
        Aeq.swap(Anew); beq.swap(bnew);
    }
}

}   // namespace anonymous

namespace mpc {

BuiltQP buildConstraints(const MpcProblem &prob, const Eigen::VectorXd &x0, const IndexMap &idx, bool use_small_angle, const Eigen::Vector2d *u_prev_opt) {
    // prob: MPC에 필요한 모든 재료(지평 N, 선형모델 A/B/c, 코리도, 한계값, 슬랙 정책 등).
    const std::size_t N  = prob.hor.N;
    const std::size_t nx = prob.ltv.nx;
    const std::size_t nu = prob.ltv.nu;
    // nx -> 한 시점의 상태차원, nu -> 한 시점의 입력차원, 

    if (prob.ltv.models.size() != N)    // 각 시점마다 선형화된 모델이 필요하니 해당 모델의 개수가 시점만큼 있는지 확인
        throw std::runtime_error("ltv.models size must be N.");
    if (x0.size() != static_cast<int>(nx))  // 현재시점의 상태벡터의 차원이 모델에서 정의한 상태 차원수와 같은지 확인
        throw std::runtime_error("x0 size mismatch.");

    // 결정변수 개수 -> 슬랙변수 줄지 말지 
    bool use_obs_slack  = prob.slack.use_obstacle_slack;    // 장애물/통로 제약에
    bool use_fric_slack = prob.slack.use_friction_slack;    // 마찰 다면제 제약에

    const std::size_t n_x_block = (N + 1) * nx;
    const std::size_t n_u_block = N * nu;
    const std::size_t n_eps_obs = use_obs_slack ? N : 0;    // 각 시점하다 하나씩 존재, corridor 제약 완화용
    const std::size_t n_eps_fric = use_fric_slack ? N : 0;  // 마찰 제약 완화용

    BuiltQP qp;
    // 각 블록의 시작 오프셋
    qp.N = N; qp.nx = nx; qp.nu = nu;
    qp.off_x = 0;
    qp.off_u = n_x_block;
    qp.off_eps_obs  = qp.off_u + n_u_block;
    qp.off_eps_fric = qp.off_eps_obs + n_eps_obs;
    qp.nvar = qp.off_eps_fric + n_eps_fric;

    // 초기화
    qp.Aeq.resize(0, qp.nvar);      // 등식 제약
    qp.beq.resize(0);
    qp.Aineq.resize(0, qp.nvar);    // 부등식 제약 
    qp.bineq.resize(0);
    qp.lb = Eigen::VectorXd::Constant(qp.nvar, -std::numeric_limits<double>::infinity());   // 부등호 제약
    qp.ub = Eigen::VectorXd::Constant(qp.nvar,  std::numeric_limits<double>::infinity());

    // [운동모델 제약]
    // 등식 제약
    for (std::size_t k = 0; k < N; k++) {
        const auto &Ak = prob.ltv.models[k].A;  // 각 스탭에 맞는 모델 꺼내기 (A_k, B_k, c_k)
        const auto &Bk = prob.ltv.models[k].B;
        const auto &ck = prob.ltv.models[k].c;

        // 차수검증
        // A_k = (nx * nx)
        // B_k = (nx * nu)
        // c_k = (nx)
        if (Ak.rows()!= (int)nx || Ak.cols()!= (int)nx) throw std::runtime_error("A size err");
        if (Bk.rows()!= (int)nx || Bk.cols()!= (int)nu) throw std::runtime_error("B size err");
        if (ck.size()!= (int)nx)                        throw std::runtime_error("c size err");

        Eigen::MatrixXd Arow = Eigen::MatrixXd::Zero(nx, qp.nvar);
        // x_{k+1}
        Arow.block(0, qp.off_x + xOffset(k + 1, nx), nx, nx).setIdentity();
        // -A_k x_k
        Arow.block(0, qp.off_x + xOffset(k, nx), nx, nx) -= Ak;
        // -B_k u_k
        Arow.block(0, qp.off_u + k * nu, nx, nu) -= Bk;
        // block -> 행렬접근
        appendRowsEq(qp.Aeq, qp.beq, Arow, ck);
    }
    // 현재상태 x_0를 실제 측정된 값으로 고정시키는 제약 -> 동역학제약이 아니라 처음을(Identity) = 측정값 으로 하는 등식 제약 추가 하는것
    {
        Eigen::MatrixXd A0 = Eigen::MatrixXd::Zero(nx, qp.nvar);
        A0.block(0, qp.off_x + xOffset(0, nx), nx, nx).setIdentity();
        appendRowsEq(qp.Aeq, qp.beq, A0, x0);
    }
    // 입력 bounds
    for (std::size_t k = 0; k < N; k++) {
        const std::size_t off = qp.off_u + k * nu;
        // ax
        qp.lb(off + idx.idx_ax) = prob.limits.ax_min;
        qp.ub(off + idx.idx_ax) = prob.limits.ax_max;
        // delta
        qp.lb(off + idx.idx_delta) = prob.limits.delta_min;
        qp.ub(off + idx.idx_delta) = prob.limits.delta_max;
    }
    // 상태 bounds (|ey|, vx, wz 등)
    for (std::size_t k = 0; k <= N; ++k) {
        const std::size_t off = qp.off_x + xOffset(k, nx);
        // |ey| <= ey_abs_max
        qp.lb(off + idx.idx_ey) = -prob.limits.ey_abs_max;  // 횡방향 -> 벗어나지 않도록
        qp.ub(off + idx.idx_ey) =  prob.limits.ey_abs_max;
        // vx in [vx_min, vx_max]
        qp.lb(off + idx.idx_vx) = prob.limits.vx_min;       // 종방향 속도 -> 급가속, 급감속 방지
        qp.ub(off + idx.idx_vx) = prob.limits.vx_max;
        // wz in [wz_min, wz_max]
        qp.lb(off + idx.idx_wz) = prob.limits.wz_min;       // yaw rate -> 급격한 조향 방지
        qp.ub(off + idx.idx_wz) = prob.limits.wz_max;
    }

    // [입력변화율 제약]
    // 부등식 제약
    if (N > 0) {
        for (std::size_t k = 0; k < N; k++) {
            Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(2 * nu, qp.nvar);    // 입력 변화율 제약식의 좌변 계수 행렬
            Eigen::VectorXd b2 = Eigen::VectorXd::Zero(2 * nu);             // 제약식의 우변 상수항

            for (int j = 0; j < (int)nu; ++j) {
                A2(2 * j, qp.off_u + k * nu + j) =  1.0;                // 행 2*j :  u_k - u_{k-1} <= dmax
                A2(2 * j + 1, qp.off_u + k * nu + j) = -1.0;            // 행 2*j+1: -(u_k - u_{k-1}) <= -dmin  → -u_k + u_{k-1} <= -dmin
            }
            if (k > 0) {
                for (int j = 0; j < (int)nu; j++) {
                    A2(2 * j, qp.off_u + (k - 1) * nu + j) = -1.0;
                    A2(2 * j + 1, qp.off_u + (k - 1) * nu + j) =  1.0;
                }
            } 
            else {
                if (u_prev_opt) {
                    const Eigen::Vector2d& u_prev = *u_prev_opt;
                    for (int j = 0; j < (int)nu; j++) {
                        b2(2 * j) =  prob.limits.dax_max; // 기본값, 아래서 각 성분으로 교체
                        b2(2 * j + 1) = -prob.limits.dax_min;
                    }
                    // 성분별 (ax, delta)
                    // u_prev가 비어 있으면 warm start 없이 순수 rate 제한만 사용
                    if (u_prev.size() >= 2) {
                        b2(0) = prob.limits.dax_max + u_prev(0);   
                        b2(1) = -prob.limits.dax_min - u_prev(0);  
                        b2(2) = prob.limits.ddelta_max + u_prev(1);
                        b2(3) = -prob.limits.ddelta_min - u_prev(1);
                    } else {
                        // warm start가 없으면 u_prev를 0으로 본다고 생각
                        b2(0) = prob.limits.dax_max;
                        b2(1) = -prob.limits.dax_min;
                        b2(2) = prob.limits.ddelta_max;
                        b2(3) = -prob.limits.ddelta_min;
                    }
                } 
                else {
                    A2.setZero(); 
                    b2.setZero();
                }
            }
            if (k > 0) {
                b2(0) =  prob.limits.dax_max;
                b2(1) = -prob.limits.dax_min;
                b2(2) =  prob.limits.ddelta_max;
                b2(3) = -prob.limits.ddelta_min;
            }
            // k=0에서 u_prev 미제공이면 skip -> 입력변화율 부등식 제약값이 있을때 제약식에 추가
            if (A2.squaredNorm() > 0.0)
                appendRows(qp.Aineq, qp.bineq, A2, b2);
        }
    }

    // [코리도 제약]
    // 부등식 제약
    const double L_full = prob.model.geom.front_overhang + prob.model.geom.wheelbase + prob.model.geom.rear_overhang;
    const double halfL  = 0.5 * L_full;
    auto push_corridor = [&](std::size_t k, double ey_min, double ey_max, double sign /* -1 rear, 0 mid, +1 front */) {
        // e_term = ey + sign*(L/2)*eψ   (rear는 sign=-1, mid 0, front +1)
        // 상한:  e_term <= ey_max (+ ε_obs)
        // 하한: -e_term <= -ey_min (+ ε_obs)
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, qp.nvar);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(2);

        const std::size_t offx = qp.off_x + xOffset(k, nx);

        // 상한
        A(0, offx + idx.idx_ey) = 1.0;                                                              // e_y 항 계수 = +1
        A(0, offx + idx.idx_epsi) =  sign * (use_small_angle ? halfL : halfL /* same here */);      // heading -> eψ 항 계수 = sign×L/2
        b(0) = ey_max;
        // 하한  -e_term <= -ey_min
        A(1, offx + idx.idx_ey) = -1.0;
        A(1, offx + idx.idx_epsi) = -sign * (use_small_angle ? halfL : halfL);
        b(1) = -ey_min;

        if (use_obs_slack) {
            const std::size_t eps_idx = qp.off_eps_obs + k;
            A(0, eps_idx) =  1.0;
            A(1, eps_idx) =  1.0;
            // ε bounds
            qp.lb(eps_idx) = 0.0;
            qp.ub(eps_idx) = prob.slack.slack_max_obs;
        }
        appendRows(qp.Aineq, qp.bineq, A, b);
    };

    // 코리도 경계 데이터 개수 검증
    if (prob.corridor.bounds.size() != N)
        throw std::runtime_error("corridor.bounds size must be N.");

    // 모든 시점 k에 대해, 뒤/중간/앞 3지점 제약 추가
    for (std::size_t k = 0; k < N; k++) {
        const auto& b = prob.corridor.bounds[k];
        // Rear (sign=-1), Mid (0), Front(+1)
        push_corridor(k, b.ey_min_R, b.ey_max_R, -1.0); // 마지막 파라메미터 => 위치 플래그 (뒤, 중앙, 앞)
        push_corridor(k, b.ey_min_M, b.ey_max_M,  0.0);
        push_corridor(k, b.ey_min_F, b.ey_max_F, +1.0);
    }

    // [마찰제약]
    // 원 안에서는 미끄러지지 않지만 마찰원을 벗어나면 미끄러진다 -> 이 원을 넘지않는 제약조건
    if (!prob.friction_poly.H.empty()) {
        // 마찰타원 형태로 나타나는데 이건 비선형 -> 원을 직선 여러개로 근사 -> 다면체라고 한다
        // Hk​uk​ ≤ hk​ 이런 제약식으로 바꾼다
        if (prob.friction_poly.H.size() != N || prob.friction_poly.h.size() != N)
            throw std::runtime_error("friction_poly size must be N.");

        for (std::size_t k = 0; k < N; k++) {
            // 시점 k에서 마찰 다면체를 구성하는 행렬 - 벡터를 꺼낸다
            const auto &Hk = prob.friction_poly.H[k]; // (rows x nu)
            const auto &hk = prob.friction_poly.h[k]; // (rows)

            // 차원 유효성 검사
            if (Hk.cols() != (int)nu || Hk.rows() != hk.size())
                throw std::runtime_error("friction_poly dims mismatch.");

            const int rows = Hk.rows();

            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(rows, qp.nvar);
            Eigen::VectorXd b = hk; // copy

            // place Hk on u_k block
            A.block(0, qp.off_u + k * nu, rows, nu) = Hk;

            // 마찰 슬랙을 쓰는 옵션 -> 완화 조건
            if (use_fric_slack) {
                const std::size_t eps = qp.off_eps_fric + k;
                A.col(eps).setOnes();
                qp.lb(eps) = 0.0;
                qp.ub(eps) = prob.slack.slack_max_friction;
            }
            appendRows(qp.Aineq, qp.bineq, A, b);
        }
    }

    return qp;
}
    
}   // namespace mpc
}   // namespace local_planner
