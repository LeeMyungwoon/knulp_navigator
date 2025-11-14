// local_mpc.cpp

#include "local_planner/types.hpp"
#include "local_planner/local_mpc.hpp"
#include "local_planner/constraint_builder.hpp"

#ifdef USE_OSQP
  #include <osqp/osqp.h>
#endif

namespace local_planner {
namespace {

struct CostBuildOut {
    Eigen::MatrixXd H;
    Eigen::VectorXd f;
};

// 비용함수 생성기
CostBuildOut buildCostHF(const MpcProblem &prob, const mpc::BuiltQP &qp, const mpc::IndexMap &idx, const Eigen::Vector2d *u_prev_opt) {
    const std::size_t N = qp.N;
    const std::size_t nx = qp.nx;
    const std::size_t nu = qp.nu;
    const std::size_t nvar = qp.nvar;

    CostBuildOut out;
    // H, f를 전부 0으로 초기화.
    out.H = Eigen::MatrixXd::Zero(nvar, nvar);
    out.f = Eigen::VectorXd::Zero(nvar);

    // --------- 상태 가중치 Q (각 시점) ---------
    // Q는 상태별 대각 가중치. 참조 (ey*=0, epsi*=0, vx*=ref, s*=ref(보통 0 가중))
    const auto &W = prob.weights;

    // 한 시점 k에 대해 상태 비용을 H, f 에 더한다
    auto put_state_quad = [&](std::size_t k, double ey_ref, double epsi_ref, double s_ref, double vx_ref, double wz_ref) {
        // off = z 벡터 안에서 x_k가 시작되는 인덱스. 상태벡터 시작 인덱스
        const std::size_t off = qp.off_x + mpc::xOffset(k, nx);

        // 대각 가중치(부호 주의 필요 없음. H에 2*Q, f에 -2*Q*x_ref)
        // 상태 순서: [ey, epsi, s, vx, wz]
        // 가중치
        const double q_ey = W.q_ey;
        const double q_epsi = W.q_epsi;
        const double q_s = W.q_s;
        const double q_vx = W.q_vx;
        const double q_wz = 0.0; // 필요시 weights에 추가

        // H 블록 (대각)
        // H = 2q
        out.H(off + idx.idx_ey,   off + idx.idx_ey)   += 2.0 * q_ey;
        out.H(off + idx.idx_epsi, off + idx.idx_epsi) += 2.0 * q_epsi;
        out.H(off + idx.idx_s,    off + idx.idx_s)    += 2.0 * q_s;
        out.H(off + idx.idx_vx,   off + idx.idx_vx)   += 2.0 * q_vx;
        out.H(off + idx.idx_wz,   off + idx.idx_wz)   += 2.0 * q_wz;

        // f 블록 (선형: -2 Q x_ref)
        // f = -2q{x_{ref}}
        out.f(off + idx.idx_ey)   += -2.0 * q_ey   * ey_ref;
        out.f(off + idx.idx_epsi) += -2.0 * q_epsi * epsi_ref;
        out.f(off + idx.idx_s)    += -2.0 * q_s    * s_ref;
        out.f(off + idx.idx_vx)   += -2.0 * q_vx   * vx_ref;
        out.f(off + idx.idx_wz)   += -2.0 * q_wz   * wz_ref;
    };

    // k = 0..N-1 : 참조는 prob.ref.*[k]
    // k = N(터미널) : 마지막 값 재사용
    auto get_ref = [&](std::size_t k, double &s_ref, double &yaw_ref, double &vx_ref, double &kappa_ref) {
        const std::size_t kk = std::min<std::size_t>(k, prob.ref.s_ref.size() ? prob.ref.s_ref.size() - 1 : 0);
        s_ref     = (prob.ref.s_ref.size()     ? prob.ref.s_ref[kk]     : 0.0);
        yaw_ref   = (prob.ref.yaw_ref.size()   ? prob.ref.yaw_ref[kk]   : 0.0);
        vx_ref    = (prob.ref.vx_ref.size()    ? prob.ref.vx_ref[kk]    : 0.0);
        kappa_ref = (prob.ref.kappa_ref.size() ? prob.ref.kappa_ref[kk] : 0.0);
    };

    for (std::size_t k = 0; k <= N; k++) {
        double s_ref, yaw_ref, vx_ref, kappa_ref;
        get_ref(k, s_ref, yaw_ref, vx_ref, kappa_ref);

        // 상태는 "오차" 변수로 정의되어 있다고 가정 (ey, epsi는 ref가 0)
        const double ey_ref   = 0.0;
        const double epsi_ref = 0.0;

        // s, vx는 필요시 참조 부여(보통 q_s=0이면 영향 없음)
        put_state_quad(k, ey_ref, epsi_ref, s_ref, vx_ref, /*wz_ref*/0.0);
    }

    // --------- 입력 가중치 R ---------
    // 각 k=0..N-1에 대해 u_k^T R u_k
    auto put_input_quad = [&](std::size_t k, double ax_ref, double delta_ref) {
        const std::size_t off_u = qp.off_u + k * nu;

        // 가중치 꺼내기 -> 대각 R
        const double r_ax = W.r_ax;
        const double r_delta = W.r_delta;

        out.H(off_u + 0, off_u + 0) += 2.0 * r_ax;
        out.H(off_u + 1, off_u + 1) += 2.0 * r_delta;

        // 입력 참조는 보통 0 (필요 시 넣을 수 있음). f += -2 R u_ref
        // 입력참조 보통 0 주므로 결국 f 행렬은 0이 되버린다
        out.f(off_u + 0) += -2.0 * r_ax * ax_ref;
        out.f(off_u + 1) += -2.0 * r_delta * delta_ref;
    };

    for (std::size_t k = 0; k < N; k++) {
        // 기본적으로 u_ref = 0
        put_input_quad(k, 0.0, 0.0);    // 입력이 작을수록 좋으므로 ref를 둘다 0으로 준다
    }

    // --------- 입력 변화율 가중치 Rd (Δu_k = u_k - u_{k-1}) ---------
    // JΔu = Σ ||Δu||^2_{Rd}. 1/2*z^T H z에서
    //  k>=1: u_k^T(2Rd)u_k + u_{k-1}^T(2Rd)u_{k-1} + u_k^T(-4Rd)u_{k-1}
    //  k=0, u_prev 제공 시: u_0^T(2Rd)u_0  +  f += -2*(2Rd)*u_prev/2 = -2*Rd*u_prev  (정확히는 전개하면 f 항에 -2*Rd*u_prev)
    
    // 가중치 
    const double rd_ax    = W.rd_ax;
    const double rd_delta = W.rd_delta;

    // H 행렬에 값 누적 대각항
    auto add_block_uu = [&](std::size_t ku, const Eigen::Matrix2d &W2) {
        const std::size_t off = qp.off_u + ku * nu;
        out.H.block(off, off, 2, 2) += W2;
    };

    // 교차항 넣기
    auto add_block_uukm = [&](std::size_t ku, std::size_t kum1, const Eigen::Matrix2d &W2) {
        const std::size_t offk  = qp.off_u + ku * nu;
        const std::size_t offkm = qp.off_u + kum1 * nu;
        out.H.block(offk, offkm, 2, 2) += W2;
        out.H.block(offkm, offk, 2, 2) += W2.transpose(); // 대칭
    };

    Eigen::Matrix2d Rd2 = (Eigen::Matrix2d() <<
                           2.0 * rd_ax, 0.0,
                           0.0, 2.0 * rd_delta).finished();
    Eigen::Matrix2d Neg4Rd_half = (Eigen::Matrix2d() <<
                                   -2.0 * rd_ax, 0.0,
                                   0.0, -2.0 * rd_delta).finished(); // 대칭 블록 한쪽 (전체로는 -4Rd가 두 블록에 분배됨)

    for (std::size_t k = 0; k < N; k++) {
        if (k == 0) {
            if (u_prev_opt) {
                // H(0,0) += 2*Rd
                add_block_uu(0, Rd2);
                // f += -2*Rd*u_prev
                const std::size_t off0 = qp.off_u + 0 * nu;
                out.f(off0 + 0) += -2.0 * rd_ax    * (*u_prev_opt)(0);
                out.f(off0 + 1) += -2.0 * rd_delta * (*u_prev_opt)(1);
            }
            // u_prev 없으면 k=0 항은 생략
        } 
        else {
            // k>=1: u_k^T 2Rd u_k, u_{k-1}^T 2Rd u_{k-1}, 교차 -4Rd는
            add_block_uu(k,   Rd2);
            add_block_uu(k-1, Rd2);
            add_block_uukm(k, k-1, Neg4Rd_half); // 위/아래 둘 다 채워서 총 -4Rd 효과
        }
    }

    // --------- 슬랙 벌점(소프트 제약) ---------
    // H에만 들어간다 -> 입실론의 제곱(제곱항)이므로 선형항은 없다
    // 장애물 슬랙: r_slack_obs * ||eps_obs||^2
    if (prob.slack.use_obstacle_slack) {
        const double rs = prob.weights.r_slack_obs;
        for (std::size_t k = 0; k < N; k++) {
            const std::size_t e = qp.off_eps_obs + k;
            out.H(e, e) += 2.0 * rs;
        }
    }

    // 마찰 슬랙: r_slack_friction * ||eps_fric||^2
    if (prob.slack.use_friction_slack) {
        const double rs = prob.weights.r_slack_friction;
        for (std::size_t k = 0; k < N; k++) {
            const std::size_t e = qp.off_eps_fric + k;
            out.H(e, e) += 2.0 * rs;
        }
    }

    return out;
}

#ifdef USE_OSQP

// Eigen Dense Matrix -> OSQP CSC 변환
// upper_triangular_only = true 이면 (r > j) 요소는 버림(상삼각만 사용)
OSQPCscMatrix* eigenToCSC(const Eigen::MatrixXd &M, bool upper_triangular_only) {
    const OSQPInt rows = static_cast<OSQPInt>(M.rows());
    const OSQPInt cols = static_cast<OSQPInt>(M.cols());

    std::vector<OSQPInt>    p(cols + 1, 0);
    std::vector<OSQPInt>    i;
    std::vector<OSQPFloat>  x;
    i.reserve(M.size());
    x.reserve(M.size());

    OSQPInt nnz = 0;
    for (OSQPInt j = 0; j < cols; ++j) {
        p[j] = nnz;
        for (OSQPInt r = 0; r < rows; ++r) {
            if (upper_triangular_only && r > j)
                continue;  // P는 상삼각만 사용

            const double v = M(r, j);
            if (std::abs(v) > 1e-12) {
                i.push_back(r);
                x.push_back(static_cast<OSQPFloat>(v));
                nnz++;
            }
        }
    }
    p[cols] = nnz;

    // OSQP가 쓸 실제 배열 (우리가 할당)
    OSQPFloat *x_data = (OSQPFloat*)std::malloc(sizeof(OSQPFloat) * nnz);
    OSQPInt *i_data = (OSQPInt*)std::malloc(sizeof(OSQPInt) * nnz);
    OSQPInt *p_data = (OSQPInt*)std::malloc(sizeof(OSQPInt) * (cols + 1));

    if (!x_data || !i_data || !p_data) {
        if (x_data) std::free(x_data);
        if (i_data) std::free(i_data);
        if (p_data) std::free(p_data);

        return OSQP_NULL;
    }

    std::copy(x.begin(), x.end(), x_data);
    std::copy(i.begin(), i.end(), i_data);
    std::copy(p.begin(), p.end(), p_data);

    // CSC 구조체 생성 (OSQP가 이 구조체와 내부 배열을 관리)
    OSQPCscMatrix *mat = OSQPCscMatrix_new(rows, cols, nnz, x_data, i_data, p_data);
    if (!mat) {
        std::free(x_data);
        std::free(i_data);
        std::free(p_data);
        return OSQP_NULL;
    }
    return mat;
}

// OSQP CSC 행렬 해제 헬퍼
void freeCSC(OSQPCscMatrix *M) {
    if (!M) return;
    OSQPCscMatrix_free(M);   // OSQP가 x, i, p까지 같이 free해줌
}

// 실제 OSQP를 호출하는 solver
bool solveWithOSQP(const Eigen::MatrixXd &H, const Eigen::VectorXd &f, const Eigen::MatrixXd &Aeq, const Eigen::VectorXd &beq, const Eigen::MatrixXd &Aineq, const Eigen::VectorXd &bineq, const Eigen::VectorXd &lb, const Eigen::VectorXd &ub, Eigen::VectorXd &z, const SolverOptions &opt) {
    // OSQP 표준형: min 1/2 z^T P z + q^T z
    //              l <= A z <= u
    const int n = static_cast<int>(H.rows());
    if (H.rows() != H.cols() || f.size() != n) {
        return false;
    }

    const int me = static_cast<int>(Aeq.rows());      // 등식 제약
    const int mi = static_cast<int>(Aineq.rows());    // 부등식 제약
    const int mb = 2 * n;                             // 박스 제약 (상/하한)

    const int m = me + mi + mb;

    Eigen::MatrixXd A(m, n);
    Eigen::VectorXd l(m);
    Eigen::VectorXd u(m);

    // 1) 등식: Aeq z = beq  → l = u = beq
    if (me > 0) {
        A.block(0, 0, me, n) = Aeq;
        l.segment(0, me) = beq;
        u.segment(0, me) = beq;
    }

    // 2) 부등식: Aineq z <= bineq  →  l = -inf, u = bineq
    if (mi > 0) {
        A.block(me, 0, mi, n) = Aineq;
        l.segment(me, mi).setConstant(-OSQP_INFTY);
        u.segment(me, mi) = bineq;
    }

    // 3) 상한:  I z <= ub → l = -inf, u = ub
    A.block(me + mi, 0, n, n) = Eigen::MatrixXd::Identity(n, n);
    l.segment(me + mi, n).setConstant(-OSQP_INFTY);
    u.segment(me + mi, n) = ub;

    // 4) 하한: -I z <= -lb → l = -inf, u = -lb
    A.block(me + mi + n, 0, n, n) = -Eigen::MatrixXd::Identity(n, n);
    l.segment(me + mi + n, n).setConstant(-OSQP_INFTY);
    u.segment(me + mi + n, n) = -lb;

    // H는 대칭으로 맞춰서 P 구성
    Eigen::MatrixXd Pmat = 0.5 * (H + H.transpose());

    // Eigen → CSC (Pmat 사용!)
    OSQPCscMatrix *P     = eigenToCSC(Pmat, /*upper_triangular_only=*/true);
    OSQPCscMatrix *A_csc = eigenToCSC(A,    /*upper_triangular_only=*/false);


    if (!P || !A_csc) {
        if (P)     freeCSC(P);
        if (A_csc) freeCSC(A_csc);
        return false;
    }

    const OSQPInt n_osqp = static_cast<OSQPInt>(n);
    const OSQPInt m_osqp = static_cast<OSQPInt>(m);

    // q, l, u 배열 할당
    OSQPFloat *q     = static_cast<OSQPFloat*>(std::malloc(sizeof(OSQPFloat) * n_osqp));
    OSQPFloat *l_arr = static_cast<OSQPFloat*>(std::malloc(sizeof(OSQPFloat) * m_osqp));
    OSQPFloat *u_arr = static_cast<OSQPFloat*>(std::malloc(sizeof(OSQPFloat) * m_osqp));

    if (!q || !l_arr || !u_arr) {
        if (q)     std::free(q);
        if (l_arr) std::free(l_arr);
        if (u_arr) std::free(u_arr);
        OSQPCscMatrix_free(P);
        OSQPCscMatrix_free(A_csc);
        return false;
    }

    for (OSQPInt i = 0; i < n_osqp; i++) {
        q[i] = static_cast<OSQPFloat>(f(i));
    }

    for (OSQPInt i = 0; i < m_osqp; i++) {
        l_arr[i] = static_cast<OSQPFloat>(l(i));
        u_arr[i] = static_cast<OSQPFloat>(u(i));
    }

    // 설정 구조체 생성 및 기본값 설정
    OSQPSettings *settings = OSQPSettings_new();
    if (!settings) {
        std::free(q);
        std::free(l_arr);
        std::free(u_arr);
        OSQPCscMatrix_free(P);
        OSQPCscMatrix_free(A_csc);
        return false;
    }

    osqp_set_default_settings(settings);

    // 옵션 적용
    settings->verbose  = opt.verbose ? 1 : 0;
    settings->max_iter = opt.max_iter;
    settings->eps_abs  = opt.tol;
    settings->eps_rel  = opt.tol;
    settings->alpha    = 1.6;

    // 솔버 생성 및 문제 셋업
    OSQPSolver *solver = OSQP_NULL;
    OSQPInt exitflag = osqp_setup(&solver,
                                  P,
                                  q,
                                  A_csc,
                                  l_arr,
                                  u_arr,
                                  m_osqp,
                                  n_osqp,
                                  settings);
    if (exitflag != 0 || !solver) {
        std::cerr << "[OSQP] setup failed. exitflag=" << exitflag << std::endl;
        OSQPSettings_free(settings);
        std::free(q);
        std::free(l_arr);
        std::free(u_arr);
        freeCSC(P);
        freeCSC(A_csc);
        return false;
    }

    // 실제 QP 풀기
    exitflag = osqp_solve(solver);

    // 디버그 출력
    if (solver && solver->info) {
        std::cerr << "[OSQP] solve exitflag=" << exitflag
                << ", status_val=" << solver->info->status_val
                << ", status=" << (solver->info->status ? solver->info->status : "null")
                << ", iter=" << solver->info->iter << std::endl;
    }

    OSQPInt st = solver->info ? solver->info->status_val : (OSQPInt)OSQP_UNSOLVED;

    bool ok = false;

    bool acceptable =
        (st == OSQP_SOLVED ||
         st == OSQP_SOLVED_INACCURATE ||
         st == OSQP_MAX_ITER_REACHED);  // ★ 이 줄 추가

    if (exitflag == 0 && acceptable && solver->solution && solver->solution->x) {
        if (z.size() != n) z.resize(n);
        for (int i = 0; i < n; i++) {
            z(i) = static_cast<double>(solver->solution->x[i]);
        }
        ok = true;
    }

    // 정리
    osqp_cleanup(solver);
    OSQPSettings_free(settings);
    std::free(q);
    std::free(l_arr);
    std::free(u_arr);
    OSQPCscMatrix_free(P);
    OSQPCscMatrix_free(A_csc);

    return ok;
}

#else  // USE_OSQP 가 정의되지 않은 빌드: 더미 구현

bool solveWithOSQP(const Eigen::MatrixXd &H, const Eigen::VectorXd &f, const Eigen::MatrixXd &Aeq, const Eigen::VectorXd &beq, const Eigen::MatrixXd &Aineq, const Eigen::VectorXd &bineq, const Eigen::VectorXd &lb, const Eigen::VectorXd &ub, Eigen::VectorXd &z, const SolverOptions &opt) {
    (void)H; (void)f;
    (void)Aeq; (void)beq;
    (void)Aineq; (void)bineq;
    (void)lb; (void)ub;
    (void)z; (void)opt;
    return false;
}
#endif  // USE_OSQP

void unpackSolution(const mpc::BuiltQP &qp, const Eigen::VectorXd &z, MPCResult &out) {
    const std::size_t N = qp.N;
    const std::size_t nx = qp.nx;
    const std::size_t nu = qp.nu;

    out.x_pred.resize(N + 1);
    out.u_pred.resize(N);

    // x
    for (std::size_t k = 0; k <= N; k++) {
        const std::size_t off = qp.off_x + mpc::xOffset(k, nx);
        State xs{};
        xs.ey = z(off + 0);
        xs.epsi = z(off + 1);
        xs.s = z(off + 2);
        xs.vx = z(off + 3);
        xs.wz = (nx > 4 ? z(off + 4) : 0.0);
        out.x_pred[k] = xs;
    }
    // u
    for (std::size_t k = 0; k < N; k++) {
        const std::size_t off = qp.off_u + k * nu;
        Control u{};
        u.ax = z(off + 0);
        u.delta = z(off + 1);
        out.u_pred[k] = u;
    }
    // 즉시 적용 입력
    if (!out.u_pred.empty())
        out.u0 = out.u_pred.front();
}

}   // namespace anonymous
namespace mpc {

MPCResult run_local_mpc(const MpcProblem &prob, const State &x_curr, const Eigen::Vector2d *u_prev_opt, const SolverOptions &opt) {
    MPCResult res; 

    // 1) 제약 만들기 (Aeq, Aineq, lb, ub 등)
    mpc::IndexMap idx; // 기본 인덱스
    Eigen::VectorXd x0(prob.ltv.nx);
    // 상태 순서: [ey, epsi, s, vx, wz]
    x0(0) = x_curr.ey; 
    x0(1) = x_curr.epsi; 
    x0(2) = x_curr.s; 
    x0(3) = x_curr.vx;
    if (prob.ltv.nx > 4) x0(4) = x_curr.wz;

    const auto qp = mpc::buildConstraints(prob, x0, idx, /*use_small_angle*/true, u_prev_opt);

    // 2) 비용(H, f) 만들기
    const auto HF = buildCostHF(prob, qp, idx, u_prev_opt);

    // 3) QP 풀기
    Eigen::VectorXd z = Eigen::VectorXd::Zero(qp.nvar); // 해
#ifdef USE_OSQP
    {
        const bool ok = solveWithOSQP(HF.H, HF.f,
                                      qp.Aeq, qp.beq,
                                      qp.Aineq, qp.bineq,
                                      qp.lb, qp.ub,
                                      z, opt);
        if (!ok) {
            res.success = false;
            return res;
        }
    }
#else
    //솔버 미연결: 성공 false
    (void)HF;
    (void)z;
    res.success = false;
    return res;
#endif

    // 4) 해 언패킹
    unpackSolution(qp, z, res);
    res.success = true;

    // 5) 최적 비용(대략 계산: 1/2 z^T H z + f^T z)
    res.optimal_cost = 0.5 * z.dot(HF.H * z) + HF.f.dot(z);

    return res;
}

}   // namespace mpc

MPCResult solve_local_mpc(const MpcProblem &prob, const State &x_curr, const std::vector<Control> *warm_u, const SolverOptions &opt) {
    Eigen::Vector2d u_prev;
    const Eigen::Vector2d *u_prev_ptr = nullptr;
    if (warm_u && !warm_u->empty()) {
        u_prev << (*warm_u)[0].ax, (*warm_u)[0].delta;
        u_prev_ptr = &u_prev;
    }

    return mpc::run_local_mpc(prob, x_curr, u_prev_ptr, opt);
}

}   // namespace local_planner
