/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: NMPC_Path_Tracking.c
 *
 * Code generated for Simulink model 'NMPC_Path_Tracking'.
 *
 * Model version                  : 8.13
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Mon Feb 17 13:01:51 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: AMD->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "NMPC_Path_Tracking.h"
#include "rtwtypes.h"
#include "NMPC_Path_Tracking_types.h"
#include <string.h>
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>
#include "NMPC_Path_Tracking_private.h"
#include "rt_defines.h"

/* Block signals (default storage) */
B_NMPC_Path_Tracking_T NMPC_Path_Tracking_B;

/* Block states (default storage) */
DW_NMPC_Path_Tracking_T NMPC_Path_Tracking_DW;

/* External inputs (root inport signals with default storage) */
ExtU_NMPC_Path_Tracking_T NMPC_Path_Tracking_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_NMPC_Path_Tracking_T NMPC_Path_Tracking_Y;

/* Real-time model */
static RT_MODEL_NMPC_Path_Tracking_T NMPC_Path_Tracking_M_;
RT_MODEL_NMPC_Path_Tracking_T *const NMPC_Path_Tracking_M =
  &NMPC_Path_Tracking_M_;

/* Forward declaration for local functions */
static void NMPC_Path_T_generateRuntimeData(const real_T x[6], const real_T
  lastMV[2], const real_T ref0[60], const real_T X0[294], const real_T MV0[98],
  real_T Slack0, s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T *runtimedata,
  s02qEHDaRbOUcZF4Nctxs8G_NMPC__T *userdata, real_T z0[401]);
static void NMPC_Path_Tracking_getZBounds(const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T *
  runtimedata, real_T zLB[401], real_T zUB[401]);
static void NMPC_Path_Tracking_isfinite(const real_T x[2], boolean_T b[2]);
static void NMPC_Path_Tracking_mtimes(const real_T A_data[], const int32_T
  A_size[2], real_T C_data[], int32_T C_size[2]);
static void NMPC_Path_Tracking_getUBounds(const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T *
  runtimedata, real_T A_data[], int32_T A_size[2], real_T Bu_data[], int32_T
  *Bu_size);
static void NMPC_Path_Tracking_getXUe(const real_T z[401], const real_T x[6],
  real_T X[306], real_T U[102], real_T *e);
static void NMPC_Path_T_VehicleStateJacFcn1(const real_T x[6], const real_T u[2],
  real_T A[36], real_T B[12]);
static void NMPC_Path_StateFunctionVehicle1(const real_T x[6], const real_T u[2],
  real_T dxdt[6]);
static void NMPC_Path_Tracki_stateEvolution(const real_T X[306], const real_T U
  [102], real_T c[300], real_T J[120300]);
static void NMPC_Path_Tracking_all(const boolean_T x[300], boolean_T y[6]);
static void NMPC_Path_Tracking_isfinite_p(const real_T x[6], boolean_T b[6]);
static boolean_T NMPC_Path_Tracking_any(const boolean_T x[12]);
static void NMPC_Path_Tracki_reformJacobian(const real_T Jx_data[], const
  int32_T Jx_size[3], const real_T Jmv_data[], const real_T Je_data[], const
  int32_T *Je_size, real_T Jc_data[], int32_T Jc_size[2]);
static void NMPC_Path_Tracking_outputBounds(const real_T runtimedata_OutputMin
  [300], const real_T runtimedata_OutputMax[300], const real_T X[306], real_T e,
  real_T c_data[], int32_T c_size[2], real_T Jc_data[], int32_T Jc_size[2]);
static void NMPC_Path_Tr_c4_mpclib_anonFcn2(const real_T runtimedata_x[6], const
  real_T runtimedata_OutputMin[300], const real_T runtimedata_OutputMax[300],
  const real_T z[401], real_T varargout_1_data[], int32_T varargout_1_size[2],
  real_T varargout_2[300], real_T varargout_3_data[], int32_T varargout_3_size[2],
  real_T varargout_4[120300]);
static void NMPC_Path_Trac_factoryConstruct(int32_T nVarMax, int32_T mConstrMax,
  int32_T mIneq, int32_T mNonlinIneq, s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *obj);
static void NMPC_Path_factoryConstruct_gzst(int32_T MaxVars, int32_T
  *obj_grad_size, int32_T *obj_Hx_size, boolean_T *obj_hasLinear, int32_T
  *obj_nvar, int32_T *obj_maxVar, real_T *obj_beta, real_T *obj_rho, int32_T
  *obj_objtype, int32_T *obj_prev_objtype, int32_T *obj_prev_nvar, boolean_T
  *obj_prev_hasLinear, real_T *obj_gammaScalar);
static void NMPC_Pat_factoryConstruct_gzstv(int32_T mIneqMax, int32_T nVarMax,
  int32_T mConstrMax, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj);
static void computeObjectiveAndUserGradient(const
  s_UduwGwOpHSPENrm1X2xchE_NMPC_T *obj_next_next_next_next_next_ne, const real_T
  x[401], real_T grad_workspace_data[], real_T *fval, int32_T *status);
static int32_T NMPC_Path__checkVectorNonFinite(int32_T N, const real_T vec_data[],
  int32_T iv0);
static int32_T NMPC_Pat_checkVectorNonFinite_l(const real_T vec[300]);
static int32_T computeConstraintsAndUserJacobi(int32_T
  obj_next_next_next_next_next_b_, const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T
  *obj_next_next_next_next_next_ne, const real_T x[401], real_T
  Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[300], real_T
  JacIneqTrans_workspace_data[], int32_T iJI_col, int32_T ldJI, real_T
  JacEqTrans_workspace_data[], int32_T ldJE);
static void evalObjAndConstrAndDerivatives(int32_T
  obj_next_next_next_next_next_b_, const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T
  *obj_next_next_next_next_next_ne, const s_UduwGwOpHSPENrm1X2xchE_NMPC_T
  *obj_next_next_next_next_next__0, const real_T x[401], real_T
  grad_workspace_data[], real_T Cineq_workspace_data[], int32_T ineq0, real_T
  Ceq_workspace[300], real_T JacIneqTrans_workspace_data[], int32_T iJI_col,
  int32_T ldJI, real_T JacEqTrans_workspace_data[], int32_T ldJE, real_T *fval,
  int32_T *status);
static void NMPC_Pa_modifyOverheadPhaseOne_(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj);
static void NMPC_Path_Tracki_setProblemType(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj,
  int32_T PROBLEM_TYPE);
static void NMPC_Path_Trackin_initActiveSet(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj);
static void NMPC_Path_T_factoryConstruct_gz(int32_T maxRows, int32_T maxCols,
  int32_T *obj_ldq, int32_T obj_QR_size[2], real_T obj_Q_data[], int32_T
  obj_Q_size[2], int32_T obj_jpvt_data[], int32_T *obj_jpvt_size, int32_T
  *obj_mrows, int32_T *obj_ncols, int32_T *obj_tau_size, int32_T *obj_minRowCol,
  boolean_T *obj_usedPivoting);
static void NMPC_Path__factoryConstruct_gzs(int32_T MaxDims, int32_T
  obj_FMat_size[2], int32_T *obj_ldm, int32_T *obj_ndims, int32_T *obj_info,
  real_T *obj_scaleFactor, boolean_T *obj_ConvexCheck, real_T *obj_regTol_,
  real_T *obj_workspace_, real_T *obj_workspace2_);
static void NMPC_Path_Tracki_computeGradLag(real_T workspace_data[], int32_T ldA,
  int32_T nVar, const real_T grad_data[], int32_T mIneq, const real_T
  AineqTrans_data[], const real_T AeqTrans_data[], const int32_T
  finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[], int32_T mLB,
  const int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[]);
static real_T NMPC_Pat_computePrimalFeasError(const real_T x[401], int32_T
  mLinIneq, int32_T mNonlinIneq, const real_T cIneq_data[], const real_T cEq[300],
  const int32_T finiteLB_data[], int32_T mLB, const real_T lb[401], const
  int32_T finiteUB_data[], int32_T mUB);
static void NMPC_Path__computeDualFeasError(int32_T nVar, const real_T
  gradLag_data[], boolean_T *gradOK, real_T *val);
static void NMPC_Path_Tracking_test_exit(sG8JZ69axY52WWR6RKyApQC_NMPC__T
  *MeritFunction, const s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *WorkingSet,
  s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *TrialState, const real_T lb[401], boolean_T
  *Flags_gradOK, boolean_T *Flags_fevalOK, boolean_T *Flags_done, boolean_T
  *Flags_stepAccepted, boolean_T *Flags_failedLineSearch, int32_T
  *Flags_stepType);
static void NMPC_Path_Tracking_saveJacobian(s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *obj,
  int32_T nVar, int32_T mIneq, const real_T JacCineqTrans_data[], int32_T
  ineqCol0, const real_T JacCeqTrans_data[], int32_T ldJ);
static real_T NMPC_Path_Tra_computeComplError(const int32_T
  *fscales_lineq_constraint_size, const int32_T *fscales_cineq_constraint_size,
  const real_T xCurrent[401], int32_T mIneq, const real_T cIneq_data[], const
  int32_T finiteLB_data[], int32_T mLB, const real_T lb[401], const int32_T
  finiteUB_data[], int32_T mUB, const real_T lambda_data[], int32_T iL0);
static void NMPC_Path_Trac_computeGradLag_h(real_T workspace_data[], int32_T ldA,
  int32_T nVar, const real_T grad_data[], int32_T mIneq, const real_T
  AineqTrans_data[], const real_T AeqTrans_data[], const int32_T
  finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[], int32_T mLB,
  const int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[]);
static void NMPC_Pat_computeDualFeasError_b(int32_T nVar, const real_T
  gradLag_data[], boolean_T *gradOK, real_T *val);
static void NMPC_P_updateWorkingSetForNewQP(const real_T xk[401],
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *WorkingSet, int32_T mIneq, int32_T
  mNonlinIneq, const real_T cIneq_data[], const real_T cEq[300], int32_T mLB,
  const real_T lb[401], int32_T mUB, int32_T mFixed);
static void NMPC_Path_Tracking_xswap(int32_T n, real_T x_data[], int32_T ix0,
  int32_T iy0);
static real_T NMPC_Path_Tracking_xnrm2(int32_T n, const real_T x_data[], int32_T
  ix0);
static real_T NMPC_Path_Tracking_xzlarfg(int32_T n, real_T *alpha1, real_T
  x_data[], int32_T ix0);
static void NMPC_Path_Tracking_xgemv(int32_T m, int32_T n, const real_T A_data[],
  int32_T ia0, int32_T lda, const real_T x_data[], int32_T ix0, real_T y_data[]);
static void NMPC_Path_Tracking_xgerc(int32_T m, int32_T n, real_T alpha1,
  int32_T ix0, const real_T y_data[], real_T A_data[], int32_T ia0, int32_T lda);
static void NMPC_Path_Tracking_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C_data[], int32_T ic0, int32_T ldc, real_T work_data[]);
static void NMPC_Path_Tracking_qrf(real_T A_data[], const int32_T A_size[2],
  int32_T m, int32_T n, int32_T nfxd, real_T tau_data[]);
static void NMPC_Path_Tracking_qrpf(real_T A_data[], const int32_T A_size[2],
  int32_T m, int32_T n, int32_T nfxd, real_T tau_data[], int32_T jpvt_data[]);
static void NMPC_Path_Tracking_xgeqp3(real_T A_data[], const int32_T A_size[2],
  int32_T m, int32_T n, int32_T jpvt_data[], real_T tau_data[], int32_T
  *tau_size);
static void NMPC_Path_Tracking_factorQRE(s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *obj,
  const real_T A_data[], int32_T mrows, int32_T ncols, int32_T ldA);
static void NMPC_Path_Tracking_xorgqr(int32_T m, int32_T n, int32_T k, real_T
  A_data[], const int32_T A_size[2], int32_T lda, const real_T tau_data[]);
static void NMPC_Path_Tracking_sortLambdaQP(real_T lambda_data[], int32_T
  WorkingSet_nActiveConstr, const int32_T WorkingSet_sizes[5], const int32_T
  WorkingSet_isActiveIdx[6], const int32_T WorkingSet_Wid_data[], const int32_T
  WorkingSet_Wlocalidx_data[], real_T workspace_data[]);
static void NMPC_Path_Tracking_test_exit_o(s7RdrPWkr8UPAUyTdDJkLaG_NMPC__T
  *Flags, s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace,
  sG8JZ69axY52WWR6RKyApQC_NMPC__T *MeritFunction, const int32_T
  *fscales_lineq_constraint_size, const int32_T *fscales_cineq_constraint_size,
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *WorkingSet, s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T
  *TrialState, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager, const real_T lb[401]);
static boolean_T NMPC_Path_Tracking_BFGSUpdate(int32_T nvar, real_T Bk[160801],
  const real_T sk_data[], real_T yk_data[], real_T workspace_data[]);
static void NMPC_Path_Tracking_factorQRE_d(s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *obj,
  int32_T mrows, int32_T ncols);
static void NMPC_Path_Tracking_countsort(int32_T x_data[], int32_T xLen, int32_T
  workspace_data[], int32_T xMin, int32_T xMax);
static void NMPC_Path_Tracking_removeConstr(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj,
  int32_T idx_global);
static int32_T NMPC_Path_Tr_RemoveDependentEq_(s_BcTDS8pFolHdhtDibl2TnF_NMPC_T
  *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *workingset,
  s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager);
static void NMPC_Path__RemoveDependentIneq_(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace);
static int32_T NMPC_Path_Tracking_rank(const real_T qrmanager_QR_data[], const
  int32_T qrmanager_QR_size[2], int32_T qrmanager_mrows, int32_T qrmanager_ncols);
static void NMPC_Path_Tracking_xgemv_c(int32_T m, int32_T n, const real_T
  A_data[], int32_T lda, const real_T x_data[], real_T y_data[]);
static real_T NMPC_P_maxConstraintViolation_l(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *obj, const real_T x_data[]);
static void NMPC_Path_Tracking_xgemv_cp(int32_T m, int32_T n, const real_T
  A_data[], int32_T lda, const real_T x_data[], int32_T ix0, real_T y_data[]);
static real_T NMPC__maxConstraintViolation_ln(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *obj, const real_T x_data[], int32_T ix0);
static boolean_T NMPC_Pa_feasibleX0ForWorkingSet(real_T workspace_data[], const
  int32_T workspace_size[2], real_T xCurrent_data[],
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T
  *qrmanager);
static void NMPC_Pat_RemoveDependentIneq__h(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace);
static void NMPC_Path_Tracking_xgemv_cpg(int32_T m, int32_T n, const real_T
  A_data[], int32_T lda, const real_T x_data[], real_T y_data[]);
static real_T maxConstraintViolation_AMats_no(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *obj, const real_T x_data[]);
static real_T maxConstraintViolation_AMats_re(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *obj, const real_T x_data[]);
static real_T NMPC_maxConstraintViolation_lng(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *obj, const real_T x_data[]);
static void NMPC_Path_Tr_PresolveWorkingSet(s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T
  *solution, s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace,
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T
  *qrmanager);
static void NMPC_Path_Tracking_xgemv_cpge(int32_T m, int32_T n, const real_T A
  [160801], int32_T lda, const real_T x_data[], real_T y_data[]);
static void NMPC_Path_T_computeGrad_StoreHx(s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *obj,
  const real_T H[160801], const real_T f_data[], const real_T x_data[]);
static real_T NMPC_Path_T_computeFval_ReuseHx(const
  s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *obj, real_T workspace_data[], const real_T
  f_data[], const real_T x_data[]);
static void NMPC_Path_Tracking_xgeqrf(real_T A_data[], const int32_T A_size[2],
  int32_T m, int32_T n, real_T tau_data[], int32_T *tau_size);
static void NMPC_Path_Tracking_factorQR(s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *obj,
  const real_T A_data[], int32_T mrows, int32_T ncols, int32_T ldA);
static void NMPC_Path_Tracking_xrotg(real_T *a, real_T *b, real_T *c, real_T *s);
static void NMPC_Path_Tra_squareQ_appendCol(s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *obj,
  const real_T vec_data[], int32_T iv0);
static void NMPC_Path_Trac_deleteColMoveEnd(s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *obj,
  int32_T idx);
static boolean_T NMPC_Path_Tracking_strcmp(const char_T a[7]);
static void NMPC_Path_Tracking_xgemm(int32_T m, int32_T n, int32_T k, const
  real_T A[160801], int32_T lda, const real_T B_data[], int32_T ib0, int32_T ldb,
  real_T C_data[], int32_T ldc);
static void NMPC_Path_Tracking_xgemm_o(int32_T m, int32_T n, int32_T k, const
  real_T A_data[], int32_T ia0, int32_T lda, const real_T B_data[], int32_T ldb,
  real_T C_data[], int32_T ldc);
static void NMPC_Path_Tracking_fullColLDL2_(s_LuoN3prQfsei9XMpifhsFF_NMPC_T *obj,
  int32_T LD_offset, int32_T NColsRemain);
static void NMPC_Path_Track_partialColLDL3_(s_LuoN3prQfsei9XMpifhsFF_NMPC_T *obj,
  int32_T LD_offset, int32_T NColsRemain);
static int32_T NMPC_Path_Tracking_xpotrf(int32_T n, real_T A_data[], int32_T lda);
static void NMPC_Path_Tracking_xgemv_cpgex(int32_T m, int32_T n, const real_T
  A_data[], int32_T ia0, int32_T lda, const real_T x_data[], real_T y_data[]);
static void NMPC_Path_Tracking_factor_l(s_LuoN3prQfsei9XMpifhsFF_NMPC_T *obj,
  const real_T A[160801], int32_T ndims, int32_T ldA);
static void NMPC_Path_Tracking_factor(s_LuoN3prQfsei9XMpifhsFF_NMPC_T *obj,
  const real_T A[160801], int32_T ndims, int32_T ldA);
static void NMPC_Path_Tracking_solve_i(const s_LuoN3prQfsei9XMpifhsFF_NMPC_T
  *obj, real_T rhs_data[]);
static void NMPC_Path_Tracking_solve(const s_LuoN3prQfsei9XMpifhsFF_NMPC_T *obj,
  real_T rhs_data[]);
static void NMPC_Path_Tracki_compute_deltax(const real_T H[160801],
  s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution, s_BcTDS8pFolHdhtDibl2TnF_NMPC_T
  *memspace, const s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *cholmanager, const
  s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *objective, boolean_T alwaysPositiveDef);
static real_T NMPC_Path_Tracking_xnrm2_o(int32_T n, const real_T x_data[]);
static void NMPC_Path_Tracking_xgemv_cpgexe(int32_T m, int32_T n, const real_T
  A_data[], int32_T lda, const real_T x_data[], real_T y_data[]);
static void NMPC_Path_Tra_feasibleratiotest(const real_T solution_xstar_data[],
  const real_T solution_searchDir_data[], real_T workspace_data[], const int32_T
  workspace_size[2], int32_T workingset_nVar, int32_T workingset_ldA, const
  real_T workingset_Aineq_data[], const real_T workingset_bineq_data[], const
  real_T workingset_lb_data[], const int32_T workingset_indexLB_data[], const
  int32_T workingset_sizes[5], const int32_T workingset_isActiveIdx[6], const
  boolean_T workingset_isActiveConstr_data[], const int32_T workingset_nWConstr
  [5], boolean_T isPhaseOne, real_T *alpha, boolean_T *newBlocking, int32_T
  *constrType, int32_T *constrIdx);
static void NMPC_P_checkUnboundedOrIllPosed(s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T
  *solution, const s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *objective);
static void NMPC_addBoundToActiveSetMatrix_(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj,
  int32_T TYPE, int32_T idx_local);
static void NMPC_Path_Tracki_addAineqConstr(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj,
  int32_T idx_local);
static void NMPC_Path_Tracki_compute_lambda(real_T workspace_data[],
  s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution, const
  s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *objective, const
  s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager);
static void NM_checkStoppingAndUpdateFval_l(int32_T *activeSetChangeID,
  s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution, s_BcTDS8pFolHdhtDibl2TnF_NMPC_T
  *memspace, const s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *objective,
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T
  *qrmanager, int32_T runTimeOptions_MaxIterations, boolean_T *updateFval);
static void NMPC_Path_Tracking_iterate_d(const real_T H[160801], const real_T
  f_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *cholmanager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  objective, const char_T options_SolverName[7], int32_T
  runTimeOptions_MaxIterations);
static void NMPC_checkStoppingAndUpdateFval(int32_T *activeSetChangeID, const
  real_T f_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, const
  s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *objective, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager, int32_T
  runTimeOptions_MaxIterations, const boolean_T *updateFval);
static void NMPC_Path_Tracking_iterate(const real_T H[160801], const real_T
  f_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *cholmanager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  objective, const char_T options_SolverName[7], int32_T
  runTimeOptions_MaxIterations);
static void NMPC_Path_Tracking_phaseone(const real_T H[160801], const real_T
  f_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *cholmanager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  objective, const char_T options_SolverName[7], const
  somzaGboVhDG7PNQS6E98jD_NMPC__T *runTimeOptions);
static void NMPC_Path_Tracking_linearForm_(boolean_T obj_hasLinear, int32_T
  obj_nvar, real_T workspace_data[], const real_T H[160801], const real_T
  f_data[], const real_T x_data[]);
static void NMPC_Path_Tracking_driver_n(const real_T H[160801], const real_T
  f_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *cholmanager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  objective, const somzaGboVhDG7PNQS6E98jD_NMPC__T *options, const
  somzaGboVhDG7PNQS6E98jD_NMPC__T *runTimeOptions);
static void NMPC_Path_Tracking_addAeqConstr(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj,
  int32_T idx_local);
static boolean_T NMPC_Path_Tracking_soc(const real_T Hessian[160801], const
  real_T grad_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *TrialState,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *WorkingSet, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *CholManager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  QPObjective, const somzaGboVhDG7PNQS6E98jD_NMPC__T *qpoptions);
static real_T NMPC_Pat_maxConstraintViolation(const
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj, const real_T x_data[]);
static void NMPC_Path_Tracking_normal(const real_T Hessian[160801], const real_T
  grad_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *TrialState,
  sG8JZ69axY52WWR6RKyApQC_NMPC__T *MeritFunction,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *WorkingSet, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *CholManager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  QPObjective, const somzaGboVhDG7PNQS6E98jD_NMPC__T *qpoptions,
  s7RdrPWkr8UPAUyTdDJkLaG_NMPC__T *stepFlags);
static void NMPC_Path_Tracking_relaxed(const real_T Hessian[160801], const
  real_T grad_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *TrialState,
  sG8JZ69axY52WWR6RKyApQC_NMPC__T *MeritFunction,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *WorkingSet, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *CholManager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  QPObjective, somzaGboVhDG7PNQS6E98jD_NMPC__T *qpoptions);
static void NMPC_Path_Tracking_step_c(s7RdrPWkr8UPAUyTdDJkLaG_NMPC__T *stepFlags,
  real_T Hessian[160801], const real_T lb[401], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *
  TrialState, sG8JZ69axY52WWR6RKyApQC_NMPC__T *MeritFunction,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *WorkingSet, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *CholManager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  QPObjective, somzaGboVhDG7PNQS6E98jD_NMPC__T *qpoptions);
static void NMPC_Path_Tracki_outputBounds_c(const real_T runtimedata_OutputMin
  [300], const real_T runtimedata_OutputMax[300], const real_T X[306], real_T e,
  real_T c_data[], int32_T c_size[2]);
static void NMPC_Path_Trac_stateEvolution_d(const real_T X[306], const real_T U
  [102], real_T c[300]);
static void NMPC_Path__c4_mpclib_anonFcn2_d(const real_T runtimedata_x[6], const
  real_T runtimedata_OutputMin[300], const real_T runtimedata_OutputMax[300],
  const real_T z[401], real_T varargout_1_data[], int32_T varargout_1_size[2],
  real_T varargout_2[300]);
static void NMPC_Path_Trac_evalObjAndConstr(int32_T
  obj_next_next_next_next_next_b_, const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T
  *obj_next_next_next_next_next_ne, const s_UduwGwOpHSPENrm1X2xchE_NMPC_T
  *obj_next_next_next_next_next__0, const real_T x[401], real_T
  Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[300], real_T *fval,
  int32_T *status);
static void NMPC_Pat_computeLinearResiduals(const real_T x[401], int32_T nVar,
  real_T workspaceIneq_data[], const int32_T *workspaceIneq_size, int32_T
  mLinIneq, const real_T AineqT_data[], const real_T bineq_data[], int32_T ldAi);
static real_T NMPC_Path_Track_computeMeritFcn(real_T obj_penaltyParam, real_T
  fval, const real_T Cineq_workspace_data[], int32_T mIneq, const real_T
  Ceq_workspace[300], boolean_T evalWellDefined);
static void NMPC_Path_Tracking_linesearch(boolean_T *evalWellDefined, const
  real_T bineq_data[], int32_T WorkingSet_nVar, int32_T WorkingSet_ldA, const
  real_T WorkingSet_Aineq_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *TrialState,
  real_T MeritFunction_penaltyParam, real_T MeritFunction_phi, real_T
  MeritFunction_phiPrimePlus, real_T MeritFunction_phiFullStep, int32_T
  FcnEvaluator_next_next_next_nex, const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T
  *FcnEvaluator_next_next_next_n_0, const s_UduwGwOpHSPENrm1X2xchE_NMPC_T
  *FcnEvaluator_next_next_next_n_1, boolean_T socTaken, real_T *alpha, int32_T
  *exitflag);
static void NMPC_Path_Tracking_driver(real_T Hessian[160801], const real_T
  bineq_data[], const real_T lb[401], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T
  *TrialState, sG8JZ69axY52WWR6RKyApQC_NMPC__T *MeritFunction, const
  coder_internal_stickyStruct_2_T *FcnEvaluator, s_BcTDS8pFolHdhtDibl2TnF_NMPC_T
  *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *WorkingSet,
  s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager, s_LuoN3prQfsei9XMpifhsFF_NMPC_T
  *CholManager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *QPObjective, const int32_T
  *fscales_lineq_constraint_size, const int32_T *fscales_cineq_constraint_size);
static void NMPC_Path_Tracking_fmincon(const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T
  *fun_workspace_runtimedata, const s02qEHDaRbOUcZF4Nctxs8G_NMPC__T
  *fun_workspace_userdata, const real_T x0[401], const real_T Aineq_data[],
  const real_T bineq_data[], const int32_T *bineq_size, const real_T lb[401],
  const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T *nonlcon_workspace_runtimedata, real_T
  x[401], real_T *fval, real_T *exitflag, sttYSJM5GCi2c1Eu0R50efC_NMPC__T
  *output);
static void NMPC_Path_Tracking_computeInfo(const real_T X[306], const real_T U
  [102], real_T e, real_T cost, real_T ExitFlag, real_T iter, real_T X0[300],
  real_T MV0[100], real_T *Slack0, stwyLkKtGfiNF3i9PDxcjkC_NMPC__T *info);
real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_T_generateRuntimeData(const real_T x[6], const real_T
  lastMV[2], const real_T ref0[60], const real_T X0[294], const real_T MV0[98],
  real_T Slack0, s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T *runtimedata,
  s02qEHDaRbOUcZF4Nctxs8G_NMPC__T *userdata, real_T z0[401])
{
  real_T X0_0[300];
  real_T MV0_0[100];
  real_T a[100];
  int32_T MV0_tmp;
  int32_T i;
  static const real_T a_1[10000] = { 0.16666666666666666, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16666666666666666, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.44247787610619471 };

  static const real_T d[300] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T e[100] = { -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5,
    -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5,
    -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5,
    -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5,
    -0.5, -0.5, -0.5, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712,
    -0.24336283185840712, -0.24336283185840712, -0.24336283185840712 };

  static const real_T f[100] = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712, 0.24336283185840712, 0.24336283185840712,
    0.24336283185840712 };

  for (i = 0; i < 6; i++) {
    memcpy(&userdata->References[i * 50], &ref0[i * 10], 10U * sizeof(real_T));
    for (MV0_tmp = 0; MV0_tmp < 40; MV0_tmp++) {
      userdata->References[(MV0_tmp + 50 * i) + 10] = ref0[10 * i + 9];
    }
  }

  memset(&userdata->MVTarget[0], 0, 100U * sizeof(real_T));
  for (i = 0; i < 49; i++) {
    for (MV0_tmp = 0; MV0_tmp < 6; MV0_tmp++) {
      X0_0[MV0_tmp + 6 * i] = X0[49 * MV0_tmp + i];
    }
  }

  for (i = 0; i < 6; i++) {
    X0_0[i + 294] = X0[49 * i + 48];
  }

  for (i = 0; i < 49; i++) {
    MV0_tmp = i << 1;
    MV0_0[MV0_tmp] = MV0[i];
    MV0_0[MV0_tmp + 1] = MV0[i + 49];
  }

  MV0_0[98] = MV0[48];
  MV0_0[99] = MV0[97];
  for (i = 0; i < 100; i++) {
    real_T a_0;
    a_0 = 0.0;
    for (MV0_tmp = 0; MV0_tmp < 100; MV0_tmp++) {
      a_0 += a_1[100 * MV0_tmp + i] * MV0_0[MV0_tmp];
    }

    a[i] = a_0;
  }

  memcpy(&z0[0], &X0_0[0], 300U * sizeof(real_T));
  memcpy(&z0[300], &a[0], 100U * sizeof(real_T));
  z0[400] = Slack0;
  userdata->Ts = 0.01;
  userdata->PredictionHorizon = 50.0;
  userdata->NumOfStates = 6.0;
  userdata->NumOfOutputs = 6.0;
  userdata->NumOfInputs = 2.0;
  userdata->LastMV[0] = lastMV[0];
  userdata->MVIndex[0] = 1.0;
  userdata->LastMV[1] = lastMV[1];
  userdata->MVIndex[1] = 2.0;
  userdata->InputPassivityIndex = 0.0;
  userdata->OutputPassivityIndex = 0.1;
  userdata->PassivityUsePredictedX = true;
  for (i = 0; i < 6; i++) {
    userdata->CurrentStates[i] = x[i];
    runtimedata->x[i] = x[i];
  }

  runtimedata->lastMV[0] = lastMV[0];
  runtimedata->lastMV[1] = lastMV[1];
  memcpy(&runtimedata->ref[0], &userdata->References[0], 300U * sizeof(real_T));
  memcpy(&runtimedata->OutputWeights[0], &d[0], 300U * sizeof(real_T));
  for (i = 0; i < 100; i++) {
    runtimedata->MVWeights[i] = 0.0;
    runtimedata->MVRateWeights[i] = 0.3;
  }

  runtimedata->ECRWeight = 100000.0;
  for (i = 0; i < 300; i++) {
    runtimedata->OutputMin[i] = (rtMinusInf);
    runtimedata->OutputMax[i] = (rtInf);
    runtimedata->StateMin[i] = (rtMinusInf);
    runtimedata->StateMax[i] = (rtInf);
  }

  for (i = 0; i < 100; i++) {
    runtimedata->MVMin[i] = e[i];
    runtimedata->MVMax[i] = f[i];
    runtimedata->MVRateMin[i] = (rtMinusInf);
    runtimedata->MVRateMax[i] = (rtInf);
    runtimedata->MVScaledTarget[i] = 0.0;
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_getZBounds(const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T *
  runtimedata, real_T zLB[401], real_T zUB[401])
{
  real_T runtimedata_0[300];
  int32_T i;
  int32_T i_0;
  for (i_0 = 0; i_0 < 50; i_0++) {
    for (i = 0; i < 6; i++) {
      runtimedata_0[i + 6 * i_0] = runtimedata->StateMin[50 * i + i_0];
    }
  }

  memcpy(&zLB[0], &runtimedata_0[0], 300U * sizeof(real_T));
  for (i_0 = 0; i_0 < 100; i_0++) {
    zLB[i_0 + 300] = (rtMinusInf);
  }

  zLB[400] = 0.0;
  for (i_0 = 0; i_0 < 50; i_0++) {
    for (i = 0; i < 6; i++) {
      runtimedata_0[i + 6 * i_0] = runtimedata->StateMax[50 * i + i_0];
    }
  }

  memcpy(&zUB[0], &runtimedata_0[0], 300U * sizeof(real_T));
  for (i_0 = 0; i_0 < 100; i_0++) {
    zUB[i_0 + 300] = (rtInf);
  }

  zUB[400] = (rtInf);
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_isfinite(const real_T x[2], boolean_T b[2])
{
  b[0] = ((!rtIsInf(x[0])) && (!rtIsNaN(x[0])));
  b[1] = ((!rtIsInf(x[1])) && (!rtIsNaN(x[1])));
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_mtimes(const real_T A_data[], const int32_T
  A_size[2], real_T C_data[], int32_T C_size[2])
{
  int32_T b_i;
  int32_T i;
  int32_T j;
  int32_T mc;
  static const real_T b[10000] = { 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26 };

  mc = A_size[0] - 1;
  C_size[0] = A_size[0];
  C_size[1] = 100;
  for (j = 0; j < 100; j++) {
    int32_T boffset;
    int32_T coffset;
    coffset = (mc + 1) * j;
    boffset = j * 100;
    for (i = 0; i <= mc; i++) {
      C_data[coffset + i] = 0.0;
    }

    for (i = 0; i < 100; i++) {
      real_T bkj;
      int32_T aoffset;
      int32_T scalarLB;
      int32_T tmp_0;
      int32_T vectorUB;
      aoffset = i * A_size[0];
      bkj = b[boffset + i];
      scalarLB = ((mc + 1) / 2) << 1;
      vectorUB = scalarLB - 2;
      for (b_i = 0; b_i <= vectorUB; b_i += 2) {
        __m128d tmp;
        tmp_0 = coffset + b_i;
        tmp = _mm_loadu_pd(&C_data[tmp_0]);
        _mm_storeu_pd(&C_data[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
          (&A_data[aoffset + b_i]), _mm_set1_pd(bkj)), tmp));
      }

      for (b_i = scalarLB; b_i <= mc; b_i++) {
        tmp_0 = coffset + b_i;
        C_data[tmp_0] += A_data[aoffset + b_i] * bkj;
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_getUBounds(const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T *
  runtimedata, real_T A_data[], int32_T A_size[2], real_T Bu_data[], int32_T
  *Bu_size)
{
  real_T b_Bu[400];
  real_T runtimedata_0[2];
  real_T runtimedata_MVMax;
  real_T runtimedata_MVMax_0;
  real_T runtimedata_MVMin;
  real_T runtimedata_MVMin_0;
  real_T runtimedata_MVRateMax;
  real_T runtimedata_MVRateMax_0;
  real_T runtimedata_MVRateMin;
  real_T runtimedata_MVRateMin_0;
  int32_T b[2];
  int32_T tmp_size[2];
  int32_T Au_tmp;
  int32_T Au_tmp_0;
  int32_T Auf_data_tmp;
  int32_T Auf_data_tmp_0;
  int32_T b_i;
  int32_T i;
  int32_T ic_idx_0;
  int32_T ic_idx_1;
  int16_T ii_data[400];
  boolean_T x[400];
  boolean_T tmp[2];
  boolean_T exitg1;
  memset(&NMPC_Path_Tracking_B.Au[0], 0, 40000U * sizeof(real_T));
  memset(&b_Bu[0], 0, 400U * sizeof(real_T));
  memset(&x[0], 0, 400U * sizeof(boolean_T));
  ic_idx_0 = 1;
  ic_idx_1 = 2;
  for (i = 0; i < 50; i++) {
    runtimedata_MVRateMin = runtimedata->MVRateMin[i];
    runtimedata_0[0] = runtimedata_MVRateMin;
    runtimedata_MVRateMin_0 = runtimedata->MVRateMin[i + 50];
    runtimedata_0[1] = runtimedata_MVRateMin_0;
    NMPC_Path_Tracking_isfinite(runtimedata_0, tmp);
    x[ic_idx_0 - 1] = tmp[0];
    runtimedata_MVRateMax = runtimedata->MVRateMax[i];
    runtimedata_0[0] = runtimedata_MVRateMax;
    x[ic_idx_1 - 1] = tmp[1];
    runtimedata_MVRateMax_0 = runtimedata->MVRateMax[i + 50];
    runtimedata_0[1] = runtimedata_MVRateMax_0;
    NMPC_Path_Tracking_isfinite(runtimedata_0, tmp);
    x[ic_idx_0 + 1] = tmp[0];
    runtimedata_MVMin = runtimedata->MVMin[i];
    runtimedata_0[0] = runtimedata_MVMin;
    x[ic_idx_1 + 1] = tmp[1];
    runtimedata_MVMin_0 = runtimedata->MVMin[i + 50];
    runtimedata_0[1] = runtimedata_MVMin_0;
    NMPC_Path_Tracking_isfinite(runtimedata_0, tmp);
    x[ic_idx_0 + 3] = tmp[0];
    runtimedata_MVMax = runtimedata->MVMax[i];
    runtimedata_0[0] = runtimedata_MVMax;
    x[ic_idx_1 + 3] = tmp[1];
    runtimedata_MVMax_0 = runtimedata->MVMax[i + 50];
    runtimedata_0[1] = runtimedata_MVMax_0;
    NMPC_Path_Tracking_isfinite(runtimedata_0, tmp);
    x[ic_idx_0 + 5] = tmp[0];
    Au_tmp = 800 * i + ic_idx_0;
    NMPC_Path_Tracking_B.Au[Au_tmp - 1] = -0.16666666666666666;
    Au_tmp_0 = 800 * i + ic_idx_1;
    NMPC_Path_Tracking_B.Au[Au_tmp_0 - 1] = -0.0;
    x[ic_idx_1 + 5] = tmp[1];
    NMPC_Path_Tracking_B.Au[Au_tmp + 399] = -0.0;
    NMPC_Path_Tracking_B.Au[Au_tmp_0 + 399] = -0.44247787610619471;
    NMPC_Path_Tracking_B.Au[Au_tmp + 1] = 0.16666666666666666;
    NMPC_Path_Tracking_B.Au[Au_tmp_0 + 1] = 0.0;
    NMPC_Path_Tracking_B.Au[Au_tmp + 401] = 0.0;
    NMPC_Path_Tracking_B.Au[Au_tmp_0 + 401] = 0.44247787610619471;
    NMPC_Path_Tracking_B.Au[Au_tmp + 3] = -0.16666666666666666;
    NMPC_Path_Tracking_B.Au[Au_tmp_0 + 3] = -0.0;
    NMPC_Path_Tracking_B.Au[Au_tmp + 403] = -0.0;
    NMPC_Path_Tracking_B.Au[Au_tmp_0 + 403] = -0.44247787610619471;
    NMPC_Path_Tracking_B.Au[Au_tmp + 5] = 0.16666666666666666;
    NMPC_Path_Tracking_B.Au[Au_tmp_0 + 5] = 0.0;
    NMPC_Path_Tracking_B.Au[Au_tmp + 405] = 0.0;
    NMPC_Path_Tracking_B.Au[Au_tmp_0 + 405] = 0.44247787610619471;
    b_Bu[ic_idx_0 - 1] = -runtimedata_MVRateMin;
    b_Bu[ic_idx_1 - 1] = -runtimedata_MVRateMin_0;
    b_Bu[ic_idx_0 + 1] = runtimedata_MVRateMax;
    b_Bu[ic_idx_1 + 1] = runtimedata_MVRateMax_0;
    b_Bu[ic_idx_0 + 3] = -runtimedata_MVMin;
    b_Bu[ic_idx_1 + 3] = -runtimedata_MVMin_0;
    b_Bu[ic_idx_0 + 5] = runtimedata_MVMax;
    b_Bu[ic_idx_1 + 5] = runtimedata_MVMax_0;
    if (i + 1 == 1) {
      runtimedata_MVRateMin = runtimedata->lastMV[0] / 6.0;
      runtimedata_MVRateMin_0 = runtimedata_MVRateMin;
      runtimedata_MVRateMax = b_Bu[ic_idx_0 - 1] - runtimedata_MVRateMin;
      runtimedata_MVRateMin = runtimedata->lastMV[1] / 2.26;
      runtimedata_MVRateMax_0 = b_Bu[ic_idx_1 - 1] - runtimedata_MVRateMin;
      b_Bu[ic_idx_0 - 1] = runtimedata_MVRateMax;
      b_Bu[ic_idx_1 - 1] = runtimedata_MVRateMax_0;
      runtimedata_MVRateMax_0 = b_Bu[ic_idx_1 + 1] + runtimedata_MVRateMin;
      b_Bu[ic_idx_0 + 1] += runtimedata_MVRateMin_0;
      b_Bu[ic_idx_1 + 1] = runtimedata_MVRateMax_0;
    } else {
      Au_tmp = (i - 1) * 800;
      Au_tmp_0 = ic_idx_0 + Au_tmp;
      NMPC_Path_Tracking_B.Au[Au_tmp_0 - 1] = 0.16666666666666666;
      Au_tmp += ic_idx_1;
      NMPC_Path_Tracking_B.Au[Au_tmp - 1] = 0.0;
      NMPC_Path_Tracking_B.Au[Au_tmp_0 + 399] = 0.0;
      NMPC_Path_Tracking_B.Au[Au_tmp + 399] = 0.44247787610619471;
      NMPC_Path_Tracking_B.Au[Au_tmp_0 + 1] = -0.16666666666666666;
      NMPC_Path_Tracking_B.Au[Au_tmp + 1] = -0.0;
      NMPC_Path_Tracking_B.Au[Au_tmp_0 + 401] = -0.0;
      NMPC_Path_Tracking_B.Au[Au_tmp + 401] = -0.44247787610619471;
    }

    ic_idx_0 += 8;
    ic_idx_1 += 8;
  }

  i = 0;
  ic_idx_0 = 0;
  exitg1 = false;
  while ((!exitg1) && (ic_idx_0 < 400)) {
    if (x[ic_idx_0]) {
      i++;
      ii_data[i - 1] = (int16_T)(ic_idx_0 + 1);
      if (i >= 400) {
        exitg1 = true;
      } else {
        ic_idx_0++;
      }
    } else {
      ic_idx_0++;
    }
  }

  if (i < 1) {
    i = 0;
  }

  if (i > 0) {
    *Bu_size = i;
    ic_idx_0 = (i / 2) << 1;
    ic_idx_1 = ic_idx_0 - 2;
    for (Au_tmp = 0; Au_tmp <= ic_idx_1; Au_tmp += 2) {
      Bu_data[Au_tmp] = b_Bu[ii_data[Au_tmp] - 1];
      Bu_data[Au_tmp + 1] = b_Bu[ii_data[Au_tmp + 1] - 1];
    }

    for (Au_tmp = ic_idx_0; Au_tmp < i; Au_tmp++) {
      Bu_data[Au_tmp] = b_Bu[ii_data[Au_tmp] - 1];
    }

    for (Au_tmp = 0; Au_tmp < 2; Au_tmp++) {
      for (Au_tmp_0 = 0; Au_tmp_0 < 50; Au_tmp_0++) {
        for (b_i = 0; b_i <= ic_idx_1; b_i += 2) {
          Auf_data_tmp = i * Au_tmp;
          Auf_data_tmp_0 = (i << 1) * Au_tmp_0;
          NMPC_Path_Tracking_B.Auf_data[(b_i + Auf_data_tmp) + Auf_data_tmp_0] =
            NMPC_Path_Tracking_B.Au[((400 * Au_tmp + ii_data[b_i]) + 800 *
            Au_tmp_0) - 1];
          NMPC_Path_Tracking_B.Auf_data[((b_i + Auf_data_tmp) + Auf_data_tmp_0)
            + 1] = NMPC_Path_Tracking_B.Au[((400 * Au_tmp + ii_data[b_i + 1]) +
            800 * Au_tmp_0) - 1];
        }

        for (b_i = ic_idx_0; b_i < i; b_i++) {
          NMPC_Path_Tracking_B.Auf_data[(b_i + i * Au_tmp) + (i << 1) * Au_tmp_0]
            = NMPC_Path_Tracking_B.Au[((400 * Au_tmp + ii_data[b_i]) + 800 *
            Au_tmp_0) - 1];
        }
      }
    }

    b[0] = i;
    b[1] = 100;
    NMPC_Path_Tracking_mtimes(NMPC_Path_Tracking_B.Auf_data, b,
      NMPC_Path_Tracking_B.tmp_data, tmp_size);
    A_size[0] = i;
    A_size[1] = 401;
    ic_idx_0 = i * 300;
    for (Au_tmp = 0; Au_tmp < ic_idx_0; Au_tmp++) {
      A_data[Au_tmp] = 0.0;
    }

    ic_idx_0 = i * 100;
    for (Au_tmp = 0; Au_tmp < ic_idx_0; Au_tmp++) {
      A_data[Au_tmp + i * 300] = NMPC_Path_Tracking_B.tmp_data[Au_tmp];
    }

    for (Au_tmp = 0; Au_tmp < i; Au_tmp++) {
      A_data[(Au_tmp + i * 300) + i * 100] = 0.0;
    }
  } else {
    *Bu_size = 0;
    A_size[0] = 0;
    A_size[1] = 401;
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_getXUe(const real_T z[401], const real_T x[6],
  real_T X[306], real_T U[102], real_T *e)
{
  real_T z_0[300];
  real_T Umv[102];
  real_T y[100];
  int32_T i;
  int32_T i_0;
  static const real_T y_1[10000] = { 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26 };

  memset(&X[0], 0, 306U * sizeof(real_T));
  memset(&Umv[0], 0, 102U * sizeof(real_T));
  for (i = 0; i < 100; i++) {
    real_T y_0;
    y_0 = 0.0;
    for (i_0 = 0; i_0 < 100; i_0++) {
      y_0 += y_1[100 * i_0 + i] * z[i_0 + 300];
    }

    y[i] = y_0;
  }

  for (i = 0; i < 2; i++) {
    for (i_0 = 0; i_0 < 50; i_0++) {
      Umv[i_0 + 51 * i] = y[(i_0 << 1) + i];
    }
  }

  *e = z[400];
  memcpy(&z_0[0], &z[0], 300U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < 50; i_0++) {
      X[(i_0 + 51 * i) + 1] = z_0[6 * i_0 + i];
    }

    X[51 * i] = x[i];
  }

  for (i_0 = 0; i_0 < 2; i_0++) {
    Umv[51 * i_0 + 50] = Umv[51 * i_0 + 49];
    memcpy(&U[i_0 * 51], &Umv[i_0 * 51], 51U * sizeof(real_T));
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_T_VehicleStateJacFcn1(const real_T x[6], const real_T u[2],
  real_T A[36], real_T B[12])
{
  __m128d tmp_0;
  real_T tmp[2];
  real_T A_tmp;
  real_T A_tmp_0;
  real_T A_tmp_1;
  real_T A_tmp_2;
  real_T A_tmp_3;
  memset(&A[0], 0, 36U * sizeof(real_T));
  memset(&B[0], 0, 12U * sizeof(real_T));
  A_tmp = cos(x[2]);
  A_tmp_0 = sin(x[2]);
  A[12] = -x[3] * A_tmp_0 - x[4] * A_tmp;
  A[18] = A_tmp;
  A[24] = -A_tmp_0;
  A[13] = x[3] * A_tmp - x[4] * A_tmp_0;
  A[19] = A_tmp_0;
  A[25] = A_tmp;
  A[32] = 1.0;
  A[27] = x[5];
  A[33] = x[4];
  tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_set_pd(-1.6, 1.4), _mm_set1_pd(x[5])),
                     _mm_set1_pd(x[4]));
  _mm_storeu_pd(&tmp[0], tmp_0);
  A_tmp = cos(u[1]);
  A_tmp_0 = x[3] * x[3];
  A_tmp_1 = -1.4 * x[5] - x[4];
  A_tmp_2 = 1.6 * x[5] - x[4];
  A[22] = (A_tmp_1 * -1.2E+6 * A_tmp / (tmp[0] * tmp[0] + A_tmp_0) - A_tmp_2 *
           1.1E+6 / (tmp[1] * tmp[1] + A_tmp_0)) * 0.001 - x[5];
  _mm_storeu_pd(&tmp[0], tmp_0);
  A[28] = (-1.2E+6 * x[3] * A_tmp / (tmp[0] * tmp[0] + A_tmp_0) - 1.1E+6 * x[3] /
           (tmp[1] * tmp[1] + A_tmp_0)) * 0.001;
  _mm_storeu_pd(&tmp[0], tmp_0);
  A_tmp_3 = -1.68E+6 * x[3] * A_tmp;
  A[34] = (A_tmp_3 / (tmp[0] * tmp[0] + A_tmp_0) + 1.76E+6 * x[3] / (tmp[1] *
            tmp[1] + A_tmp_0)) * 0.001 - x[3];
  _mm_storeu_pd(&tmp[0], tmp_0);
  A[23] = (A_tmp_1 * -1.68E+6 * A_tmp / (tmp[0] * tmp[0] + A_tmp_0) + A_tmp_2 *
           1.76E+6 / (tmp[1] * tmp[1] + A_tmp_0)) * 0.0005;
  _mm_storeu_pd(&tmp[0], tmp_0);
  A[29] = (A_tmp_3 / (tmp[0] * tmp[0] + A_tmp_0) + 1.76E+6 * x[3] / (tmp[1] *
            tmp[1] + A_tmp_0)) * 0.0005;
  _mm_storeu_pd(&tmp[0], tmp_0);
  A[35] = (-2.3519999999999995E+6 * x[3] * A_tmp / (tmp[0] * tmp[0] + A_tmp_0) -
           2.8160000000000005E+6 * x[3] / (tmp[1] * tmp[1] + A_tmp_0)) * 0.0005;
  B[3] = 1.0;
  tmp_0 = _mm_set_pd(1.68E+6, 1.2E+6);
  _mm_storeu_pd(&B[10], _mm_mul_pd(_mm_add_pd(_mm_mul_pd(_mm_mul_pd(_mm_add_pd
    (_mm_set1_pd(-u[1]), _mm_set1_pd(rt_atan2d_snf(1.4 * x[5] + x[4], x[3]))),
    tmp_0), _mm_set1_pd(sin(u[1]))), _mm_mul_pd(tmp_0, _mm_set1_pd(A_tmp))),
    _mm_set_pd(0.0005, 0.001)));
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_StateFunctionVehicle1(const real_T x[6], const real_T u[2],
  real_T dxdt[6])
{
  real_T Fyf;
  real_T Fyr;
  real_T dxdt_tmp;
  real_T dxdt_tmp_0;
  Fyf = (rt_atan2d_snf(1.4 * x[5] + x[4], x[3]) - u[1]) * -1.2E+6 * cos(u[1]);
  Fyr = rt_atan2d_snf(x[4] - 1.6 * x[5], x[3]) * -1.1E+6;
  dxdt_tmp = sin(x[2]);
  dxdt_tmp_0 = cos(x[2]);
  _mm_storeu_pd(&dxdt[0], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(x[3]), _mm_set_pd
    (dxdt_tmp, dxdt_tmp_0)), _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(x[4]), _mm_set_pd
    (dxdt_tmp_0, dxdt_tmp)), _mm_set_pd(1.0, -1.0))));
  dxdt[2] = x[5];
  dxdt[3] = x[4] * x[5] + u[0];
  dxdt[4] = (Fyf + Fyr) * 0.001 + -x[3] * x[5];
  dxdt[5] = (1.4 * Fyf - 1.6 * Fyr) * 0.0005;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracki_stateEvolution(const real_T X[306], const real_T U
  [102], real_T c[300], real_T J[120300])
{
  __m128d tmp_1;
  __m128d tmp_2;
  real_T b_X[306];
  real_T b_U[102];
  real_T Ak[36];
  real_T Ak1[36];
  real_T Bk1[12];
  real_T val[12];
  real_T ic[6];
  real_T tmp[6];
  real_T tmp_0[6];
  real_T Jmv;
  int32_T Jx_tmp;
  int32_T b_k;
  int32_T i;
  int32_T k;
  static const real_T b[10000] = { 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26 };

  int32_T c_tmp;
  memset(&NMPC_Path_Tracking_B.Jx_m[0], 0, 90000U * sizeof(real_T));
  memset(&NMPC_Path_Tracking_B.Jmv[0], 0, 30000U * sizeof(real_T));
  memset(&c[0], 0, 300U * sizeof(real_T));
  for (Jx_tmp = 0; Jx_tmp < 6; Jx_tmp++) {
    ic[Jx_tmp] = (real_T)Jx_tmp + 1.0;
  }

  for (Jx_tmp = 0; Jx_tmp < 51; Jx_tmp++) {
    i = Jx_tmp << 1;
    b_U[i] = U[Jx_tmp];
    b_U[i + 1] = U[Jx_tmp + 51];
    for (b_k = 0; b_k < 6; b_k++) {
      b_X[b_k + 6 * Jx_tmp] = X[51 * b_k + Jx_tmp];
    }
  }

  for (i = 0; i < 50; i++) {
    Jx_tmp = i << 1;
    NMPC_Path_T_VehicleStateJacFcn1(&b_X[6 * i], &b_U[Jx_tmp], Ak, val);
    b_k = (i + 1) * 6;
    NMPC_Path_T_VehicleStateJacFcn1(&b_X[b_k], &b_U[Jx_tmp], Ak1, Bk1);
    NMPC_Path_StateFunctionVehicle1(&b_X[6 * i], &b_U[Jx_tmp], tmp);
    NMPC_Path_StateFunctionVehicle1(&b_X[b_k], &b_U[Jx_tmp], tmp_0);
    for (k = 0; k < 6; k++) {
      c_tmp = (int32_T)ic[k];
      c[c_tmp - 1] = (b_X[6 * i + k] + (tmp[k] + tmp_0[k]) * 0.005) - b_X[b_k +
        k];
      if (i + 1 > 1) {
        for (Jx_tmp = 0; Jx_tmp < 6; Jx_tmp++) {
          NMPC_Path_Tracking_B.Jx_m[(((int32_T)ic[Jx_tmp] + 300 * k) + 1800 * (i
            - 1)) - 1] = Ak[6 * k + Jx_tmp] * 0.005;
        }

        Jx_tmp = ((300 * k + c_tmp) + (i - 1) * 1800) - 1;
        NMPC_Path_Tracking_B.Jx_m[Jx_tmp]++;
      }
    }

    for (b_k = 0; b_k < 6; b_k++) {
      for (Jx_tmp = 0; Jx_tmp < 6; Jx_tmp++) {
        NMPC_Path_Tracking_B.Jx_m[(((int32_T)ic[Jx_tmp] + 300 * b_k) + 1800 * i)
          - 1] = Ak1[6 * b_k + Jx_tmp] * 0.005;
      }

      Jx_tmp = ((300 * b_k + (int32_T)ic[b_k]) + 1800 * i) - 1;
      NMPC_Path_Tracking_B.Jx_m[Jx_tmp]--;
    }

    for (Jx_tmp = 0; Jx_tmp <= 10; Jx_tmp += 2) {
      tmp_1 = _mm_loadu_pd(&val[Jx_tmp]);
      tmp_2 = _mm_loadu_pd(&Bk1[Jx_tmp]);
      _mm_storeu_pd(&val[Jx_tmp], _mm_mul_pd(_mm_add_pd(tmp_1, tmp_2),
        _mm_set1_pd(0.005)));
    }

    for (b_k = 0; b_k < 2; b_k++) {
      for (Jx_tmp = 0; Jx_tmp < 6; Jx_tmp++) {
        NMPC_Path_Tracking_B.Jmv[(((int32_T)ic[Jx_tmp] + 300 * b_k) + 600 * i) -
          1] = val[6 * b_k + Jx_tmp];
      }
    }

    for (Jx_tmp = 0; Jx_tmp <= 4; Jx_tmp += 2) {
      tmp_1 = _mm_loadu_pd(&ic[Jx_tmp]);
      _mm_storeu_pd(&ic[Jx_tmp], _mm_add_pd(tmp_1, _mm_set1_pd(6.0)));
    }
  }

  for (Jx_tmp = 0; Jx_tmp < 100; Jx_tmp++) {
    for (b_k = 0; b_k < 300; b_k++) {
      Jmv = 0.0;
      for (i = 0; i < 100; i++) {
        Jmv += NMPC_Path_Tracking_B.Jmv[300 * i + b_k] * b[100 * Jx_tmp + i];
      }

      NMPC_Path_Tracking_B.Jmv_c[Jx_tmp + 100 * b_k] = Jmv;
    }
  }

  for (Jx_tmp = 0; Jx_tmp < 300; Jx_tmp++) {
    for (b_k = 0; b_k < 300; b_k++) {
      J[b_k + 401 * Jx_tmp] = NMPC_Path_Tracking_B.Jx_m[300 * b_k + Jx_tmp];
    }

    memcpy(&J[Jx_tmp * 401 + 300], &NMPC_Path_Tracking_B.Jmv_c[Jx_tmp * 100],
           100U * sizeof(real_T));
    J[401 * Jx_tmp + 400] = 0.0;
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_all(const boolean_T x[300], boolean_T y[6])
{
  int32_T i;
  int32_T i2;
  i2 = 1;
  for (i = 0; i < 6; i++) {
    int32_T a;
    int32_T ix;
    boolean_T exitg1;
    y[i] = true;
    a = i2 + 49;
    ix = i2;
    i2 += 50;
    exitg1 = false;
    while ((!exitg1) && (ix <= a)) {
      if (!x[ix - 1]) {
        y[i] = false;
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_isfinite_p(const real_T x[6], boolean_T b[6])
{
  int32_T i;
  for (i = 0; i < 6; i++) {
    real_T x_0;
    x_0 = x[i];
    b[i] = ((!rtIsInf(x_0)) && (!rtIsNaN(x_0)));
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static boolean_T NMPC_Path_Tracking_any(const boolean_T x[12])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 11)) {
    if (x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracki_reformJacobian(const real_T Jx_data[], const
  int32_T Jx_size[3], const real_T Jmv_data[], const real_T Je_data[], const
  int32_T *Je_size, real_T Jc_data[], int32_T Jc_size[2])
{
  int32_T Jx_0[2];
  int32_T tmp_size[2];
  int32_T i;
  int32_T i_0;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int32_T loop_ub_1;
  int16_T Je[2];
  int16_T Jx[2];
  int16_T varargin_2[2];
  if (Jx_size[0] == 0) {
    Jc_size[0] = 0;
    Jc_size[1] = 0;
  } else {
    Jx[0] = (int16_T)Jx_size[0];
    loop_ub_0 = Jx_size[0];
    for (i_0 = 0; i_0 < loop_ub_0; i_0++) {
      for (i = 0; i < 300; i++) {
        NMPC_Path_Tracking_B.varargin_1_data[i + 300 * i_0] = Jx_data[Jx[0] * i
          + i_0];
      }
    }

    Jx_0[0] = Jx_size[0];
    Jx_0[1] = 100;
    NMPC_Path_Tracking_mtimes(Jmv_data, Jx_0, NMPC_Path_Tracking_B.tmp_data_k,
      tmp_size);
    loop_ub_1 = tmp_size[0];
    for (i_0 = 0; i_0 < loop_ub_1; i_0++) {
      for (i = 0; i < 100; i++) {
        NMPC_Path_Tracking_B.varargin_2_data[i + 100 * i_0] =
          NMPC_Path_Tracking_B.tmp_data_k[tmp_size[0] * i + i_0];
      }
    }

    Jx[0] = 300;
    if (tmp_size[0] != 0) {
      varargin_2[0] = 100;
    } else {
      varargin_2[0] = 0;
    }

    loop_ub_1 = *Je_size;
    for (i_0 = 0; i_0 < loop_ub_1; i_0++) {
      NMPC_Path_Tracking_B.Je_data[i_0] = Je_data[i_0];
    }

    Je[0] = (int16_T)(*Je_size != 0);
    Jc_size[0] = (varargin_2[0] + Je[0]) + 300;
    Jc_size[1] = Jx_size[0];
    loop_ub_1 = varargin_2[0];
    loop_ub = Je[0];
    for (i_0 = 0; i_0 < loop_ub_0; i_0++) {
      for (i = 0; i < 300; i++) {
        Jc_data[i + Jc_size[0] * i_0] = NMPC_Path_Tracking_B.varargin_1_data[Jx
          [0] * i_0 + i];
      }

      for (i = 0; i < loop_ub_1; i++) {
        Jc_data[(i + Jc_size[0] * i_0) + 300] =
          NMPC_Path_Tracking_B.varargin_2_data[varargin_2[0] * i_0 + i];
      }

      if (loop_ub - 1 >= 0) {
        Jc_data[(varargin_2[0] + Jc_size[0] * i_0) + 300] =
          NMPC_Path_Tracking_B.Je_data[Je[0] * i_0];
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_outputBounds(const real_T runtimedata_OutputMin
  [300], const real_T runtimedata_OutputMax[300], const real_T X[306], real_T e,
  real_T c_data[], int32_T c_size[2], real_T Jc_data[], int32_T Jc_size[2])
{
  __m128d tmp_0;
  real_T Ck[36];
  real_T val[36];
  real_T ic[6];
  real_T yk[6];
  int32_T icf_tmp_1[12];
  int32_T icf_tmp[6];
  int32_T icf_tmp_0[6];
  int32_T Ck_tmp;
  int32_T c_k;
  int32_T i;
  int32_T icf_tmp_2;
  int32_T s;
  int16_T tmp_data[600];
  int8_T Je[600];
  boolean_T icf[600];
  boolean_T tmp[300];
  boolean_T icf_0[12];
  boolean_T x[6];
  boolean_T y;
  static const int8_T d[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const int8_T s_0[6] = { 1, 1, 1, 10, 1, 1 };

  int32_T Jx_size[3];
  int32_T Jx_data_tmp;
  int32_T Jx_data_tmp_0;
  int32_T tmp_size_idx_0;
  boolean_T exitg1;
  for (c_k = 0; c_k < 300; c_k++) {
    tmp[c_k] = rtIsInf(runtimedata_OutputMin[c_k]);
  }

  NMPC_Path_Tracking_all(tmp, x);
  y = true;
  c_k = 0;
  exitg1 = false;
  while ((!exitg1) && (c_k < 6)) {
    if (!x[c_k]) {
      y = false;
      exitg1 = true;
    } else {
      c_k++;
    }
  }

  if (y) {
    for (c_k = 0; c_k < 300; c_k++) {
      tmp[c_k] = rtIsInf(runtimedata_OutputMax[c_k]);
    }

    NMPC_Path_Tracking_all(tmp, x);
    y = true;
    c_k = 0;
    exitg1 = false;
    while ((!exitg1) && (c_k < 6)) {
      if (!x[c_k]) {
        y = false;
        exitg1 = true;
      } else {
        c_k++;
      }
    }
  } else {
    y = false;
  }

  if (y) {
    c_size[0] = 0;
    c_size[1] = 0;
    Jc_size[0] = 0;
    Jc_size[1] = 0;
  } else {
    for (i = 0; i < 600; i++) {
      NMPC_Path_Tracking_B.b_c[i] = 0.0;
      icf[i] = true;
    }

    memset(&NMPC_Path_Tracking_B.Jx[0], 0, 180000U * sizeof(real_T));
    memset(&Je[0], 0, 600U * sizeof(int8_T));
    for (c_k = 0; c_k < 6; c_k++) {
      ic[c_k] = (real_T)c_k + 1.0;
    }

    for (i = 0; i < 50; i++) {
      for (c_k = 0; c_k < 6; c_k++) {
        icf_tmp[c_k] = (int32_T)ic[c_k];
        yk[c_k] = runtimedata_OutputMin[50 * c_k + i];
      }

      NMPC_Path_Tracking_isfinite_p(yk, x);
      for (c_k = 0; c_k < 6; c_k++) {
        icf[icf_tmp[c_k] - 1] = x[c_k];
        icf_tmp_0[c_k] = (int32_T)(ic[c_k] + 6.0);
        yk[c_k] = runtimedata_OutputMax[50 * c_k + i];
      }

      NMPC_Path_Tracking_isfinite_p(yk, x);
      for (c_k = 0; c_k < 6; c_k++) {
        icf_tmp_2 = icf_tmp_0[c_k];
        icf[icf_tmp_2 - 1] = x[c_k];
        icf_tmp_1[c_k] = icf_tmp[c_k] - 1;
        icf_tmp_1[c_k + 6] = icf_tmp_2 - 1;
      }

      for (c_k = 0; c_k < 12; c_k++) {
        icf_0[c_k] = icf[icf_tmp_1[c_k]];
      }

      if (NMPC_Path_Tracking_any(icf_0)) {
        for (c_k = 0; c_k < 36; c_k++) {
          Ck[c_k] = d[c_k];
        }

        yk[0] = X[i + 1];
        yk[1] = X[i + 52];
        yk[2] = X[i + 103];
        yk[3] = X[i + 154] / 10.0;
        yk[4] = X[i + 205];
        yk[5] = X[i + 256];
        for (icf_tmp_2 = 0; icf_tmp_2 < 6; icf_tmp_2++) {
          s = s_0[icf_tmp_2];
          for (c_k = 0; c_k < 6; c_k++) {
            Ck_tmp = 6 * c_k + icf_tmp_2;
            Ck[Ck_tmp] /= (real_T)s;
          }

          NMPC_Path_Tracking_B.b_c[icf_tmp[icf_tmp_2] - 1] =
            (runtimedata_OutputMin[50 * icf_tmp_2 + i] - e) - yk[icf_tmp_2];
        }

        for (c_k = 0; c_k < 6; c_k++) {
          NMPC_Path_Tracking_B.b_c[icf_tmp_0[c_k] - 1] = (yk[c_k] -
            runtimedata_OutputMax[50 * c_k + i]) - e;
        }

        for (c_k = 0; c_k <= 34; c_k += 2) {
          tmp_0 = _mm_loadu_pd(&Ck[c_k]);
          _mm_storeu_pd(&val[c_k], _mm_mul_pd(tmp_0, _mm_set1_pd(-1.0)));
        }

        for (icf_tmp_2 = 0; icf_tmp_2 < 6; icf_tmp_2++) {
          for (c_k = 0; c_k < 6; c_k++) {
            NMPC_Path_Tracking_B.Jx[(((int32_T)ic[c_k] + 600 * icf_tmp_2) + 3600
              * i) - 1] = val[6 * icf_tmp_2 + c_k];
          }
        }

        for (icf_tmp_2 = 0; icf_tmp_2 < 6; icf_tmp_2++) {
          for (c_k = 0; c_k < 6; c_k++) {
            NMPC_Path_Tracking_B.Jx[(((int32_T)(ic[c_k] + 6.0) + 600 * icf_tmp_2)
              + 3600 * i) - 1] = Ck[6 * icf_tmp_2 + c_k];
          }

          Je[(int32_T)ic[icf_tmp_2] - 1] = -1;
        }

        for (c_k = 0; c_k < 6; c_k++) {
          Je[(int32_T)(ic[c_k] + 6.0) - 1] = -1;
        }
      }

      for (c_k = 0; c_k <= 4; c_k += 2) {
        tmp_0 = _mm_loadu_pd(&ic[c_k]);
        _mm_storeu_pd(&ic[c_k], _mm_add_pd(tmp_0, _mm_set1_pd(12.0)));
      }
    }

    c_k = 0;
    for (i = 0; i < 600; i++) {
      if (icf[i]) {
        c_k++;
      }
    }

    tmp_size_idx_0 = c_k;
    c_k = 0;
    for (i = 0; i < 600; i++) {
      if (icf[i]) {
        tmp_data[c_k] = (int16_T)i;
        c_k++;
      }
    }

    c_size[0] = tmp_size_idx_0;
    c_size[1] = 1;
    for (c_k = 0; c_k < tmp_size_idx_0; c_k++) {
      c_data[c_k] = NMPC_Path_Tracking_B.b_c[tmp_data[c_k]];
    }

    Jx_size[0] = tmp_size_idx_0;
    Jx_size[1] = 6;
    Jx_size[2] = 50;
    for (c_k = 0; c_k < 50; c_k++) {
      for (icf_tmp_2 = 0; icf_tmp_2 < 6; icf_tmp_2++) {
        s = (tmp_size_idx_0 / 2) << 1;
        Ck_tmp = s - 2;
        for (i = 0; i <= Ck_tmp; i += 2) {
          Jx_data_tmp = tmp_size_idx_0 * icf_tmp_2;
          Jx_data_tmp_0 = tmp_size_idx_0 * 6 * c_k;
          NMPC_Path_Tracking_B.Jx_data[(i + Jx_data_tmp) + Jx_data_tmp_0] =
            NMPC_Path_Tracking_B.Jx[(600 * icf_tmp_2 + tmp_data[i]) + 3600 * c_k];
          NMPC_Path_Tracking_B.Jx_data[((i + Jx_data_tmp) + Jx_data_tmp_0) + 1] =
            NMPC_Path_Tracking_B.Jx[(600 * icf_tmp_2 + tmp_data[i + 1]) + 3600 *
            c_k];
        }

        for (i = s; i < tmp_size_idx_0; i++) {
          NMPC_Path_Tracking_B.Jx_data[(i + tmp_size_idx_0 * icf_tmp_2) +
            tmp_size_idx_0 * 6 * c_k] = NMPC_Path_Tracking_B.Jx[(600 * icf_tmp_2
            + tmp_data[i]) + 3600 * c_k];
        }
      }
    }

    icf_tmp_2 = (tmp_size_idx_0 << 1) * 50;
    for (c_k = 0; c_k < icf_tmp_2; c_k++) {
      NMPC_Path_Tracking_B.tmp_data_c[c_k] = 0.0;
    }

    for (c_k = 0; c_k < tmp_size_idx_0; c_k++) {
      NMPC_Path_Tracking_B.b_c[c_k] = Je[tmp_data[c_k]];
    }

    NMPC_Path_Tracki_reformJacobian(NMPC_Path_Tracking_B.Jx_data, Jx_size,
      NMPC_Path_Tracking_B.tmp_data_c, NMPC_Path_Tracking_B.b_c, &tmp_size_idx_0,
      Jc_data, Jc_size);
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tr_c4_mpclib_anonFcn2(const real_T runtimedata_x[6], const
  real_T runtimedata_OutputMin[300], const real_T runtimedata_OutputMax[300],
  const real_T z[401], real_T varargout_1_data[], int32_T varargout_1_size[2],
  real_T varargout_2[300], real_T varargout_3_data[], int32_T varargout_3_size[2],
  real_T varargout_4[120300])
{
  real_T X[306];
  real_T U[102];
  real_T e;
  int32_T b_varargin_1_size[2];
  int32_T varargin_1_size[2];
  int32_T i;
  int32_T i_0;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int16_T sizes[2];
  int16_T sizes_idx_0;
  int16_T tmp;
  boolean_T sizes_idx_1_tmp;
  NMPC_Path_Tracking_getXUe(z, runtimedata_x, X, U, &e);
  NMPC_Path_Tracki_stateEvolution(X, U, varargout_2, varargout_4);
  NMPC_Path_Tracking_outputBounds(runtimedata_OutputMin, runtimedata_OutputMax,
    X, e, NMPC_Path_Tracking_B.varargin_1_data_m, varargin_1_size,
    NMPC_Path_Tracking_B.b_varargin_1_data, b_varargin_1_size);
  sizes_idx_1_tmp = ((varargin_1_size[0] != 0) && (varargin_1_size[1] != 0));
  if (!sizes_idx_1_tmp) {
    sizes[0] = (int16_T)varargin_1_size[0];
  } else if (sizes_idx_1_tmp) {
    sizes[0] = (int16_T)varargin_1_size[0];
  } else {
    sizes[0] = 0;
  }

  varargout_1_size[0] = sizes[0];
  varargout_1_size[1] = sizes_idx_1_tmp;
  loop_ub = sizes_idx_1_tmp;
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    loop_ub_0 = sizes[0];
    for (i = 0; i < loop_ub_0; i++) {
      varargout_1_data[i] = NMPC_Path_Tracking_B.varargin_1_data_m[i];
    }
  }

  sizes_idx_1_tmp = ((b_varargin_1_size[0] != 0) && (b_varargin_1_size[1] != 0));
  if (sizes_idx_1_tmp) {
    sizes_idx_0 = (int16_T)b_varargin_1_size[0];
  } else {
    sizes_idx_0 = 0;
  }

  varargout_3_size[0] = sizes_idx_0;
  if (sizes_idx_0 == 0) {
    varargout_3_size[1] = b_varargin_1_size[1];
    tmp = (int16_T)b_varargin_1_size[1];
  } else if (sizes_idx_1_tmp) {
    varargout_3_size[1] = b_varargin_1_size[1];
    tmp = (int16_T)b_varargin_1_size[1];
  } else {
    varargout_3_size[1] = 0;
    tmp = 0;
  }

  loop_ub = sizes_idx_0 * tmp;
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    varargout_3_data[i_0] = NMPC_Path_Tracking_B.b_varargin_1_data[i_0];
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Trac_factoryConstruct(int32_T nVarMax, int32_T mConstrMax,
  int32_T mIneq, int32_T mNonlinIneq, s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *obj)
{
  int32_T i;
  obj->nVarMax = nVarMax;
  obj->mNonlinIneq = mNonlinIneq;
  obj->mNonlinEq = 300;
  obj->mIneq = mIneq;
  obj->mEq = 300;
  obj->iNonIneq0 = (mIneq - mNonlinIneq) + 1;
  obj->iNonEq0 = 1;
  obj->sqpFval = 0.0;
  obj->sqpFval_old = 0.0;
  obj->cIneq.size = mIneq;
  obj->cIneq_old.size = mIneq;
  obj->grad.size = nVarMax;
  obj->grad_old.size = nVarMax;
  obj->FunctionEvaluations = 0;
  obj->sqpIterations = 0;
  obj->sqpExitFlag = 0;
  obj->lambdasqp.size = mConstrMax;
  for (i = 0; i < mConstrMax; i++) {
    obj->lambdasqp.data[i] = 0.0;
  }

  obj->lambdaStopTest.size = mConstrMax;
  obj->lambdaStopTestPrev.size = mConstrMax;
  obj->steplength = 1.0;
  obj->delta_x.size = nVarMax;
  for (i = 0; i < nVarMax; i++) {
    obj->delta_x.data[i] = 0.0;
  }

  obj->socDirection.size = nVarMax;
  obj->workingset_old.size = mConstrMax;
  if (mNonlinIneq > 0) {
    obj->JacCineqTrans_old.size[0] = nVarMax;
    obj->JacCineqTrans_old.size[1] = mNonlinIneq;
  } else {
    obj->JacCineqTrans_old.size[0] = 0;
    obj->JacCineqTrans_old.size[1] = 0;
  }

  obj->JacCeqTrans_old.size[0] = nVarMax;
  obj->JacCeqTrans_old.size[1] = 300;
  obj->gradLag.size = nVarMax;
  obj->delta_gradLag.size = nVarMax;
  obj->xstar.size = nVarMax;
  obj->fstar = 0.0;
  obj->firstorderopt = 0.0;
  obj->lambda.size = mConstrMax;
  for (i = 0; i < mConstrMax; i++) {
    obj->lambda.data[i] = 0.0;
  }

  obj->state = 0;
  obj->maxConstr = 0.0;
  obj->iterations = 0;
  obj->searchDir.size = nVarMax;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_factoryConstruct_gzst(int32_T MaxVars, int32_T
  *obj_grad_size, int32_T *obj_Hx_size, boolean_T *obj_hasLinear, int32_T
  *obj_nvar, int32_T *obj_maxVar, real_T *obj_beta, real_T *obj_rho, int32_T
  *obj_objtype, int32_T *obj_prev_objtype, int32_T *obj_prev_nvar, boolean_T
  *obj_prev_hasLinear, real_T *obj_gammaScalar)
{
  *obj_grad_size = MaxVars;
  *obj_Hx_size = MaxVars - 1;
  *obj_hasLinear = false;
  *obj_nvar = 0;
  *obj_maxVar = MaxVars;
  *obj_beta = 0.0;
  *obj_rho = 0.0;
  *obj_objtype = 3;
  *obj_prev_objtype = 3;
  *obj_prev_nvar = 0;
  *obj_prev_hasLinear = false;
  *obj_gammaScalar = 0.0;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Pat_factoryConstruct_gzstv(int32_T mIneqMax, int32_T nVarMax,
  int32_T mConstrMax, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj)
{
  int32_T i;
  obj->mConstr = 0;
  obj->mConstrOrig = 0;
  obj->mConstrMax = mConstrMax;
  obj->nVar = 401;
  obj->nVarOrig = 401;
  obj->nVarMax = nVarMax;
  obj->ldA = nVarMax;
  obj->Aineq.size = mIneqMax * nVarMax;
  obj->bineq.size = mIneqMax;
  obj->Aeq.size = 300 * nVarMax;
  obj->lb.size = nVarMax;
  obj->ub.size = nVarMax;
  obj->indexLB.size = nVarMax;
  obj->indexUB.size = nVarMax;
  obj->indexFixed.size = nVarMax;
  obj->mEqRemoved = 0;
  obj->ATwset.size = nVarMax * mConstrMax;
  obj->bwset.size = mConstrMax;
  obj->nActiveConstr = 0;
  obj->maxConstrWorkspace.size = mConstrMax;
  for (i = 0; i < 5; i++) {
    obj->sizes[i] = 0;
    obj->sizesNormal[i] = 0;
    obj->sizesPhaseOne[i] = 0;
    obj->sizesRegularized[i] = 0;
    obj->sizesRegPhaseOne[i] = 0;
  }

  for (i = 0; i < 6; i++) {
    obj->isActiveIdx[i] = 0;
    obj->isActiveIdxNormal[i] = 0;
    obj->isActiveIdxPhaseOne[i] = 0;
    obj->isActiveIdxRegularized[i] = 0;
    obj->isActiveIdxRegPhaseOne[i] = 0;
  }

  obj->isActiveConstr.size = mConstrMax;
  obj->Wid.size = mConstrMax;
  obj->Wlocalidx.size = mConstrMax;
  for (i = 0; i < 5; i++) {
    obj->nWConstr[i] = 0;
  }

  obj->probType = 3;
  obj->SLACK0 = 1.0E-5;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void computeObjectiveAndUserGradient(const
  s_UduwGwOpHSPENrm1X2xchE_NMPC_T *obj_next_next_next_next_next_ne, const real_T
  x[401], real_T grad_workspace_data[], real_T *fval, int32_T *status)
{
  __m128d tmp_0;
  __m128d tmp_1;
  real_T b_x[401];
  real_T X[306];
  real_T b_X[306];
  real_T gfX[300];
  real_T U[102];
  real_T b_U[102];
  real_T c[100];
  real_T gfU[100];
  real_T b_X_0[6];
  real_T ix[6];
  real_T obj_next_next_next_next_next__0[6];
  real_T tmp[2];
  real_T duk;
  real_T duk_idx_0;
  real_T duk_idx_1;
  real_T e;
  real_T fs;
  real_T gfU_idx_0;
  real_T iu_idx_0;
  real_T iu_idx_1;
  real_T obj_next_next_next_next_next__1;
  real_T umvk_idx_0;
  real_T umvk_idx_1;
  real_T wtYerr_tmp_0;
  int32_T b_X_tmp;
  int32_T i;
  int32_T wtYerr_tmp;
  boolean_T allFinite;
  static const real_T c_0[10000] = { 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.26 };

  static const int8_T b[6] = { 1, 1, 1, 10, 1, 1 };

  static const real_T d[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  NMPC_Path_Tracking_getXUe(x, obj_next_next_next_next_next_ne->runtimedata.x, X,
    U, &e);
  memset(&gfX[0], 0, 300U * sizeof(real_T));
  memset(&gfU[0], 0, 100U * sizeof(real_T));
  fs = 0.0;
  for (b_X_tmp = 0; b_X_tmp < 6; b_X_tmp++) {
    ix[b_X_tmp] = (real_T)b_X_tmp + 1.0;
  }

  iu_idx_0 = 1.0;
  iu_idx_1 = 2.0;
  for (b_X_tmp = 0; b_X_tmp < 51; b_X_tmp++) {
    for (wtYerr_tmp = 0; wtYerr_tmp < 6; wtYerr_tmp++) {
      b_X[wtYerr_tmp + 6 * b_X_tmp] = X[51 * wtYerr_tmp + b_X_tmp];
    }

    i = b_X_tmp << 1;
    b_U[i] = U[b_X_tmp];
    b_U[i + 1] = U[b_X_tmp + 51];
  }

  for (i = 0; i < 50; i++) {
    b_X_tmp = (i + 1) * 6;
    b_X_0[0] = b_X[b_X_tmp];
    b_X_0[1] = b_X[b_X_tmp + 1];
    b_X_0[2] = b_X[b_X_tmp + 2];
    b_X_0[3] = b_X[b_X_tmp + 3] / 10.0;
    b_X_0[4] = b_X[b_X_tmp + 4];
    b_X_0[5] = b_X[b_X_tmp + 5];
    umvk_idx_0 = 0.0;
    for (b_X_tmp = 0; b_X_tmp < 6; b_X_tmp++) {
      wtYerr_tmp = 50 * b_X_tmp + i;
      wtYerr_tmp_0 = obj_next_next_next_next_next_ne->
        runtimedata.OutputWeights[wtYerr_tmp];
      duk_idx_1 = (b_X_0[b_X_tmp] -
                   obj_next_next_next_next_next_ne->runtimedata.ref[wtYerr_tmp] /
                   (real_T)b[b_X_tmp]) * wtYerr_tmp_0;
      umvk_idx_0 += duk_idx_1 * duk_idx_1;
      obj_next_next_next_next_next__0[b_X_tmp] = wtYerr_tmp_0 * duk_idx_1;
    }

    fs += umvk_idx_0;
    for (b_X_tmp = 0; b_X_tmp < 6; b_X_tmp++) {
      umvk_idx_0 = 0.0;
      for (wtYerr_tmp = 0; wtYerr_tmp < 6; wtYerr_tmp++) {
        umvk_idx_0 += d[6 * wtYerr_tmp + b_X_tmp] *
          obj_next_next_next_next_next__0[wtYerr_tmp];
      }

      b_X_0[b_X_tmp] = gfX[(int32_T)ix[b_X_tmp] - 1] + umvk_idx_0;
    }

    for (b_X_tmp = 0; b_X_tmp < 6; b_X_tmp++) {
      umvk_idx_0 = ix[b_X_tmp];
      gfX[(int32_T)umvk_idx_0 - 1] = b_X_0[b_X_tmp];
      ix[b_X_tmp] = umvk_idx_0 + 6.0;
    }

    tmp_1 = _mm_set_pd(2.26, 6.0);
    tmp_0 = _mm_div_pd(_mm_loadu_pd(&b_U[i << 1]), tmp_1);
    _mm_storeu_pd(&tmp[0], tmp_0);
    umvk_idx_0 = tmp[0];
    umvk_idx_1 = tmp[1];
    if (i + 1 == 1) {
      _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(tmp[1], tmp[0]), _mm_div_pd
        (_mm_loadu_pd(&obj_next_next_next_next_next_ne->runtimedata.lastMV[0]),
         tmp_1)));
      duk_idx_0 = tmp[0];
      duk_idx_1 = tmp[1];
    } else {
      tmp_1 = _mm_sub_pd(_mm_set_pd(tmp[1], tmp[0]), _mm_div_pd(_mm_loadu_pd
        (&b_U[(i - 1) << 1]), tmp_1));
      _mm_storeu_pd(&tmp[0], tmp_1);
      duk_idx_0 = tmp[0];
      duk_idx_1 = tmp[1];
    }

    obj_next_next_next_next_next__1 =
      obj_next_next_next_next_next_ne->runtimedata.MVWeights[i];
    wtYerr_tmp_0 = (umvk_idx_0 -
                    obj_next_next_next_next_next_ne->
                    runtimedata.MVScaledTarget[i]) *
      obj_next_next_next_next_next__1;
    umvk_idx_0 = wtYerr_tmp_0;
    gfU_idx_0 = obj_next_next_next_next_next__1 * wtYerr_tmp_0 / 6.0 + gfU
      [(int32_T)iu_idx_0 - 1];
    obj_next_next_next_next_next__1 =
      obj_next_next_next_next_next_ne->runtimedata.MVWeights[i + 50];
    wtYerr_tmp_0 = (umvk_idx_1 -
                    obj_next_next_next_next_next_ne->
                    runtimedata.MVScaledTarget[i + 50]) *
      obj_next_next_next_next_next__1;
    umvk_idx_1 = obj_next_next_next_next_next__1 * wtYerr_tmp_0 / 2.26 + gfU
      [(int32_T)iu_idx_1 - 1];
    gfU[(int32_T)iu_idx_0 - 1] = gfU_idx_0;
    obj_next_next_next_next_next__1 =
      obj_next_next_next_next_next_ne->runtimedata.MVRateWeights[i];
    duk_idx_0 *= obj_next_next_next_next_next__1;
    gfU_idx_0 = umvk_idx_0 * umvk_idx_0;
    duk = duk_idx_0 * duk_idx_0;
    umvk_idx_0 = obj_next_next_next_next_next__1 * duk_idx_0 / 6.0;
    gfU[(int32_T)iu_idx_1 - 1] = umvk_idx_1;
    obj_next_next_next_next_next__1 =
      obj_next_next_next_next_next_ne->runtimedata.MVRateWeights[i + 50];
    duk_idx_0 = obj_next_next_next_next_next__1 * duk_idx_1;
    duk_idx_1 = obj_next_next_next_next_next__1 * duk_idx_0 / 2.26;
    fs = ((wtYerr_tmp_0 * wtYerr_tmp_0 + gfU_idx_0) + fs) + (duk_idx_0 *
      duk_idx_0 + duk);
    wtYerr_tmp_0 = gfU[(int32_T)iu_idx_1 - 1];
    gfU[(int32_T)iu_idx_0 - 1] += umvk_idx_0;
    gfU[(int32_T)iu_idx_1 - 1] = wtYerr_tmp_0 + duk_idx_1;
    if (i + 1 > 1) {
      umvk_idx_1 = gfU[(int32_T)(iu_idx_1 - 2.0) - 1] - duk_idx_1;
      gfU[(int32_T)(iu_idx_0 - 2.0) - 1] -= umvk_idx_0;
      gfU[(int32_T)(iu_idx_1 - 2.0) - 1] = umvk_idx_1;
    }

    iu_idx_0 += 2.0;
    iu_idx_1 += 2.0;
  }

  *fval = 100000.0 * e * e + fs;
  for (b_X_tmp = 0; b_X_tmp < 100; b_X_tmp++) {
    umvk_idx_0 = 0.0;
    for (wtYerr_tmp = 0; wtYerr_tmp < 100; wtYerr_tmp++) {
      umvk_idx_0 += c_0[100 * wtYerr_tmp + b_X_tmp] * (2.0 * gfU[wtYerr_tmp]);
    }

    c[b_X_tmp] = umvk_idx_0;
  }

  for (b_X_tmp = 0; b_X_tmp <= 298; b_X_tmp += 2) {
    tmp_1 = _mm_loadu_pd(&gfX[b_X_tmp]);
    _mm_storeu_pd(&b_x[b_X_tmp], _mm_mul_pd(_mm_set1_pd(2.0), tmp_1));
  }

  memcpy(&b_x[300], &c[0], 100U * sizeof(real_T));
  b_x[400] = 200000.0 * e;
  memcpy(&grad_workspace_data[0], &b_x[0], 401U * sizeof(real_T));
  *status = 1;
  allFinite = rtIsNaN(*fval);
  if (rtIsInf(*fval) || allFinite) {
    if (allFinite) {
      *status = -3;
    } else if (*fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  } else {
    allFinite = true;
    i = -1;
    while (allFinite && (i + 2 <= 401)) {
      e = grad_workspace_data[i + 1];
      allFinite = ((!rtIsInf(e)) && (!rtIsNaN(e)));
      i++;
    }

    if (!allFinite) {
      if (rtIsNaN(grad_workspace_data[i])) {
        *status = -3;
      } else if (grad_workspace_data[i] < 0.0) {
        *status = -1;
      } else {
        *status = -2;
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static int32_T NMPC_Path__checkVectorNonFinite(int32_T N, const real_T vec_data[],
  int32_T iv0)
{
  int32_T idx_current;
  int32_T idx_end;
  int32_T status;
  boolean_T allFinite;
  status = 1;
  allFinite = true;
  idx_current = iv0 - 2;
  idx_end = (iv0 + N) - 1;
  while (allFinite && (idx_current + 2 <= idx_end)) {
    real_T allFinite_tmp;
    allFinite_tmp = vec_data[idx_current + 1];
    allFinite = ((!rtIsInf(allFinite_tmp)) && (!rtIsNaN(allFinite_tmp)));
    idx_current++;
  }

  if (!allFinite) {
    if (rtIsNaN(vec_data[idx_current])) {
      status = -3;
    } else if (vec_data[idx_current] < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }

  return status;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static int32_T NMPC_Pat_checkVectorNonFinite_l(const real_T vec[300])
{
  int32_T idx_current;
  int32_T status;
  boolean_T allFinite;
  status = 1;
  allFinite = true;
  idx_current = -1;
  while (allFinite && (idx_current + 2 <= 300)) {
    real_T allFinite_tmp;
    allFinite_tmp = vec[idx_current + 1];
    allFinite = ((!rtIsInf(allFinite_tmp)) && (!rtIsNaN(allFinite_tmp)));
    idx_current++;
  }

  if (!allFinite) {
    if (rtIsNaN(vec[idx_current])) {
      status = -3;
    } else if (vec[idx_current] < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }

  return status;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static int32_T computeConstraintsAndUserJacobi(int32_T
  obj_next_next_next_next_next_b_, const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T
  *obj_next_next_next_next_next_ne, const real_T x[401], real_T
  Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[300], real_T
  JacIneqTrans_workspace_data[], int32_T iJI_col, int32_T ldJI, real_T
  JacEqTrans_workspace_data[], int32_T ldJE)
{
  real_T b_x[300];
  int32_T a__3_size[2];
  int32_T a__4_size[2];
  int32_T col;
  int32_T col_end;
  int32_T idx_mat;
  int32_T row;
  int32_T status;
  boolean_T allFinite;
  if (obj_next_next_next_next_next_b_ > 0) {
    NMPC_Path_Tr_c4_mpclib_anonFcn2(obj_next_next_next_next_next_ne->x,
      obj_next_next_next_next_next_ne->OutputMin,
      obj_next_next_next_next_next_ne->OutputMax, x,
      NMPC_Path_Tracking_B.a__3_data, a__3_size, b_x,
      NMPC_Path_Tracking_B.a__4_data, a__4_size,
      NMPC_Path_Tracking_B.JacEqTrans_tmp);
    col = (uint16_T)obj_next_next_next_next_next_b_;
    for (row = 0; row < col; row++) {
      Cineq_workspace_data[(ineq0 + row) - 1] =
        NMPC_Path_Tracking_B.a__3_data[row];
    }

    memcpy(&Ceq_workspace[0], &b_x[0], 300U * sizeof(real_T));
    col_end = a__4_size[0];
    for (row = 0; row < col_end; row++) {
      idx_mat = a__4_size[1];
      for (col = 0; col < idx_mat; col++) {
        JacIneqTrans_workspace_data[row + ldJI * ((iJI_col + col) - 1)] =
          NMPC_Path_Tracking_B.a__4_data[a__4_size[0] * col + row];
      }
    }

    for (row = 0; row < 401; row++) {
      for (col = 0; col < 300; col++) {
        JacEqTrans_workspace_data[row + ldJE * col] =
          NMPC_Path_Tracking_B.JacEqTrans_tmp[401 * col + row];
      }
    }
  } else {
    NMPC_Path_Tr_c4_mpclib_anonFcn2(obj_next_next_next_next_next_ne->x,
      obj_next_next_next_next_next_ne->OutputMin,
      obj_next_next_next_next_next_ne->OutputMax, x,
      NMPC_Path_Tracking_B.a__3_data, a__3_size, b_x,
      NMPC_Path_Tracking_B.a__4_data, a__4_size,
      NMPC_Path_Tracking_B.JacEqTrans_tmp);
    memcpy(&Ceq_workspace[0], &b_x[0], 300U * sizeof(real_T));
    for (row = 0; row < 401; row++) {
      for (col = 0; col < 300; col++) {
        JacEqTrans_workspace_data[row + ldJE * col] =
          NMPC_Path_Tracking_B.JacEqTrans_tmp[401 * col + row];
      }
    }
  }

  status = NMPC_Path__checkVectorNonFinite(obj_next_next_next_next_next_b_,
    Cineq_workspace_data, ineq0);
  if (status == 1) {
    status = NMPC_Pat_checkVectorNonFinite_l(Ceq_workspace);
    if (status == 1) {
      allFinite = true;
      row = -1;
      col = iJI_col;
      col_end = (iJI_col + obj_next_next_next_next_next_b_) - 1;
      while (allFinite && (col <= col_end)) {
        row = -1;
        while (allFinite && (row + 2 <= 401)) {
          idx_mat = ((col - 1) * ldJI + row) + 1;
          allFinite = ((!rtIsInf(JacIneqTrans_workspace_data[idx_mat])) &&
                       (!rtIsNaN(JacIneqTrans_workspace_data[idx_mat])));
          row++;
        }

        col++;
      }

      if (!allFinite) {
        idx_mat = (col - 2) * ldJI + row;
        if (rtIsNaN(JacIneqTrans_workspace_data[idx_mat])) {
          status = -3;
        } else if (JacIneqTrans_workspace_data[idx_mat] < 0.0) {
          status = -1;
        } else {
          status = -2;
        }
      } else {
        allFinite = true;
        row = -1;
        col = -1;
        while (allFinite && (col + 2 <= 300)) {
          row = -1;
          while (allFinite && (row + 2 <= 401)) {
            col_end = ((col + 1) * ldJE + row) + 1;
            allFinite = ((!rtIsInf(JacEqTrans_workspace_data[col_end])) &&
                         (!rtIsNaN(JacEqTrans_workspace_data[col_end])));
            row++;
          }

          col++;
        }

        if (!allFinite) {
          col_end = ldJE * col + row;
          if (rtIsNaN(JacEqTrans_workspace_data[col_end])) {
            status = -3;
          } else if (JacEqTrans_workspace_data[col_end] < 0.0) {
            status = -1;
          } else {
            status = -2;
          }
        }
      }
    }
  }

  return status;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void evalObjAndConstrAndDerivatives(int32_T
  obj_next_next_next_next_next_b_, const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T
  *obj_next_next_next_next_next_ne, const s_UduwGwOpHSPENrm1X2xchE_NMPC_T
  *obj_next_next_next_next_next__0, const real_T x[401], real_T
  grad_workspace_data[], real_T Cineq_workspace_data[], int32_T ineq0, real_T
  Ceq_workspace[300], real_T JacIneqTrans_workspace_data[], int32_T iJI_col,
  int32_T ldJI, real_T JacEqTrans_workspace_data[], int32_T ldJE, real_T *fval,
  int32_T *status)
{
  computeObjectiveAndUserGradient(obj_next_next_next_next_next__0, x,
    grad_workspace_data, fval, status);
  if (*status == 1) {
    *status = computeConstraintsAndUserJacobi(obj_next_next_next_next_next_b_,
      obj_next_next_next_next_next_ne, x, Cineq_workspace_data, ineq0,
      Ceq_workspace, JacIneqTrans_workspace_data, iJI_col, ldJI,
      JacEqTrans_workspace_data, ldJE);
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Pa_modifyOverheadPhaseOne_(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj)
{
  int32_T d;
  int32_T idx;
  int32_T idxEq;
  idxEq = (uint16_T)obj->sizes[0];
  for (idx = 0; idx < idxEq; idx++) {
    obj->ATwset.data[(obj->nVar + obj->ldA * idx) - 1] = 0.0;
  }

  for (idx = 0; idx < 300; idx++) {
    idxEq = (obj->ldA * idx + obj->nVar) - 1;
    obj->Aeq.data[idxEq] = 0.0;
    obj->ATwset.data[idxEq + obj->ldA * (obj->isActiveIdx[1] - 1)] = 0.0;
  }

  idxEq = (uint16_T)obj->sizes[2];
  for (idx = 0; idx < idxEq; idx++) {
    obj->Aineq.data[(obj->nVar + obj->ldA * idx) - 1] = -1.0;
  }

  obj->indexLB.data[obj->sizes[3] - 1] = obj->nVar;
  obj->lb.data[obj->nVar - 1] = 1.0E-5;
  idxEq = obj->isActiveIdx[2];
  d = obj->nActiveConstr;
  for (idx = idxEq; idx <= d; idx++) {
    obj->ATwset.data[(obj->nVar + obj->ldA * (idx - 1)) - 1] = -1.0;
  }

  idxEq = obj->isActiveIdx[4] - 1;
  if (obj->nWConstr[4] > 0) {
    d = obj->sizesNormal[4];
    for (idx = d; idx >= 1; idx--) {
      int32_T tmp;
      tmp = idxEq + idx;
      obj->isActiveConstr.data[tmp] = obj->isActiveConstr.data[tmp - 1];
    }
  } else {
    obj->isActiveConstr.data[(obj->isActiveIdx[4] + obj->sizesNormal[4]) - 1] =
      false;
  }

  obj->isActiveConstr.data[obj->isActiveIdx[4] - 1] = false;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracki_setProblemType(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj,
  int32_T PROBLEM_TYPE)
{
  int32_T c;
  int32_T colOffsetATw;
  int32_T colOffsetAineq;
  int32_T d;
  int32_T g;
  int32_T h;
  int32_T idxUpperExisting;
  int32_T idx_col;
  int32_T offsetEq1;
  int32_T offsetEq2;
  switch (PROBLEM_TYPE) {
   case 3:
    obj->nVar = 401;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      idxUpperExisting = obj->isActiveIdx[4] - 1;
      offsetEq1 = (uint16_T)obj->sizesNormal[4];
      for (colOffsetATw = 0; colOffsetATw < offsetEq1; colOffsetATw++) {
        c = idxUpperExisting + colOffsetATw;
        obj->isActiveConstr.data[(obj->isActiveIdxNormal[4] + colOffsetATw) - 1]
          = obj->isActiveConstr.data[c];
        obj->isActiveConstr.data[c] = false;
      }
    }

    for (c = 0; c < 5; c++) {
      obj->sizes[c] = obj->sizesNormal[c];
    }

    for (c = 0; c < 6; c++) {
      obj->isActiveIdx[c] = obj->isActiveIdxNormal[c];
    }
    break;

   case 1:
    obj->nVar = 402;
    obj->mConstr = obj->mConstrOrig + 1;
    for (c = 0; c < 5; c++) {
      obj->sizes[c] = obj->sizesPhaseOne[c];
    }

    NMPC_Pa_modifyOverheadPhaseOne_(obj);
    for (c = 0; c < 6; c++) {
      obj->isActiveIdx[c] = obj->isActiveIdxPhaseOne[c];
    }
    break;

   case 2:
    obj->nVar = obj->nVarMax - 1;
    obj->mConstr = obj->mConstrMax - 1;
    for (c = 0; c < 5; c++) {
      obj->sizes[c] = obj->sizesRegularized[c];
    }

    if (obj->probType != 4) {
      offsetEq2 = obj->sizes[2] + 701;
      offsetEq1 = obj->sizes[2] + 401;
      c = (uint16_T)obj->sizes[0];
      for (idx_col = 0; idx_col < c; idx_col++) {
        colOffsetATw = obj->ldA * idx_col;
        d = obj->nVar;
        for (colOffsetAineq = 402; colOffsetAineq <= d; colOffsetAineq++) {
          obj->ATwset.data[(colOffsetAineq + colOffsetATw) - 1] = 0.0;
        }
      }

      idx_col = (uint16_T)obj->sizes[2];
      for (colOffsetATw = 0; colOffsetATw < idx_col; colOffsetATw++) {
        colOffsetAineq = obj->ldA * colOffsetATw - 1;
        for (c = 402; c <= colOffsetATw + 401; c++) {
          obj->Aineq.data[c + colOffsetAineq] = 0.0;
        }

        obj->Aineq.data[(colOffsetATw + colOffsetAineq) + 402] = -1.0;
        d = obj->nVar;
        for (c = colOffsetATw + 403; c <= d; c++) {
          obj->Aineq.data[c + colOffsetAineq] = 0.0;
        }
      }

      for (idx_col = 0; idx_col < 300; idx_col++) {
        colOffsetAineq = obj->ldA * idx_col - 1;
        colOffsetATw = (obj->isActiveIdx[1] - 1) * obj->ldA + colOffsetAineq;
        for (c = 402; c <= offsetEq1; c++) {
          obj->Aeq.data[c + colOffsetAineq] = 0.0;
          obj->ATwset.data[c + colOffsetATw] = 0.0;
        }

        d = offsetEq2 + idx_col;
        g = d - 300;
        for (c = offsetEq2 - 299; c <= g; c++) {
          obj->Aeq.data[c + colOffsetAineq] = 0.0;
          obj->ATwset.data[c + colOffsetATw] = 0.0;
        }

        c = d + colOffsetAineq;
        obj->Aeq.data[c - 299] = -1.0;
        g = d + colOffsetATw;
        obj->ATwset.data[g - 299] = -1.0;
        h = d - 298;
        for (idxUpperExisting = h; idxUpperExisting <= offsetEq2;
             idxUpperExisting++) {
          obj->Aeq.data[idxUpperExisting + colOffsetAineq] = 0.0;
          obj->ATwset.data[idxUpperExisting + colOffsetATw] = 0.0;
        }

        for (idxUpperExisting = offsetEq2 + 1; idxUpperExisting <= d;
             idxUpperExisting++) {
          obj->Aeq.data[idxUpperExisting + colOffsetAineq] = 0.0;
          obj->ATwset.data[idxUpperExisting + colOffsetATw] = 0.0;
        }

        obj->Aeq.data[c + 1] = 1.0;
        obj->ATwset.data[g + 1] = 1.0;
        d += 2;
        g = obj->nVar;
        for (c = d; c <= g; c++) {
          obj->Aeq.data[c + colOffsetAineq] = 0.0;
          obj->ATwset.data[c + colOffsetATw] = 0.0;
        }
      }

      idxUpperExisting = 401;
      offsetEq1 = obj->sizesNormal[3] + 1;
      offsetEq2 = obj->sizesRegularized[3];
      for (colOffsetATw = offsetEq1; colOffsetATw <= offsetEq2; colOffsetATw++)
      {
        idxUpperExisting++;
        obj->indexLB.data[colOffsetATw - 1] = idxUpperExisting;
      }

      if (obj->nWConstr[4] > 0) {
        idxUpperExisting = (uint16_T)obj->sizesRegularized[4];
        for (colOffsetATw = 0; colOffsetATw < idxUpperExisting; colOffsetATw++)
        {
          obj->isActiveConstr.data[obj->isActiveIdxRegularized[4] + colOffsetATw]
            = obj->isActiveConstr.data[(obj->isActiveIdx[4] + colOffsetATw) - 1];
        }
      }

      idxUpperExisting = obj->isActiveIdx[4];
      offsetEq1 = obj->isActiveIdxRegularized[4];
      for (colOffsetATw = idxUpperExisting; colOffsetATw < offsetEq1;
           colOffsetATw++) {
        obj->isActiveConstr.data[colOffsetATw - 1] = false;
      }

      idxUpperExisting = obj->sizes[2] + 1001;
      for (colOffsetATw = 402; colOffsetATw <= idxUpperExisting; colOffsetATw++)
      {
        obj->lb.data[colOffsetATw - 1] = 0.0;
      }

      offsetEq1 = obj->isActiveIdx[2];
      offsetEq2 = obj->nActiveConstr;
      for (idxUpperExisting = offsetEq1; idxUpperExisting <= offsetEq2;
           idxUpperExisting++) {
        colOffsetATw = (idxUpperExisting - 1) * obj->ldA - 1;
        if (obj->Wid.data[idxUpperExisting - 1] == 3) {
          c = obj->Wlocalidx.data[idxUpperExisting - 1];
          colOffsetAineq = c + 400;
          for (idx_col = 402; idx_col <= colOffsetAineq; idx_col++) {
            obj->ATwset.data[idx_col + colOffsetATw] = 0.0;
          }

          obj->ATwset.data[(c + colOffsetATw) + 401] = -1.0;
          colOffsetAineq = c + 402;
          c = obj->nVar;
          for (idx_col = colOffsetAineq; idx_col <= c; idx_col++) {
            obj->ATwset.data[idx_col + colOffsetATw] = 0.0;
          }
        } else {
          colOffsetAineq = obj->nVar;
          for (idx_col = 402; idx_col <= colOffsetAineq; idx_col++) {
            obj->ATwset.data[idx_col + colOffsetATw] = 0.0;
          }
        }
      }
    }

    for (c = 0; c < 6; c++) {
      obj->isActiveIdx[c] = obj->isActiveIdxRegularized[c];
    }
    break;

   default:
    obj->nVar = obj->nVarMax;
    obj->mConstr = obj->mConstrMax;
    for (c = 0; c < 5; c++) {
      obj->sizes[c] = obj->sizesRegPhaseOne[c];
    }

    NMPC_Pa_modifyOverheadPhaseOne_(obj);
    for (c = 0; c < 6; c++) {
      obj->isActiveIdx[c] = obj->isActiveIdxRegPhaseOne[c];
    }
    break;
  }

  obj->probType = PROBLEM_TYPE;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Trackin_initActiveSet(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj)
{
  int32_T colOffsetATw;
  int32_T f;
  int32_T iATw0;
  int32_T iAeq0;
  int32_T idx;
  int32_T idxFillStart;
  NMPC_Path_Tracki_setProblemType(obj, 3);
  idxFillStart = obj->isActiveIdx[2];
  colOffsetATw = obj->mConstrMax;
  for (idx = idxFillStart; idx <= colOffsetATw; idx++) {
    obj->isActiveConstr.data[idx - 1] = false;
  }

  obj->nWConstr[0] = obj->sizes[0];
  obj->nWConstr[1] = 300;
  obj->nWConstr[2] = 0;
  obj->nWConstr[3] = 0;
  obj->nWConstr[4] = 0;
  obj->nActiveConstr = obj->nWConstr[0] + 300;
  idxFillStart = (uint16_T)obj->sizes[0];
  for (idx = 0; idx < idxFillStart; idx++) {
    obj->Wid.data[idx] = 1;
    obj->Wlocalidx.data[idx] = idx + 1;
    obj->isActiveConstr.data[idx] = true;
    colOffsetATw = obj->ldA * idx;
    iATw0 = (uint16_T)(obj->indexFixed.data[idx] - 1);
    for (iAeq0 = 0; iAeq0 < iATw0; iAeq0++) {
      obj->ATwset.data[iAeq0 + colOffsetATw] = 0.0;
    }

    obj->ATwset.data[(obj->indexFixed.data[idx] + colOffsetATw) - 1] = 1.0;
    iATw0 = obj->indexFixed.data[idx] + 1;
    f = obj->nVar;
    for (iAeq0 = iATw0; iAeq0 <= f; iAeq0++) {
      obj->ATwset.data[(iAeq0 + colOffsetATw) - 1] = 0.0;
    }

    obj->bwset.data[idx] = obj->ub.data[obj->indexFixed.data[idx] - 1];
  }

  for (idx = 0; idx < 300; idx++) {
    colOffsetATw = obj->sizes[0] + idx;
    obj->Wid.data[colOffsetATw] = 2;
    obj->Wlocalidx.data[colOffsetATw] = idx + 1;
    obj->isActiveConstr.data[colOffsetATw] = true;
    iAeq0 = obj->ldA * idx;
    iATw0 = obj->ldA * colOffsetATw;
    f = obj->nVar;
    for (idxFillStart = 0; idxFillStart < f; idxFillStart++) {
      obj->ATwset.data[iATw0 + idxFillStart] = obj->Aeq.data[iAeq0 +
        idxFillStart];
    }

    obj->bwset.data[colOffsetATw] = obj->beq[idx];
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_T_factoryConstruct_gz(int32_T maxRows, int32_T maxCols,
  int32_T *obj_ldq, int32_T obj_QR_size[2], real_T obj_Q_data[], int32_T
  obj_Q_size[2], int32_T obj_jpvt_data[], int32_T *obj_jpvt_size, int32_T
  *obj_mrows, int32_T *obj_ncols, int32_T *obj_tau_size, int32_T *obj_minRowCol,
  boolean_T *obj_usedPivoting)
{
  int32_T i;
  int32_T loop_ub_tmp;
  *obj_ldq = maxRows;
  obj_QR_size[0] = maxRows;
  obj_QR_size[1] = maxCols;
  obj_Q_size[0] = maxRows;
  obj_Q_size[1] = maxRows;
  loop_ub_tmp = maxRows * maxRows;
  for (i = 0; i < loop_ub_tmp; i++) {
    obj_Q_data[i] = 0.0;
  }

  *obj_jpvt_size = maxCols;
  for (i = 0; i < maxCols; i++) {
    obj_jpvt_data[i] = 0;
  }

  *obj_mrows = 0;
  *obj_ncols = 0;
  if (maxRows <= maxCols) {
    *obj_tau_size = maxRows;
  } else {
    *obj_tau_size = maxCols;
  }

  *obj_minRowCol = 0;
  *obj_usedPivoting = false;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path__factoryConstruct_gzs(int32_T MaxDims, int32_T
  obj_FMat_size[2], int32_T *obj_ldm, int32_T *obj_ndims, int32_T *obj_info,
  real_T *obj_scaleFactor, boolean_T *obj_ConvexCheck, real_T *obj_regTol_,
  real_T *obj_workspace_, real_T *obj_workspace2_)
{
  obj_FMat_size[0] = MaxDims;
  obj_FMat_size[1] = MaxDims;
  *obj_ldm = MaxDims;
  *obj_ndims = 0;
  *obj_info = 0;
  *obj_scaleFactor = 0.0;
  *obj_ConvexCheck = true;
  *obj_regTol_ = (rtInf);
  *obj_workspace_ = (rtInf);
  *obj_workspace2_ = (rtInf);
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracki_computeGradLag(real_T workspace_data[], int32_T ldA,
  int32_T nVar, const real_T grad_data[], int32_T mIneq, const real_T
  AineqTrans_data[], const real_T AeqTrans_data[], const int32_T
  finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[], int32_T mLB,
  const int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[])
{
  int32_T b;
  int32_T f;
  int32_T finiteFixed;
  int32_T g;
  int32_T iL0;
  int32_T ix;
  b = (uint16_T)nVar;
  for (iL0 = 0; iL0 < b; iL0++) {
    workspace_data[iL0] = grad_data[iL0];
  }

  b = (uint16_T)mFixed;
  for (iL0 = 0; iL0 < b; iL0++) {
    finiteFixed = finiteFixed_data[iL0];
    workspace_data[finiteFixed - 1] += lambda_data[iL0];
  }

  ix = mFixed;
  f = ldA * 299 + 1;
  for (b = 1; ldA < 0 ? b >= f : b <= f; b += ldA) {
    g = (b + nVar) - 1;
    for (finiteFixed = b; finiteFixed <= g; finiteFixed++) {
      iL0 = finiteFixed - b;
      workspace_data[iL0] += AeqTrans_data[finiteFixed - 1] * lambda_data[ix];
    }

    ix++;
  }

  if (mIneq != 0) {
    ix = mFixed + 300;
    f = (mIneq - 1) * ldA + 1;
    for (b = 1; ldA < 0 ? b >= f : b <= f; b += ldA) {
      g = (b + nVar) - 1;
      for (finiteFixed = b; finiteFixed <= g; finiteFixed++) {
        iL0 = finiteFixed - b;
        workspace_data[iL0] += AineqTrans_data[finiteFixed - 1] * lambda_data[ix];
      }

      ix++;
    }
  }

  iL0 = (mFixed + mIneq) + 300;
  finiteFixed = (uint16_T)mLB - 1;
  for (b = 0; b <= finiteFixed; b++) {
    ix = finiteLB_data[b];
    workspace_data[ix - 1] -= lambda_data[iL0 + b];
  }

  if ((uint16_T)mLB - 1 >= 0) {
    iL0 += (uint16_T)mLB;
  }

  finiteFixed = (uint16_T)mUB - 1;
  for (b = 0; b <= finiteFixed; b++) {
    ix = finiteUB_data[b];
    workspace_data[ix - 1] += lambda_data[iL0 + b];
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T NMPC_Pat_computePrimalFeasError(const real_T x[401], int32_T
  mLinIneq, int32_T mNonlinIneq, const real_T cIneq_data[], const real_T cEq[300],
  const int32_T finiteLB_data[], int32_T mLB, const real_T lb[401], const
  int32_T finiteUB_data[], int32_T mUB)
{
  real_T feasError;
  int32_T idx;
  int32_T mIneq;
  feasError = 0.0;
  mIneq = mNonlinIneq + mLinIneq;
  for (idx = 0; idx < 300; idx++) {
    feasError = fmax(feasError, fabs(cEq[idx]));
  }

  for (idx = 0; idx < mIneq; idx++) {
    feasError = fmax(feasError, cIneq_data[idx]);
  }

  mIneq = (uint16_T)mLB;
  for (idx = 0; idx < mIneq; idx++) {
    int32_T finiteLB;
    finiteLB = finiteLB_data[idx];
    feasError = fmax(feasError, lb[finiteLB - 1] - x[finiteLB - 1]);
  }

  mIneq = (uint16_T)mUB;
  for (idx = 0; idx < mIneq; idx++) {
    feasError = fmax(feasError, x[finiteUB_data[idx] - 1] - (rtInf));
  }

  return feasError;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path__computeDualFeasError(int32_T nVar, const real_T
  gradLag_data[], boolean_T *gradOK, real_T *val)
{
  int32_T idx;
  boolean_T exitg1;
  *gradOK = true;
  *val = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= (uint16_T)nVar - 1)) {
    *gradOK = ((!rtIsInf(gradLag_data[idx])) && (!rtIsNaN(gradLag_data[idx])));
    if (!*gradOK) {
      exitg1 = true;
    } else {
      *val = fmax(*val, fabs(gradLag_data[idx]));
      idx++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_test_exit(sG8JZ69axY52WWR6RKyApQC_NMPC__T
  *MeritFunction, const s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *WorkingSet,
  s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *TrialState, const real_T lb[401], boolean_T
  *Flags_gradOK, boolean_T *Flags_fevalOK, boolean_T *Flags_done, boolean_T
  *Flags_stepAccepted, boolean_T *Flags_failedLineSearch, int32_T
  *Flags_stepType)
{
  real_T s;
  real_T smax;
  int32_T idx_max;
  int32_T k;
  int32_T mLambda;
  int32_T nVar;
  boolean_T isFeasible;
  *Flags_fevalOK = true;
  *Flags_stepAccepted = false;
  *Flags_failedLineSearch = false;
  *Flags_stepType = 1;
  nVar = WorkingSet->nVar;
  mLambda = (((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes
              [3]) + WorkingSet->sizes[4]) + 299;
  for (k = 0; k <= mLambda; k++) {
    TrialState->lambdaStopTest.data[k] = TrialState->lambdasqp.data[k];
  }

  NMPC_Path_Tracki_computeGradLag(TrialState->gradLag.data, WorkingSet->ldA,
    WorkingSet->nVar, TrialState->grad.data, WorkingSet->sizes[2],
    WorkingSet->Aineq.data, WorkingSet->Aeq.data, WorkingSet->indexFixed.data,
    WorkingSet->sizes[0], WorkingSet->indexLB.data, WorkingSet->sizes[3],
    WorkingSet->indexUB.data, WorkingSet->sizes[4],
    TrialState->lambdaStopTest.data);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = fabs(TrialState->grad.data[0]);
      for (k = 2; k <= nVar; k++) {
        s = fabs(TrialState->grad.data[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }

  smax = fmax(1.0, fabs(TrialState->grad.data[idx_max - 1]));
  if (rtIsInf(smax)) {
    smax = 1.0;
  }

  MeritFunction->nlpPrimalFeasError = NMPC_Pat_computePrimalFeasError
    (TrialState->xstarsqp, WorkingSet->sizes[2] - TrialState->mNonlinIneq,
     TrialState->mNonlinIneq, TrialState->cIneq.data, TrialState->cEq,
     WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
     WorkingSet->indexUB.data, WorkingSet->sizes[4]);
  MeritFunction->feasRelativeFactor = fmax(1.0,
    MeritFunction->nlpPrimalFeasError);
  isFeasible = (MeritFunction->nlpPrimalFeasError <= 1.0E-6 *
                MeritFunction->feasRelativeFactor);
  NMPC_Path__computeDualFeasError(WorkingSet->nVar, TrialState->gradLag.data,
    Flags_gradOK, &MeritFunction->nlpDualFeasError);
  if (!*Flags_gradOK) {
    *Flags_done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = 0.0;
    MeritFunction->firstOrderOpt = fmax(MeritFunction->nlpDualFeasError, 0.0);
    for (k = 0; k <= mLambda; k++) {
      TrialState->lambdaStopTestPrev.data[k] = TrialState->lambdaStopTest.data[k];
    }

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-6 * smax)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      *Flags_done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        *Flags_done = true;
        TrialState->sqpExitFlag = -3;
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_saveJacobian(s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *obj,
  int32_T nVar, int32_T mIneq, const real_T JacCineqTrans_data[], int32_T
  ineqCol0, const real_T JacCeqTrans_data[], int32_T ldJ)
{
  int32_T b;
  int32_T iCol;
  int32_T iCol_old;
  int32_T idx_col;
  int32_T k;
  int32_T loop_ub_tmp;
  iCol = (ineqCol0 - 1) * ldJ;
  iCol_old = 0;
  b = mIneq - ineqCol0;
  for (idx_col = 0; idx_col <= b; idx_col++) {
    int32_T c;
    loop_ub_tmp = obj->JacCineqTrans_old.size[0] * obj->JacCineqTrans_old.size[1];
    for (k = 0; k < loop_ub_tmp; k++) {
      NMPC_Path_Tracking_B.y_data[k] = obj->JacCineqTrans_old.data[k];
    }

    c = (uint16_T)nVar;
    for (k = 0; k < c; k++) {
      NMPC_Path_Tracking_B.y_data[iCol_old + k] = JacCineqTrans_data[iCol + k];
    }

    for (k = 0; k < loop_ub_tmp; k++) {
      obj->JacCineqTrans_old.data[k] = NMPC_Path_Tracking_B.y_data[k];
    }

    iCol += ldJ;
    iCol_old += ldJ;
  }

  iCol = 0;
  iCol_old = 0;
  loop_ub_tmp = (uint16_T)nVar;
  for (idx_col = 0; idx_col < 300; idx_col++) {
    for (b = 0; b < loop_ub_tmp; b++) {
      obj->JacCeqTrans_old.data[iCol_old + b] = JacCeqTrans_data[iCol + b];
    }

    iCol += ldJ;
    iCol_old = iCol;
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T NMPC_Path_Tra_computeComplError(const int32_T
  *fscales_lineq_constraint_size, const int32_T *fscales_cineq_constraint_size,
  const real_T xCurrent[401], int32_T mIneq, const real_T cIneq_data[], const
  int32_T finiteLB_data[], int32_T mLB, const real_T lb[401], const int32_T
  finiteUB_data[], int32_T mUB, const real_T lambda_data[], int32_T iL0)
{
  real_T nlpComplError;
  int32_T b_idx;
  int32_T c;
  int32_T iLineq0;
  int32_T ubOffset;
  nlpComplError = 0.0;
  ubOffset = *fscales_lineq_constraint_size;
  if ((mIneq + mLB) + mUB > 0) {
    real_T lbDelta;
    real_T lbLambda;
    for (iLineq0 = 0; iLineq0 < ubOffset; iLineq0++) {
      lbDelta = cIneq_data[iLineq0];
      lbLambda = lambda_data[(iL0 + iLineq0) - 1];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda), fmin
        (fabs(lbDelta), lbLambda)));
    }

    iLineq0 = (iL0 + *fscales_lineq_constraint_size) - 1;
    c = *fscales_cineq_constraint_size;
    for (b_idx = 0; b_idx < c; b_idx++) {
      lbDelta = cIneq_data[*fscales_lineq_constraint_size + b_idx];
      lbLambda = lambda_data[iLineq0 + b_idx];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda), fmin
        (fabs(lbDelta), lbLambda)));
    }

    iLineq0 = (iL0 + mIneq) - 1;
    ubOffset = iLineq0 + mLB;
    c = (uint16_T)mLB;
    for (b_idx = 0; b_idx < c; b_idx++) {
      int32_T finiteLB;
      finiteLB = finiteLB_data[b_idx];
      lbDelta = xCurrent[finiteLB - 1] - lb[finiteLB - 1];
      lbLambda = lambda_data[iLineq0 + b_idx];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda), fmin
        (fabs(lbDelta), lbLambda)));
    }

    iLineq0 = (uint16_T)mUB;
    for (c = 0; c < iLineq0; c++) {
      lbDelta = lambda_data[ubOffset + c];
      lbLambda = (rtInf) - xCurrent[finiteUB_data[c] - 1];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbLambda * lbDelta), fmin
        (lbLambda, lbDelta)));
    }
  }

  return nlpComplError;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Trac_computeGradLag_h(real_T workspace_data[], int32_T ldA,
  int32_T nVar, const real_T grad_data[], int32_T mIneq, const real_T
  AineqTrans_data[], const real_T AeqTrans_data[], const int32_T
  finiteFixed_data[], int32_T mFixed, const int32_T finiteLB_data[], int32_T mLB,
  const int32_T finiteUB_data[], int32_T mUB, const real_T lambda_data[])
{
  int32_T f;
  int32_T finiteFixed;
  int32_T g;
  int32_T i;
  int32_T iL0;
  int32_T ix;
  iL0 = (uint16_T)nVar;
  for (i = 0; i < iL0; i++) {
    workspace_data[i] = grad_data[i];
  }

  i = (uint16_T)mFixed;
  for (iL0 = 0; iL0 < i; iL0++) {
    finiteFixed = finiteFixed_data[iL0];
    workspace_data[finiteFixed - 1] += lambda_data[iL0];
  }

  ix = mFixed;
  f = ldA * 299 + 1;
  for (i = 1; ldA < 0 ? i >= f : i <= f; i += ldA) {
    g = (i + nVar) - 1;
    for (finiteFixed = i; finiteFixed <= g; finiteFixed++) {
      iL0 = finiteFixed - i;
      workspace_data[iL0] += AeqTrans_data[finiteFixed - 1] * lambda_data[ix];
    }

    ix++;
  }

  if (mIneq != 0) {
    ix = mFixed + 300;
    f = (mIneq - 1) * ldA + 1;
    for (i = 1; ldA < 0 ? i >= f : i <= f; i += ldA) {
      g = (i + nVar) - 1;
      for (finiteFixed = i; finiteFixed <= g; finiteFixed++) {
        iL0 = finiteFixed - i;
        workspace_data[iL0] += AineqTrans_data[finiteFixed - 1] * lambda_data[ix];
      }

      ix++;
    }
  }

  iL0 = (mFixed + mIneq) + 300;
  finiteFixed = (uint16_T)mLB - 1;
  for (i = 0; i <= finiteFixed; i++) {
    ix = finiteLB_data[i];
    workspace_data[ix - 1] -= lambda_data[iL0 + i];
  }

  if ((uint16_T)mLB - 1 >= 0) {
    iL0 += (uint16_T)mLB;
  }

  finiteFixed = (uint16_T)mUB - 1;
  for (i = 0; i <= finiteFixed; i++) {
    ix = finiteUB_data[i];
    workspace_data[ix - 1] += lambda_data[iL0 + i];
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Pat_computeDualFeasError_b(int32_T nVar, const real_T
  gradLag_data[], boolean_T *gradOK, real_T *val)
{
  int32_T idx;
  boolean_T exitg1;
  *gradOK = true;
  *val = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= (uint16_T)nVar - 1)) {
    *gradOK = ((!rtIsInf(gradLag_data[idx])) && (!rtIsNaN(gradLag_data[idx])));
    if (!*gradOK) {
      exitg1 = true;
    } else {
      *val = fmax(*val, fabs(gradLag_data[idx]));
      idx++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_P_updateWorkingSetForNewQP(const real_T xk[401],
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *WorkingSet, int32_T mIneq, int32_T
  mNonlinIneq, const real_T cIneq_data[], const real_T cEq[300], int32_T mLB,
  const real_T lb[401], int32_T mUB, int32_T mFixed)
{
  real_T tmp[2];
  int32_T i;
  int32_T iEq0;
  int32_T idx;
  int32_T iw0;
  int32_T nVar;
  nVar = WorkingSet->nVar;
  iw0 = WorkingSet->ldA * mFixed;
  iEq0 = 0;
  for (idx = 0; idx < 300; idx++) {
    real_T WorkingSet_beq;
    WorkingSet_beq = -cEq[idx];
    WorkingSet->beq[idx] = WorkingSet_beq;
    WorkingSet->bwset.data[mFixed + idx] = WorkingSet_beq;
    for (i = 0; i < nVar; i++) {
      WorkingSet->ATwset.data[iw0 + i] = WorkingSet->Aeq.data[iEq0 + i];
    }

    iw0 += WorkingSet->ldA;
    iEq0 += WorkingSet->ldA;
  }

  i = (uint16_T)mIneq;
  iw0 = ((uint16_T)mIneq / 2) << 1;
  iEq0 = iw0 - 2;
  for (idx = 0; idx <= iEq0; idx += 2) {
    _mm_storeu_pd(&WorkingSet->bineq.data[idx], _mm_mul_pd(_mm_loadu_pd
      (&cIneq_data[idx]), _mm_set1_pd(-1.0)));
  }

  for (idx = iw0; idx < i; idx++) {
    WorkingSet->bineq.data[idx] = -cIneq_data[idx];
  }

  i = (uint16_T)mLB;
  for (idx = 0; idx < i; idx++) {
    WorkingSet->lb.data[WorkingSet->indexLB.data[idx] - 1] = -lb
      [WorkingSet->indexLB.data[idx] - 1] + xk[WorkingSet->indexLB.data[idx] - 1];
  }

  i = (uint16_T)mUB;
  for (idx = 0; idx < i; idx++) {
    WorkingSet->ub.data[WorkingSet->indexUB.data[idx] - 1] = (rtInf) -
      xk[WorkingSet->indexUB.data[idx] - 1];
  }

  i = (uint16_T)mFixed;
  for (idx = 0; idx < i; idx++) {
    _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set1_pd((rtInf)), _mm_set1_pd
      (xk[WorkingSet->indexFixed.data[idx] - 1])));
    WorkingSet->ub.data[WorkingSet->indexFixed.data[idx] - 1] = tmp[0];
    WorkingSet->bwset.data[idx] = tmp[1];
  }

  if (WorkingSet->nActiveConstr > mFixed + 300) {
    iw0 = WorkingSet->nActiveConstr;
    for (idx = mFixed + 301; idx <= iw0; idx++) {
      switch (WorkingSet->Wid.data[idx - 1]) {
       case 4:
        WorkingSet->bwset.data[idx - 1] = WorkingSet->lb.data
          [WorkingSet->indexLB.data[WorkingSet->Wlocalidx.data[idx - 1] - 1] - 1];
        break;

       case 5:
        WorkingSet->bwset.data[idx - 1] = WorkingSet->ub.data
          [WorkingSet->indexUB.data[WorkingSet->Wlocalidx.data[idx - 1] - 1] - 1];
        break;

       default:
        {
          i = WorkingSet->Wlocalidx.data[idx - 1];
          WorkingSet->bwset.data[idx - 1] = WorkingSet->bineq.data[i - 1];
          if ((mNonlinIneq > 0) && (i > mIneq - mNonlinIneq)) {
            int32_T g;
            int32_T ix0;
            iEq0 = (idx - 1) * WorkingSet->ldA;
            ix0 = (i - 1) * WorkingSet->ldA;
            g = (uint16_T)nVar;
            for (i = 0; i < g; i++) {
              WorkingSet->ATwset.data[iEq0 + i] = WorkingSet->Aineq.data[ix0 + i];
            }
          }
        }
        break;
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xswap(int32_T n, real_T x_data[], int32_T ix0,
  int32_T iy0)
{
  int32_T k;
  for (k = 0; k < n; k++) {
    real_T temp;
    int32_T temp_tmp;
    int32_T tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x_data[temp_tmp];
    tmp = (iy0 + k) - 1;
    x_data[temp_tmp] = x_data[tmp];
    x_data[tmp] = temp;
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T NMPC_Path_Tracking_xnrm2(int32_T n, const real_T x_data[], int32_T
  ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x_data[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (k = ix0; k < kend; k++) {
        real_T absxk;
        absxk = fabs(x_data[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = sqrt(b * b + 1.0) * a;
  } else if (rtIsNaN(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T NMPC_Path_Tracking_xzlarfg(int32_T n, real_T *alpha1, real_T
  x_data[], int32_T ix0)
{
  __m128d tmp;
  real_T a;
  real_T tau;
  real_T xnorm;
  int32_T c;
  int32_T knt;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T vectorUB_tmp;
  tau = 0.0;
  if (n > 0) {
    xnorm = NMPC_Path_Tracking_xnrm2(n - 1, x_data, ix0);
    if (xnorm != 0.0) {
      xnorm = rt_hypotd_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (fabs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        scalarLB = (ix0 + n) - 2;
        do {
          knt++;
          vectorUB = ((((scalarLB - ix0) + 1) / 2) << 1) + ix0;
          vectorUB_tmp = vectorUB - 2;
          for (c = ix0; c <= vectorUB_tmp; c += 2) {
            tmp = _mm_loadu_pd(&x_data[c - 1]);
            _mm_storeu_pd(&x_data[c - 1], _mm_mul_pd(tmp, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (c = vectorUB; c <= scalarLB; c++) {
            x_data[c - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while ((fabs(xnorm) < 1.0020841800044864E-292) && (knt < 20));

        xnorm = rt_hypotd_snf(*alpha1, NMPC_Path_Tracking_xnrm2(n - 1, x_data,
          ix0));
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        a = 1.0 / (*alpha1 - xnorm);
        for (c = ix0; c <= vectorUB_tmp; c += 2) {
          tmp = _mm_loadu_pd(&x_data[c - 1]);
          _mm_storeu_pd(&x_data[c - 1], _mm_mul_pd(tmp, _mm_set1_pd(a)));
        }

        for (c = vectorUB; c <= scalarLB; c++) {
          x_data[c - 1] *= a;
        }

        for (c = 0; c < knt; c++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        a = 1.0 / (*alpha1 - xnorm);
        c = (ix0 + n) - 2;
        scalarLB = ((((c - ix0) + 1) / 2) << 1) + ix0;
        vectorUB = scalarLB - 2;
        for (knt = ix0; knt <= vectorUB; knt += 2) {
          tmp = _mm_loadu_pd(&x_data[knt - 1]);
          _mm_storeu_pd(&x_data[knt - 1], _mm_mul_pd(tmp, _mm_set1_pd(a)));
        }

        for (knt = scalarLB; knt <= c; knt++) {
          x_data[knt - 1] *= a;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgemv(int32_T m, int32_T n, const real_T A_data[],
  int32_T ia0, int32_T lda, const real_T x_data[], int32_T ix0, real_T y_data[])
{
  int32_T b_iy;
  int32_T ia;
  if (n != 0) {
    int32_T b;
    int32_T iy;
    for (b_iy = 0; b_iy < n; b_iy++) {
      y_data[b_iy] = 0.0;
    }

    iy = 0;
    b = (n - 1) * lda + ia0;
    for (b_iy = ia0; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda) {
      real_T c;
      int32_T d;
      c = 0.0;
      d = (b_iy + m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        c += x_data[((ix0 + ia) - b_iy) - 1] * A_data[ia - 1];
      }

      y_data[iy] += c;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgerc(int32_T m, int32_T n, real_T alpha1,
  int32_T ix0, const real_T y_data[], real_T A_data[], int32_T ia0, int32_T lda)
{
  int32_T ijA;
  int32_T j;
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (j = 0; j < n; j++) {
      real_T temp;
      temp = y_data[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = (m + jA) - 1;
        for (ijA = jA; ijA <= b; ijA++) {
          A_data[ijA - 1] += A_data[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += lda;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C_data[], int32_T ic0, int32_T ldc, real_T work_data[])
{
  int32_T coltop;
  int32_T exitg1;
  int32_T ia;
  int32_T lastc;
  int32_T lastv;
  boolean_T exitg2;
  if (tau != 0.0) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C_data[lastc - 2] == 0.0)) {
      lastv--;
      lastc--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coltop = (lastc - 1) * ldc + ic0;
      ia = coltop;
      do {
        exitg1 = 0;
        if (ia <= (coltop + lastv) - 1) {
          if (C_data[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    NMPC_Path_Tracking_xgemv(lastv, lastc, C_data, ic0, ldc, C_data, iv0,
      work_data);
    NMPC_Path_Tracking_xgerc(lastv, lastc, -tau, iv0, work_data, C_data, ic0,
      ldc);
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_qrf(real_T A_data[], const int32_T A_size[2],
  int32_T m, int32_T n, int32_T nfxd, real_T tau_data[])
{
  real_T b_atmp;
  real_T tau;
  int32_T i;
  int32_T ii;
  int32_T lda;
  int32_T loop_ub;
  int32_T mmi;
  lda = A_size[0];
  loop_ub = A_size[1];
  for (i = 0; i < loop_ub; i++) {
    NMPC_Path_Tracking_B.work_data[i] = 0.0;
  }

  loop_ub = (uint16_T)nfxd;
  for (i = 0; i < loop_ub; i++) {
    ii = i * lda + i;
    mmi = m - i;
    if (i + 1 < m) {
      b_atmp = A_data[ii];
      tau = NMPC_Path_Tracking_xzlarfg(mmi, &b_atmp, A_data, ii + 2);
      tau_data[i] = tau;
      A_data[ii] = b_atmp;
    } else {
      tau = 0.0;
      tau_data[i] = 0.0;
    }

    if (i + 1 < n) {
      b_atmp = A_data[ii];
      A_data[ii] = 1.0;
      NMPC_Path_Tracking_xzlarf(mmi, (n - i) - 1, ii + 1, tau, A_data, (ii + lda)
        + 1, lda, NMPC_Path_Tracking_B.work_data);
      A_data[ii] = b_atmp;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_qrpf(real_T A_data[], const int32_T A_size[2],
  int32_T m, int32_T n, int32_T nfxd, real_T tau_data[], int32_T jpvt_data[])
{
  real_T s;
  real_T smax;
  real_T temp2;
  int32_T idxmax;
  int32_T ii;
  int32_T itemp;
  int32_T j;
  int32_T ma;
  int32_T minmn;
  int32_T mmi;
  int32_T nmi;
  int32_T pvt;
  ma = A_size[0];
  if (m <= n) {
    minmn = m;
  } else {
    minmn = n;
  }

  mmi = A_size[1];
  for (j = 0; j < mmi; j++) {
    NMPC_Path_Tracking_B.work_data_c[j] = 0.0;
    NMPC_Path_Tracking_B.vn1_data[j] = 0.0;
    NMPC_Path_Tracking_B.vn2_data[j] = 0.0;
  }

  for (j = nfxd + 1; j <= n; j++) {
    smax = NMPC_Path_Tracking_xnrm2(m - nfxd, A_data, ((j - 1) * ma + nfxd) + 1);
    NMPC_Path_Tracking_B.vn1_data[j - 1] = smax;
    NMPC_Path_Tracking_B.vn2_data[j - 1] = smax;
  }

  for (j = nfxd + 1; j <= minmn; j++) {
    itemp = (j - 1) * ma;
    ii = (itemp + j) - 1;
    nmi = n - j;
    mmi = m - j;
    if (nmi + 1 < 1) {
      idxmax = -2;
    } else {
      idxmax = -1;
      if (nmi + 1 > 1) {
        smax = fabs(NMPC_Path_Tracking_B.vn1_data[j - 1]);
        for (pvt = 2; pvt <= nmi + 1; pvt++) {
          s = fabs(NMPC_Path_Tracking_B.vn1_data[(j + pvt) - 2]);
          if (s > smax) {
            idxmax = pvt - 2;
            smax = s;
          }
        }
      }
    }

    pvt = j + idxmax;
    if (pvt + 1 != j) {
      NMPC_Path_Tracking_xswap(m, A_data, pvt * ma + 1, itemp + 1);
      itemp = jpvt_data[pvt];
      jpvt_data[pvt] = jpvt_data[j - 1];
      jpvt_data[j - 1] = itemp;
      NMPC_Path_Tracking_B.vn1_data[pvt] = NMPC_Path_Tracking_B.vn1_data[j - 1];
      NMPC_Path_Tracking_B.vn2_data[pvt] = NMPC_Path_Tracking_B.vn2_data[j - 1];
    }

    if (j < m) {
      s = A_data[ii];
      smax = NMPC_Path_Tracking_xzlarfg(mmi + 1, &s, A_data, ii + 2);
      tau_data[j - 1] = smax;
      A_data[ii] = s;
    } else {
      smax = 0.0;
      tau_data[j - 1] = 0.0;
    }

    if (j < n) {
      s = A_data[ii];
      A_data[ii] = 1.0;
      NMPC_Path_Tracking_xzlarf(mmi + 1, nmi, ii + 1, smax, A_data, (ii + ma) +
        1, ma, NMPC_Path_Tracking_B.work_data_c);
      A_data[ii] = s;
    }

    for (ii = j + 1; ii <= n; ii++) {
      nmi = (ii - 1) * ma + j;
      smax = NMPC_Path_Tracking_B.vn1_data[ii - 1];
      if (smax != 0.0) {
        s = fabs(A_data[nmi - 1]) / smax;
        s = 1.0 - s * s;
        if (s < 0.0) {
          s = 0.0;
        }

        temp2 = smax / NMPC_Path_Tracking_B.vn2_data[ii - 1];
        temp2 = temp2 * temp2 * s;
        if (temp2 <= 1.4901161193847656E-8) {
          if (j < m) {
            smax = NMPC_Path_Tracking_xnrm2(mmi, A_data, nmi + 1);
            NMPC_Path_Tracking_B.vn1_data[ii - 1] = smax;
            NMPC_Path_Tracking_B.vn2_data[ii - 1] = smax;
          } else {
            NMPC_Path_Tracking_B.vn1_data[ii - 1] = 0.0;
            NMPC_Path_Tracking_B.vn2_data[ii - 1] = 0.0;
          }
        } else {
          NMPC_Path_Tracking_B.vn1_data[ii - 1] = smax * sqrt(s);
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgeqp3(real_T A_data[], const int32_T A_size[2],
  int32_T m, int32_T n, int32_T jpvt_data[], real_T tau_data[], int32_T
  *tau_size)
{
  int32_T b_j;
  int32_T minmn;
  int32_T nfxd;
  static const int32_T offsets[4] = { 0, 1, 2, 3 };

  int32_T ma_tmp;
  ma_tmp = A_size[0];
  if (m <= n) {
    minmn = m;
  } else {
    minmn = n;
  }

  if (A_size[0] <= A_size[1]) {
    nfxd = A_size[0];
  } else {
    nfxd = A_size[1];
  }

  *tau_size = nfxd;
  for (b_j = 0; b_j < nfxd; b_j++) {
    tau_data[b_j] = 0.0;
  }

  if (minmn < 1) {
    minmn = (n / 4) << 2;
    b_j = minmn - 4;
    for (ma_tmp = 0; ma_tmp <= b_j; ma_tmp += 4) {
      _mm_storeu_si128((__m128i *)&jpvt_data[ma_tmp], _mm_add_epi32
                       (_mm_add_epi32(_mm_set1_epi32(ma_tmp), _mm_loadu_si128((
        const __m128i *)&offsets[0])), _mm_set1_epi32(1)));
    }

    for (ma_tmp = minmn; ma_tmp < n; ma_tmp++) {
      jpvt_data[ma_tmp] = ma_tmp + 1;
    }
  } else {
    nfxd = -1;
    for (b_j = 0; b_j < n; b_j++) {
      if (jpvt_data[b_j] != 0) {
        nfxd++;
        if (b_j + 1 != nfxd + 1) {
          NMPC_Path_Tracking_xswap(m, A_data, b_j * ma_tmp + 1, nfxd * ma_tmp +
            1);
          jpvt_data[b_j] = jpvt_data[nfxd];
          jpvt_data[nfxd] = b_j + 1;
        } else {
          jpvt_data[b_j] = b_j + 1;
        }
      } else {
        jpvt_data[b_j] = b_j + 1;
      }
    }

    if (nfxd + 1 <= minmn) {
      nfxd++;
    } else {
      nfxd = minmn;
    }

    NMPC_Path_Tracking_qrf(A_data, A_size, m, n, nfxd, tau_data);
    if (nfxd < minmn) {
      NMPC_Path_Tracking_qrpf(A_data, A_size, m, n, nfxd, tau_data, jpvt_data);
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_factorQRE(s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *obj,
  const real_T A_data[], int32_T mrows, int32_T ncols, int32_T ldA)
{
  int32_T b;
  int32_T idx;
  int32_T ix0;
  int32_T iy0;
  int32_T k;
  boolean_T guard1;
  idx = mrows * ncols;
  guard1 = false;
  if (idx > 0) {
    for (idx = 0; idx < ncols; idx++) {
      ix0 = ldA * idx;
      iy0 = obj->ldq * idx;
      b = (uint16_T)mrows;
      for (k = 0; k < b; k++) {
        obj->QR.data[iy0 + k] = A_data[ix0 + k];
      }
    }

    guard1 = true;
  } else if (idx == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = true;
    obj->mrows = mrows;
    obj->ncols = ncols;
    if (mrows <= ncols) {
      obj->minRowCol = mrows;
    } else {
      obj->minRowCol = ncols;
    }

    NMPC_Path_Tracking_xgeqp3(obj->QR.data, obj->QR.size, mrows, ncols,
      obj->jpvt.data, obj->tau.data, &obj->tau.size);
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xorgqr(int32_T m, int32_T n, int32_T k, real_T
  A_data[], const int32_T A_size[2], int32_T lda, const real_T tau_data[])
{
  __m128d tmp;
  int32_T b_k;
  int32_T c;
  int32_T i;
  int32_T ia;
  int32_T itau;
  int32_T scalarLB;
  int32_T vectorUB;
  if (n >= 1) {
    for (itau = k; itau < n; itau++) {
      ia = itau * lda;
      for (i = 0; i < m; i++) {
        A_data[ia + i] = 0.0;
      }

      A_data[ia + itau] = 1.0;
    }

    itau = k - 1;
    ia = A_size[1];
    for (i = 0; i < ia; i++) {
      NMPC_Path_Tracking_B.work_data_f[i] = 0.0;
    }

    for (i = k; i >= 1; i--) {
      ia = (i - 1) * lda + i;
      if (i < n) {
        A_data[ia - 1] = 1.0;
        NMPC_Path_Tracking_xzlarf((m - i) + 1, n - i, ia, tau_data[itau], A_data,
          ia + lda, lda, NMPC_Path_Tracking_B.work_data_f);
      }

      if (i < m) {
        c = (ia + m) - i;
        scalarLB = ((((c - ia) / 2) << 1) + ia) + 1;
        vectorUB = scalarLB - 2;
        for (b_k = ia + 1; b_k <= vectorUB; b_k += 2) {
          tmp = _mm_loadu_pd(&A_data[b_k - 1]);
          _mm_storeu_pd(&A_data[b_k - 1], _mm_mul_pd(tmp, _mm_set1_pd
            (-tau_data[itau])));
        }

        for (b_k = scalarLB; b_k <= c; b_k++) {
          A_data[b_k - 1] *= -tau_data[itau];
        }
      }

      A_data[ia - 1] = 1.0 - tau_data[itau];
      c = (uint16_T)(i - 1);
      for (b_k = 0; b_k < c; b_k++) {
        A_data[(ia - b_k) - 2] = 0.0;
      }

      itau--;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_sortLambdaQP(real_T lambda_data[], int32_T
  WorkingSet_nActiveConstr, const int32_T WorkingSet_sizes[5], const int32_T
  WorkingSet_isActiveIdx[6], const int32_T WorkingSet_Wid_data[], const int32_T
  WorkingSet_Wlocalidx_data[], real_T workspace_data[])
{
  int32_T currentMplier;
  if (WorkingSet_nActiveConstr != 0) {
    int32_T idxOffset;
    int32_T mAll;
    mAll = (((WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4])
            + WorkingSet_sizes[2]) + 299;
    idxOffset = (uint16_T)(mAll + 1);
    for (currentMplier = 0; currentMplier < idxOffset; currentMplier++) {
      workspace_data[currentMplier] = lambda_data[currentMplier];
    }

    for (currentMplier = 0; currentMplier <= mAll; currentMplier++) {
      lambda_data[currentMplier] = 0.0;
    }

    currentMplier = 0;
    mAll = 0;
    while ((mAll + 1 <= WorkingSet_nActiveConstr) && (WorkingSet_Wid_data[mAll] <=
            2)) {
      if (WorkingSet_Wid_data[mAll] == 1) {
        idxOffset = 1;
      } else {
        idxOffset = WorkingSet_isActiveIdx[1];
      }

      lambda_data[(idxOffset + WorkingSet_Wlocalidx_data[mAll]) - 2] =
        workspace_data[currentMplier];
      currentMplier++;
      mAll++;
    }

    while (mAll + 1 <= WorkingSet_nActiveConstr) {
      switch (WorkingSet_Wid_data[mAll]) {
       case 3:
        idxOffset = WorkingSet_isActiveIdx[2];
        break;

       case 4:
        idxOffset = WorkingSet_isActiveIdx[3];
        break;

       default:
        idxOffset = WorkingSet_isActiveIdx[4];
        break;
      }

      lambda_data[(idxOffset + WorkingSet_Wlocalidx_data[mAll]) - 2] =
        workspace_data[currentMplier];
      currentMplier++;
      mAll++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_test_exit_o(s7RdrPWkr8UPAUyTdDJkLaG_NMPC__T
  *Flags, s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace,
  sG8JZ69axY52WWR6RKyApQC_NMPC__T *MeritFunction, const int32_T
  *fscales_lineq_constraint_size, const int32_T *fscales_cineq_constraint_size,
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *WorkingSet, s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T
  *TrialState, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager, const real_T lb[401])
{
  real_T nlpComplErrorTmp;
  real_T s;
  real_T smax;
  real_T tmp;
  int32_T c_ix;
  int32_T fullRank_R;
  int32_T iQR0;
  int32_T idx_max;
  int32_T mLambda;
  int32_T nVar;
  int32_T nVar_tmp;
  int32_T rankR;
  boolean_T dxTooSmall;
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T isFeasible;
  nVar_tmp = WorkingSet->nVar;
  mLambda = (((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes
              [3]) + WorkingSet->sizes[4]) + 299;
  for (nVar = 0; nVar <= mLambda; nVar++) {
    TrialState->lambdaStopTest.data[nVar] = TrialState->lambdasqp.data[nVar];
  }

  NMPC_Path_Tracki_computeGradLag(TrialState->gradLag.data, WorkingSet->ldA,
    WorkingSet->nVar, TrialState->grad.data, WorkingSet->sizes[2],
    WorkingSet->Aineq.data, WorkingSet->Aeq.data, WorkingSet->indexFixed.data,
    WorkingSet->sizes[0], WorkingSet->indexLB.data, WorkingSet->sizes[3],
    WorkingSet->indexUB.data, WorkingSet->sizes[4],
    TrialState->lambdaStopTest.data);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = fabs(TrialState->grad.data[0]);
      for (nVar = 2; nVar <= nVar_tmp; nVar++) {
        s = fabs(TrialState->grad.data[nVar - 1]);
        if (s > smax) {
          idx_max = nVar;
          smax = s;
        }
      }
    }
  }

  smax = fmax(1.0, fabs(TrialState->grad.data[idx_max - 1]));
  if (rtIsInf(smax)) {
    smax = 1.0;
  }

  MeritFunction->nlpPrimalFeasError = NMPC_Pat_computePrimalFeasError
    (TrialState->xstarsqp, WorkingSet->sizes[2] - TrialState->mNonlinIneq,
     TrialState->mNonlinIneq, TrialState->cIneq.data, TrialState->cEq,
     WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
     WorkingSet->indexUB.data, WorkingSet->sizes[4]);
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor = fmax(1.0,
      MeritFunction->nlpPrimalFeasError);
  }

  isFeasible = (MeritFunction->nlpPrimalFeasError <= 1.0E-6 *
                MeritFunction->feasRelativeFactor);
  NMPC_Path__computeDualFeasError(WorkingSet->nVar, TrialState->gradLag.data,
    &Flags->gradOK, &MeritFunction->nlpDualFeasError);
  if (!Flags->gradOK) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = NMPC_Path_Tra_computeComplError
      (fscales_lineq_constraint_size, fscales_cineq_constraint_size,
       TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq.data,
       WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
       WorkingSet->indexUB.data, WorkingSet->sizes[4],
       TrialState->lambdaStopTest.data, WorkingSet->sizes[0] + 301);
    MeritFunction->firstOrderOpt = fmax(MeritFunction->nlpDualFeasError,
      MeritFunction->nlpComplError);
    if (TrialState->sqpIterations > 1) {
      NMPC_Path_Trac_computeGradLag_h(memspace->workspace_float.data,
        WorkingSet->ldA, WorkingSet->nVar, TrialState->grad.data,
        WorkingSet->sizes[2], WorkingSet->Aineq.data, WorkingSet->Aeq.data,
        WorkingSet->indexFixed.data, WorkingSet->sizes[0],
        WorkingSet->indexLB.data, WorkingSet->sizes[3], WorkingSet->indexUB.data,
        WorkingSet->sizes[4], TrialState->lambdaStopTestPrev.data);
      NMPC_Pat_computeDualFeasError_b(WorkingSet->nVar,
        memspace->workspace_float.data, &dxTooSmall, &s);
      nlpComplErrorTmp = NMPC_Path_Tra_computeComplError
        (fscales_lineq_constraint_size, fscales_cineq_constraint_size,
         TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq.data,
         WorkingSet->indexLB.data, WorkingSet->sizes[3], lb,
         WorkingSet->indexUB.data, WorkingSet->sizes[4],
         TrialState->lambdaStopTestPrev.data, WorkingSet->sizes[0] + 301);
      if ((s < MeritFunction->nlpDualFeasError) && (nlpComplErrorTmp <
           MeritFunction->nlpComplError)) {
        MeritFunction->nlpDualFeasError = s;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = fmax(s, nlpComplErrorTmp);
        for (nVar = 0; nVar <= mLambda; nVar++) {
          TrialState->lambdaStopTest.data[nVar] =
            TrialState->lambdaStopTestPrev.data[nVar];
        }
      } else {
        for (nVar = 0; nVar <= mLambda; nVar++) {
          TrialState->lambdaStopTestPrev.data[nVar] =
            TrialState->lambdaStopTest.data[nVar];
        }
      }
    } else {
      for (nVar = 0; nVar <= mLambda; nVar++) {
        TrialState->lambdaStopTestPrev.data[nVar] =
          TrialState->lambdaStopTest.data[nVar];
      }
    }

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-6 * smax) &&
        (MeritFunction->nlpComplError <= 1.0E-6 * smax)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          nVar = 0;
          exitg1 = false;
          while ((!exitg1) && (nVar <= (uint16_T)WorkingSet->nVar - 1)) {
            if (1.0E-6 * fmax(1.0, fabs(TrialState->xstarsqp[nVar])) <= fabs
                (TrialState->delta_x.data[nVar])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              nVar++;
            }
          }

          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags->stepType == 2) {
                Flags->done = true;
                TrialState->sqpExitFlag = -2;
              } else {
                Flags->stepType = 2;
                Flags->failedLineSearch = false;
                Flags->stepAccepted = false;
                guard1 = true;
              }
            } else {
              idx_max = WorkingSet->nActiveConstr;
              if (WorkingSet->nActiveConstr == 0) {
                Flags->done = true;
                TrialState->sqpExitFlag = 2;
              } else {
                NMPC_P_updateWorkingSetForNewQP(TrialState->xstarsqp, WorkingSet,
                  WorkingSet->sizes[2], TrialState->mNonlinIneq,
                  TrialState->cIneq.data, TrialState->cEq, WorkingSet->sizes[3],
                  lb, WorkingSet->sizes[4], WorkingSet->sizes[0]);
                for (nVar = 0; nVar < idx_max; nVar++) {
                  TrialState->lambda.data[nVar] = 0.0;
                }

                NMPC_Path_Tracking_factorQRE(QRManager, WorkingSet->ATwset.data,
                  WorkingSet->nVar, WorkingSet->nActiveConstr, WorkingSet->ldA);
                rankR = QRManager->minRowCol;
                for (idx_max = 0; idx_max < rankR; idx_max++) {
                  iQR0 = QRManager->ldq * idx_max + idx_max;
                  c_ix = QRManager->mrows - idx_max;
                  for (fullRank_R = 0; fullRank_R <= c_ix - 2; fullRank_R++) {
                    nVar = fullRank_R + iQR0;
                    QRManager->Q.data[nVar + 1] = QRManager->QR.data[nVar + 1];
                  }
                }

                NMPC_Path_Tracking_xorgqr(QRManager->mrows, QRManager->mrows,
                  QRManager->minRowCol, QRManager->Q.data, QRManager->Q.size,
                  QRManager->ldq, QRManager->tau.data);
                fullRank_R = QRManager->ldq;
                idx_max = (uint16_T)WorkingSet->nVar;
                for (nVar = 0; nVar < idx_max; nVar++) {
                  memspace->workspace_float.data[nVar] = 0.0;
                }

                rankR = 0;
                iQR0 = (WorkingSet->nVar - 1) * QRManager->ldq + 1;
                for (idx_max = 1; fullRank_R < 0 ? idx_max >= iQR0 : idx_max <=
                     iQR0; idx_max += fullRank_R) {
                  s = 0.0;
                  c_ix = (idx_max + nVar_tmp) - 1;
                  for (nVar = idx_max; nVar <= c_ix; nVar++) {
                    s += QRManager->Q.data[nVar - 1] * TrialState->
                      grad.data[nVar - idx_max];
                  }

                  memspace->workspace_float.data[rankR] -= s;
                  rankR++;
                }

                if (WorkingSet->nVar >= WorkingSet->nActiveConstr) {
                  nVar = WorkingSet->nVar;
                } else {
                  nVar = WorkingSet->nActiveConstr;
                }

                s = fmin(1.4901161193847656E-8, (real_T)nVar *
                         2.2204460492503131E-16) * fabs(QRManager->QR.data[0]);
                if (WorkingSet->nVar <= WorkingSet->nActiveConstr) {
                  fullRank_R = WorkingSet->nVar;
                } else {
                  fullRank_R = WorkingSet->nActiveConstr;
                }

                rankR = 0;
                nVar = 0;
                while ((rankR < fullRank_R) && (fabs(QRManager->QR.data[nVar]) >
                        s)) {
                  rankR++;
                  nVar = (nVar + QRManager->ldq) + 1;
                }

                if (rankR != 0) {
                  for (nVar = rankR; nVar >= 1; nVar--) {
                    iQR0 = ((nVar - 1) * QRManager->ldq + nVar) - 2;
                    memspace->workspace_float.data[nVar - 1] /=
                      QRManager->QR.data[iQR0 + 1];
                    for (idx_max = 0; idx_max <= nVar - 2; idx_max++) {
                      c_ix = (nVar - idx_max) - 2;
                      memspace->workspace_float.data[c_ix] -=
                        memspace->workspace_float.data[nVar - 1] *
                        QRManager->QR.data[iQR0 - idx_max];
                    }
                  }
                }

                if (WorkingSet->nActiveConstr <= fullRank_R) {
                  fullRank_R = WorkingSet->nActiveConstr;
                }

                for (nVar = 0; nVar < fullRank_R; nVar++) {
                  TrialState->lambda.data[QRManager->jpvt.data[nVar] - 1] =
                    memspace->workspace_float.data[nVar];
                }

                NMPC_Path_Tracking_sortLambdaQP(TrialState->lambda.data,
                  WorkingSet->nActiveConstr, WorkingSet->sizes,
                  WorkingSet->isActiveIdx, WorkingSet->Wid.data,
                  WorkingSet->Wlocalidx.data, memspace->workspace_float.data);
                NMPC_Path_Trac_computeGradLag_h(memspace->workspace_float.data,
                  WorkingSet->ldA, WorkingSet->nVar, TrialState->grad.data,
                  WorkingSet->sizes[2], WorkingSet->Aineq.data,
                  WorkingSet->Aeq.data, WorkingSet->indexFixed.data,
                  WorkingSet->sizes[0], WorkingSet->indexLB.data,
                  WorkingSet->sizes[3], WorkingSet->indexUB.data,
                  WorkingSet->sizes[4], TrialState->lambda.data);
                NMPC_Pat_computeDualFeasError_b(WorkingSet->nVar,
                  memspace->workspace_float.data, &isFeasible, &s);
                nlpComplErrorTmp = NMPC_Path_Tra_computeComplError
                  (fscales_lineq_constraint_size, fscales_cineq_constraint_size,
                   TrialState->xstarsqp, WorkingSet->sizes[2],
                   TrialState->cIneq.data, WorkingSet->indexLB.data,
                   WorkingSet->sizes[3], lb, WorkingSet->indexUB.data,
                   WorkingSet->sizes[4], TrialState->lambda.data,
                   WorkingSet->sizes[0] + 301);
                tmp = fmax(s, nlpComplErrorTmp);
                if (tmp <= fmax(MeritFunction->nlpDualFeasError,
                                MeritFunction->nlpComplError)) {
                  MeritFunction->nlpDualFeasError = s;
                  MeritFunction->nlpComplError = nlpComplErrorTmp;
                  MeritFunction->firstOrderOpt = tmp;
                  for (nVar = 0; nVar <= mLambda; nVar++) {
                    TrialState->lambdaStopTest.data[nVar] =
                      TrialState->lambda.data[nVar];
                  }
                }

                if ((MeritFunction->nlpDualFeasError <= 1.0E-6 * smax) &&
                    (MeritFunction->nlpComplError <= 1.0E-6 * smax)) {
                  TrialState->sqpExitFlag = 1;
                } else {
                  TrialState->sqpExitFlag = 2;
                }

                Flags->done = true;
                guard1 = true;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          if (TrialState->sqpIterations >= 400) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 40100) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static boolean_T NMPC_Path_Tracking_BFGSUpdate(int32_T nvar, real_T Bk[160801],
  const real_T sk_data[], real_T yk_data[], real_T workspace_data[])
{
  __m128d tmp_0;
  real_T curvatureS;
  real_T dotSY;
  real_T theta;
  int32_T b_iy;
  int32_T b_jA;
  int32_T ia;
  int32_T iac;
  int32_T ix;
  int32_T jA;
  int32_T k;
  boolean_T success;
  dotSY = 0.0;
  if (nvar >= 1) {
    for (k = 0; k < nvar; k++) {
      dotSY += sk_data[k] * yk_data[k];
    }
  }

  k = (uint16_T)nvar;
  for (b_iy = 0; b_iy < k; b_iy++) {
    workspace_data[b_iy] = 0.0;
  }

  ix = 0;
  b_jA = (nvar - 1) * 401 + 1;
  for (iac = 1; iac <= b_jA; iac += 401) {
    jA = (iac + nvar) - 1;
    for (ia = iac; ia <= jA; ia++) {
      b_iy = ia - iac;
      workspace_data[b_iy] += Bk[ia - 1] * sk_data[ix];
    }

    ix++;
  }

  curvatureS = 0.0;
  if (nvar >= 1) {
    for (b_iy = 0; b_iy < nvar; b_iy++) {
      curvatureS += workspace_data[b_iy] * sk_data[b_iy];
    }
  }

  if (dotSY < 0.2 * curvatureS) {
    theta = 0.8 * curvatureS / (curvatureS - dotSY);
    iac = ((uint16_T)nvar / 2) << 1;
    ia = iac - 2;
    for (b_iy = 0; b_iy <= ia; b_iy += 2) {
      tmp_0 = _mm_loadu_pd(&yk_data[b_iy]);
      _mm_storeu_pd(&yk_data[b_iy], _mm_mul_pd(_mm_set1_pd(theta), tmp_0));
    }

    for (b_iy = iac; b_iy < k; b_iy++) {
      yk_data[b_iy] *= theta;
    }

    dotSY = 0.0;
    for (b_iy = 0; b_iy < nvar; b_iy++) {
      if (!(1.0 - theta == 0.0)) {
        yk_data[b_iy] += (1.0 - theta) * workspace_data[b_iy];
      }

      if (nvar >= 1) {
        dotSY += sk_data[b_iy] * yk_data[b_iy];
      }
    }
  }

  success = ((curvatureS > 2.2204460492503131E-16) && (dotSY >
              2.2204460492503131E-16));
  if (success) {
    __m128d tmp;
    curvatureS = -1.0 / curvatureS;
    if (!(curvatureS == 0.0)) {
      jA = 1;
      for (ix = 0; ix < k; ix++) {
        if (workspace_data[ix] != 0.0) {
          theta = workspace_data[ix] * curvatureS;
          b_iy = (nvar + jA) - 1;
          iac = ((((b_iy - jA) + 1) / 2) << 1) + jA;
          ia = iac - 2;
          for (b_jA = jA; b_jA <= ia; b_jA += 2) {
            tmp_0 = _mm_loadu_pd(&workspace_data[b_jA - jA]);
            tmp = _mm_loadu_pd(&Bk[b_jA - 1]);
            _mm_storeu_pd(&Bk[b_jA - 1], _mm_add_pd(_mm_mul_pd(tmp_0,
              _mm_set1_pd(theta)), tmp));
          }

          for (b_jA = iac; b_jA <= b_iy; b_jA++) {
            Bk[b_jA - 1] += workspace_data[b_jA - jA] * theta;
          }
        }

        jA += 401;
      }
    }

    dotSY = 1.0 / dotSY;
    if (!(dotSY == 0.0)) {
      b_jA = 1;
      for (b_iy = 0; b_iy < k; b_iy++) {
        curvatureS = yk_data[b_iy];
        if (curvatureS != 0.0) {
          curvatureS *= dotSY;
          jA = (nvar + b_jA) - 1;
          iac = ((((jA - b_jA) + 1) / 2) << 1) + b_jA;
          ia = iac - 2;
          for (ix = b_jA; ix <= ia; ix += 2) {
            tmp_0 = _mm_loadu_pd(&yk_data[ix - b_jA]);
            tmp = _mm_loadu_pd(&Bk[ix - 1]);
            _mm_storeu_pd(&Bk[ix - 1], _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd
              (curvatureS)), tmp));
          }

          for (ix = iac; ix <= jA; ix++) {
            Bk[ix - 1] += yk_data[ix - b_jA] * curvatureS;
          }
        }

        b_jA += 401;
      }
    }
  }

  return success;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_factorQRE_d(s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *obj,
  int32_T mrows, int32_T ncols)
{
  if (mrows * ncols == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    obj->usedPivoting = true;
    obj->mrows = mrows;
    obj->ncols = ncols;
    if (mrows <= ncols) {
      obj->minRowCol = mrows;
    } else {
      obj->minRowCol = ncols;
    }

    NMPC_Path_Tracking_xgeqp3(obj->QR.data, obj->QR.size, mrows, ncols,
      obj->jpvt.data, obj->tau.data, &obj->tau.size);
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_countsort(int32_T x_data[], int32_T xLen, int32_T
  workspace_data[], int32_T xMin, int32_T xMax)
{
  int32_T b_tmp;
  int32_T idxFill;
  int32_T maxOffset;
  if ((xLen > 1) && (xMax > xMin)) {
    int32_T idxEnd;
    int32_T idxStart;
    b_tmp = xMax - xMin;
    for (maxOffset = 0; maxOffset <= b_tmp; maxOffset++) {
      workspace_data[maxOffset] = 0;
    }

    maxOffset = b_tmp - 1;
    for (b_tmp = 0; b_tmp < xLen; b_tmp++) {
      idxFill = x_data[b_tmp] - xMin;
      workspace_data[idxFill]++;
    }

    for (b_tmp = 2; b_tmp <= maxOffset + 2; b_tmp++) {
      workspace_data[b_tmp - 1] += workspace_data[b_tmp - 2];
    }

    idxStart = 1;
    idxEnd = workspace_data[0];
    for (b_tmp = 0; b_tmp <= maxOffset; b_tmp++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x_data[idxFill - 1] = b_tmp + xMin;
      }

      idxStart = workspace_data[b_tmp] + 1;
      idxEnd = workspace_data[b_tmp + 1];
    }

    for (maxOffset = idxStart; maxOffset <= idxEnd; maxOffset++) {
      x_data[maxOffset - 1] = xMax;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_removeConstr(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj,
  int32_T idx_global)
{
  int32_T TYPE_tmp;
  int32_T idx;
  TYPE_tmp = obj->Wid.data[idx_global - 1] - 1;
  obj->isActiveConstr.data[(obj->isActiveIdx[TYPE_tmp] + obj->
    Wlocalidx.data[idx_global - 1]) - 2] = false;
  if (idx_global < obj->nActiveConstr) {
    int32_T b;
    obj->Wid.data[idx_global - 1] = obj->Wid.data[obj->nActiveConstr - 1];
    obj->Wlocalidx.data[idx_global - 1] = obj->Wlocalidx.data[obj->nActiveConstr
      - 1];
    b = (uint16_T)obj->nVar;
    for (idx = 0; idx < b; idx++) {
      obj->ATwset.data[idx + obj->ldA * (idx_global - 1)] = obj->ATwset.data
        [(obj->nActiveConstr - 1) * obj->ldA + idx];
    }

    obj->bwset.data[idx_global - 1] = obj->bwset.data[obj->nActiveConstr - 1];
  }

  obj->nActiveConstr--;
  obj->nWConstr[TYPE_tmp]--;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static int32_T NMPC_Path_Tr_RemoveDependentEq_(s_BcTDS8pFolHdhtDibl2TnF_NMPC_T
  *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *workingset,
  s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager)
{
  real_T qtb;
  real_T tol;
  int32_T i;
  int32_T iQR0;
  int32_T ix;
  int32_T mTotalWorkingEq_tmp;
  int32_T mWorkingFixed;
  int32_T n;
  int32_T nDepInd;
  int32_T totalEq;
  int32_T totalEq_tmp;
  int32_T totalRank;
  boolean_T exitg1;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp > 0) {
    totalEq_tmp = (uint16_T)workingset->nVar;
    for (totalRank = 0; totalRank < mTotalWorkingEq_tmp; totalRank++) {
      for (totalEq = 0; totalEq < totalEq_tmp; totalEq++) {
        qrmanager->QR.data[totalRank + qrmanager->ldq * totalEq] =
          workingset->ATwset.data[workingset->ldA * totalRank + totalEq];
      }
    }

    nDepInd = mTotalWorkingEq_tmp - workingset->nVar;
    if (nDepInd <= 0) {
      nDepInd = 0;
    }

    for (totalRank = 0; totalRank < totalEq_tmp; totalRank++) {
      qrmanager->jpvt.data[totalRank] = 0;
    }

    NMPC_Path_Tracking_factorQRE_d(qrmanager, mTotalWorkingEq_tmp,
      workingset->nVar);
    if (mTotalWorkingEq_tmp >= workingset->nVar) {
      ix = mTotalWorkingEq_tmp;
    } else {
      ix = workingset->nVar;
    }

    tol = fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * (real_T)ix);
    if (workingset->nVar <= mTotalWorkingEq_tmp) {
      totalRank = workingset->nVar;
    } else {
      totalRank = mTotalWorkingEq_tmp;
    }

    totalRank += (totalRank - 1) * qrmanager->ldq;
    while ((totalRank > 0) && (fabs(qrmanager->QR.data[totalRank - 1]) < fabs
            (qrmanager->QR.data[0]) * tol)) {
      totalRank = (totalRank - qrmanager->ldq) - 1;
      nDepInd++;
    }

    if (nDepInd > 0) {
      i = qrmanager->minRowCol;
      for (totalRank = 0; totalRank < i; totalRank++) {
        iQR0 = qrmanager->ldq * totalRank + totalRank;
        n = qrmanager->mrows - totalRank;
        for (totalEq = 0; totalEq <= n - 2; totalEq++) {
          ix = totalEq + iQR0;
          qrmanager->Q.data[ix + 1] = qrmanager->QR.data[ix + 1];
        }
      }

      NMPC_Path_Tracking_xorgqr(qrmanager->mrows, qrmanager->mrows,
        qrmanager->minRowCol, qrmanager->Q.data, qrmanager->Q.size,
        qrmanager->ldq, qrmanager->tau.data);
      totalEq = 0;
      exitg1 = false;
      while ((!exitg1) && (totalEq <= nDepInd - 1)) {
        ix = ((mTotalWorkingEq_tmp - totalEq) - 1) * qrmanager->ldq;
        qtb = 0.0;
        for (totalRank = 0; totalRank < mTotalWorkingEq_tmp; totalRank++) {
          qtb += qrmanager->Q.data[ix + totalRank] * workingset->
            bwset.data[totalRank];
        }

        if (fabs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          totalEq++;
        }
      }
    }

    if (nDepInd > 0) {
      for (totalRank = 0; totalRank < mTotalWorkingEq_tmp; totalRank++) {
        ix = qrmanager->ldq * totalRank;
        i = workingset->ldA * totalRank;
        for (totalEq = 0; totalEq < totalEq_tmp; totalEq++) {
          qrmanager->QR.data[ix + totalEq] = workingset->ATwset.data[i + totalEq];
        }
      }

      for (totalEq = 0; totalEq < mWorkingFixed; totalEq++) {
        qrmanager->jpvt.data[totalEq] = 1;
      }

      totalEq = workingset->nWConstr[0] + 1;
      for (mWorkingFixed = totalEq; mWorkingFixed <= mTotalWorkingEq_tmp;
           mWorkingFixed++) {
        qrmanager->jpvt.data[mWorkingFixed - 1] = 0;
      }

      NMPC_Path_Tracking_factorQRE_d(qrmanager, workingset->nVar,
        mTotalWorkingEq_tmp);
      for (mWorkingFixed = 0; mWorkingFixed < nDepInd; mWorkingFixed++) {
        memspace->workspace_int.data[mWorkingFixed] = qrmanager->jpvt.data
          [(mTotalWorkingEq_tmp - nDepInd) + mWorkingFixed];
      }

      NMPC_Path_Tracking_countsort(memspace->workspace_int.data, nDepInd,
        memspace->workspace_sort.data, 1, mTotalWorkingEq_tmp);
      for (totalRank = nDepInd; totalRank >= 1; totalRank--) {
        mTotalWorkingEq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
        if (mTotalWorkingEq_tmp != 0) {
          totalEq = memspace->workspace_int.data[totalRank - 1];
          if (totalEq <= mTotalWorkingEq_tmp) {
            if ((mTotalWorkingEq_tmp == workingset->nActiveConstr) ||
                (mTotalWorkingEq_tmp == totalEq)) {
              workingset->mEqRemoved++;
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                workingset->Wlocalidx.data[totalEq - 1];
              NMPC_Path_Tracking_removeConstr(workingset,
                memspace->workspace_int.data[totalRank - 1]);
            } else {
              workingset->mEqRemoved++;
              ix = workingset->Wid.data[totalEq - 1] - 1;
              mWorkingFixed = workingset->Wlocalidx.data[totalEq - 1];
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                mWorkingFixed;
              workingset->isActiveConstr.data[(workingset->isActiveIdx[ix] +
                mWorkingFixed) - 2] = false;
              workingset->Wid.data[totalEq - 1] = workingset->
                Wid.data[mTotalWorkingEq_tmp - 1];
              workingset->Wlocalidx.data[totalEq - 1] =
                workingset->Wlocalidx.data[mTotalWorkingEq_tmp - 1];
              for (mWorkingFixed = 0; mWorkingFixed < totalEq_tmp; mWorkingFixed
                   ++) {
                workingset->ATwset.data[mWorkingFixed + workingset->ldA *
                  (totalEq - 1)] = workingset->ATwset.data[(mTotalWorkingEq_tmp
                  - 1) * workingset->ldA + mWorkingFixed];
              }

              workingset->bwset.data[totalEq - 1] = workingset->
                bwset.data[mTotalWorkingEq_tmp - 1];
              workingset->Wid.data[mTotalWorkingEq_tmp - 1] =
                workingset->Wid.data[workingset->nActiveConstr - 1];
              workingset->Wlocalidx.data[mTotalWorkingEq_tmp - 1] =
                workingset->Wlocalidx.data[workingset->nActiveConstr - 1];
              for (mWorkingFixed = 0; mWorkingFixed < totalEq_tmp; mWorkingFixed
                   ++) {
                workingset->ATwset.data[mWorkingFixed + workingset->ldA *
                  (mTotalWorkingEq_tmp - 1)] = workingset->ATwset.data
                  [(workingset->nActiveConstr - 1) * workingset->ldA +
                  mWorkingFixed];
              }

              workingset->bwset.data[mTotalWorkingEq_tmp - 1] =
                workingset->bwset.data[workingset->nActiveConstr - 1];
              workingset->nActiveConstr--;
              workingset->nWConstr[ix]--;
            }
          }
        }
      }
    }
  }

  return nDepInd;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path__RemoveDependentIneq_(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace)
{
  real_T maxDiag;
  real_T tol;
  int32_T b_idx;
  int32_T d;
  int32_T idxDiag;
  int32_T idxDiag_tmp;
  int32_T ix0;
  int32_T iy0;
  int32_T nDepIneq;
  int32_T nFixedConstr;
  nDepIneq = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  idxDiag_tmp = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    if (workingset->nVar >= workingset->nActiveConstr) {
      b_idx = workingset->nVar;
    } else {
      b_idx = workingset->nActiveConstr;
    }

    tol = fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * (real_T)b_idx);
    for (b_idx = 0; b_idx < nFixedConstr; b_idx++) {
      qrmanager->jpvt.data[b_idx] = 1;
    }

    for (b_idx = nFixedConstr + 1; b_idx <= nDepIneq; b_idx++) {
      qrmanager->jpvt.data[b_idx - 1] = 0;
    }

    for (b_idx = 0; b_idx < nDepIneq; b_idx++) {
      iy0 = qrmanager->ldq * b_idx;
      ix0 = workingset->ldA * b_idx;
      d = (uint16_T)idxDiag_tmp;
      for (idxDiag = 0; idxDiag < d; idxDiag++) {
        qrmanager->QR.data[iy0 + idxDiag] = workingset->ATwset.data[ix0 +
          idxDiag];
      }
    }

    NMPC_Path_Tracking_factorQRE_d(qrmanager, workingset->nVar,
      workingset->nActiveConstr);
    nDepIneq = 0;
    for (b_idx = workingset->nActiveConstr - 1; b_idx + 1 > idxDiag_tmp; b_idx--)
    {
      nDepIneq++;
      memspace->workspace_int.data[nDepIneq - 1] = qrmanager->jpvt.data[b_idx];
    }

    maxDiag = fabs(qrmanager->QR.data[0]);
    for (idxDiag = 0; idxDiag < b_idx; idxDiag++) {
      maxDiag = fmax(maxDiag, fabs(qrmanager->QR.data[((idxDiag + 1) *
        qrmanager->ldq + idxDiag) + 1]));
    }

    if (b_idx + 1 <= workingset->nVar) {
      idxDiag = qrmanager->ldq * b_idx + b_idx;
      while ((b_idx + 1 > nFixedConstr) && (fabs(qrmanager->QR.data[idxDiag]) <
              tol * maxDiag)) {
        nDepIneq++;
        memspace->workspace_int.data[nDepIneq - 1] = qrmanager->jpvt.data[b_idx];
        b_idx--;
        idxDiag = (idxDiag - qrmanager->ldq) - 1;
      }
    }

    NMPC_Path_Tracking_countsort(memspace->workspace_int.data, nDepIneq,
      memspace->workspace_sort.data, nFixedConstr + 1, workingset->nActiveConstr);
    for (nFixedConstr = nDepIneq; nFixedConstr >= 1; nFixedConstr--) {
      NMPC_Path_Tracking_removeConstr(workingset, memspace->
        workspace_int.data[nFixedConstr - 1]);
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static int32_T NMPC_Path_Tracking_rank(const real_T qrmanager_QR_data[], const
  int32_T qrmanager_QR_size[2], int32_T qrmanager_mrows, int32_T qrmanager_ncols)
{
  int32_T minmn;
  int32_T r;
  r = 0;
  if (qrmanager_mrows <= qrmanager_ncols) {
    minmn = qrmanager_mrows;
  } else {
    minmn = qrmanager_ncols;
  }

  if (minmn > 0) {
    real_T tol;
    int32_T tmp;
    if (qrmanager_mrows >= qrmanager_ncols) {
      tmp = qrmanager_mrows;
    } else {
      tmp = qrmanager_ncols;
    }

    tol = fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * (real_T)tmp) *
      fabs(qrmanager_QR_data[0]);
    while ((r < minmn) && (!(fabs(qrmanager_QR_data[qrmanager_QR_size[0] * r + r])
             <= tol))) {
      r++;
    }
  }

  return r;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgemv_c(int32_T m, int32_T n, const real_T
  A_data[], int32_T lda, const real_T x_data[], real_T y_data[])
{
  int32_T b;
  int32_T b_iy;
  if (n != 0) {
    int32_T scalarLB;
    int32_T vectorUB;
    b = (uint16_T)n;
    scalarLB = ((uint16_T)n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (b_iy = 0; b_iy <= vectorUB; b_iy += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&y_data[b_iy]);
      _mm_storeu_pd(&y_data[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
    }

    for (b_iy = scalarLB; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    scalarLB = 0;
    vectorUB = (n - 1) * lda + 1;
    for (b_iy = 1; lda < 0 ? b_iy >= vectorUB : b_iy <= vectorUB; b_iy += lda) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += x_data[b - b_iy] * A_data[b - 1];
      }

      y_data[scalarLB] += c;
      scalarLB++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T NMPC_P_maxConstraintViolation_l(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *obj, const real_T x_data[])
{
  real_T obj_maxConstrWorkspace;
  real_T v;
  int32_T b_mIneq;
  int32_T f;
  int32_T mIneq;
  if (obj->probType == 2) {
    v = 0.0;
    b_mIneq = obj->sizes[2];
    if (obj->Aineq.size != 0) {
      for (mIneq = 0; mIneq < b_mIneq; mIneq++) {
        obj->maxConstrWorkspace.data[mIneq] = obj->bineq.data[mIneq];
      }

      NMPC_Path_Tracking_xgemv_c(401, obj->sizes[2], obj->Aineq.data, obj->ldA,
        x_data, obj->maxConstrWorkspace.data);
      f = (uint16_T)obj->sizes[2];
      for (mIneq = 0; mIneq < f; mIneq++) {
        obj_maxConstrWorkspace = obj->maxConstrWorkspace.data[mIneq] -
          x_data[mIneq + 401];
        obj->maxConstrWorkspace.data[mIneq] = obj_maxConstrWorkspace;
        v = fmax(v, obj_maxConstrWorkspace);
      }
    }

    memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 300U * sizeof(real_T));
    NMPC_Path_Tracking_xgemv_c(401, 300, obj->Aeq.data, obj->ldA, x_data,
      obj->maxConstrWorkspace.data);
    for (mIneq = 0; mIneq < 300; mIneq++) {
      obj->maxConstrWorkspace.data[mIneq] = (obj->maxConstrWorkspace.data[mIneq]
        - x_data[(b_mIneq + mIneq) + 401]) + x_data[(obj->sizes[2] + mIneq) +
        701];
      v = fmax(v, fabs(obj->maxConstrWorkspace.data[mIneq]));
    }
  } else {
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size != 0) {
      for (b_mIneq = 0; b_mIneq < mIneq; b_mIneq++) {
        obj->maxConstrWorkspace.data[b_mIneq] = obj->bineq.data[b_mIneq];
      }

      NMPC_Path_Tracking_xgemv_c(obj->nVar, obj->sizes[2], obj->Aineq.data,
        obj->ldA, x_data, obj->maxConstrWorkspace.data);
      mIneq = (uint16_T)obj->sizes[2];
      for (b_mIneq = 0; b_mIneq < mIneq; b_mIneq++) {
        v = fmax(v, obj->maxConstrWorkspace.data[b_mIneq]);
      }
    }

    memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 300U * sizeof(real_T));
    NMPC_Path_Tracking_xgemv_c(obj->nVar, 300, obj->Aeq.data, obj->ldA, x_data,
      obj->maxConstrWorkspace.data);
    for (b_mIneq = 0; b_mIneq < 300; b_mIneq++) {
      v = fmax(v, fabs(obj->maxConstrWorkspace.data[b_mIneq]));
    }
  }

  if (obj->sizes[3] > 0) {
    mIneq = (uint16_T)obj->sizes[3];
    for (b_mIneq = 0; b_mIneq < mIneq; b_mIneq++) {
      v = fmax(v, -x_data[obj->indexLB.data[b_mIneq] - 1] - obj->lb.data
               [obj->indexLB.data[b_mIneq] - 1]);
    }
  }

  if (obj->sizes[4] > 0) {
    mIneq = (uint16_T)obj->sizes[4];
    for (b_mIneq = 0; b_mIneq < mIneq; b_mIneq++) {
      v = fmax(v, x_data[obj->indexUB.data[b_mIneq] - 1] - obj->ub.data
               [obj->indexUB.data[b_mIneq] - 1]);
    }
  }

  if (obj->sizes[0] > 0) {
    mIneq = (uint16_T)obj->sizes[0];
    for (b_mIneq = 0; b_mIneq < mIneq; b_mIneq++) {
      v = fmax(v, fabs(x_data[obj->indexFixed.data[b_mIneq] - 1] - obj->
                       ub.data[obj->indexFixed.data[b_mIneq] - 1]));
    }
  }

  return v;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgemv_cp(int32_T m, int32_T n, const real_T
  A_data[], int32_T lda, const real_T x_data[], int32_T ix0, real_T y_data[])
{
  int32_T b;
  int32_T b_iy;
  if (n != 0) {
    int32_T scalarLB;
    int32_T vectorUB;
    b = (uint16_T)n;
    scalarLB = ((uint16_T)n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (b_iy = 0; b_iy <= vectorUB; b_iy += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&y_data[b_iy]);
      _mm_storeu_pd(&y_data[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
    }

    for (b_iy = scalarLB; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    scalarLB = 0;
    vectorUB = (n - 1) * lda + 1;
    for (b_iy = 1; lda < 0 ? b_iy >= vectorUB : b_iy <= vectorUB; b_iy += lda) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += x_data[((ix0 + b) - b_iy) - 1] * A_data[b - 1];
      }

      y_data[scalarLB] += c;
      scalarLB++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T NMPC__maxConstraintViolation_ln(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *obj, const real_T x_data[], int32_T ix0)
{
  real_T obj_maxConstrWorkspace;
  real_T v;
  int32_T b_mIneq;
  int32_T f;
  int32_T mIneq;
  if (obj->probType == 2) {
    v = 0.0;
    b_mIneq = obj->sizes[2];
    if (obj->Aineq.size != 0) {
      for (mIneq = 0; mIneq < b_mIneq; mIneq++) {
        obj->maxConstrWorkspace.data[mIneq] = obj->bineq.data[mIneq];
      }

      NMPC_Path_Tracking_xgemv_cp(401, obj->sizes[2], obj->Aineq.data, obj->ldA,
        x_data, ix0, obj->maxConstrWorkspace.data);
      f = (uint16_T)obj->sizes[2];
      for (mIneq = 0; mIneq < f; mIneq++) {
        obj_maxConstrWorkspace = obj->maxConstrWorkspace.data[mIneq] - x_data
          [(ix0 + mIneq) + 400];
        obj->maxConstrWorkspace.data[mIneq] = obj_maxConstrWorkspace;
        v = fmax(v, obj_maxConstrWorkspace);
      }
    }

    memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 300U * sizeof(real_T));
    NMPC_Path_Tracking_xgemv_cp(401, 300, obj->Aeq.data, obj->ldA, x_data, ix0,
      obj->maxConstrWorkspace.data);
    for (mIneq = 0; mIneq < 300; mIneq++) {
      obj->maxConstrWorkspace.data[mIneq] = (obj->maxConstrWorkspace.data[mIneq]
        - x_data[((ix0 + b_mIneq) + mIneq) + 400]) + x_data[((ix0 + obj->sizes[2])
        + mIneq) + 700];
      v = fmax(v, fabs(obj->maxConstrWorkspace.data[mIneq]));
    }
  } else {
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size != 0) {
      for (b_mIneq = 0; b_mIneq < mIneq; b_mIneq++) {
        obj->maxConstrWorkspace.data[b_mIneq] = obj->bineq.data[b_mIneq];
      }

      NMPC_Path_Tracking_xgemv_cp(obj->nVar, obj->sizes[2], obj->Aineq.data,
        obj->ldA, x_data, ix0, obj->maxConstrWorkspace.data);
      mIneq = (uint16_T)obj->sizes[2];
      for (b_mIneq = 0; b_mIneq < mIneq; b_mIneq++) {
        v = fmax(v, obj->maxConstrWorkspace.data[b_mIneq]);
      }
    }

    memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 300U * sizeof(real_T));
    NMPC_Path_Tracking_xgemv_cp(obj->nVar, 300, obj->Aeq.data, obj->ldA, x_data,
      ix0, obj->maxConstrWorkspace.data);
    for (b_mIneq = 0; b_mIneq < 300; b_mIneq++) {
      v = fmax(v, fabs(obj->maxConstrWorkspace.data[b_mIneq]));
    }
  }

  if (obj->sizes[3] > 0) {
    mIneq = (uint16_T)obj->sizes[3];
    for (b_mIneq = 0; b_mIneq < mIneq; b_mIneq++) {
      v = fmax(v, -x_data[(ix0 + obj->indexLB.data[b_mIneq]) - 2] - obj->
               lb.data[obj->indexLB.data[b_mIneq] - 1]);
    }
  }

  if (obj->sizes[4] > 0) {
    mIneq = (uint16_T)obj->sizes[4];
    for (b_mIneq = 0; b_mIneq < mIneq; b_mIneq++) {
      v = fmax(v, x_data[(ix0 + obj->indexUB.data[b_mIneq]) - 2] - obj->
               ub.data[obj->indexUB.data[b_mIneq] - 1]);
    }
  }

  if (obj->sizes[0] > 0) {
    mIneq = (uint16_T)obj->sizes[0];
    for (b_mIneq = 0; b_mIneq < mIneq; b_mIneq++) {
      v = fmax(v, fabs(x_data[(ix0 + obj->indexFixed.data[b_mIneq]) - 2] -
                       obj->ub.data[obj->indexFixed.data[b_mIneq] - 1]));
    }
  }

  return v;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static boolean_T NMPC_Pa_feasibleX0ForWorkingSet(real_T workspace_data[], const
  int32_T workspace_size[2], real_T xCurrent_data[],
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T
  *qrmanager)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T constrViolation_basicX;
  real_T temp;
  int32_T b_br;
  int32_T b_iQR0;
  int32_T br;
  int32_T exitg1;
  int32_T h;
  int32_T iAcol;
  int32_T iQR0;
  int32_T i_k;
  int32_T jBcol;
  int32_T ldq;
  int32_T mWConstr;
  int32_T n;
  int32_T nVar;
  int32_T rankQR;
  boolean_T nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (workingset->nActiveConstr != 0) {
    if (workingset->nActiveConstr >= workingset->nVar) {
      i_k = (uint16_T)workingset->nVar;
      for (rankQR = 0; rankQR < i_k; rankQR++) {
        iQR0 = qrmanager->ldq * rankQR;
        for (ldq = 0; ldq < mWConstr; ldq++) {
          qrmanager->QR.data[ldq + iQR0] = workingset->ATwset.data
            [workingset->ldA * ldq + rankQR];
        }
      }

      for (rankQR = 0; rankQR < i_k; rankQR++) {
        qrmanager->jpvt.data[rankQR] = 0;
      }

      NMPC_Path_Tracking_factorQRE_d(qrmanager, workingset->nActiveConstr,
        workingset->nVar);
      iQR0 = qrmanager->minRowCol;
      for (rankQR = 0; rankQR < iQR0; rankQR++) {
        b_iQR0 = qrmanager->ldq * rankQR + rankQR;
        jBcol = qrmanager->mrows - rankQR;
        for (ldq = 0; ldq <= jBcol - 2; ldq++) {
          h = ldq + b_iQR0;
          qrmanager->Q.data[h + 1] = qrmanager->QR.data[h + 1];
        }
      }

      NMPC_Path_Tracking_xorgqr(qrmanager->mrows, qrmanager->mrows,
        qrmanager->minRowCol, qrmanager->Q.data, qrmanager->Q.size,
        qrmanager->ldq, qrmanager->tau.data);
      rankQR = NMPC_Path_Tracking_rank(qrmanager->QR.data, qrmanager->QR.size,
        qrmanager->mrows, qrmanager->ncols);
      for (ldq = 0; ldq < mWConstr; ldq++) {
        workspace_data[ldq] = workingset->bwset.data[ldq];
        workspace_data[ldq + workspace_size[0]] = workingset->bwset.data[ldq];
      }

      b_iQR0 = workingset->ldA;
      jBcol = 0;
      iAcol = (workingset->nActiveConstr - 1) * workingset->ldA + 1;
      for (ldq = 1; b_iQR0 < 0 ? ldq >= iAcol : ldq <= iAcol; ldq += b_iQR0) {
        temp = 0.0;
        h = (ldq + nVar) - 1;
        for (iQR0 = ldq; iQR0 <= h; iQR0++) {
          temp += workingset->ATwset.data[iQR0 - 1] * xCurrent_data[iQR0 - ldq];
        }

        workspace_data[jBcol] -= temp;
        jBcol++;
      }

      ldq = qrmanager->ldq;
      iQR0 = workspace_size[0];
      b_iQR0 = workspace_size[0] * workspace_size[1];
      for (h = 0; h < b_iQR0; h++) {
        NMPC_Path_Tracking_B.B_data[h] = workspace_data[h];
      }

      for (h = 0; iQR0 < 0 ? h >= iQR0 : h <= iQR0; h += iQR0) {
        iAcol = h + nVar;
        for (jBcol = h + 1; jBcol <= iAcol; jBcol++) {
          workspace_data[jBcol - 1] = 0.0;
        }
      }

      b_br = -1;
      for (h = 0; iQR0 < 0 ? h >= iQR0 : h <= iQR0; h += iQR0) {
        br = -1;
        b_iQR0 = h + nVar;
        for (jBcol = h + 1; jBcol <= b_iQR0; jBcol++) {
          temp = 0.0;
          for (iAcol = 0; iAcol < mWConstr; iAcol++) {
            temp += qrmanager->Q.data[(iAcol + br) + 1] *
              NMPC_Path_Tracking_B.B_data[(iAcol + b_br) + 1];
          }

          workspace_data[jBcol - 1] += temp;
          br += ldq;
        }

        b_br += iQR0;
      }

      for (mWConstr = 0; mWConstr < 2; mWConstr++) {
        b_br = iQR0 * mWConstr - 1;
        for (jBcol = rankQR; jBcol >= 1; jBcol--) {
          br = (jBcol - 1) * ldq;
          h = jBcol + b_br;
          temp = workspace_data[h];
          if (temp != 0.0) {
            workspace_data[h] = temp / qrmanager->QR.data[(jBcol + br) - 1];
            b_iQR0 = (uint16_T)(jBcol - 1);
            for (iAcol = 0; iAcol < b_iQR0; iAcol++) {
              n = (iAcol + b_br) + 1;
              workspace_data[n] -= qrmanager->QR.data[iAcol + br] *
                workspace_data[h];
            }
          }
        }
      }

      for (ldq = rankQR + 1; ldq <= nVar; ldq++) {
        workspace_data[ldq - 1] = 0.0;
        workspace_data[(ldq + workspace_size[0]) - 1] = 0.0;
      }

      for (rankQR = 0; rankQR < i_k; rankQR++) {
        workspace_data[(qrmanager->jpvt.data[rankQR] + (workspace_size[0] << 1))
          - 1] = workspace_data[rankQR];
      }

      for (rankQR = 0; rankQR < i_k; rankQR++) {
        workspace_data[rankQR] = workspace_data[(workspace_size[0] << 1) +
          rankQR];
      }

      for (rankQR = 0; rankQR < i_k; rankQR++) {
        workspace_data[(qrmanager->jpvt.data[rankQR] + (workspace_size[0] << 1))
          - 1] = workspace_data[rankQR + workspace_size[0]];
      }

      for (rankQR = 0; rankQR < i_k; rankQR++) {
        workspace_data[rankQR + workspace_size[0]] = workspace_data
          [(workspace_size[0] << 1) + rankQR];
      }
    } else {
      for (rankQR = 0; rankQR < mWConstr; rankQR++) {
        qrmanager->jpvt.data[rankQR] = 0;
      }

      NMPC_Path_Tracking_factorQRE(qrmanager, workingset->ATwset.data,
        workingset->nVar, workingset->nActiveConstr, workingset->ldA);
      ldq = qrmanager->minRowCol;
      for (rankQR = 0; rankQR < ldq; rankQR++) {
        iQR0 = qrmanager->ldq * rankQR + rankQR;
        b_iQR0 = qrmanager->mrows - rankQR;
        for (i_k = 0; i_k <= b_iQR0 - 2; i_k++) {
          h = i_k + iQR0;
          qrmanager->Q.data[h + 1] = qrmanager->QR.data[h + 1];
        }
      }

      NMPC_Path_Tracking_xorgqr(qrmanager->mrows, qrmanager->minRowCol,
        qrmanager->minRowCol, qrmanager->Q.data, qrmanager->Q.size,
        qrmanager->ldq, qrmanager->tau.data);
      rankQR = NMPC_Path_Tracking_rank(qrmanager->QR.data, qrmanager->QR.size,
        qrmanager->mrows, qrmanager->ncols);
      for (i_k = 0; i_k < mWConstr; i_k++) {
        iQR0 = (qrmanager->jpvt.data[i_k] - 1) * workingset->ldA;
        temp = 0.0;
        b_iQR0 = (uint16_T)nVar;
        for (ldq = 0; ldq < b_iQR0; ldq++) {
          temp += workingset->ATwset.data[iQR0 + ldq] * xCurrent_data[ldq];
        }

        constrViolation_basicX = workingset->bwset.data[qrmanager->jpvt.data[i_k]
          - 1];
        workspace_data[i_k] = constrViolation_basicX - temp;
        workspace_data[i_k + workspace_size[0]] = constrViolation_basicX;
      }

      ldq = qrmanager->ldq;
      iQR0 = workspace_size[0];
      h = (uint16_T)rankQR;
      for (i_k = 0; i_k < 2; i_k++) {
        jBcol = iQR0 * i_k;
        for (mWConstr = 0; mWConstr < h; mWConstr++) {
          iAcol = ldq * mWConstr;
          br = mWConstr + jBcol;
          temp = workspace_data[br];
          for (b_iQR0 = 0; b_iQR0 < mWConstr; b_iQR0++) {
            temp -= qrmanager->QR.data[b_iQR0 + iAcol] * workspace_data[b_iQR0 +
              jBcol];
          }

          workspace_data[br] = temp / qrmanager->QR.data[mWConstr + iAcol];
        }
      }

      mWConstr = workspace_size[0] * workspace_size[1];
      for (h = 0; h < mWConstr; h++) {
        NMPC_Path_Tracking_B.B_data[h] = workspace_data[h];
      }

      for (i_k = 0; iQR0 < 0 ? i_k >= iQR0 : i_k <= iQR0; i_k += iQR0) {
        h = i_k + nVar;
        for (mWConstr = i_k + 1; mWConstr <= h; mWConstr++) {
          workspace_data[mWConstr - 1] = 0.0;
        }
      }

      br = 1;
      for (i_k = 0; iQR0 < 0 ? i_k >= iQR0 : i_k <= iQR0; i_k += iQR0) {
        b_iQR0 = -1;
        n = br + rankQR;
        for (mWConstr = br; mWConstr < n; mWConstr++) {
          h = i_k + nVar;
          iAcol = ((((h - i_k) / 2) << 1) + i_k) + 1;
          b_br = iAcol - 2;
          for (jBcol = i_k + 1; jBcol <= b_br; jBcol += 2) {
            tmp = _mm_loadu_pd(&qrmanager->Q.data[(b_iQR0 + jBcol) - i_k]);
            tmp_0 = _mm_loadu_pd(&workspace_data[jBcol - 1]);
            _mm_storeu_pd(&workspace_data[jBcol - 1], _mm_add_pd(_mm_mul_pd
              (_mm_set1_pd(NMPC_Path_Tracking_B.B_data[mWConstr - 1]), tmp),
              tmp_0));
          }

          for (jBcol = iAcol; jBcol <= h; jBcol++) {
            workspace_data[jBcol - 1] += qrmanager->Q.data[(b_iQR0 + jBcol) -
              i_k] * NMPC_Path_Tracking_B.B_data[mWConstr - 1];
          }

          b_iQR0 += ldq;
        }

        br += iQR0;
      }
    }

    rankQR = 0;
    do {
      exitg1 = 0;
      if (rankQR <= (uint16_T)nVar - 1) {
        temp = workspace_data[rankQR];
        if (rtIsInf(temp) || rtIsNaN(temp)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          temp = workspace_data[rankQR + workspace_size[0]];
          if (rtIsInf(temp) || rtIsNaN(temp)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            rankQR++;
          }
        }
      } else {
        iAcol = (nVar / 2) << 1;
        b_br = iAcol - 2;
        for (rankQR = 0; rankQR <= b_br; rankQR += 2) {
          tmp = _mm_loadu_pd(&workspace_data[rankQR]);
          tmp_0 = _mm_loadu_pd(&xCurrent_data[rankQR]);
          _mm_storeu_pd(&workspace_data[rankQR], _mm_add_pd(tmp, tmp_0));
        }

        for (rankQR = iAcol; rankQR < nVar; rankQR++) {
          workspace_data[rankQR] += xCurrent_data[rankQR];
        }

        temp = NMPC_P_maxConstraintViolation_l(workingset, workspace_data);
        constrViolation_basicX = NMPC__maxConstraintViolation_ln(workingset,
          workspace_data, workspace_size[0] + 1);
        if ((temp <= 2.2204460492503131E-16) || (temp < constrViolation_basicX))
        {
          rankQR = (uint16_T)nVar;
          for (nVar = 0; nVar < rankQR; nVar++) {
            xCurrent_data[nVar] = workspace_data[nVar];
          }
        } else {
          rankQR = (uint16_T)nVar;
          for (nVar = 0; nVar < rankQR; nVar++) {
            xCurrent_data[nVar] = workspace_data[workspace_size[0] + nVar];
          }
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return nonDegenerateWset;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Pat_RemoveDependentIneq__h(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace)
{
  real_T maxDiag;
  real_T tol;
  int32_T b_idx;
  int32_T d;
  int32_T idxDiag;
  int32_T idxDiag_tmp;
  int32_T ix0;
  int32_T iy0;
  int32_T nDepIneq;
  int32_T nFixedConstr;
  nDepIneq = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  idxDiag_tmp = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    if (workingset->nVar >= workingset->nActiveConstr) {
      b_idx = workingset->nVar;
    } else {
      b_idx = workingset->nActiveConstr;
    }

    tol = fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * (real_T)b_idx) *
      10.0;
    for (b_idx = 0; b_idx < nFixedConstr; b_idx++) {
      qrmanager->jpvt.data[b_idx] = 1;
    }

    for (b_idx = nFixedConstr + 1; b_idx <= nDepIneq; b_idx++) {
      qrmanager->jpvt.data[b_idx - 1] = 0;
    }

    for (b_idx = 0; b_idx < nDepIneq; b_idx++) {
      iy0 = qrmanager->ldq * b_idx;
      ix0 = workingset->ldA * b_idx;
      d = (uint16_T)idxDiag_tmp;
      for (idxDiag = 0; idxDiag < d; idxDiag++) {
        qrmanager->QR.data[iy0 + idxDiag] = workingset->ATwset.data[ix0 +
          idxDiag];
      }
    }

    NMPC_Path_Tracking_factorQRE_d(qrmanager, workingset->nVar,
      workingset->nActiveConstr);
    nDepIneq = 0;
    for (b_idx = workingset->nActiveConstr - 1; b_idx + 1 > idxDiag_tmp; b_idx--)
    {
      nDepIneq++;
      memspace->workspace_int.data[nDepIneq - 1] = qrmanager->jpvt.data[b_idx];
    }

    maxDiag = fabs(qrmanager->QR.data[0]);
    for (idxDiag = 0; idxDiag < b_idx; idxDiag++) {
      maxDiag = fmax(maxDiag, fabs(qrmanager->QR.data[((idxDiag + 1) *
        qrmanager->ldq + idxDiag) + 1]));
    }

    if (b_idx + 1 <= workingset->nVar) {
      idxDiag = qrmanager->ldq * b_idx + b_idx;
      while ((b_idx + 1 > nFixedConstr) && (fabs(qrmanager->QR.data[idxDiag]) <
              tol * maxDiag)) {
        nDepIneq++;
        memspace->workspace_int.data[nDepIneq - 1] = qrmanager->jpvt.data[b_idx];
        b_idx--;
        idxDiag = (idxDiag - qrmanager->ldq) - 1;
      }
    }

    NMPC_Path_Tracking_countsort(memspace->workspace_int.data, nDepIneq,
      memspace->workspace_sort.data, nFixedConstr + 1, workingset->nActiveConstr);
    for (nFixedConstr = nDepIneq; nFixedConstr >= 1; nFixedConstr--) {
      NMPC_Path_Tracking_removeConstr(workingset, memspace->
        workspace_int.data[nFixedConstr - 1]);
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgemv_cpg(int32_T m, int32_T n, const real_T
  A_data[], int32_T lda, const real_T x_data[], real_T y_data[])
{
  int32_T b;
  int32_T b_iy;
  if (n != 0) {
    int32_T scalarLB;
    int32_T vectorUB;
    b = (uint16_T)n;
    scalarLB = ((uint16_T)n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (b_iy = 0; b_iy <= vectorUB; b_iy += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&y_data[b_iy]);
      _mm_storeu_pd(&y_data[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
    }

    for (b_iy = scalarLB; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    scalarLB = 0;
    vectorUB = (n - 1) * lda + 1;
    for (b_iy = 1; lda < 0 ? b_iy >= vectorUB : b_iy <= vectorUB; b_iy += lda) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += A_data[b - 1] * x_data[b - b_iy];
      }

      y_data[scalarLB] += c;
      scalarLB++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T maxConstraintViolation_AMats_no(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *obj, const real_T x_data[])
{
  real_T v;
  int32_T k;
  int32_T mIneq;
  v = 0.0;
  mIneq = obj->sizes[2];
  if (obj->Aineq.size != 0) {
    for (k = 0; k < mIneq; k++) {
      obj->maxConstrWorkspace.data[k] = obj->bineq.data[k];
    }

    NMPC_Path_Tracking_xgemv_cpg(obj->nVar, obj->sizes[2], obj->Aineq.data,
      obj->ldA, x_data, obj->maxConstrWorkspace.data);
    mIneq = (uint16_T)obj->sizes[2];
    for (k = 0; k < mIneq; k++) {
      v = fmax(v, obj->maxConstrWorkspace.data[k]);
    }
  }

  memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 300U * sizeof(real_T));
  NMPC_Path_Tracking_xgemv_cpg(obj->nVar, 300, obj->Aeq.data, obj->ldA, x_data,
    obj->maxConstrWorkspace.data);
  for (k = 0; k < 300; k++) {
    v = fmax(v, fabs(obj->maxConstrWorkspace.data[k]));
  }

  return v;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T maxConstraintViolation_AMats_re(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *obj, const real_T x_data[])
{
  real_T obj_maxConstrWorkspace;
  real_T v;
  int32_T b;
  int32_T k;
  int32_T mIneq;
  v = 0.0;
  mIneq = obj->sizes[2];
  if (obj->Aineq.size != 0) {
    for (k = 0; k < mIneq; k++) {
      obj->maxConstrWorkspace.data[k] = obj->bineq.data[k];
    }

    NMPC_Path_Tracking_xgemv_cpg(401, obj->sizes[2], obj->Aineq.data, obj->ldA,
      x_data, obj->maxConstrWorkspace.data);
    b = (uint16_T)obj->sizes[2];
    for (k = 0; k < b; k++) {
      obj_maxConstrWorkspace = obj->maxConstrWorkspace.data[k] - x_data[k + 401];
      obj->maxConstrWorkspace.data[k] = obj_maxConstrWorkspace;
      v = fmax(v, obj_maxConstrWorkspace);
    }
  }

  memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0], 300U * sizeof(real_T));
  NMPC_Path_Tracking_xgemv_cpg(401, 300, obj->Aeq.data, obj->ldA, x_data,
    obj->maxConstrWorkspace.data);
  for (k = 0; k < 300; k++) {
    obj->maxConstrWorkspace.data[k] = (obj->maxConstrWorkspace.data[k] - x_data
      [(mIneq + k) + 401]) + x_data[(obj->sizes[2] + k) + 701];
    v = fmax(v, fabs(obj->maxConstrWorkspace.data[k]));
  }

  return v;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T NMPC_maxConstraintViolation_lng(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *obj, const real_T x_data[])
{
  real_T v;
  int32_T b;
  int32_T idx;
  if (obj->probType == 2) {
    v = maxConstraintViolation_AMats_re(obj, x_data);
  } else {
    v = maxConstraintViolation_AMats_no(obj, x_data);
  }

  if (obj->sizes[3] > 0) {
    b = (uint16_T)obj->sizes[3];
    for (idx = 0; idx < b; idx++) {
      v = fmax(v, -x_data[obj->indexLB.data[idx] - 1] - obj->lb.data
               [obj->indexLB.data[idx] - 1]);
    }
  }

  if (obj->sizes[4] > 0) {
    b = (uint16_T)obj->sizes[4];
    for (idx = 0; idx < b; idx++) {
      v = fmax(v, x_data[obj->indexUB.data[idx] - 1] - obj->ub.data
               [obj->indexUB.data[idx] - 1]);
    }
  }

  if (obj->sizes[0] > 0) {
    b = (uint16_T)obj->sizes[0];
    for (idx = 0; idx < b; idx++) {
      v = fmax(v, fabs(x_data[obj->indexFixed.data[idx] - 1] - obj->ub.data
                       [obj->indexFixed.data[idx] - 1]));
    }
  }

  return v;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tr_PresolveWorkingSet(s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T
  *solution, s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace,
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T
  *qrmanager)
{
  real_T constrViolation;
  int32_T b;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T idxStartIneq_tmp;
  boolean_T guard1;
  boolean_T okWorkingSet;
  solution->state = 82;
  b = NMPC_Path_Tr_RemoveDependentEq_(memspace, workingset, qrmanager);
  if ((b != -1) && (workingset->nActiveConstr <= qrmanager->ldq)) {
    NMPC_Path__RemoveDependentIneq_(workingset, qrmanager, memspace);
    okWorkingSet = NMPC_Pa_feasibleX0ForWorkingSet
      (memspace->workspace_float.data, memspace->workspace_float.size,
       solution->xstar.data, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      NMPC_Pat_RemoveDependentIneq__h(workingset, qrmanager, memspace);
      okWorkingSet = NMPC_Pa_feasibleX0ForWorkingSet
        (memspace->workspace_float.data, memspace->workspace_float.size,
         solution->xstar.data, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if (workingset->nWConstr[0] + workingset->nWConstr[1] == workingset->nVar)
      {
        constrViolation = NMPC_maxConstraintViolation_lng(workingset,
          solution->xstar.data);
        if (constrViolation > 1.0E-6) {
          solution->state = -2;
        }
      }
    }
  } else {
    solution->state = -3;
    idxStartIneq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
    idxStartIneq = idxStartIneq_tmp + 1;
    idxEndIneq = workingset->nActiveConstr;
    for (b = idxStartIneq; b <= idxEndIneq; b++) {
      workingset->isActiveConstr.data[(workingset->isActiveIdx
        [workingset->Wid.data[b - 1] - 1] + workingset->Wlocalidx.data[b - 1]) -
        2] = false;
    }

    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = idxStartIneq_tmp;
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgemv_cpge(int32_T m, int32_T n, const real_T A
  [160801], int32_T lda, const real_T x_data[], real_T y_data[])
{
  int32_T b_iy;
  int32_T ia;
  if ((m != 0) && (n != 0)) {
    int32_T b;
    int32_T ix;
    for (b_iy = 0; b_iy < m; b_iy++) {
      y_data[b_iy] = 0.0;
    }

    ix = 0;
    b = (n - 1) * lda + 1;
    for (b_iy = 1; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda) {
      int32_T c;
      c = (b_iy + m) - 1;
      for (ia = b_iy; ia <= c; ia++) {
        int32_T tmp;
        tmp = ia - b_iy;
        y_data[tmp] += A[ia - 1] * x_data[ix];
      }

      ix++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_T_computeGrad_StoreHx(s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *obj,
  const real_T H[160801], const real_T f_data[], const real_T x_data[])
{
  __m128d tmp;
  int32_T b;
  int32_T i;
  int32_T i_tmp;
  int32_T ixlast;
  int32_T iy;
  int32_T k;
  switch (obj->objtype) {
   case 5:
    b = obj->nvar;
    for (i = 0; i <= b - 2; i++) {
      obj->grad.data[i] = 0.0;
    }

    obj->grad.data[obj->nvar - 1] = obj->gammaScalar;
    break;

   case 3:
    NMPC_Path_Tracking_xgemv_cpge(obj->nvar, obj->nvar, H, obj->nvar, x_data,
      obj->Hx.data);
    b = obj->nvar;
    for (i = 0; i < b; i++) {
      obj->grad.data[i] = obj->Hx.data[i];
    }

    if (obj->hasLinear) {
      iy = obj->grad.size;
      i = obj->grad.size;
      for (k = 0; k < i; k++) {
        NMPC_Path_Tracking_B.y_data_g[k] = obj->grad.data[k];
      }

      if (obj->nvar >= 1) {
        ixlast = obj->nvar;
        i = (obj->nvar / 2) << 1;
        b = i - 2;
        for (k = 0; k <= b; k += 2) {
          tmp = _mm_loadu_pd(&NMPC_Path_Tracking_B.y_data_g[k]);
          _mm_storeu_pd(&NMPC_Path_Tracking_B.y_data_g[k], _mm_add_pd(tmp,
            _mm_loadu_pd(&f_data[k])));
        }

        for (k = i; k < ixlast; k++) {
          NMPC_Path_Tracking_B.y_data_g[k] += f_data[k];
        }
      }

      for (k = 0; k < iy; k++) {
        obj->grad.data[k] = NMPC_Path_Tracking_B.y_data_g[k];
      }
    }
    break;

   case 4:
    ixlast = obj->maxVar;
    NMPC_Path_Tracking_xgemv_cpge(obj->nvar, obj->nvar, H, obj->nvar, x_data,
      obj->Hx.data);
    iy = obj->nvar + 1;
    i_tmp = (obj->maxVar - obj->nvar) - 1;
    i = (((i_tmp / 2) << 1) + obj->nvar) + 1;
    b = i - 2;
    for (k = iy; k <= b; k += 2) {
      _mm_storeu_pd(&obj->Hx.data[k - 1], _mm_mul_pd(_mm_loadu_pd(&x_data[k - 1]),
        _mm_set1_pd(obj->beta)));
    }

    for (k = i; k < ixlast; k++) {
      obj->Hx.data[k - 1] = x_data[k - 1] * obj->beta;
    }

    b = (uint16_T)(obj->maxVar - 1);
    for (i = 0; i < b; i++) {
      obj->grad.data[i] = obj->Hx.data[i];
    }

    if (obj->hasLinear) {
      iy = obj->grad.size;
      i = obj->grad.size;
      for (k = 0; k < i; k++) {
        NMPC_Path_Tracking_B.y_data_g[k] = obj->grad.data[k];
      }

      if (obj->nvar >= 1) {
        ixlast = obj->nvar;
        i = (obj->nvar / 2) << 1;
        b = i - 2;
        for (k = 0; k <= b; k += 2) {
          tmp = _mm_loadu_pd(&NMPC_Path_Tracking_B.y_data_g[k]);
          _mm_storeu_pd(&NMPC_Path_Tracking_B.y_data_g[k], _mm_add_pd(tmp,
            _mm_loadu_pd(&f_data[k])));
        }

        for (k = i; k < ixlast; k++) {
          NMPC_Path_Tracking_B.y_data_g[k] += f_data[k];
        }
      }

      for (k = 0; k < iy; k++) {
        obj->grad.data[k] = NMPC_Path_Tracking_B.y_data_g[k];
      }
    }

    if (i_tmp >= 1) {
      iy = obj->nvar;
      i = (i_tmp / 2) << 1;
      b = i - 2;
      for (ixlast = 0; ixlast <= b; ixlast += 2) {
        k = iy + ixlast;
        tmp = _mm_loadu_pd(&obj->grad.data[k]);
        _mm_storeu_pd(&obj->grad.data[k], _mm_add_pd(tmp, _mm_set1_pd(obj->rho)));
      }

      for (ixlast = i; ixlast < i_tmp; ixlast++) {
        k = iy + ixlast;
        obj->grad.data[k] += obj->rho;
      }
    }
    break;
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T NMPC_Path_T_computeFval_ReuseHx(const
  s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *obj, real_T workspace_data[], const real_T
  f_data[], const real_T x_data[])
{
  real_T val;
  int32_T b_ixlast;
  int32_T ixlast;
  int32_T maxRegVar;
  val = 0.0;
  switch (obj->objtype) {
   case 5:
    val = x_data[obj->nvar - 1] * obj->gammaScalar;
    break;

   case 3:
    {
      if (obj->hasLinear) {
        int32_T b;
        b = obj->nvar;
        maxRegVar = (obj->nvar / 2) << 1;
        ixlast = maxRegVar - 2;
        for (b_ixlast = 0; b_ixlast <= ixlast; b_ixlast += 2) {
          __m128d tmp_0;
          tmp_0 = _mm_loadu_pd(&obj->Hx.data[b_ixlast]);
          _mm_storeu_pd(&workspace_data[b_ixlast], _mm_add_pd(_mm_mul_pd
            (_mm_set1_pd(0.5), tmp_0), _mm_loadu_pd(&f_data[b_ixlast])));
        }

        for (b_ixlast = maxRegVar; b_ixlast < b; b_ixlast++) {
          workspace_data[b_ixlast] = 0.5 * obj->Hx.data[b_ixlast] +
            f_data[b_ixlast];
        }

        if (obj->nvar >= 1) {
          ixlast = obj->nvar;
          for (maxRegVar = 0; maxRegVar < ixlast; maxRegVar++) {
            val += workspace_data[maxRegVar] * x_data[maxRegVar];
          }
        }
      } else {
        if (obj->nvar >= 1) {
          ixlast = obj->nvar;
          for (maxRegVar = 0; maxRegVar < ixlast; maxRegVar++) {
            val += x_data[maxRegVar] * obj->Hx.data[maxRegVar];
          }
        }

        val *= 0.5;
      }
    }
    break;

   case 4:
    {
      int32_T b;
      b = obj->maxVar;
      if (obj->hasLinear) {
        int32_T b_tmp;
        ixlast = obj->nvar;
        for (maxRegVar = 0; maxRegVar < ixlast; maxRegVar++) {
          workspace_data[maxRegVar] = f_data[maxRegVar];
        }

        ixlast = obj->maxVar - obj->nvar;
        for (maxRegVar = 0; maxRegVar <= ixlast - 2; maxRegVar++) {
          workspace_data[obj->nvar + maxRegVar] = obj->rho;
        }

        b_tmp = (uint16_T)(obj->maxVar - 1);
        maxRegVar = ((uint16_T)(obj->maxVar - 1) / 2) << 1;
        ixlast = maxRegVar - 2;
        for (b_ixlast = 0; b_ixlast <= ixlast; b_ixlast += 2) {
          __m128d tmp;
          __m128d tmp_0;
          tmp_0 = _mm_loadu_pd(&obj->Hx.data[b_ixlast]);
          tmp = _mm_loadu_pd(&workspace_data[b_ixlast]);
          _mm_storeu_pd(&workspace_data[b_ixlast], _mm_add_pd(tmp, _mm_mul_pd
            (_mm_set1_pd(0.5), tmp_0)));
        }

        for (b_ixlast = maxRegVar; b_ixlast < b_tmp; b_ixlast++) {
          workspace_data[b_ixlast] += 0.5 * obj->Hx.data[b_ixlast];
        }

        if (obj->maxVar - 1 >= 1) {
          for (maxRegVar = 0; maxRegVar <= b - 2; maxRegVar++) {
            val += workspace_data[maxRegVar] * x_data[maxRegVar];
          }
        }
      } else {
        if (obj->maxVar - 1 >= 1) {
          for (ixlast = 0; ixlast <= b - 2; ixlast++) {
            val += x_data[ixlast] * obj->Hx.data[ixlast];
          }
        }

        val *= 0.5;
        b_ixlast = obj->nvar + 1;
        for (ixlast = b_ixlast; ixlast < b; ixlast++) {
          val += x_data[ixlast - 1] * obj->rho;
        }
      }
    }
    break;
  }

  return val;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgeqrf(real_T A_data[], const int32_T A_size[2],
  int32_T m, int32_T n, real_T tau_data[], int32_T *tau_size)
{
  int32_T i;
  int32_T loop_ub;
  int32_T minmn;
  if (m <= n) {
    minmn = m;
  } else {
    minmn = n;
  }

  if (A_size[0] <= A_size[1]) {
    loop_ub = A_size[0];
  } else {
    loop_ub = A_size[1];
  }

  *tau_size = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    tau_data[i] = 0.0;
  }

  if (minmn >= 1) {
    NMPC_Path_Tracking_qrf(A_data, A_size, m, n, minmn, tau_data);
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_factorQR(s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *obj,
  const real_T A_data[], int32_T mrows, int32_T ncols, int32_T ldA)
{
  int32_T b;
  int32_T idx;
  int32_T ix0;
  int32_T iy0;
  int32_T k;
  static const int32_T offsets[4] = { 0, 1, 2, 3 };

  boolean_T guard1;
  idx = mrows * ncols;
  guard1 = false;
  if (idx > 0) {
    for (idx = 0; idx < ncols; idx++) {
      ix0 = ldA * idx;
      iy0 = obj->ldq * idx;
      b = (uint16_T)mrows;
      for (k = 0; k < b; k++) {
        obj->QR.data[iy0 + k] = A_data[ix0 + k];
      }
    }

    guard1 = true;
  } else if (idx == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    k = (ncols / 4) << 2;
    ix0 = k - 4;
    for (idx = 0; idx <= ix0; idx += 4) {
      _mm_storeu_si128((__m128i *)&obj->jpvt.data[idx], _mm_add_epi32
                       (_mm_add_epi32(_mm_set1_epi32(idx), _mm_loadu_si128((
        const __m128i *)&offsets[0])), _mm_set1_epi32(1)));
    }

    for (idx = k; idx < ncols; idx++) {
      obj->jpvt.data[idx] = idx + 1;
    }

    if (mrows <= ncols) {
      obj->minRowCol = mrows;
    } else {
      obj->minRowCol = ncols;
    }

    NMPC_Path_Tracking_xgeqrf(obj->QR.data, obj->QR.size, mrows, ncols,
      obj->tau.data, &obj->tau.size);
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xrotg(real_T *a, real_T *b, real_T *c, real_T *s)
{
  real_T absa;
  real_T absb;
  real_T roe;
  real_T scale;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *a = 0.0;
    *b = 0.0;
  } else {
    real_T ads;
    real_T bds;
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }

    *a = scale;
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tra_squareQ_appendCol(s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *obj,
  const real_T vec_data[], int32_T iv0)
{
  real_T b_c;
  real_T s;
  real_T temp;
  int32_T Qk0;
  int32_T b_iy;
  int32_T e;
  int32_T idx;
  int32_T iy;
  int32_T iyend;
  int32_T temp_tmp;
  if (obj->mrows <= obj->ncols + 1) {
    obj->minRowCol = obj->mrows;
  } else {
    obj->minRowCol = obj->ncols + 1;
  }

  b_iy = obj->ldq * obj->ncols;
  idx = obj->ldq;
  if (obj->mrows != 0) {
    iyend = b_iy + obj->mrows;
    for (Qk0 = b_iy + 1; Qk0 <= iyend; Qk0++) {
      obj->QR.data[Qk0 - 1] = 0.0;
    }

    iy = (obj->mrows - 1) * obj->ldq + 1;
    for (Qk0 = 1; idx < 0 ? Qk0 >= iy : Qk0 <= iy; Qk0 += idx) {
      b_c = 0.0;
      e = (Qk0 + obj->mrows) - 1;
      for (iyend = Qk0; iyend <= e; iyend++) {
        b_c += vec_data[((iv0 + iyend) - Qk0) - 1] * obj->Q.data[iyend - 1];
      }

      obj->QR.data[b_iy] += b_c;
      b_iy++;
    }
  }

  obj->ncols++;
  obj->jpvt.data[obj->ncols - 1] = obj->ncols;
  for (idx = obj->mrows - 2; idx + 2 > obj->ncols; idx--) {
    b_iy = (obj->ncols - 1) * obj->ldq + idx;
    temp = obj->QR.data[b_iy + 1];
    NMPC_Path_Tracking_xrotg(&obj->QR.data[b_iy], &temp, &b_c, &s);
    obj->QR.data[b_iy + 1] = temp;
    Qk0 = obj->ldq * idx;
    iyend = obj->mrows;
    if (obj->mrows >= 1) {
      iy = obj->ldq + Qk0;
      for (b_iy = 0; b_iy < iyend; b_iy++) {
        e = iy + b_iy;
        temp_tmp = Qk0 + b_iy;
        temp = obj->Q.data[temp_tmp] * b_c + obj->Q.data[e] * s;
        obj->Q.data[e] = obj->Q.data[e] * b_c - obj->Q.data[temp_tmp] * s;
        obj->Q.data[temp_tmp] = temp;
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Trac_deleteColMoveEnd(s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *obj,
  int32_T idx)
{
  real_T b_s;
  real_T b_temp;
  real_T c_c;
  int32_T QRk0;
  int32_T QRk0_tmp;
  int32_T b_ix;
  int32_T b_n;
  int32_T d_temp_tmp;
  int32_T i;
  int32_T idxRotGCol;
  int32_T ix;
  int32_T k;
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt.data[i - 1] != idx)) {
      i++;
    }

    idx = i;
  }

  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    obj->jpvt.data[idx - 1] = obj->jpvt.data[obj->ncols - 1];
    QRk0 = obj->minRowCol;
    for (i = 0; i < QRk0; i++) {
      obj->QR.data[i + obj->ldq * (idx - 1)] = obj->QR.data[(obj->ncols - 1) *
        obj->ldq + i];
    }

    obj->ncols--;
    if (obj->mrows <= obj->ncols) {
      obj->minRowCol = obj->mrows;
    } else {
      obj->minRowCol = obj->ncols;
    }

    if (idx < obj->mrows) {
      if (obj->mrows - 1 <= obj->ncols) {
        i = obj->mrows - 1;
      } else {
        i = obj->ncols;
      }

      k = i;
      idxRotGCol = (idx - 1) * obj->ldq;
      while (k >= idx) {
        QRk0 = k + idxRotGCol;
        b_temp = obj->QR.data[QRk0];
        NMPC_Path_Tracking_xrotg(&obj->QR.data[QRk0 - 1], &b_temp, &c_c, &b_s);
        obj->QR.data[QRk0] = b_temp;
        obj->QR.data[k + obj->ldq * (k - 1)] = 0.0;
        QRk0 = obj->ldq * idx + k;
        b_ix = obj->ncols - idx;
        if (b_ix >= 1) {
          ix = QRk0 - 1;
          for (b_n = 0; b_n < b_ix; b_n++) {
            b_temp = obj->QR.data[ix] * c_c + obj->QR.data[QRk0] * b_s;
            obj->QR.data[QRk0] = obj->QR.data[QRk0] * c_c - obj->QR.data[ix] *
              b_s;
            obj->QR.data[ix] = b_temp;
            QRk0 += obj->ldq;
            ix += obj->ldq;
          }
        }

        QRk0 = (k - 1) * obj->ldq;
        b_ix = obj->mrows;
        if (obj->mrows >= 1) {
          ix = obj->ldq + QRk0;
          for (b_n = 0; b_n < b_ix; b_n++) {
            d_temp_tmp = ix + b_n;
            QRk0_tmp = QRk0 + b_n;
            b_temp = obj->Q.data[QRk0_tmp] * c_c + obj->Q.data[d_temp_tmp] * b_s;
            obj->Q.data[d_temp_tmp] = obj->Q.data[d_temp_tmp] * c_c -
              obj->Q.data[QRk0_tmp] * b_s;
            obj->Q.data[QRk0_tmp] = b_temp;
          }
        }

        k--;
      }

      for (k = idx + 1; k <= i; k++) {
        QRk0_tmp = (k - 1) * obj->ldq;
        QRk0 = k + QRk0_tmp;
        b_temp = obj->QR.data[QRk0];
        NMPC_Path_Tracking_xrotg(&obj->QR.data[QRk0 - 1], &b_temp, &c_c, &b_s);
        obj->QR.data[QRk0] = b_temp;
        QRk0 = (obj->ldq + 1) * k;
        b_n = obj->ncols - k;
        if (b_n >= 1) {
          b_ix = QRk0 - 1;
          for (idxRotGCol = 0; idxRotGCol < b_n; idxRotGCol++) {
            b_temp = obj->QR.data[b_ix] * c_c + obj->QR.data[QRk0] * b_s;
            obj->QR.data[QRk0] = obj->QR.data[QRk0] * c_c - obj->QR.data[b_ix] *
              b_s;
            obj->QR.data[b_ix] = b_temp;
            QRk0 += obj->ldq;
            b_ix += obj->ldq;
          }
        }

        b_n = obj->mrows;
        if (obj->mrows >= 1) {
          b_ix = obj->ldq + QRk0_tmp;
          for (idxRotGCol = 0; idxRotGCol < b_n; idxRotGCol++) {
            ix = b_ix + idxRotGCol;
            d_temp_tmp = QRk0_tmp + idxRotGCol;
            b_temp = obj->Q.data[d_temp_tmp] * c_c + obj->Q.data[ix] * b_s;
            obj->Q.data[ix] = obj->Q.data[ix] * c_c - obj->Q.data[d_temp_tmp] *
              b_s;
            obj->Q.data[d_temp_tmp] = b_temp;
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static boolean_T NMPC_Path_Tracking_strcmp(const char_T a[7])
{
  int32_T ret;
  static const char_T b_b[7] = { 'f', 'm', 'i', 'n', 'c', 'o', 'n' };

  ret = memcmp(&a[0], &b_b[0], 7);
  return ret == 0;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgemm(int32_T m, int32_T n, int32_T k, const
  real_T A[160801], int32_T lda, const real_T B_data[], int32_T ib0, int32_T ldb,
  real_T C_data[], int32_T ldc)
{
  int32_T b;
  int32_T cr;
  int32_T ic;
  if ((m != 0) && (n != 0)) {
    int32_T br;
    int32_T lastColC;
    br = ib0;
    lastColC = (n - 1) * ldc;
    for (cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      b = cr + m;
      for (ic = cr + 1; ic <= b; ic++) {
        C_data[ic - 1] = 0.0;
      }
    }

    for (cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      int32_T ar;
      int32_T c;
      ar = -1;
      c = br + k;
      for (ic = br; ic < c; ic++) {
        int32_T d;
        int32_T scalarLB;
        int32_T vectorUB;
        d = cr + m;
        scalarLB = ((((d - cr) / 2) << 1) + cr) + 1;
        vectorUB = scalarLB - 2;
        for (b = cr + 1; b <= vectorUB; b += 2) {
          __m128d tmp;
          tmp = _mm_loadu_pd(&C_data[b - 1]);
          _mm_storeu_pd(&C_data[b - 1], _mm_add_pd(_mm_mul_pd(_mm_set1_pd
            (B_data[ic - 1]), _mm_loadu_pd(&A[(ar + b) - cr])), tmp));
        }

        for (b = scalarLB; b <= d; b++) {
          C_data[b - 1] += A[(ar + b) - cr] * B_data[ic - 1];
        }

        ar += lda;
      }

      br += ldb;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgemm_o(int32_T m, int32_T n, int32_T k, const
  real_T A_data[], int32_T ia0, int32_T lda, const real_T B_data[], int32_T ldb,
  real_T C_data[], int32_T ldc)
{
  int32_T b_w;
  int32_T cr;
  int32_T ic;
  if ((m != 0) && (n != 0)) {
    int32_T br;
    int32_T lastColC;
    lastColC = (n - 1) * ldc;
    for (cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      br = cr + m;
      for (ic = cr + 1; ic <= br; ic++) {
        C_data[ic - 1] = 0.0;
      }
    }

    br = -1;
    for (cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      int32_T ar;
      int32_T c;
      ar = ia0;
      c = cr + m;
      for (ic = cr + 1; ic <= c; ic++) {
        real_T temp;
        temp = 0.0;
        for (b_w = 0; b_w < k; b_w++) {
          temp += A_data[(b_w + ar) - 1] * B_data[(b_w + br) + 1];
        }

        C_data[ic - 1] += temp;
        ar += lda;
      }

      br += ldb;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_fullColLDL2_(s_LuoN3prQfsei9XMpifhsFF_NMPC_T *obj,
  int32_T LD_offset, int32_T NColsRemain)
{
  int32_T LDimSizeP1;
  int32_T b_k;
  int32_T ijA;
  int32_T j;
  int32_T k;
  int32_T subMatrixDim;
  LDimSizeP1 = obj->ldm;
  for (k = 0; k < NColsRemain; k++) {
    __m128d tmp;
    real_T alpha1;
    real_T y;
    int32_T LD_diagOffset;
    int32_T alpha1_tmp;
    LD_diagOffset = (LDimSizeP1 + 1) * k + LD_offset;
    alpha1 = -1.0 / obj->FMat.data[LD_diagOffset - 1];
    subMatrixDim = (NColsRemain - k) - 2;
    for (b_k = 0; b_k <= subMatrixDim; b_k++) {
      obj->workspace_ = obj->FMat.data[LD_diagOffset + b_k];
    }

    y = obj->workspace_;
    if (!(alpha1 == 0.0)) {
      int32_T jA;
      jA = LD_diagOffset + LDimSizeP1;
      for (j = 0; j <= subMatrixDim; j++) {
        if (y != 0.0) {
          real_T temp;
          int32_T b;
          temp = y * alpha1;
          b = (subMatrixDim + jA) + 1;
          b_k = ((((b - jA) / 2) << 1) + jA) + 1;
          alpha1_tmp = b_k - 2;
          for (ijA = jA + 1; ijA <= alpha1_tmp; ijA += 2) {
            tmp = _mm_loadu_pd(&obj->FMat.data[ijA - 1]);
            _mm_storeu_pd(&obj->FMat.data[ijA - 1], _mm_add_pd(tmp, _mm_set1_pd
              (obj->workspace_ * temp)));
          }

          for (ijA = b_k; ijA <= b; ijA++) {
            obj->FMat.data[ijA - 1] += obj->workspace_ * temp;
          }
        }

        jA += obj->ldm;
      }
    }

    alpha1 = 1.0 / obj->FMat.data[LD_diagOffset - 1];
    j = (LD_diagOffset + subMatrixDim) + 1;
    b_k = ((((j - LD_diagOffset) / 2) << 1) + LD_diagOffset) + 1;
    alpha1_tmp = b_k - 2;
    for (subMatrixDim = LD_diagOffset + 1; subMatrixDim <= alpha1_tmp;
         subMatrixDim += 2) {
      tmp = _mm_loadu_pd(&obj->FMat.data[subMatrixDim - 1]);
      _mm_storeu_pd(&obj->FMat.data[subMatrixDim - 1], _mm_mul_pd(tmp,
        _mm_set1_pd(alpha1)));
    }

    for (subMatrixDim = b_k; subMatrixDim <= j; subMatrixDim++) {
      obj->FMat.data[subMatrixDim - 1] *= alpha1;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Track_partialColLDL3_(s_LuoN3prQfsei9XMpifhsFF_NMPC_T *obj,
  int32_T LD_offset, int32_T NColsRemain)
{
  __m128d tmp;
  int32_T LD_diagOffset;
  int32_T LDimSizeP1;
  int32_T br;
  int32_T c;
  int32_T d;
  int32_T e;
  int32_T ia;
  int32_T ix;
  int32_T k;
  int32_T subBlockSize;
  int32_T subRows;
  LDimSizeP1 = obj->ldm + 1;
  for (k = 0; k < 48; k++) {
    real_T y;
    subRows = (NColsRemain - k) - 1;
    LD_diagOffset = (LDimSizeP1 * k + LD_offset) - 1;
    for (subBlockSize = 0; subBlockSize <= subRows; subBlockSize++) {
      obj->workspace_ = obj->FMat.data[LD_diagOffset + subBlockSize];
    }

    for (subBlockSize = 0; subBlockSize < NColsRemain; subBlockSize++) {
      obj->workspace2_ = obj->workspace_;
    }

    br = obj->ldm;
    y = obj->workspace2_;
    if ((NColsRemain != 0) && (k != 0)) {
      ix = LD_offset + k;
      c = (k - 1) * obj->ldm + 1;
      for (subBlockSize = 1; br < 0 ? subBlockSize >= c : subBlockSize <= c;
           subBlockSize += br) {
        d = subBlockSize + NColsRemain;
        for (ia = subBlockSize; ia < d; ia++) {
          y += -obj->FMat.data[ix - 1] * obj->workspace_;
        }

        ix += obj->ldm;
      }
    }

    obj->workspace2_ = y;
    for (subBlockSize = 0; subBlockSize < NColsRemain; subBlockSize++) {
      obj->workspace_ = y;
    }

    for (subBlockSize = 0; subBlockSize <= subRows; subBlockSize++) {
      obj->FMat.data[LD_diagOffset + subBlockSize] = obj->workspace_;
    }

    subBlockSize = (subRows / 2) << 1;
    ia = subBlockSize - 2;
    for (ix = 0; ix <= ia; ix += 2) {
      c = (ix + LD_diagOffset) + 1;
      tmp = _mm_loadu_pd(&obj->FMat.data[c]);
      _mm_storeu_pd(&obj->FMat.data[c], _mm_div_pd(tmp, _mm_set1_pd
        (obj->FMat.data[LD_diagOffset])));
    }

    for (ix = subBlockSize; ix < subRows; ix++) {
      c = (ix + LD_diagOffset) + 1;
      obj->FMat.data[c] /= obj->FMat.data[LD_diagOffset];
    }
  }

  for (k = 48; k <= NColsRemain - 1; k += 48) {
    int32_T lastColC;
    br = NColsRemain - k;
    if (br >= 48) {
      subBlockSize = 48;
    } else {
      subBlockSize = br;
    }

    subRows = k + subBlockSize;
    for (ia = k; ia < subRows; ia++) {
      LD_diagOffset = subRows - ia;
      for (ix = 0; ix < 48; ix++) {
        obj->workspace2_ = obj->FMat.data[((LD_offset + ia) + ix * obj->ldm) - 1];
      }

      d = obj->ldm;
      if (LD_diagOffset != 0) {
        e = (obj->ldm * 47 + ia) + 1;
        for (ix = ia + 1; d < 0 ? ix >= e : ix <= e; ix += d) {
          lastColC = ix + LD_diagOffset;
          for (c = ix; c < lastColC; c++) {
            /* Check node always fails. would cause program termination and was eliminated */
          }
        }
      }
    }

    if (subRows < NColsRemain) {
      subRows = br - subBlockSize;
      LD_diagOffset = ((LD_offset + subBlockSize) + LDimSizeP1 * k) - 1;
      for (ia = 0; ia < 48; ia++) {
        ix = (LD_offset + k) + ia * obj->ldm;
        for (br = 0; br < subBlockSize; br++) {
          obj->workspace2_ = obj->FMat.data[(ix + br) - 1];
        }
      }

      ix = obj->ldm;
      if ((subRows != 0) && (subBlockSize != 0)) {
        lastColC = (subBlockSize - 1) * obj->ldm + LD_diagOffset;
        br = 0;
        for (c = LD_diagOffset; ix < 0 ? c >= lastColC : c <= lastColC; c += ix)
        {
          int32_T g;
          br++;
          g = ix * 47 + br;
          for (d = br; ix < 0 ? d >= g : d <= g; d += ix) {
            int32_T h;
            h = c + subRows;
            subBlockSize = ((((h - c) / 2) << 1) + c) + 1;
            ia = subBlockSize - 2;
            for (e = c + 1; e <= ia; e += 2) {
              tmp = _mm_loadu_pd(&obj->FMat.data[e - 1]);
              _mm_storeu_pd(&obj->FMat.data[e - 1], _mm_add_pd(tmp, _mm_set1_pd(
                -obj->workspace2_ * obj->workspace_)));
            }

            for (e = subBlockSize; e <= h; e++) {
              obj->FMat.data[e - 1] += -obj->workspace2_ * obj->workspace_;
            }
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static int32_T NMPC_Path_Tracking_xpotrf(int32_T n, real_T A_data[], int32_T lda)
{
  int32_T b_j;
  int32_T b_k;
  int32_T ia;
  int32_T idxA1j;
  int32_T info;
  boolean_T exitg1;
  info = 0;
  b_j = 0;
  exitg1 = false;
  while ((!exitg1) && (b_j <= n - 1)) {
    real_T c;
    real_T ssq;
    int32_T idxAjj;
    idxA1j = b_j * lda;
    idxAjj = idxA1j + b_j;
    ssq = 0.0;
    if (b_j >= 1) {
      for (b_k = 0; b_k < b_j; b_k++) {
        c = A_data[idxA1j + b_k];
        ssq += c * c;
      }
    }

    ssq = A_data[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = sqrt(ssq);
      A_data[idxAjj] = ssq;
      if (b_j + 1 < n) {
        int32_T ia0;
        int32_T nmj;
        nmj = (n - b_j) - 2;
        ia0 = (idxA1j + lda) + 1;
        idxAjj += lda;
        if ((b_j != 0) && (nmj + 1 != 0)) {
          int32_T b;
          int32_T iy;
          iy = idxAjj;
          b = lda * nmj + ia0;
          for (b_k = ia0; lda < 0 ? b_k >= b : b_k <= b; b_k += lda) {
            int32_T d;
            c = 0.0;
            d = (b_k + b_j) - 1;
            for (ia = b_k; ia <= d; ia++) {
              c += A_data[(idxA1j + ia) - b_k] * A_data[ia - 1];
            }

            A_data[iy] -= c;
            iy += lda;
          }
        }

        ssq = 1.0 / ssq;
        nmj = (lda * nmj + idxAjj) + 1;
        for (idxA1j = idxAjj + 1; lda < 0 ? idxA1j >= nmj : idxA1j <= nmj;
             idxA1j += lda) {
          A_data[idxA1j - 1] *= ssq;
        }
      }

      b_j++;
    } else {
      A_data[idxAjj] = ssq;
      info = b_j + 1;
      exitg1 = true;
    }
  }

  return info;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgemv_cpgex(int32_T m, int32_T n, const real_T
  A_data[], int32_T ia0, int32_T lda, const real_T x_data[], real_T y_data[])
{
  int32_T b_iy;
  int32_T ia;
  if (m != 0) {
    int32_T b;
    int32_T ix;
    for (b_iy = 0; b_iy < m; b_iy++) {
      y_data[b_iy] = 0.0;
    }

    ix = 0;
    b = (n - 1) * lda + ia0;
    for (b_iy = ia0; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda) {
      int32_T c;
      c = (b_iy + m) - 1;
      for (ia = b_iy; ia <= c; ia++) {
        int32_T tmp;
        tmp = ia - b_iy;
        y_data[tmp] += A_data[ia - 1] * x_data[ix];
      }

      ix++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_factor_l(s_LuoN3prQfsei9XMpifhsFF_NMPC_T *obj,
  const real_T A[160801], int32_T ndims, int32_T ldA)
{
  real_T s;
  real_T smax;
  int32_T A_maxDiag_idx;
  int32_T LDimSizeP1;
  int32_T exitg2;
  int32_T iy0;
  int32_T k;
  int32_T order;
  boolean_T exitg1;
  LDimSizeP1 = obj->ldm + 1;
  obj->ndims = ndims;
  for (k = 0; k < ndims; k++) {
    order = ldA * k;
    iy0 = obj->ldm * k;
    for (A_maxDiag_idx = 0; A_maxDiag_idx < ndims; A_maxDiag_idx++) {
      obj->FMat.data[iy0 + A_maxDiag_idx] = A[A_maxDiag_idx + order];
    }
  }

  if (ndims < 1) {
    A_maxDiag_idx = -1;
  } else {
    A_maxDiag_idx = 0;
    if (ndims > 1) {
      smax = fabs(obj->FMat.data[0]);
      for (k = 2; k <= ndims; k++) {
        s = fabs(obj->FMat.data[(k - 1) * LDimSizeP1]);
        if (s > smax) {
          A_maxDiag_idx = k - 1;
          smax = s;
        }
      }
    }
  }

  obj->regTol_ = fmax(fabs(obj->FMat.data[obj->ldm * A_maxDiag_idx +
    A_maxDiag_idx]) * 2.2204460492503131E-16, 0.0);
  if (ndims > 128) {
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < ndims)) {
      A_maxDiag_idx = LDimSizeP1 * k + 1;
      order = ndims - k;
      if (k + 48 <= ndims) {
        NMPC_Path_Track_partialColLDL3_(obj, A_maxDiag_idx, order);
        k += 48;
      } else {
        NMPC_Path_Tracking_fullColLDL2_(obj, A_maxDiag_idx, order);
        exitg1 = true;
      }
    }
  } else {
    NMPC_Path_Tracking_fullColLDL2_(obj, 1, ndims);
  }

  if (obj->ConvexCheck) {
    LDimSizeP1 = 0;
    do {
      exitg2 = 0;
      if (LDimSizeP1 <= ndims - 1) {
        if (obj->FMat.data[obj->ldm * LDimSizeP1 + LDimSizeP1] <= 0.0) {
          obj->info = -LDimSizeP1 - 1;
          exitg2 = 1;
        } else {
          LDimSizeP1++;
        }
      } else {
        obj->ConvexCheck = false;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_factor(s_LuoN3prQfsei9XMpifhsFF_NMPC_T *obj,
  const real_T A[160801], int32_T ndims, int32_T ldA)
{
  int32_T b_k;
  int32_T idx;
  int32_T ix0;
  int32_T iy0;
  obj->ndims = ndims;
  for (idx = 0; idx < ndims; idx++) {
    ix0 = ldA * idx;
    iy0 = obj->ldm * idx;
    for (b_k = 0; b_k < ndims; b_k++) {
      obj->FMat.data[iy0 + b_k] = A[b_k + ix0];
    }
  }

  obj->info = NMPC_Path_Tracking_xpotrf(ndims, obj->FMat.data, obj->ldm);
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_solve_i(const s_LuoN3prQfsei9XMpifhsFF_NMPC_T
  *obj, real_T rhs_data[])
{
  int32_T b_i;
  int32_T b_j;
  int32_T jjA;
  int32_T n_tmp;
  n_tmp = obj->ndims;
  if (obj->ndims != 0) {
    for (b_j = 0; b_j < n_tmp; b_j++) {
      int32_T c;
      jjA = b_j * obj->ldm + b_j;
      c = (n_tmp - b_j) - 2;
      for (b_i = 0; b_i <= c; b_i++) {
        int32_T ix;
        ix = (b_i + b_j) + 1;
        rhs_data[ix] -= obj->FMat.data[(b_i + jjA) + 1] * rhs_data[b_j];
      }
    }
  }

  for (b_j = 0; b_j < n_tmp; b_j++) {
    rhs_data[b_j] /= obj->FMat.data[obj->ldm * b_j + b_j];
  }

  if (obj->ndims != 0) {
    for (b_j = n_tmp; b_j >= 1; b_j--) {
      real_T temp;
      jjA = (b_j - 1) * obj->ldm;
      temp = rhs_data[b_j - 1];
      for (b_i = n_tmp; b_i >= b_j + 1; b_i--) {
        temp -= obj->FMat.data[(jjA + b_i) - 1] * rhs_data[b_i - 1];
      }

      rhs_data[b_j - 1] = temp;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_solve(const s_LuoN3prQfsei9XMpifhsFF_NMPC_T *obj,
  real_T rhs_data[])
{
  int32_T i;
  int32_T j;
  int32_T n_tmp;
  n_tmp = obj->ndims;
  if (obj->ndims != 0) {
    int32_T jA;
    for (j = 0; j < n_tmp; j++) {
      real_T temp;
      jA = j * obj->ldm;
      temp = rhs_data[j];
      for (i = 0; i < j; i++) {
        temp -= obj->FMat.data[jA + i] * rhs_data[i];
      }

      rhs_data[j] = temp / obj->FMat.data[jA + j];
    }

    for (j = n_tmp; j >= 1; j--) {
      jA = ((j - 1) * obj->ldm + j) - 2;
      rhs_data[j - 1] /= obj->FMat.data[jA + 1];
      for (i = 0; i <= j - 2; i++) {
        int32_T ix;
        ix = (j - i) - 2;
        rhs_data[ix] -= obj->FMat.data[jA - i] * rhs_data[j - 1];
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracki_compute_deltax(const real_T H[160801],
  s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution, s_BcTDS8pFolHdhtDibl2TnF_NMPC_T
  *memspace, const s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *cholmanager, const
  s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *objective, boolean_T alwaysPositiveDef)
{
  __m128d tmp;
  real_T s;
  real_T smax;
  int32_T A_maxDiag_idx;
  int32_T A_maxDiag_idx_tmp;
  int32_T LDimSizeP1;
  int32_T b_idx;
  int32_T c_ix;
  int32_T exitg2;
  int32_T k;
  int32_T mNull_tmp;
  int32_T nVar;
  int32_T nVars;
  int32_T order;
  boolean_T exitg1;
  nVar = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    for (mNull_tmp = 0; mNull_tmp <= nVar; mNull_tmp++) {
      solution->searchDir.data[mNull_tmp] = 0.0;
    }
  } else {
    A_maxDiag_idx = (qrmanager->mrows / 2) << 1;
    order = A_maxDiag_idx - 2;
    for (b_idx = 0; b_idx <= order; b_idx += 2) {
      tmp = _mm_loadu_pd(&objective->grad.data[b_idx]);
      _mm_storeu_pd(&solution->searchDir.data[b_idx], _mm_mul_pd(tmp,
        _mm_set1_pd(-1.0)));
    }

    for (b_idx = A_maxDiag_idx; b_idx <= nVar; b_idx++) {
      solution->searchDir.data[b_idx] = -objective->grad.data[b_idx];
    }

    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
       case 5:
        break;

       case 3:
        if (alwaysPositiveDef) {
          NMPC_Path_Tracking_factor(cholmanager, H, qrmanager->mrows,
            qrmanager->mrows);
        } else {
          NMPC_Path_Tracking_factor_l(cholmanager, H, qrmanager->mrows,
            qrmanager->mrows);
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else if (alwaysPositiveDef) {
          NMPC_Path_Tracking_solve(cholmanager, solution->searchDir.data);
        } else {
          NMPC_Path_Tracking_solve_i(cholmanager, solution->searchDir.data);
        }
        break;

       case 4:
        if (alwaysPositiveDef) {
          NMPC_Path_Tracking_factor(cholmanager, H, objective->nvar,
            objective->nvar);
          if (cholmanager->info != 0) {
            solution->state = -6;
          } else {
            NMPC_Path_Tracking_solve(cholmanager, solution->searchDir.data);
            smax = 1.0 / objective->beta;
            b_idx = objective->nvar + 1;
            nVar = qrmanager->mrows;
            A_maxDiag_idx = ((((qrmanager->mrows - objective->nvar) / 2) << 1) +
                             objective->nvar) + 1;
            order = A_maxDiag_idx - 2;
            for (mNull_tmp = b_idx; mNull_tmp <= order; mNull_tmp += 2) {
              tmp = _mm_loadu_pd(&solution->searchDir.data[mNull_tmp - 1]);
              _mm_storeu_pd(&solution->searchDir.data[mNull_tmp - 1], _mm_mul_pd
                            (tmp, _mm_set1_pd(smax)));
            }

            for (mNull_tmp = A_maxDiag_idx; mNull_tmp <= nVar; mNull_tmp++) {
              solution->searchDir.data[mNull_tmp - 1] *= smax;
            }
          }
        }
        break;
      }
    } else {
      b_idx = qrmanager->ldq * qrmanager->ncols + 1;
      if (objective->objtype == 5) {
        for (k = 0; k < mNull_tmp; k++) {
          memspace->workspace_float.data[k] = -qrmanager->Q.data
            [(qrmanager->ncols + k) * qrmanager->ldq + nVar];
        }

        NMPC_Path_Tracking_xgemv_cpgex(qrmanager->mrows, mNull_tmp,
          qrmanager->Q.data, b_idx, qrmanager->ldq,
          memspace->workspace_float.data, solution->searchDir.data);
      } else {
        if (objective->objtype == 3) {
          NMPC_Path_Tracking_xgemm(qrmanager->mrows, mNull_tmp, qrmanager->mrows,
            H, qrmanager->mrows, qrmanager->Q.data, b_idx, qrmanager->ldq,
            memspace->workspace_float.data, memspace->workspace_float.size[0]);
          NMPC_Path_Tracking_xgemm_o(mNull_tmp, mNull_tmp, qrmanager->mrows,
            qrmanager->Q.data, b_idx, qrmanager->ldq,
            memspace->workspace_float.data, memspace->workspace_float.size[0],
            cholmanager->FMat.data, cholmanager->ldm);
        } else if (alwaysPositiveDef) {
          nVars = qrmanager->mrows;
          NMPC_Path_Tracking_xgemm(objective->nvar, mNull_tmp, objective->nvar,
            H, objective->nvar, qrmanager->Q.data, b_idx, qrmanager->ldq,
            memspace->workspace_float.data, memspace->workspace_float.size[0]);
          for (k = 0; k < mNull_tmp; k++) {
            c_ix = objective->nvar + 1;
            A_maxDiag_idx = ((((nVars - objective->nvar) / 2) << 1) +
                             objective->nvar) + 1;
            order = A_maxDiag_idx - 2;
            for (LDimSizeP1 = c_ix; LDimSizeP1 <= order; LDimSizeP1 += 2) {
              tmp = _mm_loadu_pd(&qrmanager->Q.data[((k + qrmanager->ncols) *
                qrmanager->Q.size[0] + LDimSizeP1) - 1]);
              _mm_storeu_pd(&memspace->workspace_float.data[(LDimSizeP1 +
                memspace->workspace_float.size[0] * k) - 1], _mm_mul_pd(tmp,
                _mm_set1_pd(objective->beta)));
            }

            for (LDimSizeP1 = A_maxDiag_idx; LDimSizeP1 <= nVars; LDimSizeP1++)
            {
              memspace->workspace_float.data[(LDimSizeP1 +
                memspace->workspace_float.size[0] * k) - 1] = qrmanager->Q.data
                [((k + qrmanager->ncols) * qrmanager->Q.size[0] + LDimSizeP1) -
                1] * objective->beta;
            }
          }

          NMPC_Path_Tracking_xgemm_o(mNull_tmp, mNull_tmp, qrmanager->mrows,
            qrmanager->Q.data, b_idx, qrmanager->ldq,
            memspace->workspace_float.data, memspace->workspace_float.size[0],
            cholmanager->FMat.data, cholmanager->ldm);
        }

        if (alwaysPositiveDef) {
          cholmanager->ndims = mNull_tmp;
          cholmanager->info = NMPC_Path_Tracking_xpotrf(mNull_tmp,
            cholmanager->FMat.data, cholmanager->ldm);
        } else {
          LDimSizeP1 = cholmanager->ldm + 1;
          cholmanager->ndims = mNull_tmp;
          A_maxDiag_idx = 0;
          if (mNull_tmp > 1) {
            smax = fabs(cholmanager->FMat.data[0]);
            for (k = 2; k <= mNull_tmp; k++) {
              s = fabs(cholmanager->FMat.data[(k - 1) * LDimSizeP1]);
              if (s > smax) {
                A_maxDiag_idx = k - 1;
                smax = s;
              }
            }
          }

          cholmanager->regTol_ = fmax(fabs(cholmanager->FMat.data
            [cholmanager->ldm * A_maxDiag_idx + A_maxDiag_idx]) *
            2.2204460492503131E-16, 0.0);
          if (mNull_tmp > 128) {
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k < mNull_tmp)) {
              A_maxDiag_idx = LDimSizeP1 * k + 1;
              order = mNull_tmp - k;
              if (k + 48 <= mNull_tmp) {
                NMPC_Path_Track_partialColLDL3_(cholmanager, A_maxDiag_idx,
                  order);
                k += 48;
              } else {
                NMPC_Path_Tracking_fullColLDL2_(cholmanager, A_maxDiag_idx,
                  order);
                exitg1 = true;
              }
            }
          } else {
            NMPC_Path_Tracking_fullColLDL2_(cholmanager, 1, mNull_tmp);
          }

          if (cholmanager->ConvexCheck) {
            k = 0;
            do {
              exitg2 = 0;
              if (k <= mNull_tmp - 1) {
                if (cholmanager->FMat.data[cholmanager->ldm * k + k] <= 0.0) {
                  cholmanager->info = -k - 1;
                  exitg2 = 1;
                } else {
                  k++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          A_maxDiag_idx = qrmanager->ldq;
          if (qrmanager->mrows != 0) {
            for (k = 0; k < mNull_tmp; k++) {
              memspace->workspace_float.data[k] = 0.0;
            }

            order = 0;
            nVars = (mNull_tmp - 1) * qrmanager->ldq + b_idx;
            for (LDimSizeP1 = b_idx; A_maxDiag_idx < 0 ? LDimSizeP1 >= nVars :
                 LDimSizeP1 <= nVars; LDimSizeP1 += A_maxDiag_idx) {
              smax = 0.0;
              c_ix = LDimSizeP1 + nVar;
              for (k = LDimSizeP1; k <= c_ix; k++) {
                smax += qrmanager->Q.data[k - 1] * objective->grad.data[k -
                  LDimSizeP1];
              }

              memspace->workspace_float.data[order] -= smax;
              order++;
            }
          }

          if (alwaysPositiveDef) {
            c_ix = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (nVar = 0; nVar < c_ix; nVar++) {
                A_maxDiag_idx = nVar * cholmanager->ldm;
                smax = memspace->workspace_float.data[nVar];
                for (k = 0; k < nVar; k++) {
                  smax -= cholmanager->FMat.data[A_maxDiag_idx + k] *
                    memspace->workspace_float.data[k];
                }

                memspace->workspace_float.data[nVar] = smax /
                  cholmanager->FMat.data[A_maxDiag_idx + nVar];
              }
            }

            if (cholmanager->ndims != 0) {
              for (nVar = c_ix; nVar >= 1; nVar--) {
                order = ((nVar - 1) * cholmanager->ldm + nVar) - 2;
                memspace->workspace_float.data[nVar - 1] /=
                  cholmanager->FMat.data[order + 1];
                for (LDimSizeP1 = 0; LDimSizeP1 <= nVar - 2; LDimSizeP1++) {
                  nVars = (nVar - LDimSizeP1) - 2;
                  memspace->workspace_float.data[nVars] -=
                    memspace->workspace_float.data[nVar - 1] *
                    cholmanager->FMat.data[order - LDimSizeP1];
                }
              }
            }
          } else {
            A_maxDiag_idx_tmp = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (nVar = 0; nVar < A_maxDiag_idx_tmp; nVar++) {
                order = nVar * cholmanager->ldm + nVar;
                nVars = (A_maxDiag_idx_tmp - nVar) - 2;
                for (LDimSizeP1 = 0; LDimSizeP1 <= nVars; LDimSizeP1++) {
                  c_ix = (LDimSizeP1 + nVar) + 1;
                  memspace->workspace_float.data[c_ix] -= cholmanager->
                    FMat.data[(LDimSizeP1 + order) + 1] *
                    memspace->workspace_float.data[nVar];
                }
              }
            }

            for (nVar = 0; nVar < A_maxDiag_idx_tmp; nVar++) {
              memspace->workspace_float.data[nVar] /= cholmanager->
                FMat.data[cholmanager->ldm * nVar + nVar];
            }

            if (cholmanager->ndims != 0) {
              for (nVar = A_maxDiag_idx_tmp; nVar >= 1; nVar--) {
                A_maxDiag_idx = (nVar - 1) * cholmanager->ldm;
                smax = memspace->workspace_float.data[nVar - 1];
                for (k = A_maxDiag_idx_tmp; k >= nVar + 1; k--) {
                  smax -= cholmanager->FMat.data[(A_maxDiag_idx + k) - 1] *
                    memspace->workspace_float.data[k - 1];
                }

                memspace->workspace_float.data[nVar - 1] = smax;
              }
            }
          }

          NMPC_Path_Tracking_xgemv_cpgex(qrmanager->mrows, mNull_tmp,
            qrmanager->Q.data, b_idx, qrmanager->ldq,
            memspace->workspace_float.data, solution->searchDir.data);
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T NMPC_Path_Tracking_xnrm2_o(int32_T n, const real_T x_data[])
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x_data[0]);
    } else {
      real_T scale;
      scale = 3.3121686421112381E-170;
      for (k = 0; k < n; k++) {
        real_T absxk;
        absxk = fabs(x_data[k]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_xgemv_cpgexe(int32_T m, int32_T n, const real_T
  A_data[], int32_T lda, const real_T x_data[], real_T y_data[])
{
  int32_T b_iy;
  int32_T y_tmp;
  if (n != 0) {
    int32_T b;
    int32_T scalarLB;
    int32_T vectorUB;
    b = (uint16_T)n;
    scalarLB = ((uint16_T)n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (b_iy = 0; b_iy <= vectorUB; b_iy += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&y_data[b_iy]);
      _mm_storeu_pd(&y_data[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
    }

    for (b_iy = scalarLB; b_iy < b; b_iy++) {
      y_data[b_iy] = -y_data[b_iy];
    }

    b = 0;
    scalarLB = (n - 1) * lda + 1;
    for (b_iy = 1; lda < 0 ? b_iy >= scalarLB : b_iy <= scalarLB; b_iy += lda) {
      real_T c;
      c = 0.0;
      vectorUB = (b_iy + m) - 1;
      for (y_tmp = b_iy; y_tmp <= vectorUB; y_tmp++) {
        c += A_data[y_tmp - 1] * x_data[y_tmp - b_iy];
      }

      y_data[b] += c;
      b++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tra_feasibleratiotest(const real_T solution_xstar_data[],
  const real_T solution_searchDir_data[], real_T workspace_data[], const int32_T
  workspace_size[2], int32_T workingset_nVar, int32_T workingset_ldA, const
  real_T workingset_Aineq_data[], const real_T workingset_bineq_data[], const
  real_T workingset_lb_data[], const int32_T workingset_indexLB_data[], const
  int32_T workingset_sizes[5], const int32_T workingset_isActiveIdx[6], const
  boolean_T workingset_isActiveConstr_data[], const int32_T workingset_nWConstr
  [5], boolean_T isPhaseOne, real_T *alpha, boolean_T *newBlocking, int32_T
  *constrType, int32_T *constrIdx)
{
  real_T tmp[2];
  real_T alphaTemp;
  real_T b_c;
  real_T denomTol;
  real_T phaseOneCorrectionP;
  real_T pk_corrected;
  int32_T b_iy;
  int32_T d_tmp;
  int32_T e;
  int32_T f;
  int32_T ia;
  int32_T iyend;
  int32_T k;
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 * NMPC_Path_Tracking_xnrm2_o(workingset_nVar,
    solution_searchDir_data);
  if (workingset_nWConstr[2] < workingset_sizes[2]) {
    d_tmp = (uint16_T)workingset_sizes[2];
    for (k = 0; k < d_tmp; k++) {
      workspace_data[k] = workingset_bineq_data[k];
    }

    NMPC_Path_Tracking_xgemv_cpgexe(workingset_nVar, workingset_sizes[2],
      workingset_Aineq_data, workingset_ldA, solution_xstar_data, workspace_data);
    k = workspace_size[0];
    if (workingset_sizes[2] != 0) {
      iyend = workspace_size[0] + workingset_sizes[2];
      for (b_iy = k + 1; b_iy <= iyend; b_iy++) {
        workspace_data[b_iy - 1] = 0.0;
      }

      iyend = workspace_size[0];
      e = (workingset_sizes[2] - 1) * workingset_ldA + 1;
      for (b_iy = 1; workingset_ldA < 0 ? b_iy >= e : b_iy <= e; b_iy +=
           workingset_ldA) {
        b_c = 0.0;
        f = (b_iy + workingset_nVar) - 1;
        for (ia = b_iy; ia <= f; ia++) {
          b_c += workingset_Aineq_data[ia - 1] * solution_searchDir_data[ia -
            b_iy];
        }

        workspace_data[iyend] += b_c;
        iyend++;
      }
    }

    for (b_iy = 0; b_iy < d_tmp; b_iy++) {
      b_c = workspace_data[k + b_iy];
      if ((b_c > denomTol) && (!workingset_isActiveConstr_data
           [(workingset_isActiveIdx[2] + b_iy) - 1])) {
        alphaTemp = fmin(fabs(workspace_data[b_iy]), 1.0E-6 -
                         workspace_data[b_iy]) / b_c;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = b_iy + 1;
          *newBlocking = true;
        }
      }
    }
  }

  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_set_pd
      (solution_searchDir_data[workingset_nVar - 1],
       solution_xstar_data[workingset_nVar - 1]), _mm_set1_pd(isPhaseOne)));
    b_c = tmp[0];
    phaseOneCorrectionP = tmp[1];
    k = workingset_sizes[3];
    for (d_tmp = 0; d_tmp <= k - 2; d_tmp++) {
      b_iy = workingset_indexLB_data[d_tmp];
      pk_corrected = -solution_searchDir_data[b_iy - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr_data
           [(workingset_isActiveIdx[3] + d_tmp) - 1])) {
        alphaTemp = (-solution_xstar_data[b_iy - 1] - workingset_lb_data[b_iy -
                     1]) - b_c;
        alphaTemp = fmin(fabs(alphaTemp), 1.0E-6 - alphaTemp) / pk_corrected;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = d_tmp + 1;
          *newBlocking = true;
        }
      }
    }

    iyend = workingset_indexLB_data[workingset_sizes[3] - 1] - 1;
    b_c = -solution_searchDir_data[iyend];
    if ((b_c > denomTol) && (!workingset_isActiveConstr_data
         [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      alphaTemp = -solution_xstar_data[iyend] - workingset_lb_data[iyend];
      alphaTemp = fmin(fabs(alphaTemp), 1.0E-6 - alphaTemp) / b_c;
      if (alphaTemp < *alpha) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }

  if (!isPhaseOne) {
    *newBlocking = (((!*newBlocking) || (!(*alpha > 1.0))) && (*newBlocking));
    *alpha = fmin(*alpha, 1.0);
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_P_checkUnboundedOrIllPosed(s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T
  *solution, const s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *objective)
{
  if (objective->objtype == 5) {
    if (NMPC_Path_Tracking_xnrm2_o(objective->nvar, solution->searchDir.data) >
        100.0 * (real_T)objective->nvar * 1.4901161193847656E-8) {
      solution->state = 3;
    } else {
      solution->state = 4;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_addBoundToActiveSetMatrix_(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj,
  int32_T TYPE, int32_T idx_local)
{
  int32_T b;
  int32_T colOffset;
  int32_T idx;
  int32_T idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr.data[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid.data[obj->nActiveConstr - 1] = TYPE;
  obj->Wlocalidx.data[obj->nActiveConstr - 1] = idx_local;
  colOffset = (obj->nActiveConstr - 1) * obj->ldA - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj->indexUB.data[idx_local - 1];
    obj->bwset.data[obj->nActiveConstr - 1] = obj->ub.data[idx_bnd_local - 1];
  } else {
    idx_bnd_local = obj->indexLB.data[idx_local - 1];
    obj->bwset.data[obj->nActiveConstr - 1] = obj->lb.data[idx_bnd_local - 1];
  }

  b = (uint16_T)(idx_bnd_local - 1);
  for (idx = 0; idx < b; idx++) {
    obj->ATwset.data[(idx + colOffset) + 1] = 0.0;
  }

  obj->ATwset.data[idx_bnd_local + colOffset] = (real_T)(TYPE == 5) * 2.0 - 1.0;
  b = obj->nVar;
  for (idx = idx_bnd_local + 1; idx <= b; idx++) {
    obj->ATwset.data[idx + colOffset] = 0.0;
  }

  switch (obj->probType) {
   case 3:
   case 2:
    break;

   default:
    obj->ATwset.data[obj->nVar + colOffset] = -1.0;
    break;
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracki_addAineqConstr(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj,
  int32_T idx_local)
{
  int32_T b;
  int32_T iAineq0;
  int32_T iAw0;
  int32_T idx;
  obj->nWConstr[2]++;
  obj->isActiveConstr.data[(obj->isActiveIdx[2] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid.data[obj->nActiveConstr - 1] = 3;
  obj->Wlocalidx.data[obj->nActiveConstr - 1] = idx_local;
  iAineq0 = (idx_local - 1) * obj->ldA;
  iAw0 = (obj->nActiveConstr - 1) * obj->ldA;
  b = obj->nVar;
  for (idx = 0; idx < b; idx++) {
    obj->ATwset.data[iAw0 + idx] = obj->Aineq.data[iAineq0 + idx];
  }

  obj->bwset.data[obj->nActiveConstr - 1] = obj->bineq.data[idx_local - 1];
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracki_compute_lambda(real_T workspace_data[],
  s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution, const
  s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *objective, const
  s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager)
{
  int32_T b_idx;
  int32_T ia;
  int32_T idxQR;
  int32_T nActiveConstr_tmp_tmp;
  nActiveConstr_tmp_tmp = qrmanager->ncols;
  if (qrmanager->ncols > 0) {
    real_T c;
    int32_T b_ix;
    boolean_T guard1;
    guard1 = false;
    if (objective->objtype != 4) {
      boolean_T nonDegenerate;
      if (qrmanager->mrows >= qrmanager->ncols) {
        b_ix = qrmanager->mrows;
      } else {
        b_ix = qrmanager->ncols;
      }

      c = fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * (real_T)b_ix);
      nonDegenerate = ((qrmanager->mrows > 0) && (qrmanager->ncols > 0));
      if (nonDegenerate) {
        boolean_T guard2;
        b_idx = qrmanager->ncols;
        guard2 = false;
        if (qrmanager->mrows < qrmanager->ncols) {
          idxQR = (qrmanager->ncols - 1) * qrmanager->ldq + qrmanager->mrows;
          while ((b_idx > qrmanager->mrows) && (fabs(qrmanager->QR.data[idxQR -
                   1]) >= c)) {
            b_idx--;
            idxQR -= qrmanager->ldq;
          }

          nonDegenerate = (b_idx == qrmanager->mrows);
          if (!nonDegenerate) {
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          idxQR = (b_idx - 1) * qrmanager->ldq + b_idx;
          while ((b_idx >= 1) && (fabs(qrmanager->QR.data[idxQR - 1]) >= c)) {
            b_idx--;
            idxQR = (idxQR - qrmanager->ldq) - 1;
          }

          nonDegenerate = (b_idx == 0);
        }
      }

      if (!nonDegenerate) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      int32_T jjA;
      b_idx = qrmanager->ldq;
      if (qrmanager->mrows != 0) {
        for (idxQR = 0; idxQR < nActiveConstr_tmp_tmp; idxQR++) {
          workspace_data[idxQR] = 0.0;
        }

        b_ix = 0;
        jjA = (qrmanager->ncols - 1) * qrmanager->ldq + 1;
        for (idxQR = 1; b_idx < 0 ? idxQR >= jjA : idxQR <= jjA; idxQR += b_idx)
        {
          int32_T d;
          c = 0.0;
          d = (idxQR + qrmanager->mrows) - 1;
          for (ia = idxQR; ia <= d; ia++) {
            c += qrmanager->Q.data[ia - 1] * objective->grad.data[ia - idxQR];
          }

          workspace_data[b_ix] += c;
          b_ix++;
        }
      }

      if (qrmanager->ncols != 0) {
        for (idxQR = nActiveConstr_tmp_tmp; idxQR >= 1; idxQR--) {
          jjA = ((idxQR - 1) * b_idx + idxQR) - 2;
          workspace_data[idxQR - 1] /= qrmanager->QR.data[jjA + 1];
          for (ia = 0; ia <= idxQR - 2; ia++) {
            b_ix = (idxQR - ia) - 2;
            workspace_data[b_ix] -= workspace_data[idxQR - 1] *
              qrmanager->QR.data[jjA - ia];
          }
        }
      }

      idxQR = (qrmanager->ncols / 2) << 1;
      ia = idxQR - 2;
      for (b_idx = 0; b_idx <= ia; b_idx += 2) {
        __m128d tmp;
        tmp = _mm_loadu_pd(&workspace_data[b_idx]);
        _mm_storeu_pd(&solution->lambda.data[b_idx], _mm_mul_pd(tmp, _mm_set1_pd
          (-1.0)));
      }

      for (b_idx = idxQR; b_idx < nActiveConstr_tmp_tmp; b_idx++) {
        solution->lambda.data[b_idx] = -workspace_data[b_idx];
      }
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NM_checkStoppingAndUpdateFval_l(int32_T *activeSetChangeID,
  s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution, s_BcTDS8pFolHdhtDibl2TnF_NMPC_T
  *memspace, const s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *objective,
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T
  *qrmanager, int32_T runTimeOptions_MaxIterations, boolean_T *updateFval)
{
  real_T tempMaxConstr;
  int32_T k;
  int32_T nVar;
  boolean_T nonDegenerateWset;
  solution->iterations++;
  nVar = objective->nvar;
  if ((solution->iterations >= runTimeOptions_MaxIterations) &&
      ((solution->state != 1) || (objective->objtype == 5))) {
    solution->state = 0;
  }

  if (solution->iterations - solution->iterations / 50 * 50 == 0) {
    tempMaxConstr = NMPC_maxConstraintViolation_lng(workingset,
      solution->xstar.data);
    solution->maxConstr = tempMaxConstr;
    if (objective->objtype == 5) {
      tempMaxConstr = solution->maxConstr - solution->xstar.data[objective->nvar
        - 1];
    }

    if (tempMaxConstr > 1.0E-6) {
      for (k = 0; k < nVar; k++) {
        solution->searchDir.data[k] = solution->xstar.data[k];
      }

      nonDegenerateWset = NMPC_Pa_feasibleX0ForWorkingSet
        (memspace->workspace_float.data, memspace->workspace_float.size,
         solution->searchDir.data, workingset, qrmanager);
      if ((!nonDegenerateWset) && (solution->state != 0)) {
        solution->state = -2;
      }

      *activeSetChangeID = 0;
      tempMaxConstr = NMPC_maxConstraintViolation_lng(workingset,
        solution->searchDir.data);
      if (tempMaxConstr < solution->maxConstr) {
        for (k = 0; k < nVar; k++) {
          solution->xstar.data[k] = solution->searchDir.data[k];
        }

        solution->maxConstr = tempMaxConstr;
      }
    }
  }

  if (*updateFval) {
    *updateFval = false;
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_iterate_d(const real_T H[160801], const real_T
  f_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *cholmanager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  objective, const char_T options_SolverName[7], int32_T
  runTimeOptions_MaxIterations)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T normDelta;
  real_T solution_lambda;
  int32_T activeSetChangeID;
  int32_T b_n;
  int32_T exitg1;
  int32_T g;
  int32_T globalActiveConstrIdx;
  int32_T iQR0;
  int32_T idxMinLambda;
  int32_T k;
  int32_T nVar;
  int32_T tmp_1;
  boolean_T guard1;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  NMPC_Path_T_computeGrad_StoreHx(objective, H, f_data, solution->xstar.data);
  solution->fstar = NMPC_Path_T_computeFval_ReuseHx(objective,
    memspace->workspace_float.data, f_data, solution->xstar.data);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }

  idxMinLambda = workingset->mConstrMax;
  for (k = 0; k < idxMinLambda; k++) {
    solution->lambda.data[k] = 0.0;
  }

  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          NMPC_Path_Tra_squareQ_appendCol(qrmanager, workingset->ATwset.data,
            workingset->ldA * (workingset->nActiveConstr - 1) + 1);
          break;

         case -1:
          NMPC_Path_Trac_deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          NMPC_Path_Tracking_factorQR(qrmanager, workingset->ATwset.data, nVar,
            workingset->nActiveConstr, workingset->ldA);
          g = qrmanager->minRowCol;
          for (k = 0; k < g; k++) {
            iQR0 = qrmanager->ldq * k + k;
            b_n = qrmanager->mrows - k;
            for (idxMinLambda = 0; idxMinLambda <= b_n - 2; idxMinLambda++) {
              tmp_1 = idxMinLambda + iQR0;
              qrmanager->Q.data[tmp_1 + 1] = qrmanager->QR.data[tmp_1 + 1];
            }
          }

          NMPC_Path_Tracking_xorgqr(qrmanager->mrows, qrmanager->mrows,
            qrmanager->minRowCol, qrmanager->Q.data, qrmanager->Q.size,
            qrmanager->ldq, qrmanager->tau.data);
          break;
        }

        NMPC_Path_Tracki_compute_deltax(H, solution, memspace, qrmanager,
          cholmanager, objective, NMPC_Path_Tracking_strcmp(options_SolverName));
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          normDelta = NMPC_Path_Tracking_xnrm2_o(nVar, solution->searchDir.data);
          guard1 = true;
        }
      } else {
        for (k = 0; k < nVar; k++) {
          solution->searchDir.data[k] = 0.0;
        }

        normDelta = 0.0;
        guard1 = true;
      }

      if (guard1) {
        if ((!subProblemChanged) || (normDelta < 1.0E-6) ||
            (workingset->nActiveConstr >= nVar)) {
          NMPC_Path_Tracki_compute_lambda(memspace->workspace_float.data,
            solution, objective, qrmanager);
          if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
            idxMinLambda = 0;
            normDelta = 0.0;
            g = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
            iQR0 = workingset->nActiveConstr;
            for (k = g; k <= iQR0; k++) {
              solution_lambda = solution->lambda.data[k - 1];
              if (solution_lambda < normDelta) {
                normDelta = solution_lambda;
                idxMinLambda = k;
              }
            }

            if (idxMinLambda == 0) {
              solution->state = 1;
            } else {
              activeSetChangeID = -1;
              globalActiveConstrIdx = idxMinLambda;
              subProblemChanged = true;
              NMPC_Path_Tracking_removeConstr(workingset, idxMinLambda);
              if (idxMinLambda < workingset->nActiveConstr + 1) {
                solution->lambda.data[idxMinLambda - 1] = solution->
                  lambda.data[workingset->nActiveConstr];
              }

              solution->lambda.data[workingset->nActiveConstr] = 0.0;
            }
          } else {
            idxMinLambda = workingset->nActiveConstr;
            activeSetChangeID = 0;
            globalActiveConstrIdx = workingset->nActiveConstr;
            subProblemChanged = true;
            NMPC_Path_Tracking_removeConstr(workingset,
              workingset->nActiveConstr);
            solution->lambda.data[idxMinLambda - 1] = 0.0;
          }

          updateFval = false;
        } else {
          NMPC_Path_Tra_feasibleratiotest(solution->xstar.data,
            solution->searchDir.data, memspace->workspace_float.data,
            memspace->workspace_float.size, workingset->nVar, workingset->ldA,
            workingset->Aineq.data, workingset->bineq.data, workingset->lb.data,
            workingset->indexLB.data, workingset->sizes, workingset->isActiveIdx,
            workingset->isActiveConstr.data, workingset->nWConstr,
            (objective->objtype == 5), &normDelta, &updateFval, &k,
            &idxMinLambda);
          if (updateFval) {
            switch (k) {
             case 3:
              NMPC_Path_Tracki_addAineqConstr(workingset, idxMinLambda);
              break;

             case 4:
              NMPC_addBoundToActiveSetMatrix_(workingset, 4, idxMinLambda);
              break;

             default:
              NMPC_addBoundToActiveSetMatrix_(workingset, 5, idxMinLambda);
              break;
            }

            activeSetChangeID = 1;
          } else {
            NMPC_P_checkUnboundedOrIllPosed(solution, objective);
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if ((nVar >= 1) && (!(normDelta == 0.0))) {
            idxMinLambda = (nVar / 2) << 1;
            g = idxMinLambda - 2;
            for (k = 0; k <= g; k += 2) {
              tmp = _mm_loadu_pd(&solution->searchDir.data[k]);
              tmp_0 = _mm_loadu_pd(&solution->xstar.data[k]);
              _mm_storeu_pd(&solution->xstar.data[k], _mm_add_pd(_mm_mul_pd
                (_mm_set1_pd(normDelta), tmp), tmp_0));
            }

            for (k = idxMinLambda; k < nVar; k++) {
              solution->xstar.data[k] += normDelta * solution->searchDir.data[k];
            }
          }

          NMPC_Path_T_computeGrad_StoreHx(objective, H, f_data,
            solution->xstar.data);
          updateFval = true;
        }

        NM_checkStoppingAndUpdateFval_l(&activeSetChangeID, solution, memspace,
          objective, workingset, qrmanager, runTimeOptions_MaxIterations,
          &updateFval);
      }
    } else {
      if (!updateFval) {
        solution->fstar = NMPC_Path_T_computeFval_ReuseHx(objective,
          memspace->workspace_float.data, f_data, solution->xstar.data);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_checkStoppingAndUpdateFval(int32_T *activeSetChangeID, const
  real_T f_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, const
  s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *objective, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager, int32_T
  runTimeOptions_MaxIterations, const boolean_T *updateFval)
{
  real_T tempMaxConstr;
  int32_T k;
  int32_T nVar;
  boolean_T nonDegenerateWset;
  solution->iterations++;
  nVar = objective->nvar;
  if ((solution->iterations >= runTimeOptions_MaxIterations) &&
      ((solution->state != 1) || (objective->objtype == 5))) {
    solution->state = 0;
  }

  if (solution->iterations - solution->iterations / 50 * 50 == 0) {
    solution->maxConstr = NMPC_maxConstraintViolation_lng(workingset,
      solution->xstar.data);
    tempMaxConstr = solution->maxConstr;
    if (objective->objtype == 5) {
      tempMaxConstr = solution->maxConstr - solution->xstar.data[objective->nvar
        - 1];
    }

    if (tempMaxConstr > 1.0E-6) {
      for (k = 0; k < nVar; k++) {
        solution->searchDir.data[k] = solution->xstar.data[k];
      }

      nonDegenerateWset = NMPC_Pa_feasibleX0ForWorkingSet
        (memspace->workspace_float.data, memspace->workspace_float.size,
         solution->searchDir.data, workingset, qrmanager);
      if ((!nonDegenerateWset) && (solution->state != 0)) {
        solution->state = -2;
      }

      *activeSetChangeID = 0;
      tempMaxConstr = NMPC_maxConstraintViolation_lng(workingset,
        solution->searchDir.data);
      if (tempMaxConstr < solution->maxConstr) {
        for (k = 0; k < nVar; k++) {
          solution->xstar.data[k] = solution->searchDir.data[k];
        }

        solution->maxConstr = tempMaxConstr;
      }
    }
  }

  if (*updateFval) {
    solution->fstar = NMPC_Path_T_computeFval_ReuseHx(objective,
      memspace->workspace_float.data, f_data, solution->xstar.data);
    if ((solution->fstar < 1.0E-6) && ((solution->state != 0) ||
         (objective->objtype != 5))) {
      solution->state = 2;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_iterate(const real_T H[160801], const real_T
  f_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *cholmanager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  objective, const char_T options_SolverName[7], int32_T
  runTimeOptions_MaxIterations)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T normDelta;
  real_T solution_lambda;
  int32_T activeSetChangeID;
  int32_T b_n;
  int32_T exitg1;
  int32_T g;
  int32_T globalActiveConstrIdx;
  int32_T iQR0;
  int32_T idxMinLambda;
  int32_T k;
  int32_T nVar;
  int32_T tmp_1;
  boolean_T guard1;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  NMPC_Path_T_computeGrad_StoreHx(objective, H, f_data, solution->xstar.data);
  solution->fstar = NMPC_Path_T_computeFval_ReuseHx(objective,
    memspace->workspace_float.data, f_data, solution->xstar.data);
  solution->state = -5;
  idxMinLambda = workingset->mConstrMax;
  for (k = 0; k < idxMinLambda; k++) {
    solution->lambda.data[k] = 0.0;
  }

  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          NMPC_Path_Tra_squareQ_appendCol(qrmanager, workingset->ATwset.data,
            workingset->ldA * (workingset->nActiveConstr - 1) + 1);
          break;

         case -1:
          NMPC_Path_Trac_deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          NMPC_Path_Tracking_factorQR(qrmanager, workingset->ATwset.data, nVar,
            workingset->nActiveConstr, workingset->ldA);
          g = qrmanager->minRowCol;
          for (k = 0; k < g; k++) {
            iQR0 = qrmanager->ldq * k + k;
            b_n = qrmanager->mrows - k;
            for (idxMinLambda = 0; idxMinLambda <= b_n - 2; idxMinLambda++) {
              tmp_1 = idxMinLambda + iQR0;
              qrmanager->Q.data[tmp_1 + 1] = qrmanager->QR.data[tmp_1 + 1];
            }
          }

          NMPC_Path_Tracking_xorgqr(qrmanager->mrows, qrmanager->mrows,
            qrmanager->minRowCol, qrmanager->Q.data, qrmanager->Q.size,
            qrmanager->ldq, qrmanager->tau.data);
          break;
        }

        NMPC_Path_Tracki_compute_deltax(H, solution, memspace, qrmanager,
          cholmanager, objective, NMPC_Path_Tracking_strcmp(options_SolverName));
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          normDelta = NMPC_Path_Tracking_xnrm2_o(nVar, solution->searchDir.data);
          guard1 = true;
        }
      } else {
        for (k = 0; k < nVar; k++) {
          solution->searchDir.data[k] = 0.0;
        }

        normDelta = 0.0;
        guard1 = true;
      }

      if (guard1) {
        if ((!subProblemChanged) || (normDelta < 1.4901161193847657E-10) ||
            (workingset->nActiveConstr >= nVar)) {
          NMPC_Path_Tracki_compute_lambda(memspace->workspace_float.data,
            solution, objective, qrmanager);
          if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
            idxMinLambda = 0;
            normDelta = 0.0;
            g = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
            iQR0 = workingset->nActiveConstr;
            for (k = g; k <= iQR0; k++) {
              solution_lambda = solution->lambda.data[k - 1];
              if (solution_lambda < normDelta) {
                normDelta = solution_lambda;
                idxMinLambda = k;
              }
            }

            if (idxMinLambda == 0) {
              solution->state = 1;
            } else {
              activeSetChangeID = -1;
              globalActiveConstrIdx = idxMinLambda;
              subProblemChanged = true;
              NMPC_Path_Tracking_removeConstr(workingset, idxMinLambda);
              if (idxMinLambda < workingset->nActiveConstr + 1) {
                solution->lambda.data[idxMinLambda - 1] = solution->
                  lambda.data[workingset->nActiveConstr];
              }

              solution->lambda.data[workingset->nActiveConstr] = 0.0;
            }
          } else {
            idxMinLambda = workingset->nActiveConstr;
            activeSetChangeID = 0;
            globalActiveConstrIdx = workingset->nActiveConstr;
            subProblemChanged = true;
            NMPC_Path_Tracking_removeConstr(workingset,
              workingset->nActiveConstr);
            solution->lambda.data[idxMinLambda - 1] = 0.0;
          }

          updateFval = false;
        } else {
          NMPC_Path_Tra_feasibleratiotest(solution->xstar.data,
            solution->searchDir.data, memspace->workspace_float.data,
            memspace->workspace_float.size, workingset->nVar, workingset->ldA,
            workingset->Aineq.data, workingset->bineq.data, workingset->lb.data,
            workingset->indexLB.data, workingset->sizes, workingset->isActiveIdx,
            workingset->isActiveConstr.data, workingset->nWConstr, true,
            &normDelta, &updateFval, &k, &idxMinLambda);
          if (updateFval) {
            switch (k) {
             case 3:
              NMPC_Path_Tracki_addAineqConstr(workingset, idxMinLambda);
              break;

             case 4:
              NMPC_addBoundToActiveSetMatrix_(workingset, 4, idxMinLambda);
              break;

             default:
              NMPC_addBoundToActiveSetMatrix_(workingset, 5, idxMinLambda);
              break;
            }

            activeSetChangeID = 1;
          } else {
            NMPC_P_checkUnboundedOrIllPosed(solution, objective);
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if ((nVar >= 1) && (!(normDelta == 0.0))) {
            idxMinLambda = (nVar / 2) << 1;
            g = idxMinLambda - 2;
            for (k = 0; k <= g; k += 2) {
              tmp = _mm_loadu_pd(&solution->searchDir.data[k]);
              tmp_0 = _mm_loadu_pd(&solution->xstar.data[k]);
              _mm_storeu_pd(&solution->xstar.data[k], _mm_add_pd(_mm_mul_pd
                (_mm_set1_pd(normDelta), tmp), tmp_0));
            }

            for (k = idxMinLambda; k < nVar; k++) {
              solution->xstar.data[k] += normDelta * solution->searchDir.data[k];
            }
          }

          NMPC_Path_T_computeGrad_StoreHx(objective, H, f_data,
            solution->xstar.data);
          updateFval = true;
        }

        NMPC_checkStoppingAndUpdateFval(&activeSetChangeID, f_data, solution,
          memspace, objective, workingset, qrmanager,
          runTimeOptions_MaxIterations, &updateFval);
      }
    } else {
      if (!updateFval) {
        solution->fstar = NMPC_Path_T_computeFval_ReuseHx(objective,
          memspace->workspace_float.data, f_data, solution->xstar.data);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_phaseone(const real_T H[160801], const real_T
  f_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *cholmanager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  objective, const char_T options_SolverName[7], const
  somzaGboVhDG7PNQS6E98jD_NMPC__T *runTimeOptions)
{
  int32_T PROBTYPE_ORIG;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T idxStartIneq_tmp;
  int32_T mConstr;
  int32_T nVar_tmp;
  boolean_T exitg1;
  PROBTYPE_ORIG = workingset->probType;
  nVar_tmp = workingset->nVar;
  solution->xstar.data[workingset->nVar] = solution->maxConstr + 1.0;
  if (workingset->probType == 3) {
    mConstr = 1;
  } else {
    mConstr = 4;
  }

  NMPC_Path_Tracki_setProblemType(workingset, mConstr);
  idxStartIneq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  idxStartIneq = idxStartIneq_tmp + 1;
  idxEndIneq = workingset->nActiveConstr;
  for (mConstr = idxStartIneq; mConstr <= idxEndIneq; mConstr++) {
    workingset->isActiveConstr.data[(workingset->isActiveIdx
      [workingset->Wid.data[mConstr - 1] - 1] + workingset->
      Wlocalidx.data[mConstr - 1]) - 2] = false;
  }

  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = idxStartIneq_tmp;
  objective->prev_objtype = objective->objtype;
  objective->prev_nvar = objective->nvar;
  objective->prev_hasLinear = objective->hasLinear;
  objective->objtype = 5;
  objective->nvar = nVar_tmp + 1;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  solution->fstar = solution->xstar.data[nVar_tmp];
  solution->state = 5;
  NMPC_Path_Tracking_iterate(H, f_data, solution, memspace, workingset,
    qrmanager, cholmanager, objective, options_SolverName,
    runTimeOptions->MaxIterations);
  if (workingset->isActiveConstr.data[(workingset->isActiveIdx[3] +
       workingset->sizes[3]) - 2]) {
    mConstr = workingset->sizes[0] + 301;
    exitg1 = false;
    while ((!exitg1) && (mConstr <= workingset->nActiveConstr)) {
      if ((workingset->Wid.data[mConstr - 1] == 4) &&
          (workingset->Wlocalidx.data[mConstr - 1] == workingset->sizes[3])) {
        NMPC_Path_Tracking_removeConstr(workingset, mConstr);
        exitg1 = true;
      } else {
        mConstr++;
      }
    }
  }

  mConstr = workingset->nActiveConstr;
  while ((mConstr > workingset->sizes[0] + 300) && (mConstr > nVar_tmp)) {
    NMPC_Path_Tracking_removeConstr(workingset, mConstr);
    mConstr--;
  }

  solution->maxConstr = solution->xstar.data[nVar_tmp];
  NMPC_Path_Tracki_setProblemType(workingset, PROBTYPE_ORIG);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_linearForm_(boolean_T obj_hasLinear, int32_T
  obj_nvar, real_T workspace_data[], const real_T H[160801], const real_T
  f_data[], const real_T x_data[])
{
  int32_T beta1;
  int32_T ia;
  beta1 = 0;
  if (obj_hasLinear) {
    for (beta1 = 0; beta1 < obj_nvar; beta1++) {
      workspace_data[beta1] = f_data[beta1];
    }

    beta1 = 1;
  }

  if (obj_nvar != 0) {
    int32_T d;
    int32_T ix;
    if (beta1 != 1) {
      for (beta1 = 0; beta1 < obj_nvar; beta1++) {
        workspace_data[beta1] = 0.0;
      }
    }

    ix = 0;
    d = (obj_nvar - 1) * obj_nvar + 1;
    for (beta1 = 1; obj_nvar < 0 ? beta1 >= d : beta1 <= d; beta1 += obj_nvar) {
      real_T c;
      int32_T e;
      c = 0.5 * x_data[ix];
      e = (beta1 + obj_nvar) - 1;
      for (ia = beta1; ia <= e; ia++) {
        int32_T tmp;
        tmp = ia - beta1;
        workspace_data[tmp] += H[ia - 1] * c;
      }

      ix++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_driver_n(const real_T H[160801], const real_T
  f_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *solution,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *workingset, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *qrmanager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *cholmanager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  objective, const somzaGboVhDG7PNQS6E98jD_NMPC__T *options, const
  somzaGboVhDG7PNQS6E98jD_NMPC__T *runTimeOptions)
{
  __m128d tmp;
  real_T maxConstr_new;
  int32_T c;
  int32_T ixlast;
  int32_T nVar;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T guard1;
  solution->iterations = 0;
  nVar = workingset->nVar;
  guard1 = false;
  if (workingset->probType == 3) {
    c = (uint16_T)workingset->sizes[0];
    for (ixlast = 0; ixlast < c; ixlast++) {
      solution->xstar.data[workingset->indexFixed.data[ixlast] - 1] =
        workingset->ub.data[workingset->indexFixed.data[ixlast] - 1];
    }

    c = (uint16_T)workingset->sizes[3];
    for (ixlast = 0; ixlast < c; ixlast++) {
      if (workingset->isActiveConstr.data[(workingset->isActiveIdx[3] + ixlast)
          - 1]) {
        solution->xstar.data[workingset->indexLB.data[ixlast] - 1] =
          -workingset->lb.data[workingset->indexLB.data[ixlast] - 1];
      }
    }

    c = (uint16_T)workingset->sizes[4];
    for (ixlast = 0; ixlast < c; ixlast++) {
      if (workingset->isActiveConstr.data[(workingset->isActiveIdx[4] + ixlast)
          - 1]) {
        solution->xstar.data[workingset->indexUB.data[ixlast] - 1] =
          workingset->ub.data[workingset->indexUB.data[ixlast] - 1];
      }
    }

    NMPC_Path_Tr_PresolveWorkingSet(solution, memspace, workingset, qrmanager);
    if (solution->state < 0) {
    } else {
      guard1 = true;
    }
  } else {
    solution->state = 82;
    guard1 = true;
  }

  if (guard1) {
    solution->iterations = 0;
    solution->maxConstr = NMPC_maxConstraintViolation_lng(workingset,
      solution->xstar.data);
    if (solution->maxConstr > 1.0E-6) {
      NMPC_Path_Tracking_phaseone(H, f_data, solution, memspace, workingset,
        qrmanager, cholmanager, objective, options->SolverName, runTimeOptions);
      if (solution->state != 0) {
        solution->maxConstr = NMPC_maxConstraintViolation_lng(workingset,
          solution->xstar.data);
        if (solution->maxConstr > 1.0E-6) {
          ixlast = workingset->mConstrMax;
          for (nVar = 0; nVar < ixlast; nVar++) {
            solution->lambda.data[nVar] = 0.0;
          }

          maxConstr_new = 0.0;
          switch (objective->objtype) {
           case 5:
            maxConstr_new = solution->xstar.data[objective->nvar - 1] *
              objective->gammaScalar;
            break;

           case 3:
            NMPC_Path_Tracking_linearForm_(objective->hasLinear, objective->nvar,
              memspace->workspace_float.data, H, f_data, solution->xstar.data);
            if (objective->nvar >= 1) {
              ixlast = objective->nvar;
              for (nVar = 0; nVar < ixlast; nVar++) {
                maxConstr_new += memspace->workspace_float.data[nVar] *
                  solution->xstar.data[nVar];
              }
            }
            break;

           case 4:
            NMPC_Path_Tracking_linearForm_(objective->hasLinear, objective->nvar,
              memspace->workspace_float.data, H, f_data, solution->xstar.data);
            ixlast = objective->nvar + 1;
            c = objective->maxVar;
            scalarLB = (((((objective->maxVar - objective->nvar) - 1) / 2) << 1)
                        + objective->nvar) + 1;
            vectorUB = scalarLB - 2;
            for (nVar = ixlast; nVar <= vectorUB; nVar += 2) {
              tmp = _mm_loadu_pd(&solution->xstar.data[nVar - 1]);
              _mm_storeu_pd(&memspace->workspace_float.data[nVar - 1],
                            _mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.5 *
                objective->beta), tmp), _mm_set1_pd(objective->rho)));
            }

            for (nVar = scalarLB; nVar < c; nVar++) {
              memspace->workspace_float.data[nVar - 1] = 0.5 * objective->beta *
                solution->xstar.data[nVar - 1] + objective->rho;
            }

            if (objective->maxVar - 1 >= 1) {
              for (nVar = 0; nVar <= c - 2; nVar++) {
                maxConstr_new += memspace->workspace_float.data[nVar] *
                  solution->xstar.data[nVar];
              }
            }
            break;
          }

          solution->fstar = maxConstr_new;
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            for (ixlast = 0; ixlast < nVar; ixlast++) {
              solution->searchDir.data[ixlast] = solution->xstar.data[ixlast];
            }

            NMPC_Path_Tr_PresolveWorkingSet(solution, memspace, workingset,
              qrmanager);
            maxConstr_new = NMPC_maxConstraintViolation_lng(workingset,
              solution->xstar.data);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              for (ixlast = 0; ixlast < nVar; ixlast++) {
                solution->xstar.data[ixlast] = solution->searchDir.data[ixlast];
              }
            }
          }

          NMPC_Path_Tracking_iterate_d(H, f_data, solution, memspace, workingset,
            qrmanager, cholmanager, objective, options->SolverName,
            runTimeOptions->MaxIterations);
        }
      }
    } else {
      NMPC_Path_Tracking_iterate_d(H, f_data, solution, memspace, workingset,
        qrmanager, cholmanager, objective, options->SolverName,
        runTimeOptions->MaxIterations);
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_addAeqConstr(s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj,
  int32_T idx_local)
{
  int32_T b_idx;
  int32_T iAeq0;
  int32_T totalEq;
  totalEq = obj->nWConstr[0] + obj->nWConstr[1];
  if ((obj->nActiveConstr == totalEq) && (idx_local > obj->nWConstr[1])) {
    int32_T iAw0;
    obj->nWConstr[1]++;
    obj->isActiveConstr.data[(obj->isActiveIdx[1] + idx_local) - 2] = true;
    obj->nActiveConstr++;
    obj->Wid.data[obj->nActiveConstr - 1] = 2;
    obj->Wlocalidx.data[obj->nActiveConstr - 1] = idx_local;
    iAeq0 = (idx_local - 1) * obj->ldA;
    iAw0 = (obj->nActiveConstr - 1) * obj->ldA;
    b_idx = (uint16_T)obj->nVar;
    for (totalEq = 0; totalEq < b_idx; totalEq++) {
      obj->ATwset.data[iAw0 + totalEq] = obj->Aeq.data[iAeq0 + totalEq];
    }

    obj->bwset.data[obj->nActiveConstr - 1] = obj->beq[idx_local - 1];
  } else {
    int32_T iAw0;
    int32_T iAw0_tmp;
    obj->nActiveConstr++;
    obj->Wid.data[obj->nActiveConstr - 1] = obj->Wid.data[totalEq];
    obj->Wlocalidx.data[obj->nActiveConstr - 1] = obj->Wlocalidx.data[totalEq];
    iAw0_tmp = (uint16_T)obj->nVar;
    for (iAeq0 = 0; iAeq0 < iAw0_tmp; iAeq0++) {
      obj->ATwset.data[iAeq0 + obj->ldA * (obj->nActiveConstr - 1)] =
        obj->ATwset.data[obj->ldA * totalEq + iAeq0];
    }

    obj->bwset.data[obj->nActiveConstr - 1] = obj->bwset.data[totalEq];
    obj->nWConstr[1]++;
    obj->isActiveConstr.data[(obj->isActiveIdx[1] + idx_local) - 2] = true;
    obj->Wid.data[totalEq] = 2;
    obj->Wlocalidx.data[totalEq] = idx_local;
    iAeq0 = (idx_local - 1) * obj->ldA;
    iAw0 = obj->ldA * totalEq;
    for (b_idx = 0; b_idx < iAw0_tmp; b_idx++) {
      obj->ATwset.data[iAw0 + b_idx] = obj->Aeq.data[iAeq0 + b_idx];
    }

    obj->bwset.data[totalEq] = obj->beq[idx_local - 1];
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static boolean_T NMPC_Path_Tracking_soc(const real_T Hessian[160801], const
  real_T grad_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *TrialState,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *WorkingSet, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *CholManager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  QPObjective, const somzaGboVhDG7PNQS6E98jD_NMPC__T *qpoptions)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T b_c;
  int32_T idxIneqOffset;
  int32_T idxStartIneq;
  int32_T idxStartIneq_tmp;
  int32_T idx_Aineq;
  int32_T idx_Partition;
  int32_T idx_lower;
  int32_T idx_lower_tmp;
  int32_T iy;
  int32_T l;
  int32_T mConstrMax;
  int32_T nVar;
  int32_T nWIneq_old;
  int32_T nWLower_old;
  int32_T nWUpper_old;
  boolean_T success;
  nWIneq_old = WorkingSet->nWConstr[2];
  nWLower_old = WorkingSet->nWConstr[3];
  nWUpper_old = WorkingSet->nWConstr[4];
  nVar = WorkingSet->nVar;
  mConstrMax = WorkingSet->mConstrMax;
  idxStartIneq_tmp = (uint16_T)WorkingSet->nVar;
  for (idxIneqOffset = 0; idxIneqOffset < idxStartIneq_tmp; idxIneqOffset++) {
    TrialState->xstarsqp[idxIneqOffset] = TrialState->xstarsqp_old[idxIneqOffset];
  }

  for (idxIneqOffset = 0; idxIneqOffset < idxStartIneq_tmp; idxIneqOffset++) {
    TrialState->socDirection.data[idxIneqOffset] = TrialState->
      xstar.data[idxIneqOffset];
  }

  for (idxIneqOffset = 0; idxIneqOffset < mConstrMax; idxIneqOffset++) {
    TrialState->lambdaStopTest.data[idxIneqOffset] = TrialState->
      lambda.data[idxIneqOffset];
  }

  idxIneqOffset = WorkingSet->isActiveIdx[2];
  for (idxStartIneq = 0; idxStartIneq <= 298; idxStartIneq += 2) {
    tmp_0 = _mm_loadu_pd(&TrialState->cEq[idxStartIneq]);
    _mm_storeu_pd(&WorkingSet->beq[idxStartIneq], _mm_mul_pd(tmp_0, _mm_set1_pd(
      -1.0)));
  }

  idx_lower_tmp = WorkingSet->ldA;
  iy = 0;
  l = WorkingSet->ldA * 299 + 1;
  for (idxStartIneq = 1; idx_lower_tmp < 0 ? idxStartIneq >= l : idxStartIneq <=
       l; idxStartIneq += idx_lower_tmp) {
    b_c = 0.0;
    idx_Partition = (idxStartIneq + WorkingSet->nVar) - 1;
    for (idx_Aineq = idxStartIneq; idx_Aineq <= idx_Partition; idx_Aineq++) {
      b_c += WorkingSet->Aeq.data[idx_Aineq - 1] * TrialState->
        searchDir.data[idx_Aineq - idxStartIneq];
    }

    WorkingSet->beq[iy] += b_c;
    iy++;
  }

  for (idxStartIneq = 0; idxStartIneq < 300; idxStartIneq++) {
    WorkingSet->bwset.data[WorkingSet->sizes[0] + idxStartIneq] =
      WorkingSet->beq[idxStartIneq];
  }

  if (WorkingSet->sizes[2] > 0) {
    iy = (uint16_T)WorkingSet->sizes[2];
    idx_Aineq = ((uint16_T)WorkingSet->sizes[2] / 2) << 1;
    idx_lower = idx_Aineq - 2;
    for (idxStartIneq = 0; idxStartIneq <= idx_lower; idxStartIneq += 2) {
      tmp_0 = _mm_loadu_pd(&TrialState->cIneq.data[idxStartIneq]);
      _mm_storeu_pd(&WorkingSet->bineq.data[idxStartIneq], _mm_mul_pd(tmp_0,
        _mm_set1_pd(-1.0)));
    }

    for (idxStartIneq = idx_Aineq; idxStartIneq < iy; idxStartIneq++) {
      WorkingSet->bineq.data[idxStartIneq] = -TrialState->
        cIneq.data[idxStartIneq];
    }

    iy = 0;
    l = (WorkingSet->sizes[2] - 1) * WorkingSet->ldA + 1;
    for (idxStartIneq = 1; idx_lower_tmp < 0 ? idxStartIneq >= l : idxStartIneq <=
         l; idxStartIneq += idx_lower_tmp) {
      b_c = 0.0;
      idx_Partition = (idxStartIneq + WorkingSet->nVar) - 1;
      for (idx_Aineq = idxStartIneq; idx_Aineq <= idx_Partition; idx_Aineq++) {
        b_c += WorkingSet->Aineq.data[idx_Aineq - 1] *
          TrialState->searchDir.data[idx_Aineq - idxStartIneq];
      }

      WorkingSet->bineq.data[iy] += b_c;
      iy++;
    }

    idx_Aineq = 1;
    idx_lower = WorkingSet->sizes[2] + 1;
    iy = (WorkingSet->sizes[2] + WorkingSet->sizes[3]) + 1;
    l = WorkingSet->nActiveConstr;
    for (idxStartIneq = idxIneqOffset; idxStartIneq <= l; idxStartIneq++) {
      switch (WorkingSet->Wid.data[idxStartIneq - 1]) {
       case 3:
        idx_Partition = idx_Aineq;
        idx_Aineq++;
        WorkingSet->bwset.data[idxStartIneq - 1] = WorkingSet->
          bineq.data[WorkingSet->Wlocalidx.data[idxStartIneq - 1] - 1];
        break;

       case 4:
        idx_Partition = idx_lower;
        idx_lower++;
        break;

       default:
        idx_Partition = iy;
        iy++;
        break;
      }

      TrialState->workingset_old.data[idx_Partition - 1] =
        WorkingSet->Wlocalidx.data[idxStartIneq - 1];
    }
  }

  for (idxIneqOffset = 0; idxIneqOffset < idxStartIneq_tmp; idxIneqOffset++) {
    TrialState->xstar.data[idxIneqOffset] = TrialState->xstarsqp[idxIneqOffset];
  }

  NMPC_Path_Tracking_driver_n(Hessian, grad_data, TrialState, memspace,
    WorkingSet, QRManager, CholManager, QPObjective, qpoptions, qpoptions);
  while ((WorkingSet->mEqRemoved > 0) && (WorkingSet->indexEqRemoved
          [WorkingSet->mEqRemoved - 1] >= 1)) {
    NMPC_Path_Tracking_addAeqConstr(WorkingSet, WorkingSet->
      indexEqRemoved[WorkingSet->mEqRemoved - 1]);
    WorkingSet->mEqRemoved--;
  }

  idxStartIneq = (uint16_T)nVar;
  idx_Aineq = ((uint16_T)nVar / 2) << 1;
  idx_lower = idx_Aineq - 2;
  for (idxIneqOffset = 0; idxIneqOffset <= idx_lower; idxIneqOffset += 2) {
    tmp_0 = _mm_loadu_pd(&TrialState->socDirection.data[idxIneqOffset]);
    tmp = _mm_loadu_pd(&TrialState->xstar.data[idxIneqOffset]);
    _mm_storeu_pd(&TrialState->socDirection.data[idxIneqOffset], _mm_sub_pd(tmp,
      tmp_0));
    _mm_storeu_pd(&TrialState->xstar.data[idxIneqOffset], tmp_0);
  }

  for (idxIneqOffset = idx_Aineq; idxIneqOffset < idxStartIneq; idxIneqOffset++)
  {
    b_c = TrialState->socDirection.data[idxIneqOffset];
    TrialState->socDirection.data[idxIneqOffset] = TrialState->
      xstar.data[idxIneqOffset] - b_c;
    TrialState->xstar.data[idxIneqOffset] = b_c;
  }

  success = (NMPC_Path_Tracking_xnrm2_o(nVar, TrialState->socDirection.data) <=
             2.0 * NMPC_Path_Tracking_xnrm2_o(nVar, TrialState->xstar.data));
  nVar = WorkingSet->sizes[2];
  for (idxIneqOffset = 0; idxIneqOffset <= 298; idxIneqOffset += 2) {
    tmp_0 = _mm_loadu_pd(&TrialState->cEq[idxIneqOffset]);
    _mm_storeu_pd(&WorkingSet->beq[idxIneqOffset], _mm_mul_pd(tmp_0, _mm_set1_pd
      (-1.0)));
  }

  for (idxIneqOffset = 0; idxIneqOffset < 300; idxIneqOffset++) {
    WorkingSet->bwset.data[WorkingSet->sizes[0] + idxIneqOffset] =
      WorkingSet->beq[idxIneqOffset];
  }

  if (WorkingSet->sizes[2] > 0) {
    idxStartIneq = (uint16_T)WorkingSet->sizes[2];
    idx_Aineq = ((uint16_T)WorkingSet->sizes[2] / 2) << 1;
    idx_lower = idx_Aineq - 2;
    for (idxIneqOffset = 0; idxIneqOffset <= idx_lower; idxIneqOffset += 2) {
      tmp_0 = _mm_loadu_pd(&TrialState->cIneq.data[idxIneqOffset]);
      _mm_storeu_pd(&WorkingSet->bineq.data[idxIneqOffset], _mm_mul_pd(tmp_0,
        _mm_set1_pd(-1.0)));
    }

    for (idxIneqOffset = idx_Aineq; idxIneqOffset < idxStartIneq; idxIneqOffset
         ++) {
      WorkingSet->bineq.data[idxIneqOffset] = -TrialState->
        cIneq.data[idxIneqOffset];
    }

    if (!success) {
      idxStartIneq_tmp = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
      idxStartIneq = idxStartIneq_tmp + 1;
      idx_Aineq = WorkingSet->nActiveConstr;
      for (idxIneqOffset = idxStartIneq; idxIneqOffset <= idx_Aineq;
           idxIneqOffset++) {
        WorkingSet->isActiveConstr.data[(WorkingSet->isActiveIdx
          [WorkingSet->Wid.data[idxIneqOffset - 1] - 1] +
          WorkingSet->Wlocalidx.data[idxIneqOffset - 1]) - 2] = false;
      }

      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = idxStartIneq_tmp;
      for (idxIneqOffset = 0; idxIneqOffset < nWIneq_old; idxIneqOffset++) {
        NMPC_Path_Tracki_addAineqConstr(WorkingSet,
          TrialState->workingset_old.data[idxIneqOffset]);
      }

      for (nWIneq_old = 0; nWIneq_old < nWLower_old; nWIneq_old++) {
        NMPC_addBoundToActiveSetMatrix_(WorkingSet, 4,
          TrialState->workingset_old.data[nWIneq_old + nVar]);
      }

      for (nWLower_old = 0; nWLower_old < nWUpper_old; nWLower_old++) {
        NMPC_addBoundToActiveSetMatrix_(WorkingSet, 5,
          TrialState->workingset_old.data[(nWLower_old + nVar) +
          WorkingSet->sizes[3]]);
      }
    }
  }

  if (!success) {
    for (nWUpper_old = 0; nWUpper_old < mConstrMax; nWUpper_old++) {
      TrialState->lambda.data[nWUpper_old] = TrialState->
        lambdaStopTest.data[nWUpper_old];
    }
  } else {
    NMPC_Path_Tracking_sortLambdaQP(TrialState->lambda.data,
      WorkingSet->nActiveConstr, WorkingSet->sizes, WorkingSet->isActiveIdx,
      WorkingSet->Wid.data, WorkingSet->Wlocalidx.data,
      memspace->workspace_float.data);
  }

  return success;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T NMPC_Pat_maxConstraintViolation(const
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *obj, const real_T x_data[])
{
  real_T v;
  int32_T b;
  int32_T idx;
  if (obj->probType == 2) {
    NMPC_Path_Tracking_B.b_obj = *obj;
    v = maxConstraintViolation_AMats_re(&NMPC_Path_Tracking_B.b_obj, x_data);
  } else {
    NMPC_Path_Tracking_B.b_obj = *obj;
    v = maxConstraintViolation_AMats_no(&NMPC_Path_Tracking_B.b_obj, x_data);
  }

  if (obj->sizes[3] > 0) {
    b = (uint16_T)obj->sizes[3];
    for (idx = 0; idx < b; idx++) {
      v = fmax(v, -x_data[NMPC_Path_Tracking_B.b_obj.indexLB.data[idx] - 1] -
               NMPC_Path_Tracking_B.b_obj.lb.data[NMPC_Path_Tracking_B.b_obj.indexLB.data
               [idx] - 1]);
    }
  }

  if (obj->sizes[4] > 0) {
    b = (uint16_T)obj->sizes[4];
    for (idx = 0; idx < b; idx++) {
      v = fmax(v, x_data[NMPC_Path_Tracking_B.b_obj.indexUB.data[idx] - 1] -
               NMPC_Path_Tracking_B.b_obj.ub.data[NMPC_Path_Tracking_B.b_obj.indexUB.data
               [idx] - 1]);
    }
  }

  if (obj->sizes[0] > 0) {
    b = (uint16_T)obj->sizes[0];
    for (idx = 0; idx < b; idx++) {
      v = fmax(v, fabs(x_data[NMPC_Path_Tracking_B.b_obj.indexFixed.data[idx] -
                       1] -
                       NMPC_Path_Tracking_B.b_obj.ub.data[NMPC_Path_Tracking_B.b_obj.indexFixed.data
                       [idx] - 1]));
    }
  }

  return v;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_normal(const real_T Hessian[160801], const real_T
  grad_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *TrialState,
  sG8JZ69axY52WWR6RKyApQC_NMPC__T *MeritFunction,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *WorkingSet, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *CholManager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  QPObjective, const somzaGboVhDG7PNQS6E98jD_NMPC__T *qpoptions,
  s7RdrPWkr8UPAUyTdDJkLaG_NMPC__T *stepFlags)
{
  real_T TrialState_cIneq;
  real_T constrViolationEq;
  real_T constrViolationIneq;
  real_T penaltyParamTrial;
  int32_T b;
  int32_T k;
  boolean_T isEqAndIneqFeasible;
  NMPC_Path_Tracking_driver_n(Hessian, grad_data, TrialState, memspace,
    WorkingSet, QRManager, CholManager, QPObjective, qpoptions, qpoptions);
  isEqAndIneqFeasible = (NMPC_Pat_maxConstraintViolation(WorkingSet,
    TrialState->xstar.data) <= 1.0E-6);
  if ((TrialState->state > 0) || ((TrialState->state == 0) &&
       isEqAndIneqFeasible)) {
    penaltyParamTrial = MeritFunction->penaltyParam;
    constrViolationEq = 0.0;
    for (k = 0; k < 300; k++) {
      constrViolationEq += fabs(TrialState->cEq[k]);
    }

    constrViolationIneq = 0.0;
    b = (uint16_T)WorkingSet->sizes[2];
    for (k = 0; k < b; k++) {
      TrialState_cIneq = TrialState->cIneq.data[k];
      if (TrialState_cIneq > 0.0) {
        constrViolationIneq += TrialState_cIneq;
      }
    }

    constrViolationEq += constrViolationIneq;
    constrViolationIneq = MeritFunction->linearizedConstrViol;
    MeritFunction->linearizedConstrViol = 0.0;
    constrViolationIneq += constrViolationEq;
    if ((constrViolationIneq > 2.2204460492503131E-16) && (TrialState->fstar >
         0.0)) {
      if (TrialState->sqpFval == 0.0) {
        penaltyParamTrial = 1.0;
      } else {
        penaltyParamTrial = 1.5;
      }

      penaltyParamTrial = penaltyParamTrial * TrialState->fstar /
        constrViolationIneq;
    }

    if (penaltyParamTrial < MeritFunction->penaltyParam) {
      MeritFunction->phi = penaltyParamTrial * constrViolationEq +
        TrialState->sqpFval;
      if (((MeritFunction->initConstrViolationEq +
            MeritFunction->initConstrViolationIneq) * penaltyParamTrial +
           MeritFunction->initFval) - MeritFunction->phi > (real_T)
          MeritFunction->nPenaltyDecreases * MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) > TrialState->sqpIterations)
        {
          MeritFunction->threshold *= 10.0;
        }

        MeritFunction->penaltyParam = fmax(penaltyParamTrial, 1.0E-10);
      } else {
        MeritFunction->phi = MeritFunction->penaltyParam * constrViolationEq +
          TrialState->sqpFval;
      }
    } else {
      MeritFunction->penaltyParam = fmax(penaltyParamTrial, 1.0E-10);
      MeritFunction->phi = MeritFunction->penaltyParam * constrViolationEq +
        TrialState->sqpFval;
    }

    MeritFunction->phiPrimePlus = fmin(TrialState->fstar -
      MeritFunction->penaltyParam * constrViolationEq, 0.0);
  } else if (TrialState->state != -6) {
    stepFlags->stepType = 2;
  }

  NMPC_Path_Tracking_sortLambdaQP(TrialState->lambda.data,
    WorkingSet->nActiveConstr, WorkingSet->sizes, WorkingSet->isActiveIdx,
    WorkingSet->Wid.data, WorkingSet->Wlocalidx.data,
    memspace->workspace_float.data);
  isEqAndIneqFeasible = (WorkingSet->mEqRemoved > 0);
  while ((WorkingSet->mEqRemoved > 0) && (WorkingSet->indexEqRemoved
          [WorkingSet->mEqRemoved - 1] >= 1)) {
    NMPC_Path_Tracking_addAeqConstr(WorkingSet, WorkingSet->
      indexEqRemoved[WorkingSet->mEqRemoved - 1]);
    WorkingSet->mEqRemoved--;
  }

  if (isEqAndIneqFeasible) {
    for (k = 0; k < 300; k++) {
      WorkingSet->Wlocalidx.data[WorkingSet->sizes[0] + k] = k + 1;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_relaxed(const real_T Hessian[160801], const
  real_T grad_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *TrialState,
  sG8JZ69axY52WWR6RKyApQC_NMPC__T *MeritFunction,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *WorkingSet, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *CholManager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  QPObjective, somzaGboVhDG7PNQS6E98jD_NMPC__T *qpoptions)
{
  real_T beta;
  real_T qpfvalQuadExcess;
  real_T qpfvalQuadExcess_tmp;
  real_T s;
  real_T smax;
  int32_T idx_max;
  int32_T mFiniteLBOrig;
  int32_T mIneq;
  int32_T mIneq_tmp;
  int32_T mLBOrig;
  int32_T nVarOrig;
  boolean_T b_tf;
  boolean_T tf;
  nVarOrig = WorkingSet->nVar - 1;
  beta = 0.0;
  idx_max = (uint16_T)WorkingSet->nVar;
  for (mFiniteLBOrig = 0; mFiniteLBOrig < idx_max; mFiniteLBOrig++) {
    beta += Hessian[401 * mFiniteLBOrig + mFiniteLBOrig];
  }

  beta /= (real_T)WorkingSet->nVar;
  if (TrialState->sqpIterations <= 1) {
    mIneq = QPObjective->nvar;
    if (QPObjective->nvar < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (QPObjective->nvar > 1) {
        smax = fabs(grad_data[0]);
        for (mFiniteLBOrig = 2; mFiniteLBOrig <= mIneq; mFiniteLBOrig++) {
          s = fabs(grad_data[mFiniteLBOrig - 1]);
          if (s > smax) {
            idx_max = mFiniteLBOrig;
            smax = s;
          }
        }
      }
    }

    smax = fmax(1.0, fabs(grad_data[idx_max - 1])) * 100.0;
  } else {
    mIneq = WorkingSet->mConstr;
    if (WorkingSet->mConstr < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (WorkingSet->mConstr > 1) {
        smax = fabs(TrialState->lambdasqp.data[0]);
        for (mFiniteLBOrig = 2; mFiniteLBOrig <= mIneq; mFiniteLBOrig++) {
          s = fabs(TrialState->lambdasqp.data[mFiniteLBOrig - 1]);
          if (s > smax) {
            idx_max = mFiniteLBOrig;
            smax = s;
          }
        }
      }
    }

    smax = fabs(TrialState->lambdasqp.data[idx_max - 1]);
  }

  QPObjective->nvar = WorkingSet->nVar;
  QPObjective->beta = beta;
  QPObjective->rho = smax;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 4;
  NMPC_Path_Tracking_B.c_WorkingSet = *WorkingSet;
  NMPC_Path_Tracki_setProblemType(&NMPC_Path_Tracking_B.c_WorkingSet, 2);
  mIneq = NMPC_Path_Tracking_B.c_WorkingSet.sizes[2] + 1;
  mLBOrig = (NMPC_Path_Tracking_B.c_WorkingSet.sizes[3] -
             NMPC_Path_Tracking_B.c_WorkingSet.sizes[2]) - 600;
  idx_max = (uint16_T)NMPC_Path_Tracking_B.c_WorkingSet.sizes[2];
  for (mFiniteLBOrig = 0; mFiniteLBOrig < idx_max; mFiniteLBOrig++) {
    memspace->workspace_float.data[mFiniteLBOrig] =
      NMPC_Path_Tracking_B.c_WorkingSet.bineq.data[mFiniteLBOrig];
  }

  NMPC_Path_Tracking_xgemv_cpgexe(WorkingSet->nVar,
    NMPC_Path_Tracking_B.c_WorkingSet.sizes[2],
    NMPC_Path_Tracking_B.c_WorkingSet.Aineq.data,
    NMPC_Path_Tracking_B.c_WorkingSet.ldA, TrialState->xstar.data,
    memspace->workspace_float.data);
  for (mFiniteLBOrig = 0; mFiniteLBOrig < idx_max; mFiniteLBOrig++) {
    TrialState->xstar.data[(nVarOrig + mFiniteLBOrig) + 1] = (real_T)
      (memspace->workspace_float.data[mFiniteLBOrig] > 0.0) *
      memspace->workspace_float.data[mFiniteLBOrig];
  }

  memcpy(&memspace->workspace_float.data[0],
         &NMPC_Path_Tracking_B.c_WorkingSet.beq[0], 300U * sizeof(real_T));
  NMPC_Path_Tracking_xgemv_cpgexe(WorkingSet->nVar, 300,
    NMPC_Path_Tracking_B.c_WorkingSet.Aeq.data,
    NMPC_Path_Tracking_B.c_WorkingSet.ldA, TrialState->xstar.data,
    memspace->workspace_float.data);
  for (mFiniteLBOrig = 0; mFiniteLBOrig < 300; mFiniteLBOrig++) {
    idx_max = mIneq + mFiniteLBOrig;
    if (memspace->workspace_float.data[mFiniteLBOrig] <= 0.0) {
      TrialState->xstar.data[nVarOrig + idx_max] = 0.0;
      TrialState->xstar.data[(nVarOrig + idx_max) + 300] =
        -memspace->workspace_float.data[mFiniteLBOrig];
      NMPC_addBoundToActiveSetMatrix_(&NMPC_Path_Tracking_B.c_WorkingSet, 4,
        mLBOrig + idx_max);
      if (memspace->workspace_float.data[mFiniteLBOrig] >= -1.0E-6) {
        NMPC_addBoundToActiveSetMatrix_(&NMPC_Path_Tracking_B.c_WorkingSet, 4,
          (mLBOrig + idx_max) + 300);
      }
    } else {
      mIneq_tmp = nVarOrig + idx_max;
      TrialState->xstar.data[mIneq_tmp] = memspace->
        workspace_float.data[mFiniteLBOrig];
      TrialState->xstar.data[mIneq_tmp + 300] = 0.0;
      NMPC_addBoundToActiveSetMatrix_(&NMPC_Path_Tracking_B.c_WorkingSet, 4,
        (mLBOrig + idx_max) + 300);
      if (memspace->workspace_float.data[mFiniteLBOrig] <= 1.0E-6) {
        NMPC_addBoundToActiveSetMatrix_(&NMPC_Path_Tracking_B.c_WorkingSet, 4,
          mLBOrig + idx_max);
      }
    }
  }

  nVarOrig = qpoptions->MaxIterations;
  qpoptions->MaxIterations = (qpoptions->MaxIterations +
    NMPC_Path_Tracking_B.c_WorkingSet.nVar) - WorkingSet->nVar;
  NMPC_Path_Tracking_driver_n(Hessian, grad_data, TrialState, memspace,
    &NMPC_Path_Tracking_B.c_WorkingSet, QRManager, CholManager, QPObjective,
    qpoptions, qpoptions);
  qpoptions->MaxIterations = nVarOrig;
  mIneq = NMPC_Path_Tracking_B.c_WorkingSet.sizes[3] - 601;
  nVarOrig = 0;
  for (mFiniteLBOrig = 0; mFiniteLBOrig < 300; mFiniteLBOrig++) {
    idx_max = (NMPC_Path_Tracking_B.c_WorkingSet.isActiveIdx[3] + mIneq) +
      mFiniteLBOrig;
    tf = NMPC_Path_Tracking_B.c_WorkingSet.isActiveConstr.data[idx_max];
    b_tf = NMPC_Path_Tracking_B.c_WorkingSet.isActiveConstr.data[idx_max + 300];
    memspace->workspace_int.data[mFiniteLBOrig] = tf;
    memspace->workspace_int.data[mFiniteLBOrig + 300] = b_tf;
    nVarOrig = (nVarOrig + tf) + b_tf;
  }

  mLBOrig = (uint16_T)NMPC_Path_Tracking_B.c_WorkingSet.sizes[2];
  for (mFiniteLBOrig = 0; mFiniteLBOrig < mLBOrig; mFiniteLBOrig++) {
    idx_max = NMPC_Path_Tracking_B.c_WorkingSet.isActiveConstr.data
      [((NMPC_Path_Tracking_B.c_WorkingSet.isActiveIdx[3] + mIneq) -
        NMPC_Path_Tracking_B.c_WorkingSet.sizes[2]) + mFiniteLBOrig];
    memspace->workspace_int.data[mFiniteLBOrig + 600] = idx_max;
    nVarOrig += idx_max;
  }

  if (TrialState->state != -6) {
    idx_max = (NMPC_Path_Tracking_B.c_WorkingSet.nVarMax - WorkingSet->nVar) - 1;
    mIneq_tmp = WorkingSet->nVar + 1;
    s = 0.0;
    qpfvalQuadExcess = 0.0;
    if (idx_max >= 1) {
      mLBOrig = WorkingSet->nVar + idx_max;
      for (mFiniteLBOrig = mIneq_tmp; mFiniteLBOrig <= mLBOrig; mFiniteLBOrig++)
      {
        s += fabs(TrialState->xstar.data[mFiniteLBOrig - 1]);
      }

      idx_max = (uint16_T)idx_max;
      for (mFiniteLBOrig = 0; mFiniteLBOrig < idx_max; mFiniteLBOrig++) {
        qpfvalQuadExcess_tmp = TrialState->xstar.data[WorkingSet->nVar +
          mFiniteLBOrig];
        qpfvalQuadExcess += qpfvalQuadExcess_tmp * qpfvalQuadExcess_tmp;
      }
    }

    beta = (TrialState->fstar - smax * s) - beta / 2.0 * qpfvalQuadExcess;
    mIneq = (WorkingSet->nVarMax - WorkingSet->nVar) - 1;
    smax = MeritFunction->penaltyParam;
    s = 0.0;
    for (mFiniteLBOrig = 0; mFiniteLBOrig < 300; mFiniteLBOrig++) {
      s += fabs(TrialState->cEq[mFiniteLBOrig]);
    }

    qpfvalQuadExcess = 0.0;
    mLBOrig = (uint16_T)WorkingSet->sizes[2];
    for (mFiniteLBOrig = 0; mFiniteLBOrig < mLBOrig; mFiniteLBOrig++) {
      qpfvalQuadExcess_tmp = TrialState->cIneq.data[mFiniteLBOrig];
      if (qpfvalQuadExcess_tmp > 0.0) {
        qpfvalQuadExcess += qpfvalQuadExcess_tmp;
      }
    }

    s += qpfvalQuadExcess;
    qpfvalQuadExcess = MeritFunction->linearizedConstrViol;
    qpfvalQuadExcess_tmp = 0.0;
    if (mIneq >= 1) {
      mIneq += WorkingSet->nVar;
      for (mFiniteLBOrig = mIneq_tmp; mFiniteLBOrig <= mIneq; mFiniteLBOrig++) {
        qpfvalQuadExcess_tmp += fabs(TrialState->xstar.data[mFiniteLBOrig - 1]);
      }
    }

    MeritFunction->linearizedConstrViol = qpfvalQuadExcess_tmp;
    qpfvalQuadExcess = (s + qpfvalQuadExcess) - qpfvalQuadExcess_tmp;
    if ((qpfvalQuadExcess > 2.2204460492503131E-16) && (beta > 0.0)) {
      if (TrialState->sqpFval == 0.0) {
        smax = 1.0;
      } else {
        smax = 1.5;
      }

      smax = smax * beta / qpfvalQuadExcess;
    }

    if (smax < MeritFunction->penaltyParam) {
      MeritFunction->phi = smax * s + TrialState->sqpFval;
      if (((MeritFunction->initConstrViolationEq +
            MeritFunction->initConstrViolationIneq) * smax +
           MeritFunction->initFval) - MeritFunction->phi > (real_T)
          MeritFunction->nPenaltyDecreases * MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) > TrialState->sqpIterations)
        {
          MeritFunction->threshold *= 10.0;
        }

        MeritFunction->penaltyParam = fmax(smax, 1.0E-10);
      } else {
        MeritFunction->phi = MeritFunction->penaltyParam * s +
          TrialState->sqpFval;
      }
    } else {
      MeritFunction->penaltyParam = fmax(smax, 1.0E-10);
      MeritFunction->phi = MeritFunction->penaltyParam * s + TrialState->sqpFval;
    }

    MeritFunction->phiPrimePlus = fmin(beta - MeritFunction->penaltyParam * s,
      0.0);
    mIneq = NMPC_Path_Tracking_B.c_WorkingSet.isActiveIdx[1] - 1;
    for (mFiniteLBOrig = 0; mFiniteLBOrig < 300; mFiniteLBOrig++) {
      if (memspace->workspace_int.data[mFiniteLBOrig] != 0) {
        tf = (memspace->workspace_int.data[mFiniteLBOrig + 300] != 0);
      } else {
        tf = false;
      }

      idx_max = mIneq + mFiniteLBOrig;
      TrialState->lambda.data[idx_max] *= (real_T)tf;
    }

    idx_max = NMPC_Path_Tracking_B.c_WorkingSet.isActiveIdx[2];
    mIneq = NMPC_Path_Tracking_B.c_WorkingSet.nActiveConstr;
    for (mFiniteLBOrig = idx_max; mFiniteLBOrig <= mIneq; mFiniteLBOrig++) {
      if (NMPC_Path_Tracking_B.c_WorkingSet.Wid.data[mFiniteLBOrig - 1] == 3) {
        TrialState->lambda.data[mFiniteLBOrig - 1] *= (real_T)
          memspace->
          workspace_int.data[NMPC_Path_Tracking_B.c_WorkingSet.Wlocalidx.data[mFiniteLBOrig
          - 1] + 599];
      }
    }
  }

  mFiniteLBOrig = (NMPC_Path_Tracking_B.c_WorkingSet.sizes[3] -
                   NMPC_Path_Tracking_B.c_WorkingSet.sizes[2]) - 600;
  idx_max = NMPC_Path_Tracking_B.c_WorkingSet.nActiveConstr;
  while ((idx_max > NMPC_Path_Tracking_B.c_WorkingSet.sizes[0] + 300) &&
         (nVarOrig > 0)) {
    if ((NMPC_Path_Tracking_B.c_WorkingSet.Wid.data[idx_max - 1] == 4) &&
        (NMPC_Path_Tracking_B.c_WorkingSet.Wlocalidx.data[idx_max - 1] >
         mFiniteLBOrig)) {
      beta = TrialState->
        lambda.data[NMPC_Path_Tracking_B.c_WorkingSet.nActiveConstr - 1];
      TrialState->lambda.data[NMPC_Path_Tracking_B.c_WorkingSet.nActiveConstr -
        1] = 0.0;
      TrialState->lambda.data[idx_max - 1] = beta;
      NMPC_Path_Tracking_removeConstr(&NMPC_Path_Tracking_B.c_WorkingSet,
        idx_max);
      nVarOrig--;
    }

    idx_max--;
  }

  QPObjective->nvar = WorkingSet->nVar;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 3;
  *WorkingSet = NMPC_Path_Tracking_B.c_WorkingSet;
  NMPC_Path_Tracki_setProblemType(WorkingSet, 3);
  NMPC_Path_Tracking_sortLambdaQP(TrialState->lambda.data,
    WorkingSet->nActiveConstr, WorkingSet->sizes, WorkingSet->isActiveIdx,
    WorkingSet->Wid.data, WorkingSet->Wlocalidx.data,
    memspace->workspace_float.data);
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_step_c(s7RdrPWkr8UPAUyTdDJkLaG_NMPC__T *stepFlags,
  real_T Hessian[160801], const real_T lb[401], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *
  TrialState, sG8JZ69axY52WWR6RKyApQC_NMPC__T *MeritFunction,
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
  *WorkingSet, s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager,
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T *CholManager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *
  QPObjective, somzaGboVhDG7PNQS6E98jD_NMPC__T *qpoptions)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T tmp_1[2];
  real_T nrmDirInf;
  real_T nrmGradInf;
  int32_T exitg1;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T k;
  int32_T nVar;
  int32_T vectorUB;
  boolean_T checkBoundViolation;
  boolean_T guard1;
  stepFlags->stepAccepted = true;
  checkBoundViolation = true;
  nVar = WorkingSet->nVar - 1;
  if (stepFlags->stepType != 3) {
    idxStartIneq = (uint16_T)WorkingSet->nVar;
    for (k = 0; k < idxStartIneq; k++) {
      TrialState->xstar.data[k] = TrialState->xstarsqp[k];
    }
  } else {
    for (k = 0; k <= nVar; k++) {
      TrialState->searchDir.data[k] = TrialState->xstar.data[k];
    }
  }

  do {
    exitg1 = 0;
    guard1 = false;
    switch (stepFlags->stepType) {
     case 1:
      NMPC_Path_Tracking_normal(Hessian, TrialState->grad.data, TrialState,
        MeritFunction, memspace, WorkingSet, QRManager, CholManager, QPObjective,
        qpoptions, stepFlags);
      if (stepFlags->stepType == 2) {
      } else {
        for (k = 0; k <= nVar; k++) {
          TrialState->delta_x.data[k] = TrialState->xstar.data[k];
        }

        guard1 = true;
      }
      break;

     case 2:
      vectorUB = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
      idxStartIneq = vectorUB + 1;
      idxEndIneq = WorkingSet->nActiveConstr;
      for (k = idxStartIneq; k <= idxEndIneq; k++) {
        WorkingSet->isActiveConstr.data[(WorkingSet->isActiveIdx
          [WorkingSet->Wid.data[k - 1] - 1] + WorkingSet->Wlocalidx.data[k - 1])
          - 2] = false;
      }

      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = vectorUB;
      idxStartIneq = TrialState->xstar.size;
      idxEndIneq = TrialState->xstar.size;
      for (k = 0; k < idxEndIneq; k++) {
        NMPC_Path_Tracking_B.b_data[k] = TrialState->xstar.data[k];
      }

      idxEndIneq = (uint16_T)WorkingSet->sizes[3];
      for (k = 0; k < idxEndIneq; k++) {
        nrmGradInf = WorkingSet->lb.data[WorkingSet->indexLB.data[k] - 1];
        if (-NMPC_Path_Tracking_B.b_data[WorkingSet->indexLB.data[k] - 1] >
            nrmGradInf) {
          NMPC_Path_Tracking_B.b_data[WorkingSet->indexLB.data[k] - 1] =
            -nrmGradInf + fabs(nrmGradInf);
        }
      }

      for (k = 0; k < idxStartIneq; k++) {
        TrialState->xstar.data[k] = NMPC_Path_Tracking_B.b_data[k];
      }

      NMPC_Path_Tracking_relaxed(Hessian, TrialState->grad.data, TrialState,
        MeritFunction, memspace, WorkingSet, QRManager, CholManager, QPObjective,
        qpoptions);
      for (k = 0; k <= nVar; k++) {
        TrialState->delta_x.data[k] = TrialState->xstar.data[k];
      }

      guard1 = true;
      break;

     default:
      checkBoundViolation = NMPC_Path_Tracking_soc(Hessian,
        TrialState->grad.data, TrialState, memspace, WorkingSet, QRManager,
        CholManager, QPObjective, qpoptions);
      stepFlags->stepAccepted = checkBoundViolation;
      if (stepFlags->stepAccepted && (TrialState->state != -6)) {
        idxStartIneq = (uint16_T)(nVar + 1);
        idxEndIneq = ((uint16_T)(nVar + 1) / 2) << 1;
        vectorUB = idxEndIneq - 2;
        for (k = 0; k <= vectorUB; k += 2) {
          tmp = _mm_loadu_pd(&TrialState->xstar.data[k]);
          tmp_0 = _mm_loadu_pd(&TrialState->socDirection.data[k]);
          _mm_storeu_pd(&TrialState->delta_x.data[k], _mm_add_pd(tmp, tmp_0));
        }

        for (k = idxEndIneq; k < idxStartIneq; k++) {
          TrialState->delta_x.data[k] = TrialState->xstar.data[k] +
            TrialState->socDirection.data[k];
        }
      }

      guard1 = true;
      break;
    }

    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        nrmGradInf = 0.0;
        nrmDirInf = 1.0;
        for (k = 0; k < 401; k++) {
          nrmGradInf = fmax(nrmGradInf, fabs(TrialState->grad.data[k]));
          nrmDirInf = fmax(nrmDirInf, fabs(TrialState->xstar.data[k]));
        }

        nrmGradInf = fmax(2.2204460492503131E-16, nrmGradInf / nrmDirInf);
        for (k = 0; k < 401; k++) {
          idxEndIneq = 401 * k;
          for (idxStartIneq = 0; idxStartIneq < k; idxStartIneq++) {
            Hessian[idxEndIneq + idxStartIneq] = 0.0;
          }

          idxEndIneq = 401 * k + k;
          Hessian[idxEndIneq] = nrmGradInf;
          vectorUB = 399 - k;
          for (idxStartIneq = 0; idxStartIneq <= vectorUB; idxStartIneq++) {
            Hessian[(idxEndIneq + idxStartIneq) + 1] = 0.0;
          }
        }
      }
    }
  } while (exitg1 == 0);

  if (checkBoundViolation) {
    idxStartIneq = TrialState->delta_x.size;
    idxEndIneq = TrialState->delta_x.size;
    for (k = 0; k < idxEndIneq; k++) {
      NMPC_Path_Tracking_B.b_data[k] = TrialState->delta_x.data[k];
    }

    k = (uint16_T)WorkingSet->sizes[3];
    for (nVar = 0; nVar < k; nVar++) {
      nrmDirInf = NMPC_Path_Tracking_B.b_data[WorkingSet->indexLB.data[nVar] - 1];
      nrmGradInf = (TrialState->xstarsqp[WorkingSet->indexLB.data[nVar] - 1] +
                    nrmDirInf) - lb[WorkingSet->indexLB.data[nVar] - 1];
      if (nrmGradInf < 0.0) {
        _mm_storeu_pd(&tmp_1[0], _mm_sub_pd(_mm_set_pd(TrialState->
          xstar.data[WorkingSet->indexLB.data[nVar] - 1], nrmDirInf),
          _mm_set1_pd(nrmGradInf)));
        NMPC_Path_Tracking_B.b_data[WorkingSet->indexLB.data[nVar] - 1] = tmp_1
          [0];
        TrialState->xstar.data[WorkingSet->indexLB.data[nVar] - 1] = tmp_1[1];
      }
    }

    for (k = 0; k < idxStartIneq; k++) {
      TrialState->delta_x.data[k] = NMPC_Path_Tracking_B.b_data[k];
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracki_outputBounds_c(const real_T runtimedata_OutputMin
  [300], const real_T runtimedata_OutputMax[300], const real_T X[306], real_T e,
  real_T c_data[], int32_T c_size[2])
{
  __m128d tmp_0;
  real_T ic[6];
  real_T yk[6];
  real_T ic_0;
  real_T runtimedata_OutputMin_0;
  int32_T icf_tmp_1[12];
  int32_T icf_tmp[6];
  int32_T icf_tmp_0[6];
  int32_T i;
  int32_T icf_tmp_2;
  int32_T trueCount;
  int16_T tmp_data[600];
  boolean_T icf[600];
  boolean_T tmp[300];
  boolean_T icf_0[12];
  boolean_T x[6];
  boolean_T exitg1;
  boolean_T y;
  for (trueCount = 0; trueCount < 300; trueCount++) {
    tmp[trueCount] = rtIsInf(runtimedata_OutputMin[trueCount]);
  }

  NMPC_Path_Tracking_all(tmp, x);
  y = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 6)) {
    if (!x[i]) {
      y = false;
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (y) {
    for (trueCount = 0; trueCount < 300; trueCount++) {
      tmp[trueCount] = rtIsInf(runtimedata_OutputMax[trueCount]);
    }

    NMPC_Path_Tracking_all(tmp, x);
    y = true;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 6)) {
      if (!x[i]) {
        y = false;
        exitg1 = true;
      } else {
        i++;
      }
    }
  } else {
    y = false;
  }

  if (y) {
    c_size[0] = 0;
    c_size[1] = 0;
  } else {
    for (i = 0; i < 600; i++) {
      NMPC_Path_Tracking_B.b_c_p[i] = 0.0;
      icf[i] = true;
    }

    for (trueCount = 0; trueCount < 6; trueCount++) {
      ic[trueCount] = (real_T)trueCount + 1.0;
    }

    for (i = 0; i < 50; i++) {
      for (trueCount = 0; trueCount < 6; trueCount++) {
        ic_0 = ic[trueCount];
        icf_tmp[trueCount] = (int32_T)ic_0;
        runtimedata_OutputMin_0 = runtimedata_OutputMin[50 * trueCount + i];
        icf[(int32_T)ic_0 - 1] = ((!rtIsInf(runtimedata_OutputMin_0)) &&
          (!rtIsNaN(runtimedata_OutputMin_0)));
        icf_tmp_0[trueCount] = (int32_T)(ic_0 + 6.0);
      }

      for (trueCount = 0; trueCount < 6; trueCount++) {
        icf_tmp_2 = icf_tmp_0[trueCount];
        ic_0 = runtimedata_OutputMax[50 * trueCount + i];
        icf[icf_tmp_2 - 1] = ((!rtIsInf(ic_0)) && (!rtIsNaN(ic_0)));
        icf_tmp_1[trueCount] = icf_tmp[trueCount] - 1;
        icf_tmp_1[trueCount + 6] = icf_tmp_2 - 1;
      }

      for (trueCount = 0; trueCount < 12; trueCount++) {
        icf_0[trueCount] = icf[icf_tmp_1[trueCount]];
      }

      if (NMPC_Path_Tracking_any(icf_0)) {
        yk[0] = X[i + 1];
        yk[1] = X[i + 52];
        yk[2] = X[i + 103];
        yk[3] = X[i + 154] / 10.0;
        yk[4] = X[i + 205];
        yk[5] = X[i + 256];
        for (trueCount = 0; trueCount < 6; trueCount++) {
          NMPC_Path_Tracking_B.b_c_p[icf_tmp[trueCount] - 1] =
            (runtimedata_OutputMin[50 * trueCount + i] - e) - yk[trueCount];
        }

        for (trueCount = 0; trueCount < 6; trueCount++) {
          NMPC_Path_Tracking_B.b_c_p[icf_tmp_0[trueCount] - 1] = (yk[trueCount]
            - runtimedata_OutputMax[50 * trueCount + i]) - e;
        }
      }

      for (trueCount = 0; trueCount <= 4; trueCount += 2) {
        tmp_0 = _mm_loadu_pd(&ic[trueCount]);
        _mm_storeu_pd(&ic[trueCount], _mm_add_pd(tmp_0, _mm_set1_pd(12.0)));
      }
    }

    trueCount = 0;
    for (i = 0; i < 600; i++) {
      if (icf[i]) {
        trueCount++;
      }
    }

    icf_tmp_2 = trueCount;
    trueCount = 0;
    for (i = 0; i < 600; i++) {
      if (icf[i]) {
        tmp_data[trueCount] = (int16_T)i;
        trueCount++;
      }
    }

    c_size[0] = icf_tmp_2;
    c_size[1] = 1;
    for (trueCount = 0; trueCount < icf_tmp_2; trueCount++) {
      c_data[trueCount] = NMPC_Path_Tracking_B.b_c_p[tmp_data[trueCount]];
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Trac_stateEvolution_d(const real_T X[306], const real_T U
  [102], real_T c[300])
{
  real_T b_X[306];
  real_T b_U[102];
  real_T ic[6];
  real_T tmp[6];
  real_T tmp_0[6];
  real_T ic_0;
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  memset(&c[0], 0, 300U * sizeof(real_T));
  for (i_0 = 0; i_0 < 6; i_0++) {
    ic[i_0] = (real_T)i_0 + 1.0;
  }

  for (i_0 = 0; i_0 < 51; i_0++) {
    i = i_0 << 1;
    b_U[i] = U[i_0];
    b_U[i + 1] = U[i_0 + 51];
    for (i_1 = 0; i_1 < 6; i_1++) {
      b_X[i_1 + 6 * i_0] = X[51 * i_1 + i_0];
    }
  }

  for (i = 0; i < 50; i++) {
    i_0 = i << 1;
    NMPC_Path_StateFunctionVehicle1(&b_X[6 * i], &b_U[i_0], tmp);
    i_1 = (i + 1) * 6;
    NMPC_Path_StateFunctionVehicle1(&b_X[i_1], &b_U[i_0], tmp_0);
    for (i_0 = 0; i_0 < 6; i_0++) {
      ic_0 = ic[i_0];
      c[(int32_T)ic_0 - 1] = (b_X[6 * i + i_0] + (tmp[i_0] + tmp_0[i_0]) * 0.005)
        - b_X[i_1 + i_0];
      ic[i_0] = ic_0 + 6.0;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path__c4_mpclib_anonFcn2_d(const real_T runtimedata_x[6], const
  real_T runtimedata_OutputMin[300], const real_T runtimedata_OutputMax[300],
  const real_T z[401], real_T varargout_1_data[], int32_T varargout_1_size[2],
  real_T varargout_2[300])
{
  real_T X[306];
  real_T U[102];
  real_T e;
  int32_T varargin_1_size[2];
  int32_T i;
  int32_T i_0;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int16_T sizes[2];
  boolean_T sizes_idx_1_tmp;
  NMPC_Path_Tracking_getXUe(z, runtimedata_x, X, U, &e);
  NMPC_Path_Tracki_outputBounds_c(runtimedata_OutputMin, runtimedata_OutputMax,
    X, e, NMPC_Path_Tracking_B.varargin_1_data_n, varargin_1_size);
  sizes_idx_1_tmp = ((varargin_1_size[0] != 0) && (varargin_1_size[1] != 0));
  if (!sizes_idx_1_tmp) {
    sizes[0] = (int16_T)varargin_1_size[0];
  } else if (sizes_idx_1_tmp) {
    sizes[0] = (int16_T)varargin_1_size[0];
  } else {
    sizes[0] = 0;
  }

  varargout_1_size[0] = sizes[0];
  varargout_1_size[1] = sizes_idx_1_tmp;
  loop_ub = sizes_idx_1_tmp;
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    loop_ub_0 = sizes[0];
    for (i = 0; i < loop_ub_0; i++) {
      varargout_1_data[i] = NMPC_Path_Tracking_B.varargin_1_data_n[i];
    }
  }

  NMPC_Path_Trac_stateEvolution_d(X, U, varargout_2);
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Trac_evalObjAndConstr(int32_T
  obj_next_next_next_next_next_b_, const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T
  *obj_next_next_next_next_next_ne, const s_UduwGwOpHSPENrm1X2xchE_NMPC_T
  *obj_next_next_next_next_next__0, const real_T x[401], real_T
  Cineq_workspace_data[], int32_T ineq0, real_T Ceq_workspace[300], real_T *fval,
  int32_T *status)
{
  __m128d tmp_2;
  __m128d tmp_3;
  __m128i tmp_0;
  real_T X[306];
  real_T b_X[306];
  real_T c[300];
  real_T U[102];
  real_T b_U[102];
  real_T b_X_0[6];
  real_T tmp_1[2];
  real_T duk;
  real_T duk_idx_0;
  real_T duk_idx_1;
  real_T e;
  real_T fs;
  real_T umvk;
  real_T umvk_idx_0;
  real_T umvk_idx_1;
  int32_T ineqRange_data[600];
  int32_T b_X_tmp;
  int32_T n;
  int32_T vectorUB;
  int32_T yk;
  boolean_T tmp;
  static const int8_T d[6] = { 1, 1, 1, 10, 1, 1 };

  int32_T b_size[2];
  int32_T ineqRange_size_idx_1;
  NMPC_Path_Tracking_getXUe(x, obj_next_next_next_next_next__0->runtimedata.x, X,
    U, &e);
  fs = 0.0;
  for (b_X_tmp = 0; b_X_tmp < 51; b_X_tmp++) {
    for (n = 0; n < 6; n++) {
      b_X[n + 6 * b_X_tmp] = X[51 * n + b_X_tmp];
    }

    n = b_X_tmp << 1;
    b_U[n] = U[b_X_tmp];
    b_U[n + 1] = U[b_X_tmp + 51];
  }

  for (n = 0; n < 50; n++) {
    b_X_tmp = (n + 1) * 6;
    b_X_0[0] = b_X[b_X_tmp];
    b_X_0[1] = b_X[b_X_tmp + 1];
    b_X_0[2] = b_X[b_X_tmp + 2];
    b_X_0[3] = b_X[b_X_tmp + 3] / 10.0;
    b_X_0[4] = b_X[b_X_tmp + 4];
    b_X_0[5] = b_X[b_X_tmp + 5];
    umvk_idx_1 = 0.0;
    for (b_X_tmp = 0; b_X_tmp < 6; b_X_tmp++) {
      yk = 50 * b_X_tmp + n;
      duk_idx_1 = (b_X_0[b_X_tmp] -
                   obj_next_next_next_next_next__0->runtimedata.ref[yk] /
                   (real_T)d[b_X_tmp]) *
        obj_next_next_next_next_next__0->runtimedata.OutputWeights[yk];
      umvk_idx_1 += duk_idx_1 * duk_idx_1;
    }

    fs += umvk_idx_1;
    tmp_3 = _mm_set_pd(2.26, 6.0);
    tmp_2 = _mm_div_pd(_mm_loadu_pd(&b_U[n << 1]), tmp_3);
    _mm_storeu_pd(&tmp_1[0], tmp_2);
    umvk_idx_0 = tmp_1[0];
    umvk_idx_1 = tmp_1[1];
    if (n + 1 == 1) {
      _mm_storeu_pd(&tmp_1[0], _mm_sub_pd(_mm_set_pd(tmp_1[1], tmp_1[0]),
        _mm_div_pd(_mm_loadu_pd
                   (&obj_next_next_next_next_next__0->runtimedata.lastMV[0]),
                   tmp_3)));
      duk_idx_0 = tmp_1[0];
      duk_idx_1 = tmp_1[1];
    } else {
      tmp_3 = _mm_sub_pd(_mm_set_pd(tmp_1[1], tmp_1[0]), _mm_div_pd(_mm_loadu_pd
        (&b_U[(n - 1) << 1]), tmp_3));
      _mm_storeu_pd(&tmp_1[0], tmp_3);
      duk_idx_0 = tmp_1[0];
      duk_idx_1 = tmp_1[1];
    }

    umvk_idx_0 = (umvk_idx_0 -
                  obj_next_next_next_next_next__0->runtimedata.MVScaledTarget[n])
      * obj_next_next_next_next_next__0->runtimedata.MVWeights[n];
    duk_idx_0 *= obj_next_next_next_next_next__0->runtimedata.MVRateWeights[n];
    umvk = umvk_idx_0 * umvk_idx_0;
    duk = duk_idx_0 * duk_idx_0;
    umvk_idx_0 = (umvk_idx_1 -
                  obj_next_next_next_next_next__0->runtimedata.MVScaledTarget[n
                  + 50]) *
      obj_next_next_next_next_next__0->runtimedata.MVWeights[n + 50];
    duk_idx_0 = obj_next_next_next_next_next__0->runtimedata.MVRateWeights[n +
      50] * duk_idx_1;
    fs = ((umvk_idx_0 * umvk_idx_0 + umvk) + fs) + (duk_idx_0 * duk_idx_0 + duk);
  }

  *fval = 100000.0 * e * e + fs;
  *status = 1;
  tmp = rtIsNaN(*fval);
  if (rtIsInf(*fval) || tmp) {
    if (tmp) {
      *status = -3;
    } else if (*fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  }

  if (*status == 1) {
    if (obj_next_next_next_next_next_b_ - 1 < 0) {
      n = 0;
    } else {
      n = (uint16_T)(obj_next_next_next_next_next_b_ - 1) + 1;
    }

    ineqRange_size_idx_1 = n;
    if (n > 0) {
      ineqRange_data[0] = 0;
      yk = 0;
      for (b_X_tmp = 2; b_X_tmp <= n; b_X_tmp++) {
        yk++;
        ineqRange_data[b_X_tmp - 1] = yk;
      }
    }

    n--;
    yk = (ineqRange_size_idx_1 / 4) << 2;
    vectorUB = yk - 4;
    for (b_X_tmp = 0; b_X_tmp <= vectorUB; b_X_tmp += 4) {
      tmp_0 = _mm_loadu_si128((const __m128i *)&ineqRange_data[b_X_tmp]);
      _mm_storeu_si128((__m128i *)&ineqRange_data[b_X_tmp], _mm_add_epi32(tmp_0,
        _mm_set1_epi32(ineq0)));
    }

    for (b_X_tmp = yk; b_X_tmp <= n; b_X_tmp++) {
      ineqRange_data[b_X_tmp] += ineq0;
    }

    NMPC_Path__c4_mpclib_anonFcn2_d(obj_next_next_next_next_next_ne->x,
      obj_next_next_next_next_next_ne->OutputMin,
      obj_next_next_next_next_next_ne->OutputMax, x,
      NMPC_Path_Tracking_B.b_data_l, b_size, c);
    for (b_X_tmp = 0; b_X_tmp < ineqRange_size_idx_1; b_X_tmp++) {
      Cineq_workspace_data[ineqRange_data[b_X_tmp] - 1] =
        NMPC_Path_Tracking_B.b_data_l[b_X_tmp];
    }

    memcpy(&Ceq_workspace[0], &c[0], 300U * sizeof(real_T));
    *status = NMPC_Path__checkVectorNonFinite(obj_next_next_next_next_next_b_,
      Cineq_workspace_data, ineq0);
    if (*status == 1) {
      *status = NMPC_Pat_checkVectorNonFinite_l(Ceq_workspace);
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Pat_computeLinearResiduals(const real_T x[401], int32_T nVar,
  real_T workspaceIneq_data[], const int32_T *workspaceIneq_size, int32_T
  mLinIneq, const real_T AineqT_data[], const real_T bineq_data[], int32_T ldAi)
{
  int32_T k;
  int32_T loop_ub;
  if (mLinIneq > 0) {
    int32_T scalarLB;
    int32_T vectorUB;
    loop_ub = *workspaceIneq_size;
    for (k = 0; k < loop_ub; k++) {
      NMPC_Path_Tracking_B.y_data_g1[k] = workspaceIneq_data[k];
    }

    for (k = 0; k < mLinIneq; k++) {
      NMPC_Path_Tracking_B.y_data_g1[k] = bineq_data[k];
    }

    for (k = 0; k < loop_ub; k++) {
      workspaceIneq_data[k] = NMPC_Path_Tracking_B.y_data_g1[k];
    }

    k = (uint16_T)mLinIneq;
    scalarLB = ((uint16_T)mLinIneq / 2) << 1;
    vectorUB = scalarLB - 2;
    for (loop_ub = 0; loop_ub <= vectorUB; loop_ub += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&workspaceIneq_data[loop_ub]);
      _mm_storeu_pd(&workspaceIneq_data[loop_ub], _mm_mul_pd(tmp, _mm_set1_pd
        (-1.0)));
    }

    for (loop_ub = scalarLB; loop_ub < k; loop_ub++) {
      workspaceIneq_data[loop_ub] = -workspaceIneq_data[loop_ub];
    }

    scalarLB = 0;
    vectorUB = (mLinIneq - 1) * ldAi + 1;
    for (loop_ub = 1; ldAi < 0 ? loop_ub >= vectorUB : loop_ub <= vectorUB;
         loop_ub += ldAi) {
      real_T c;
      int32_T e;
      c = 0.0;
      e = (loop_ub + nVar) - 1;
      for (k = loop_ub; k <= e; k++) {
        c += AineqT_data[k - 1] * x[k - loop_ub];
      }

      workspaceIneq_data[scalarLB] += c;
      scalarLB++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static real_T NMPC_Path_Track_computeMeritFcn(real_T obj_penaltyParam, real_T
  fval, const real_T Cineq_workspace_data[], int32_T mIneq, const real_T
  Ceq_workspace[300], boolean_T evalWellDefined)
{
  real_T val;
  int32_T idx;
  int32_T k;
  if (evalWellDefined) {
    real_T constrViolationEq;
    real_T constrViolationIneq;
    constrViolationEq = 0.0;
    for (k = 0; k < 300; k++) {
      constrViolationEq += fabs(Ceq_workspace[k]);
    }

    constrViolationIneq = 0.0;
    k = (uint16_T)mIneq;
    for (idx = 0; idx < k; idx++) {
      real_T Cineq_workspace;
      Cineq_workspace = Cineq_workspace_data[idx];
      if (Cineq_workspace > 0.0) {
        constrViolationIneq += Cineq_workspace;
      }
    }

    val = (constrViolationEq + constrViolationIneq) * obj_penaltyParam + fval;
  } else {
    val = (rtInf);
  }

  return val;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_linesearch(boolean_T *evalWellDefined, const
  real_T bineq_data[], int32_T WorkingSet_nVar, int32_T WorkingSet_ldA, const
  real_T WorkingSet_Aineq_data[], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T *TrialState,
  real_T MeritFunction_penaltyParam, real_T MeritFunction_phi, real_T
  MeritFunction_phiPrimePlus, real_T MeritFunction_phiFullStep, int32_T
  FcnEvaluator_next_next_next_nex, const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T
  *FcnEvaluator_next_next_next_n_0, const s_UduwGwOpHSPENrm1X2xchE_NMPC_T
  *FcnEvaluator_next_next_next_n_1, boolean_T socTaken, real_T *alpha, int32_T
  *exitflag)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T phi_alpha;
  int32_T exitg1;
  int32_T idx;
  int32_T k;
  int32_T mLinIneq;
  int32_T scalarLB;
  int32_T scalarLB_tmp;
  int32_T vectorUB;
  int32_T vectorUB_tmp;
  boolean_T exitg2;
  boolean_T tooSmallX;
  mLinIneq = TrialState->mIneq - TrialState->mNonlinIneq;
  *alpha = 1.0;
  *exitflag = 1;
  phi_alpha = MeritFunction_phiFullStep;
  for (k = 0; k < WorkingSet_nVar; k++) {
    TrialState->searchDir.data[k] = TrialState->delta_x.data[k];
  }

  do {
    exitg1 = 0;
    if (TrialState->FunctionEvaluations < 40100) {
      if ((*evalWellDefined) && (phi_alpha <= *alpha * 0.0001 *
           MeritFunction_phiPrimePlus + MeritFunction_phi)) {
        exitg1 = 1;
      } else {
        *alpha *= 0.7;
        k = (uint16_T)WorkingSet_nVar;
        scalarLB_tmp = ((uint16_T)WorkingSet_nVar / 2) << 1;
        vectorUB_tmp = scalarLB_tmp - 2;
        for (idx = 0; idx <= vectorUB_tmp; idx += 2) {
          tmp_0 = _mm_loadu_pd(&TrialState->xstar.data[idx]);
          _mm_storeu_pd(&TrialState->delta_x.data[idx], _mm_mul_pd(_mm_set1_pd
            (*alpha), tmp_0));
        }

        for (idx = scalarLB_tmp; idx < k; idx++) {
          TrialState->delta_x.data[idx] = *alpha * TrialState->xstar.data[idx];
        }

        if (socTaken) {
          phi_alpha = *alpha * *alpha;
          if ((WorkingSet_nVar >= 1) && (!(phi_alpha == 0.0))) {
            scalarLB = (WorkingSet_nVar / 2) << 1;
            vectorUB = scalarLB - 2;
            for (idx = 0; idx <= vectorUB; idx += 2) {
              tmp_0 = _mm_loadu_pd(&TrialState->socDirection.data[idx]);
              tmp = _mm_loadu_pd(&TrialState->delta_x.data[idx]);
              _mm_storeu_pd(&TrialState->delta_x.data[idx], _mm_add_pd
                            (_mm_mul_pd(_mm_set1_pd(phi_alpha), tmp_0), tmp));
            }

            for (idx = scalarLB; idx < WorkingSet_nVar; idx++) {
              TrialState->delta_x.data[idx] += phi_alpha *
                TrialState->socDirection.data[idx];
            }
          }
        }

        tooSmallX = true;
        idx = 0;
        exitg2 = false;
        while ((!exitg2) && (idx <= (uint16_T)WorkingSet_nVar - 1)) {
          if (1.0E-6 * fmax(1.0, fabs(TrialState->xstarsqp[idx])) <= fabs
              (TrialState->delta_x.data[idx])) {
            tooSmallX = false;
            exitg2 = true;
          } else {
            idx++;
          }
        }

        if (tooSmallX) {
          *exitflag = -2;
          exitg1 = 1;
        } else {
          for (idx = 0; idx <= vectorUB_tmp; idx += 2) {
            tmp_0 = _mm_loadu_pd(&TrialState->xstarsqp_old[idx]);
            tmp = _mm_loadu_pd(&TrialState->delta_x.data[idx]);
            _mm_storeu_pd(&TrialState->xstarsqp[idx], _mm_add_pd(tmp_0, tmp));
          }

          for (idx = scalarLB_tmp; idx < k; idx++) {
            TrialState->xstarsqp[idx] = TrialState->xstarsqp_old[idx] +
              TrialState->delta_x.data[idx];
          }

          NMPC_Path_Trac_evalObjAndConstr(FcnEvaluator_next_next_next_nex,
            FcnEvaluator_next_next_next_n_0, FcnEvaluator_next_next_next_n_1,
            TrialState->xstarsqp, TrialState->cIneq.data, TrialState->iNonIneq0,
            TrialState->cEq, &TrialState->sqpFval, &k);
          NMPC_Pat_computeLinearResiduals(TrialState->xstarsqp, WorkingSet_nVar,
            TrialState->cIneq.data, &TrialState->cIneq.size, mLinIneq,
            WorkingSet_Aineq_data, bineq_data, WorkingSet_ldA);
          TrialState->FunctionEvaluations++;
          *evalWellDefined = (k == 1);
          phi_alpha = NMPC_Path_Track_computeMeritFcn(MeritFunction_penaltyParam,
            TrialState->sqpFval, TrialState->cIneq.data, TrialState->mIneq,
            TrialState->cEq, *evalWellDefined);
        }
      }
    } else {
      *exitflag = 0;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_driver(real_T Hessian[160801], const real_T
  bineq_data[], const real_T lb[401], s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T
  *TrialState, sG8JZ69axY52WWR6RKyApQC_NMPC__T *MeritFunction, const
  coder_internal_stickyStruct_2_T *FcnEvaluator, s_BcTDS8pFolHdhtDibl2TnF_NMPC_T
  *memspace, s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T *WorkingSet,
  s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T *QRManager, s_LuoN3prQfsei9XMpifhsFF_NMPC_T
  *CholManager, s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T *QPObjective, const int32_T
  *fscales_lineq_constraint_size, const int32_T *fscales_cineq_constraint_size)
{
  __m128d tmp;
  __m128d tmp_0;
  s7RdrPWkr8UPAUyTdDJkLaG_NMPC__T Flags;
  somzaGboVhDG7PNQS6E98jD_NMPC__T expl_temp;
  real_T TrialState_lambdasqp;
  int32_T d_ix;
  int32_T h;
  int32_T ix;
  int32_T k;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int32_T mConstr;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mLinIneq;
  int32_T mUB;
  int32_T o;
  int32_T u1;
  int32_T vectorUB;
  static const char_T r[7] = { 'f', 'm', 'i', 'n', 'c', 'o', 'n' };

  somzaGboVhDG7PNQS6E98jD_NMPC__T expl_temp_0;
  int32_T d_ix_tmp;
  int32_T h_tmp;
  int32_T nVar_tmp_tmp;
  int32_T y_size_idx_0;
  nVar_tmp_tmp = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mIneq = WorkingSet->sizes[2];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mConstr = (((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes
              [3]) + WorkingSet->sizes[4]) + 300;
  mLinIneq = WorkingSet->sizes[2] - TrialState->mNonlinIneq;
  u1 = ((WorkingSet->sizes[2] + WorkingSet->sizes[3]) + WorkingSet->sizes[4]) +
    (WorkingSet->sizes[0] << 1);
  if (WorkingSet->nVar >= u1) {
    u1 = WorkingSet->nVar;
  }

  u1 *= 10;
  TrialState->steplength = 1.0;
  NMPC_Path_Tracking_test_exit(MeritFunction, WorkingSet, TrialState, lb,
    &Flags.gradOK, &Flags.fevalOK, &Flags.done, &Flags.stepAccepted,
    &Flags.failedLineSearch, &Flags.stepType);
  NMPC_Path_Tracking_saveJacobian(TrialState, WorkingSet->nVar,
    WorkingSet->sizes[2], WorkingSet->Aineq.data, TrialState->iNonIneq0,
    WorkingSet->Aeq.data, WorkingSet->ldA);
  TrialState->sqpFval_old = TrialState->sqpFval;
  for (k = 0; k < 401; k++) {
    TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
    TrialState->grad_old.data[k] = TrialState->grad.data[k];
  }

  d_ix_tmp = TrialState->mIneq;
  loop_ub_0 = TrialState->cIneq_old.size;
  loop_ub = TrialState->cIneq_old.size;
  for (k = 0; k < loop_ub; k++) {
    NMPC_Path_Tracking_B.y_data_p[k] = TrialState->cIneq_old.data[k];
  }

  for (k = 0; k < d_ix_tmp; k++) {
    NMPC_Path_Tracking_B.y_data_p[k] = TrialState->cIneq.data[k];
  }

  for (k = 0; k < loop_ub_0; k++) {
    TrialState->cIneq_old.data[k] = NMPC_Path_Tracking_B.y_data_p[k];
  }

  memcpy(&TrialState->cEq_old[0], &TrialState->cEq[0], 300U * sizeof(real_T));
  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }

  while (!Flags.done) {
    if ((!Flags.stepAccepted) && (!Flags.failedLineSearch)) {
      expl_temp.IterDisplayQP = false;
      expl_temp.RemainFeasible = false;
      expl_temp.ProbRelTolFactor = 1.0;
      expl_temp.ConstrRelTolFactor = 1.0;
      expl_temp.PricingTolerance = 0.0;
      expl_temp.ObjectiveLimit = (rtMinusInf);
      expl_temp.ConstraintTolerance = 1.0E-6;
      expl_temp.OptimalityTolerance = 2.2204460492503131E-14;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = u1;
      for (k = 0; k < 7; k++) {
        expl_temp.SolverName[k] = r[k];
      }
    }

    while ((!Flags.stepAccepted) && (!Flags.failedLineSearch)) {
      if (Flags.stepType != 3) {
        NMPC_P_updateWorkingSetForNewQP(TrialState->xstarsqp, WorkingSet, mIneq,
          TrialState->mNonlinIneq, TrialState->cIneq.data, TrialState->cEq, mLB,
          lb, mUB, mFixed);
      }

      expl_temp_0 = expl_temp;
      NMPC_Path_Tracking_step_c(&Flags, Hessian, lb, TrialState, MeritFunction,
        memspace, WorkingSet, QRManager, CholManager, QPObjective, &expl_temp_0);
      if (Flags.stepAccepted) {
        loop_ub = (uint16_T)nVar_tmp_tmp;
        loop_ub_0 = ((uint16_T)nVar_tmp_tmp / 2) << 1;
        vectorUB = loop_ub_0 - 2;
        for (k = 0; k <= vectorUB; k += 2) {
          tmp = _mm_loadu_pd(&TrialState->xstarsqp[k]);
          tmp_0 = _mm_loadu_pd(&TrialState->delta_x.data[k]);
          _mm_storeu_pd(&TrialState->xstarsqp[k], _mm_add_pd(tmp, tmp_0));
        }

        for (k = loop_ub_0; k < loop_ub; k++) {
          TrialState->xstarsqp[k] += TrialState->delta_x.data[k];
        }

        NMPC_Path_Trac_evalObjAndConstr
          (FcnEvaluator->next.next.next.next.next.b_value,
           &FcnEvaluator->next.next.next.next.next.next.next.b_value.workspace.runtimedata,
           &FcnEvaluator->next.next.next.next.next.next.next.next.b_value.workspace,
           TrialState->xstarsqp, TrialState->cIneq.data, TrialState->iNonIneq0,
           TrialState->cEq, &TrialState->sqpFval, &k);
        Flags.fevalOK = (k == 1);
        TrialState->FunctionEvaluations++;
        NMPC_Pat_computeLinearResiduals(TrialState->xstarsqp, nVar_tmp_tmp,
          TrialState->cIneq.data, &TrialState->cIneq.size, mLinIneq,
          WorkingSet->Aineq.data, bineq_data, WorkingSet->ldA);
        MeritFunction->phiFullStep = NMPC_Path_Track_computeMeritFcn
          (MeritFunction->penaltyParam, TrialState->sqpFval,
           TrialState->cIneq.data, mIneq, TrialState->cEq, Flags.fevalOK);
      }

      if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
          (MeritFunction->phi < MeritFunction->phiFullStep) &&
          (TrialState->sqpFval < TrialState->sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        NMPC_Path_Tracking_linesearch(&Flags.fevalOK, bineq_data,
          WorkingSet->nVar, WorkingSet->ldA, WorkingSet->Aineq.data, TrialState,
          MeritFunction->penaltyParam, MeritFunction->phi,
          MeritFunction->phiPrimePlus, MeritFunction->phiFullStep,
          FcnEvaluator->next.next.next.next.next.b_value,
          &FcnEvaluator->next.next.next.next.next.next.next.b_value.workspace.runtimedata,
          &FcnEvaluator->next.next.next.next.next.next.next.next.b_value.workspace,
          ((Flags.stepType == 3) && Flags.stepAccepted), &TrialState_lambdasqp,
          &k);
        TrialState->steplength = TrialState_lambdasqp;
        if (k > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }

    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      loop_ub = (uint16_T)nVar_tmp_tmp;
      loop_ub_0 = ((uint16_T)nVar_tmp_tmp / 2) << 1;
      vectorUB = loop_ub_0 - 2;
      for (k = 0; k <= vectorUB; k += 2) {
        tmp = _mm_loadu_pd(&TrialState->xstarsqp_old[k]);
        tmp_0 = _mm_loadu_pd(&TrialState->delta_x.data[k]);
        _mm_storeu_pd(&TrialState->xstarsqp[k], _mm_add_pd(tmp, tmp_0));
      }

      for (k = loop_ub_0; k < loop_ub; k++) {
        TrialState->xstarsqp[k] = TrialState->xstarsqp_old[k] +
          TrialState->delta_x.data[k];
      }

      loop_ub = (uint16_T)mConstr;
      loop_ub_0 = ((uint16_T)mConstr / 2) << 1;
      vectorUB = loop_ub_0 - 2;
      for (k = 0; k <= vectorUB; k += 2) {
        tmp = _mm_loadu_pd(&TrialState->lambda.data[k]);
        tmp_0 = _mm_loadu_pd(&TrialState->lambdasqp.data[k]);
        _mm_storeu_pd(&TrialState->lambdasqp.data[k], _mm_add_pd(_mm_mul_pd
          (_mm_sub_pd(tmp, tmp_0), _mm_set1_pd(TrialState->steplength)), tmp_0));
      }

      for (k = loop_ub_0; k < loop_ub; k++) {
        TrialState_lambdasqp = TrialState->lambdasqp.data[k];
        TrialState->lambdasqp.data[k] = (TrialState->lambda.data[k] -
          TrialState_lambdasqp) * TrialState->steplength + TrialState_lambdasqp;
      }

      TrialState->sqpFval_old = TrialState->sqpFval;
      for (k = 0; k < 401; k++) {
        TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
        TrialState->grad_old.data[k] = TrialState->grad.data[k];
      }

      loop_ub_0 = TrialState->cIneq_old.size;
      loop_ub = TrialState->cIneq_old.size;
      for (k = 0; k < loop_ub; k++) {
        NMPC_Path_Tracking_B.y_data_p[k] = TrialState->cIneq_old.data[k];
      }

      for (k = 0; k < d_ix_tmp; k++) {
        NMPC_Path_Tracking_B.y_data_p[k] = TrialState->cIneq.data[k];
      }

      for (k = 0; k < loop_ub_0; k++) {
        TrialState->cIneq_old.data[k] = NMPC_Path_Tracking_B.y_data_p[k];
      }

      memcpy(&TrialState->cEq_old[0], &TrialState->cEq[0], 300U * sizeof(real_T));
      Flags.gradOK = true;
      evalObjAndConstrAndDerivatives
        (FcnEvaluator->next.next.next.next.next.b_value,
         &FcnEvaluator->next.next.next.next.next.next.next.b_value.workspace.runtimedata,
         &FcnEvaluator->next.next.next.next.next.next.next.next.b_value.workspace,
         TrialState->xstarsqp, TrialState->grad.data, TrialState->cIneq.data,
         TrialState->iNonIneq0, TrialState->cEq, WorkingSet->Aineq.data,
         TrialState->iNonIneq0, WorkingSet->ldA, WorkingSet->Aeq.data,
         WorkingSet->ldA, &TrialState->sqpFval, &k);
      TrialState->FunctionEvaluations++;
      Flags.fevalOK = (k == 1);
    } else {
      TrialState->sqpFval = TrialState->sqpFval_old;
      memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0], 401U *
             sizeof(real_T));
      loop_ub_0 = TrialState->cIneq.size;
      loop_ub = TrialState->cIneq.size;
      for (k = 0; k < loop_ub; k++) {
        NMPC_Path_Tracking_B.y_data_p[k] = TrialState->cIneq.data[k];
      }

      for (k = 0; k < d_ix_tmp; k++) {
        NMPC_Path_Tracking_B.y_data_p[k] = TrialState->cIneq_old.data[k];
      }

      for (k = 0; k < loop_ub_0; k++) {
        TrialState->cIneq.data[k] = NMPC_Path_Tracking_B.y_data_p[k];
      }

      memcpy(&TrialState->cEq[0], &TrialState->cEq_old[0], 300U * sizeof(real_T));
    }

    NMPC_Path_Tracking_test_exit_o(&Flags, memspace, MeritFunction,
      fscales_lineq_constraint_size, fscales_cineq_constraint_size, WorkingSet,
      TrialState, QRManager, lb);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      d_ix = (mFixed + TrialState->iNonIneq0) + 299;
      loop_ub = WorkingSet->ldA;
      loop_ub_0 = (uint16_T)nVar_tmp_tmp;
      for (k = 0; k < loop_ub_0; k++) {
        TrialState->delta_gradLag.data[k] = TrialState->grad.data[k];
      }

      y_size_idx_0 = TrialState->delta_gradLag.size;
      loop_ub_0 = TrialState->delta_gradLag.size;
      for (k = 0; k < loop_ub_0; k++) {
        NMPC_Path_Tracking_B.y_data_p[k] = TrialState->delta_gradLag.data[k];
      }

      if (nVar_tmp_tmp >= 1) {
        loop_ub_0 = (nVar_tmp_tmp / 2) << 1;
        vectorUB = loop_ub_0 - 2;
        for (k = 0; k <= vectorUB; k += 2) {
          tmp = _mm_loadu_pd(&NMPC_Path_Tracking_B.y_data_p[k]);
          tmp_0 = _mm_loadu_pd(&TrialState->grad_old.data[k]);
          _mm_storeu_pd(&NMPC_Path_Tracking_B.y_data_p[k], _mm_sub_pd(tmp, tmp_0));
        }

        for (k = loop_ub_0; k < nVar_tmp_tmp; k++) {
          NMPC_Path_Tracking_B.y_data_p[k] -= TrialState->grad_old.data[k];
        }
      }

      ix = mFixed;
      for (k = 0; k < y_size_idx_0; k++) {
        TrialState->delta_gradLag.data[k] = NMPC_Path_Tracking_B.y_data_p[k];
      }

      y_size_idx_0 = WorkingSet->ldA * 299 + 1;
      for (loop_ub_0 = 1; loop_ub < 0 ? loop_ub_0 >= y_size_idx_0 : loop_ub_0 <=
           y_size_idx_0; loop_ub_0 += loop_ub) {
        h = (loop_ub_0 + nVar_tmp_tmp) - 1;
        for (vectorUB = loop_ub_0; vectorUB <= h; vectorUB++) {
          k = vectorUB - loop_ub_0;
          TrialState->delta_gradLag.data[k] += WorkingSet->Aeq.data[vectorUB - 1]
            * TrialState->lambdasqp.data[ix];
        }

        ix++;
      }

      ix = mFixed;
      for (loop_ub_0 = 1; loop_ub < 0 ? loop_ub_0 >= y_size_idx_0 : loop_ub_0 <=
           y_size_idx_0; loop_ub_0 += loop_ub) {
        h = (loop_ub_0 + nVar_tmp_tmp) - 1;
        for (vectorUB = loop_ub_0; vectorUB <= h; vectorUB++) {
          k = vectorUB - loop_ub_0;
          TrialState->delta_gradLag.data[k] += TrialState->
            JacCeqTrans_old.data[vectorUB - 1] * -TrialState->lambdasqp.data[ix];
        }

        ix++;
      }

      if (TrialState->mNonlinIneq > 0) {
        y_size_idx_0 = (TrialState->iNonIneq0 - 1) * WorkingSet->ldA + 1;
        ix = d_ix;
        h_tmp = (TrialState->mNonlinIneq - 1) * WorkingSet->ldA;
        h = h_tmp + y_size_idx_0;
        for (loop_ub_0 = y_size_idx_0; loop_ub < 0 ? loop_ub_0 >= h : loop_ub_0 <=
             h; loop_ub_0 += loop_ub) {
          o = (loop_ub_0 + nVar_tmp_tmp) - 1;
          for (vectorUB = loop_ub_0; vectorUB <= o; vectorUB++) {
            k = vectorUB - loop_ub_0;
            TrialState->delta_gradLag.data[k] += WorkingSet->Aineq.data[vectorUB
              - 1] * TrialState->lambdasqp.data[ix];
          }

          ix++;
        }

        y_size_idx_0 = h_tmp + 1;
        for (loop_ub_0 = 1; loop_ub < 0 ? loop_ub_0 >= y_size_idx_0 : loop_ub_0 <=
             y_size_idx_0; loop_ub_0 += loop_ub) {
          ix = (loop_ub_0 + nVar_tmp_tmp) - 1;
          for (vectorUB = loop_ub_0; vectorUB <= ix; vectorUB++) {
            k = vectorUB - loop_ub_0;
            TrialState->delta_gradLag.data[k] +=
              TrialState->JacCineqTrans_old.data[vectorUB - 1] *
              -TrialState->lambdasqp.data[d_ix];
          }

          d_ix++;
        }
      }

      NMPC_Path_Tracking_saveJacobian(TrialState, nVar_tmp_tmp, mIneq,
        WorkingSet->Aineq.data, TrialState->iNonIneq0, WorkingSet->Aeq.data,
        WorkingSet->ldA);
      NMPC_Path_Tracking_BFGSUpdate(nVar_tmp_tmp, Hessian,
        TrialState->delta_x.data, TrialState->delta_gradLag.data,
        memspace->workspace_float.data);
      TrialState->sqpIterations++;
    }
  }
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_fmincon(const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T
  *fun_workspace_runtimedata, const s02qEHDaRbOUcZF4Nctxs8G_NMPC__T
  *fun_workspace_userdata, const real_T x0[401], const real_T Aineq_data[],
  const real_T bineq_data[], const int32_T *bineq_size, const real_T lb[401],
  const s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T *nonlcon_workspace_runtimedata, real_T
  x[401], real_T *fval, real_T *exitflag, sttYSJM5GCi2c1Eu0R50efC_NMPC__T
  *output)
{
  __m128d tmp;
  sG8JZ69axY52WWR6RKyApQC_NMPC__T MeritFunction;
  real_T Ceq[300];
  real_T absxk;
  real_T b_c;
  real_T scale;
  real_T t;
  int32_T Cineq_size[2];
  int32_T JacCineqTrans_size[2];
  int32_T b_iy;
  int32_T i;
  int32_T iEq0;
  int32_T i_k;
  int32_T k;
  int32_T mConstrMax;
  int32_T mIneq;
  int32_T mLinIneq_tmp;
  int32_T maxDims;
  int16_T WorkingSet_tmp[5];
  int16_T WorkingSet_tmp_0;
  NMPC_Path_Tr_c4_mpclib_anonFcn2(nonlcon_workspace_runtimedata->x,
    nonlcon_workspace_runtimedata->OutputMin,
    nonlcon_workspace_runtimedata->OutputMax, x0,
    NMPC_Path_Tracking_B.Cineq_data, Cineq_size, Ceq,
    NMPC_Path_Tracking_B.JacCineqTrans_data, JacCineqTrans_size,
    NMPC_Path_Tracking_B.JacCeqTrans);
  i_k = Cineq_size[0] * Cineq_size[1];
  mLinIneq_tmp = *bineq_size;
  mIneq = *bineq_size + i_k;
  mConstrMax = (mIneq + mIneq) + 1703;
  if (mIneq + 1002 >= mConstrMax) {
    maxDims = mIneq + 1002;
  } else {
    maxDims = mConstrMax;
  }

  memset(&NMPC_Path_Tracking_B.Hessian[0], 0, 160801U * sizeof(real_T));
  NMPC_Path_Trac_factoryConstruct(mIneq + 1002, mConstrMax, mIneq, i_k,
    &NMPC_Path_Tracking_B.TrialState);
  for (k = 0; k < 401; k++) {
    NMPC_Path_Tracking_B.Hessian[k + 401 * k] = 1.0;
    NMPC_Path_Tracking_B.TrialState.xstarsqp[k] = x0[k];
  }

  NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.b_value = i_k;
  for (i = 0; i < 6; i++) {
    NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.x
      [i] = nonlcon_workspace_runtimedata->x[i];
  }

  NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.lastMV
    [0] = nonlcon_workspace_runtimedata->lastMV[0];
  NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.lastMV
    [1] = nonlcon_workspace_runtimedata->lastMV[1];
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.ref
     [0], &nonlcon_workspace_runtimedata->ref[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.OutputWeights
     [0], &nonlcon_workspace_runtimedata->OutputWeights[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVWeights
     [0], &nonlcon_workspace_runtimedata->MVWeights[0], 100U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVRateWeights
     [0], &nonlcon_workspace_runtimedata->MVRateWeights[0], 100U * sizeof(real_T));
  NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.ECRWeight
    = nonlcon_workspace_runtimedata->ECRWeight;
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.OutputMin
     [0], &nonlcon_workspace_runtimedata->OutputMin[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.OutputMax
     [0], &nonlcon_workspace_runtimedata->OutputMax[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.StateMin
     [0], &nonlcon_workspace_runtimedata->StateMin[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.StateMax
     [0], &nonlcon_workspace_runtimedata->StateMax[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVMin
     [0], &nonlcon_workspace_runtimedata->MVMin[0], 100U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVMax
     [0], &nonlcon_workspace_runtimedata->MVMax[0], 100U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVRateMin
     [0], &nonlcon_workspace_runtimedata->MVRateMin[0], 100U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVRateMax
     [0], &nonlcon_workspace_runtimedata->MVRateMax[0], 100U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVScaledTarget
     [0], &nonlcon_workspace_runtimedata->MVScaledTarget[0], 100U * sizeof
     (real_T));
  for (i = 0; i < 6; i++) {
    NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.x
      [i] = fun_workspace_runtimedata->x[i];
  }

  NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.lastMV
    [0] = fun_workspace_runtimedata->lastMV[0];
  NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.lastMV
    [1] = fun_workspace_runtimedata->lastMV[1];
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.ref
     [0], &fun_workspace_runtimedata->ref[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.OutputWeights
     [0], &fun_workspace_runtimedata->OutputWeights[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVWeights
     [0], &fun_workspace_runtimedata->MVWeights[0], 100U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVRateWeights
     [0], &fun_workspace_runtimedata->MVRateWeights[0], 100U * sizeof(real_T));
  NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.ECRWeight
    = fun_workspace_runtimedata->ECRWeight;
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.OutputMin
     [0], &fun_workspace_runtimedata->OutputMin[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.OutputMax
     [0], &fun_workspace_runtimedata->OutputMax[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.StateMin
     [0], &fun_workspace_runtimedata->StateMin[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.StateMax
     [0], &fun_workspace_runtimedata->StateMax[0], 300U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVMin
     [0], &fun_workspace_runtimedata->MVMin[0], 100U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVMax
     [0], &fun_workspace_runtimedata->MVMax[0], 100U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVRateMin
     [0], &fun_workspace_runtimedata->MVRateMin[0], 100U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVRateMax
     [0], &fun_workspace_runtimedata->MVRateMax[0], 100U * sizeof(real_T));
  memcpy
    (&NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.runtimedata.MVScaledTarget
     [0], &fun_workspace_runtimedata->MVScaledTarget[0], 100U * sizeof(real_T));
  NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace.userdata
    = *fun_workspace_userdata;
  NMPC_Path_factoryConstruct_gzst(mIneq + 1002,
    &NMPC_Path_Tracking_B.QPObjective.grad.size,
    &NMPC_Path_Tracking_B.QPObjective.Hx.size,
    &NMPC_Path_Tracking_B.QPObjective.hasLinear,
    &NMPC_Path_Tracking_B.QPObjective.nvar,
    &NMPC_Path_Tracking_B.QPObjective.maxVar,
    &NMPC_Path_Tracking_B.QPObjective.beta,
    &NMPC_Path_Tracking_B.QPObjective.rho,
    &NMPC_Path_Tracking_B.QPObjective.objtype,
    &NMPC_Path_Tracking_B.QPObjective.prev_objtype,
    &NMPC_Path_Tracking_B.QPObjective.prev_nvar,
    &NMPC_Path_Tracking_B.QPObjective.prev_hasLinear,
    &NMPC_Path_Tracking_B.QPObjective.gammaScalar);
  NMPC_Path_Tracking_B.QPObjective.nvar = 401;
  NMPC_Path_Tracking_B.QPObjective.hasLinear = true;
  NMPC_Path_Tracking_B.QPObjective.objtype = 3;
  NMPC_Path_Tracking_B.memspace.workspace_float.size[0] = maxDims;
  NMPC_Path_Tracking_B.memspace.workspace_float.size[1] = mIneq + 1002;
  NMPC_Path_Tracking_B.memspace.workspace_int.size = maxDims;
  NMPC_Path_Tracking_B.memspace.workspace_sort.size = maxDims;
  NMPC_Pat_factoryConstruct_gzstv(mIneq, mIneq + 1002, mConstrMax,
    &NMPC_Path_Tracking_B.WorkingSet);
  k = -1;
  for (i = 0; i < 401; i++) {
    b_c = lb[i];
    if ((!rtIsInf(b_c)) && (!rtIsNaN(b_c))) {
      k++;
      NMPC_Path_Tracking_B.WorkingSet.indexLB.data[k] = i + 1;
    }
  }

  NMPC_Path_Tracking_B.WorkingSet.mConstrMax = mConstrMax;
  mConstrMax = mIneq + k;
  NMPC_Path_Tracking_B.WorkingSet.mConstr = mConstrMax + 301;
  NMPC_Path_Tracking_B.WorkingSet.mConstrOrig = mConstrMax + 301;
  WorkingSet_tmp[0] = 0;
  WorkingSet_tmp[1] = 300;
  WorkingSet_tmp[2] = (int16_T)mIneq;
  WorkingSet_tmp[3] = (int16_T)(k + 1);
  WorkingSet_tmp[4] = 0;
  for (i = 0; i < 5; i++) {
    WorkingSet_tmp_0 = WorkingSet_tmp[i];
    NMPC_Path_Tracking_B.WorkingSet.sizes[i] = WorkingSet_tmp_0;
    NMPC_Path_Tracking_B.WorkingSet.sizesNormal[i] = WorkingSet_tmp_0;
  }

  NMPC_Path_Tracking_B.WorkingSet.sizesPhaseOne[0] = 0;
  NMPC_Path_Tracking_B.WorkingSet.sizesPhaseOne[1] = 300;
  NMPC_Path_Tracking_B.WorkingSet.sizesPhaseOne[2] = mIneq;
  NMPC_Path_Tracking_B.WorkingSet.sizesPhaseOne[3] = k + 2;
  NMPC_Path_Tracking_B.WorkingSet.sizesPhaseOne[4] = 0;
  NMPC_Path_Tracking_B.WorkingSet.sizesRegularized[0] = 0;
  NMPC_Path_Tracking_B.WorkingSet.sizesRegularized[1] = 300;
  NMPC_Path_Tracking_B.WorkingSet.sizesRegularized[2] = mIneq;
  NMPC_Path_Tracking_B.WorkingSet.sizesRegularized[3] = mConstrMax + 601;
  NMPC_Path_Tracking_B.WorkingSet.sizesRegularized[4] = 0;
  NMPC_Path_Tracking_B.WorkingSet.sizesRegPhaseOne[0] = 0;
  NMPC_Path_Tracking_B.WorkingSet.sizesRegPhaseOne[1] = 300;
  NMPC_Path_Tracking_B.WorkingSet.sizesRegPhaseOne[2] = mIneq;
  NMPC_Path_Tracking_B.WorkingSet.sizesRegPhaseOne[3] = mConstrMax + 602;
  NMPC_Path_Tracking_B.WorkingSet.sizesRegPhaseOne[4] = 0;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxNormal[0] = 1;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxNormal[1] = 0;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxNormal[2] = 300;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxNormal[3] = mIneq;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxNormal[4] = k + 1;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxNormal[5] = 0;
  for (iEq0 = 0; iEq0 < 6; iEq0++) {
    NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[iEq0] =
      NMPC_Path_Tracking_B.WorkingSet.isActiveIdxNormal[iEq0];
  }

  for (i = 0; i < 5; i++) {
    NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
      NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[i];
  }

  for (iEq0 = 0; iEq0 < 6; iEq0++) {
    NMPC_Path_Tracking_B.WorkingSet.isActiveIdx[iEq0] =
      NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[iEq0];
  }

  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[1] = 0;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[2] = 300;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[4] = k + 2;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[5] = 0;
  for (i = 0; i < 5; i++) {
    NMPC_Path_Tracking_B.WorkingSet.isActiveIdxNormal[i + 1] +=
      NMPC_Path_Tracking_B.WorkingSet.isActiveIdxNormal[i];
    NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
      NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[i];
  }

  for (iEq0 = 0; iEq0 < 6; iEq0++) {
    NMPC_Path_Tracking_B.WorkingSet.isActiveIdxPhaseOne[iEq0] =
      NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[iEq0];
  }

  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[1] = 0;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[2] = 300;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[4] = mConstrMax + 601;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[5] = 0;
  for (i = 0; i < 5; i++) {
    NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
      NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[i];
  }

  for (iEq0 = 0; iEq0 < 6; iEq0++) {
    NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegularized[iEq0] =
      NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[iEq0];
  }

  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[1] = 0;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[2] = 300;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[4] = mConstrMax + 602;
  NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[5] = 0;
  for (i = 0; i < 5; i++) {
    NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
      NMPC_Path_Tracking_B.WorkingSet.isActiveIdxRegPhaseOne[i];
  }

  if (mIneq > 0) {
    for (i = 0; i < mLinIneq_tmp; i++) {
      for (mConstrMax = 0; mConstrMax < 401; mConstrMax++) {
        NMPC_Path_Tracking_B.WorkingSet.Aineq.data[mConstrMax +
          NMPC_Path_Tracking_B.WorkingSet.ldA * i] = Aineq_data[*bineq_size *
          mConstrMax + i];
      }
    }
  }

  for (i = 0; i <= k; i++) {
    mConstrMax = NMPC_Path_Tracking_B.WorkingSet.indexLB.data[i];
    NMPC_Path_Tracking_B.TrialState.xstarsqp[mConstrMax - 1] = fmax
      (NMPC_Path_Tracking_B.TrialState.xstarsqp[mConstrMax - 1], lb[mConstrMax -
       1]);
  }

  evalObjAndConstrAndDerivatives(i_k,
    &NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.b_value.workspace.runtimedata,
    &NMPC_Path_Tracking_B.FcnEvaluator.next.next.next.next.next.next.next.next.b_value.workspace,
    NMPC_Path_Tracking_B.TrialState.xstarsqp,
    NMPC_Path_Tracking_B.TrialState.grad.data,
    NMPC_Path_Tracking_B.TrialState.cIneq.data,
    NMPC_Path_Tracking_B.TrialState.iNonIneq0,
    NMPC_Path_Tracking_B.TrialState.cEq,
    NMPC_Path_Tracking_B.WorkingSet.Aineq.data,
    NMPC_Path_Tracking_B.TrialState.iNonIneq0,
    NMPC_Path_Tracking_B.WorkingSet.ldA,
    NMPC_Path_Tracking_B.WorkingSet.Aeq.data,
    NMPC_Path_Tracking_B.WorkingSet.ldA,
    &NMPC_Path_Tracking_B.TrialState.sqpFval, &i);
  NMPC_Path_Tracking_B.TrialState.FunctionEvaluations = 1;
  i = NMPC_Path_Tracking_B.WorkingSet.ldA;
  if (*bineq_size > 0) {
    mConstrMax = NMPC_Path_Tracking_B.TrialState.cIneq.size;
    for (iEq0 = 0; iEq0 < mConstrMax; iEq0++) {
      NMPC_Path_Tracking_B.y_data_b[iEq0] =
        NMPC_Path_Tracking_B.TrialState.cIneq.data[iEq0];
    }

    for (iEq0 = 0; iEq0 < mLinIneq_tmp; iEq0++) {
      NMPC_Path_Tracking_B.y_data_b[iEq0] = bineq_data[iEq0];
    }

    for (iEq0 = 0; iEq0 < mConstrMax; iEq0++) {
      NMPC_Path_Tracking_B.TrialState.cIneq.data[iEq0] =
        NMPC_Path_Tracking_B.y_data_b[iEq0];
    }

    mConstrMax = (*bineq_size / 2) << 1;
    iEq0 = mConstrMax - 2;
    for (b_iy = 0; b_iy <= iEq0; b_iy += 2) {
      tmp = _mm_loadu_pd(&NMPC_Path_Tracking_B.TrialState.cIneq.data[b_iy]);
      _mm_storeu_pd(&NMPC_Path_Tracking_B.TrialState.cIneq.data[b_iy],
                    _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
    }

    for (b_iy = mConstrMax; b_iy < mLinIneq_tmp; b_iy++) {
      NMPC_Path_Tracking_B.TrialState.cIneq.data[b_iy] =
        -NMPC_Path_Tracking_B.TrialState.cIneq.data[b_iy];
    }

    mConstrMax = 0;
    iEq0 = (*bineq_size - 1) * NMPC_Path_Tracking_B.WorkingSet.ldA + 1;
    for (b_iy = 1; i < 0 ? b_iy >= iEq0 : b_iy <= iEq0; b_iy += i) {
      b_c = 0.0;
      for (mLinIneq_tmp = b_iy; mLinIneq_tmp <= b_iy + 400; mLinIneq_tmp++) {
        b_c += NMPC_Path_Tracking_B.WorkingSet.Aineq.data[mLinIneq_tmp - 1] *
          NMPC_Path_Tracking_B.TrialState.xstarsqp[mLinIneq_tmp - b_iy];
      }

      NMPC_Path_Tracking_B.TrialState.cIneq.data[mConstrMax] += b_c;
      mConstrMax++;
    }
  }

  mConstrMax = 0;
  iEq0 = 0;
  for (b_iy = 0; b_iy < 300; b_iy++) {
    b_c = -NMPC_Path_Tracking_B.TrialState.cEq[b_iy];
    NMPC_Path_Tracking_B.WorkingSet.beq[b_iy] = b_c;
    NMPC_Path_Tracking_B.WorkingSet.bwset.data[b_iy] = b_c;
    memcpy(&NMPC_Path_Tracking_B.WorkingSet.ATwset.data[mConstrMax],
           &NMPC_Path_Tracking_B.WorkingSet.Aeq.data[iEq0], 401U * sizeof(real_T));
    mConstrMax += NMPC_Path_Tracking_B.WorkingSet.ldA;
    iEq0 = mConstrMax;
  }

  mConstrMax = (mIneq / 2) << 1;
  iEq0 = mConstrMax - 2;
  for (i = 0; i <= iEq0; i += 2) {
    tmp = _mm_loadu_pd(&NMPC_Path_Tracking_B.TrialState.cIneq.data[i]);
    _mm_storeu_pd(&NMPC_Path_Tracking_B.WorkingSet.bineq.data[i], _mm_mul_pd(tmp,
      _mm_set1_pd(-1.0)));
  }

  for (i = mConstrMax; i < mIneq; i++) {
    NMPC_Path_Tracking_B.WorkingSet.bineq.data[i] =
      -NMPC_Path_Tracking_B.TrialState.cIneq.data[i];
  }

  for (i = 0; i <= k; i++) {
    NMPC_Path_Tracking_B.WorkingSet.lb.data[NMPC_Path_Tracking_B.WorkingSet.indexLB.data
      [i] - 1] = -lb[NMPC_Path_Tracking_B.WorkingSet.indexLB.data[i] - 1] +
      x0[NMPC_Path_Tracking_B.WorkingSet.indexLB.data[i] - 1];
  }

  NMPC_Path_Trackin_initActiveSet(&NMPC_Path_Tracking_B.WorkingSet);
  MeritFunction.initFval = NMPC_Path_Tracking_B.TrialState.sqpFval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  b_c = 0.0;
  for (k = 0; k < 300; k++) {
    b_c += fabs(NMPC_Path_Tracking_B.TrialState.cEq[k]);
  }

  MeritFunction.initConstrViolationEq = b_c;
  b_c = 0.0;
  for (k = 0; k < mIneq; k++) {
    scale = NMPC_Path_Tracking_B.TrialState.cIneq.data[k];
    if (scale > 0.0) {
      b_c += scale;
    }
  }

  MeritFunction.initConstrViolationIneq = b_c;
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpPrimalFeasError = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  MeritFunction.firstOrderOpt = 0.0;
  NMPC_Path_T_factoryConstruct_gz(maxDims, maxDims,
    &NMPC_Path_Tracking_B.QRManager.ldq, NMPC_Path_Tracking_B.QRManager.QR.size,
    NMPC_Path_Tracking_B.QRManager.Q.data, NMPC_Path_Tracking_B.QRManager.Q.size,
    NMPC_Path_Tracking_B.QRManager.jpvt.data,
    &NMPC_Path_Tracking_B.QRManager.jpvt.size,
    &NMPC_Path_Tracking_B.QRManager.mrows, &NMPC_Path_Tracking_B.QRManager.ncols,
    &NMPC_Path_Tracking_B.QRManager.tau.size,
    &NMPC_Path_Tracking_B.QRManager.minRowCol,
    &NMPC_Path_Tracking_B.QRManager.usedPivoting);
  NMPC_Path__factoryConstruct_gzs(maxDims,
    NMPC_Path_Tracking_B.CholManager.FMat.size,
    &NMPC_Path_Tracking_B.CholManager.ldm,
    &NMPC_Path_Tracking_B.CholManager.ndims,
    &NMPC_Path_Tracking_B.CholManager.info,
    &NMPC_Path_Tracking_B.CholManager.scaleFactor,
    &NMPC_Path_Tracking_B.CholManager.ConvexCheck,
    &NMPC_Path_Tracking_B.CholManager.regTol_,
    &NMPC_Path_Tracking_B.CholManager.workspace_,
    &NMPC_Path_Tracking_B.CholManager.workspace2_);
  NMPC_Path_Tracking_driver(NMPC_Path_Tracking_B.Hessian, bineq_data, lb,
    &NMPC_Path_Tracking_B.TrialState, &MeritFunction,
    &NMPC_Path_Tracking_B.FcnEvaluator, &NMPC_Path_Tracking_B.memspace,
    &NMPC_Path_Tracking_B.WorkingSet, &NMPC_Path_Tracking_B.QRManager,
    &NMPC_Path_Tracking_B.CholManager, &NMPC_Path_Tracking_B.QPObjective,
    bineq_size, &i_k);
  *fval = NMPC_Path_Tracking_B.TrialState.sqpFval;
  *exitflag = NMPC_Path_Tracking_B.TrialState.sqpExitFlag;
  output->iterations = NMPC_Path_Tracking_B.TrialState.sqpIterations;
  output->funcCount = NMPC_Path_Tracking_B.TrialState.FunctionEvaluations;
  output->algorithm[0] = 's';
  output->algorithm[1] = 'q';
  output->algorithm[2] = 'p';
  output->constrviolation = MeritFunction.nlpPrimalFeasError;
  b_c = 0.0;
  scale = 3.3121686421112381E-170;
  for (i_k = 0; i_k < 401; i_k++) {
    x[i_k] = NMPC_Path_Tracking_B.TrialState.xstarsqp[i_k];
    absxk = fabs(NMPC_Path_Tracking_B.TrialState.delta_x.data[i_k]);
    if (absxk > scale) {
      t = scale / absxk;
      b_c = b_c * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      b_c += t * t;
    }
  }

  output->stepsize = scale * sqrt(b_c);
  output->lssteplength = NMPC_Path_Tracking_B.TrialState.steplength;
  output->firstorderopt = MeritFunction.firstOrderOpt;
}

/* Function for MATLAB Function: '<S6>/NLMPC' */
static void NMPC_Path_Tracking_computeInfo(const real_T X[306], const real_T U
  [102], real_T e, real_T cost, real_T ExitFlag, real_T iter, real_T X0[300],
  real_T MV0[100], real_T *Slack0, stwyLkKtGfiNF3i9PDxcjkC_NMPC__T *info)
{
  int32_T i;
  int32_T k;
  static const int8_T b[50] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34,
    35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50 };

  for (k = 0; k < 2; k++) {
    memcpy(&MV0[k * 50], &U[k * 51 + 1], 50U * sizeof(real_T));
  }

  for (k = 0; k < 6; k++) {
    for (i = 0; i <= 48; i += 2) {
      int32_T X0_tmp;
      X0_tmp = 50 * k + i;
      X0[X0_tmp] = X[51 * k + b[i]];
      X0[X0_tmp + 1] = X[51 * k + b[i + 1]];
    }
  }

  *Slack0 = fmax(0.0, e);
  for (k = 0; k < 51; k++) {
    info->Yopt[k] = X[k];
    info->Yopt[k + 51] = X[k + 51];
    info->Yopt[k + 102] = X[k + 102];
    info->Yopt[k + 153] = X[k + 153];
    info->Yopt[k + 204] = X[k + 204];
    info->Yopt[k + 255] = X[k + 255];
  }

  memcpy(&info->MVopt[0], &U[0], 102U * sizeof(real_T));
  memcpy(&info->Xopt[0], &X[0], 306U * sizeof(real_T));
  for (k = 0; k < 51; k++) {
    info->Topt[k] = 0.01 * (real_T)k;
  }

  info->Slack = e;
  info->ExitFlag = ExitFlag;
  info->Iterations = iter;
  info->Cost = cost;
}

/* Model step function */
void NMPC_Path_Tracking_step(void)
{
  __m128d tmp_0;
  __m128d tmp_1;
  s02qEHDaRbOUcZF4Nctxs8G_NMPC__T userdata;
  sttYSJM5GCi2c1Eu0R50efC_NMPC__T Out;
  real_T z0[401];
  real_T zLB[401];
  real_T zUB[401];
  real_T B_data[400];
  real_T rtb_x_Delay[306];
  real_T a__1[300];
  real_T tmp[294];
  real_T U[102];
  real_T x[100];
  real_T rtb_x_Delay_0[98];
  real_T rtb_ref_traj[60];
  real_T rtb_Reshape5[6];
  real_T a__3;
  real_T current_position;
  real_T e;
  real_T v_ref;
  int32_T A_size[2];
  int32_T d_k;
  int32_T e_k;
  int32_T i;
  int32_T selected_indices_size_idx_1;
  uint8_T selected_indices_data[100];
  boolean_T exitg1;

  /* Outputs for Atomic SubSystem: '<Root>/NMPC_Path_Tracking' */
  /* Reshape: '<S1>/Reshape5' incorporates:
   *  Inport: '<Root>/current_position'
   *  Inport: '<Root>/vx'
   *  Inport: '<Root>/vy'
   *  Inport: '<Root>/yaw'
   *  Inport: '<Root>/yaw_rate'
   *  SignalConversion generated from: '<S1>/current_position'
   */
  rtb_Reshape5[0] = NMPC_Path_Tracking_U.current_position[0];
  rtb_Reshape5[1] = NMPC_Path_Tracking_U.current_position[1];
  rtb_Reshape5[2] = NMPC_Path_Tracking_U.psi;
  rtb_Reshape5[3] = NMPC_Path_Tracking_U.vx;
  rtb_Reshape5[4] = NMPC_Path_Tracking_U.vy;
  rtb_Reshape5[5] = NMPC_Path_Tracking_U.yaw_rate;

  /* MATLAB Function: '<S1>/MATLAB Function2' */
  memset(&rtb_ref_traj[0], 0, 60U * sizeof(real_T));

  /* SignalConversion generated from: '<S1>/current_position' incorporates:
   *  Inport: '<Root>/current_position'
   *  MATLAB Function: '<S1>/MATLAB Function2'
   */
  current_position = NMPC_Path_Tracking_U.current_position[0];
  v_ref = NMPC_Path_Tracking_U.current_position[1];

  /* MATLAB Function: '<S1>/MATLAB Function2' incorporates:
   *  Inport: '<Root>/PredictionHorizon'
   *  Inport: '<Root>/current_position'
   *  Inport: '<Root>/curvature'
   *  Inport: '<Root>/v_max'
   *  Inport: '<Root>/x_path'
   *  Inport: '<Root>/y_path'
   *  SignalConversion generated from: '<S1>/current_position'
   */
  for (i = 0; i <= 98; i += 2) {
    tmp_0 = _mm_sub_pd(_mm_loadu_pd(&NMPC_Path_Tracking_U.x_path[i]),
                       _mm_set1_pd(current_position));
    tmp_1 = _mm_sub_pd(_mm_loadu_pd(&NMPC_Path_Tracking_U.y_path[i]),
                       _mm_set1_pd(v_ref));
    _mm_storeu_pd(&NMPC_Path_Tracking_Y.distances[i], _mm_add_pd(_mm_mul_pd
      (tmp_1, tmp_1), _mm_mul_pd(tmp_0, tmp_0)));
    tmp_0 = _mm_loadu_pd(&NMPC_Path_Tracking_Y.distances[i]);
    _mm_storeu_pd(&NMPC_Path_Tracking_Y.distances[i], _mm_sqrt_pd(tmp_0));
  }

  if (!rtIsNaN(NMPC_Path_Tracking_Y.distances[0])) {
    i = 1;
  } else {
    i = 0;
    d_k = 2;
    exitg1 = false;
    while ((!exitg1) && (d_k < 101)) {
      if (!rtIsNaN(NMPC_Path_Tracking_Y.distances[d_k - 1])) {
        i = d_k;
        exitg1 = true;
      } else {
        d_k++;
      }
    }
  }

  if (i == 0) {
    d_k = 1;
  } else {
    current_position = NMPC_Path_Tracking_Y.distances[i - 1];
    d_k = i;
    for (e_k = i + 1; e_k < 101; e_k++) {
      v_ref = NMPC_Path_Tracking_Y.distances[e_k - 1];
      if (current_position > v_ref) {
        current_position = v_ref;
        d_k = e_k;
      }
    }
  }

  current_position = fmin(100.0, ((real_T)d_k +
    NMPC_Path_Tracking_U.PredictionHorizon) - 1.0);
  if (current_position < d_k) {
    selected_indices_size_idx_1 = 0;
  } else {
    e_k = (int32_T)(current_position - (real_T)d_k);
    selected_indices_size_idx_1 = e_k + 1;
    for (i = 0; i <= e_k; i++) {
      selected_indices_data[i] = (uint8_T)(d_k + i);
    }
  }

  for (i = 0; i < selected_indices_size_idx_1; i++) {
    e_k = selected_indices_data[i];
    rtb_ref_traj[i] = NMPC_Path_Tracking_U.x_path[e_k - 1];
    rtb_ref_traj[i + 10] = NMPC_Path_Tracking_U.y_path[e_k - 1];
  }

  for (i = 0; i < selected_indices_size_idx_1; i++) {
    current_position = NMPC_Path_Tracking_U.curvature[selected_indices_data[i] -
      1];
    v_ref = fabs(current_position);
    if (v_ref <= 0.01) {
      v_ref = NMPC_Path_Tracking_U.v_max;
    } else {
      v_ref = fmin(NMPC_Path_Tracking_U.v_max, sqrt(1.0 / v_ref * 9.81 * 0.2));
    }

    rtb_ref_traj[i + 30] = v_ref;
    if (i + 1 < selected_indices_size_idx_1) {
      e_k = selected_indices_data[i];
      d_k = selected_indices_data[i + 1];
      rtb_ref_traj[i + 20] = rt_atan2d_snf(NMPC_Path_Tracking_U.y_path[d_k - 1]
        - NMPC_Path_Tracking_U.y_path[e_k - 1], NMPC_Path_Tracking_U.x_path[d_k
        - 1] - NMPC_Path_Tracking_U.x_path[e_k - 1]);
    } else {
      rtb_ref_traj[i + 20] = rtb_ref_traj[i + 19];
    }

    rtb_ref_traj[i + 40] = 0.0;
    rtb_ref_traj[i + 50] = current_position * v_ref;
  }

  /* Delay: '<S7>/mv_Delay' incorporates:
   *  Constant: '<S7>/ones'
   *  Delay: '<S1>/Delay'
   *  Product: '<S7>/Product'
   */
  if (NMPC_Path_Tracking_DW.icLoad) {
    for (i = 0; i < 2; i++) {
      for (d_k = 0; d_k < 51; d_k++) {
        NMPC_Path_Tracking_DW.mv_Delay_DSTATE[d_k + 51 * i] =
          NMPC_Path_Tracking_DW.Delay_DSTATE[i];
      }
    }
  }

  memcpy(&rtb_x_Delay[0], &NMPC_Path_Tracking_DW.mv_Delay_DSTATE[0], 102U *
         sizeof(real_T));

  /* End of Delay: '<S7>/mv_Delay' */

  /* Delay: '<S7>/x_Delay' incorporates:
   *  Constant: '<S7>/ones'
   *  Product: '<S7>/Product1'
   *  Reshape: '<S1>/Reshape5'
   */
  if (NMPC_Path_Tracking_DW.icLoad_c) {
    for (i = 0; i < 6; i++) {
      for (d_k = 0; d_k < 51; d_k++) {
        NMPC_Path_Tracking_DW.x_Delay_DSTATE[d_k + 51 * i] = rtb_Reshape5[i];
      }
    }
  }

  /* Delay: '<S7>/slack_delay' incorporates:
   *  Constant: '<S5>/e.init_zero'
   */
  if (NMPC_Path_Tracking_DW.icLoad_p) {
    NMPC_Path_Tracking_DW.slack_delay_DSTATE = 0.0;
  }

  /* Selector: '<S7>/Selector' incorporates:
   *  Constant: '<S7>/Constant'
   *  Delay: '<S7>/x_Delay'
   */
  for (i = 0; i < 6; i++) {
    for (d_k = 0; d_k <= 46; d_k += 2) {
      e_k = 49 * i + d_k;
      tmp[e_k] = NMPC_Path_Tracking_DW.x_Delay_DSTATE[(51 * i + (int32_T)
        NMPC_Path_Tracking_ConstP.Constant_Value[d_k]) - 1];
      tmp[e_k + 1] = NMPC_Path_Tracking_DW.x_Delay_DSTATE[(51 * i + (int32_T)
        NMPC_Path_Tracking_ConstP.Constant_Value[d_k + 1]) - 1];
    }

    for (d_k = 48; d_k < 49; d_k++) {
      tmp[d_k + 49 * i] = NMPC_Path_Tracking_DW.x_Delay_DSTATE[(51 * i +
        (int32_T)NMPC_Path_Tracking_ConstP.Constant_Value[d_k]) - 1];
    }
  }

  /* End of Selector: '<S7>/Selector' */

  /* Selector: '<S7>/Selector1' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  for (i = 0; i < 2; i++) {
    for (d_k = 0; d_k <= 46; d_k += 2) {
      e_k = 49 * i + d_k;
      rtb_x_Delay_0[e_k] = rtb_x_Delay[(51 * i + (int32_T)
        NMPC_Path_Tracking_ConstP.Constant1_Value[d_k]) - 1];
      rtb_x_Delay_0[e_k + 1] = rtb_x_Delay[(51 * i + (int32_T)
        NMPC_Path_Tracking_ConstP.Constant1_Value[d_k + 1]) - 1];
    }

    for (d_k = 48; d_k < 49; d_k++) {
      rtb_x_Delay_0[d_k + 49 * i] = rtb_x_Delay[(51 * i + (int32_T)
        NMPC_Path_Tracking_ConstP.Constant1_Value[d_k]) - 1];
    }
  }

  /* End of Selector: '<S7>/Selector1' */

  /* MATLAB Function: '<S6>/NLMPC' incorporates:
   *  Delay: '<S1>/Delay'
   *  Delay: '<S7>/slack_delay'
   *  Reshape: '<S1>/Reshape5'
   */
  NMPC_Path_T_generateRuntimeData(rtb_Reshape5,
    NMPC_Path_Tracking_DW.Delay_DSTATE, rtb_ref_traj, tmp, rtb_x_Delay_0,
    NMPC_Path_Tracking_DW.slack_delay_DSTATE, &NMPC_Path_Tracking_B.runtimedata,
    &userdata, z0);
  NMPC_Path_Tracking_getZBounds(&NMPC_Path_Tracking_B.runtimedata, zLB, zUB);
  NMPC_Path_Tracking_getUBounds(&NMPC_Path_Tracking_B.runtimedata,
    NMPC_Path_Tracking_B.A_data, A_size, B_data, &i);
  NMPC_Path_Tracking_fmincon(&NMPC_Path_Tracking_B.runtimedata, &userdata, z0,
    NMPC_Path_Tracking_B.A_data, B_data, &i, zLB,
    &NMPC_Path_Tracking_B.runtimedata, zUB, &current_position, &v_ref, &Out);
  if ((v_ref == 0.0) && (Out.constrviolation > 1.0E-6)) {
    v_ref = -2.0;
  }

  NMPC_Path_Tracking_getXUe(zUB, rtb_Reshape5, rtb_x_Delay, U, &e);
  if (v_ref > 0.0) {
    NMPC_Path_Tracking_DW.Delay_DSTATE[0] = U[0];
    NMPC_Path_Tracking_DW.Delay_DSTATE[1] = U[51];
  }

  NMPC_Path_Tracking_computeInfo(rtb_x_Delay, U, e, current_position, v_ref,
    Out.iterations, a__1, x, &a__3, &NMPC_Path_Tracking_B.info);

  /* MATLAB Function: '<S1>/MATLAB Function6' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function7'
   */
  if (NMPC_Path_Tracking_DW.Delay_DSTATE[0] > 0.0) {
    /* Outport: '<Root>/accel' */
    NMPC_Path_Tracking_Y.accel = NMPC_Path_Tracking_DW.Delay_DSTATE[0];

    /* Outport: '<Root>/deccel' */
    NMPC_Path_Tracking_Y.deccel = 0.0;
  } else {
    /* Outport: '<Root>/accel' */
    NMPC_Path_Tracking_Y.accel = 0.0;

    /* Outport: '<Root>/deccel' */
    NMPC_Path_Tracking_Y.deccel = -NMPC_Path_Tracking_DW.Delay_DSTATE[0];
  }

  /* End of MATLAB Function: '<S1>/MATLAB Function6' */

  /* Update for Delay: '<S7>/mv_Delay' incorporates:
   *  MATLAB Function: '<S6>/NLMPC'
   */
  NMPC_Path_Tracking_DW.icLoad = false;
  memcpy(&NMPC_Path_Tracking_DW.mv_Delay_DSTATE[0],
         &NMPC_Path_Tracking_B.info.MVopt[0], 102U * sizeof(real_T));

  /* Update for Delay: '<S7>/x_Delay' incorporates:
   *  MATLAB Function: '<S6>/NLMPC'
   */
  NMPC_Path_Tracking_DW.icLoad_c = false;
  memcpy(&NMPC_Path_Tracking_DW.x_Delay_DSTATE[0],
         &NMPC_Path_Tracking_B.info.Xopt[0], 306U * sizeof(real_T));

  /* Update for Delay: '<S7>/slack_delay' incorporates:
   *  MATLAB Function: '<S6>/NLMPC'
   */
  NMPC_Path_Tracking_DW.icLoad_p = false;
  NMPC_Path_Tracking_DW.slack_delay_DSTATE = NMPC_Path_Tracking_B.info.Slack;

  /* Outport: '<Root>/Out1' incorporates:
   *  Inport: '<Root>/current_position'
   *  SignalConversion generated from: '<S1>/current_position'
   */
  NMPC_Path_Tracking_Y.Out1 = NMPC_Path_Tracking_U.current_position[2];

  /* Outport: '<Root>/Steer' incorporates:
   *  Gain: '<S1>/Gain1'
   *  Gain: '<S1>/Gain2'
   *  MATLAB Function: '<S1>/MATLAB Function7'
   */
  NMPC_Path_Tracking_Y.Steer = 57.295779513082323 *
    NMPC_Path_Tracking_DW.Delay_DSTATE[1] * -17.175933082564711;

  /* End of Outputs for SubSystem: '<Root>/NMPC_Path_Tracking' */
}

/* Model initialize function */
void NMPC_Path_Tracking_initialize(void)
{
  /* SystemInitialize for Atomic SubSystem: '<Root>/NMPC_Path_Tracking' */
  /* InitializeConditions for Delay: '<S7>/mv_Delay' */
  NMPC_Path_Tracking_DW.icLoad = true;

  /* InitializeConditions for Delay: '<S7>/x_Delay' */
  NMPC_Path_Tracking_DW.icLoad_c = true;

  /* InitializeConditions for Delay: '<S7>/slack_delay' */
  NMPC_Path_Tracking_DW.icLoad_p = true;

  /* End of SystemInitialize for SubSystem: '<Root>/NMPC_Path_Tracking' */
}

/* Model terminate function */
void NMPC_Path_Tracking_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
