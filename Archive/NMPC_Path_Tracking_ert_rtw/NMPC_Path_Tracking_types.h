/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: NMPC_Path_Tracking_types.h
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

#ifndef NMPC_Path_Tracking_types_h_
#define NMPC_Path_Tracking_types_h_
#include "rtwtypes.h"

/* Custom Type definition for MATLAB Function: '<S6>/NLMPC' */
#ifndef struct_tag_sG8JZ69axY52WWR6RKyApQC
#define struct_tag_sG8JZ69axY52WWR6RKyApQC

struct tag_sG8JZ69axY52WWR6RKyApQC
{
  real_T penaltyParam;
  real_T threshold;
  int32_T nPenaltyDecreases;
  real_T linearizedConstrViol;
  real_T initFval;
  real_T initConstrViolationEq;
  real_T initConstrViolationIneq;
  real_T phi;
  real_T phiPrimePlus;
  real_T phiFullStep;
  real_T feasRelativeFactor;
  real_T nlpPrimalFeasError;
  real_T nlpDualFeasError;
  real_T nlpComplError;
  real_T firstOrderOpt;
  boolean_T hasObjective;
};

#endif                                 /* struct_tag_sG8JZ69axY52WWR6RKyApQC */

#ifndef typedef_sG8JZ69axY52WWR6RKyApQC_NMPC__T
#define typedef_sG8JZ69axY52WWR6RKyApQC_NMPC__T

typedef struct tag_sG8JZ69axY52WWR6RKyApQC sG8JZ69axY52WWR6RKyApQC_NMPC__T;

#endif                             /* typedef_sG8JZ69axY52WWR6RKyApQC_NMPC__T */

#ifndef struct_tag_s7RdrPWkr8UPAUyTdDJkLaG
#define struct_tag_s7RdrPWkr8UPAUyTdDJkLaG

struct tag_s7RdrPWkr8UPAUyTdDJkLaG
{
  boolean_T gradOK;
  boolean_T fevalOK;
  boolean_T done;
  boolean_T stepAccepted;
  boolean_T failedLineSearch;
  int32_T stepType;
};

#endif                                 /* struct_tag_s7RdrPWkr8UPAUyTdDJkLaG */

#ifndef typedef_s7RdrPWkr8UPAUyTdDJkLaG_NMPC__T
#define typedef_s7RdrPWkr8UPAUyTdDJkLaG_NMPC__T

typedef struct tag_s7RdrPWkr8UPAUyTdDJkLaG s7RdrPWkr8UPAUyTdDJkLaG_NMPC__T;

#endif                             /* typedef_s7RdrPWkr8UPAUyTdDJkLaG_NMPC__T */

#ifndef struct_tag_sttYSJM5GCi2c1Eu0R50efC
#define struct_tag_sttYSJM5GCi2c1Eu0R50efC

struct tag_sttYSJM5GCi2c1Eu0R50efC
{
  real_T iterations;
  real_T funcCount;
  char_T algorithm[3];
  real_T constrviolation;
  real_T stepsize;
  real_T lssteplength;
  real_T firstorderopt;
};

#endif                                 /* struct_tag_sttYSJM5GCi2c1Eu0R50efC */

#ifndef typedef_sttYSJM5GCi2c1Eu0R50efC_NMPC__T
#define typedef_sttYSJM5GCi2c1Eu0R50efC_NMPC__T

typedef struct tag_sttYSJM5GCi2c1Eu0R50efC sttYSJM5GCi2c1Eu0R50efC_NMPC__T;

#endif                             /* typedef_sttYSJM5GCi2c1Eu0R50efC_NMPC__T */

#ifndef struct_tag_somzaGboVhDG7PNQS6E98jD
#define struct_tag_somzaGboVhDG7PNQS6E98jD

struct tag_somzaGboVhDG7PNQS6E98jD
{
  char_T SolverName[7];
  int32_T MaxIterations;
  real_T StepTolerance;
  real_T OptimalityTolerance;
  real_T ConstraintTolerance;
  real_T ObjectiveLimit;
  real_T PricingTolerance;
  real_T ConstrRelTolFactor;
  real_T ProbRelTolFactor;
  boolean_T RemainFeasible;
  boolean_T IterDisplayQP;
};

#endif                                 /* struct_tag_somzaGboVhDG7PNQS6E98jD */

#ifndef typedef_somzaGboVhDG7PNQS6E98jD_NMPC__T
#define typedef_somzaGboVhDG7PNQS6E98jD_NMPC__T

typedef struct tag_somzaGboVhDG7PNQS6E98jD somzaGboVhDG7PNQS6E98jD_NMPC__T;

#endif                             /* typedef_somzaGboVhDG7PNQS6E98jD_NMPC__T */

#ifndef struct_tag_stwyLkKtGfiNF3i9PDxcjkC
#define struct_tag_stwyLkKtGfiNF3i9PDxcjkC

struct tag_stwyLkKtGfiNF3i9PDxcjkC
{
  real_T MVopt[102];
  real_T Xopt[306];
  real_T Yopt[306];
  real_T Topt[51];
  real_T Slack;
  real_T ExitFlag;
  real_T Iterations;
  real_T Cost;
};

#endif                                 /* struct_tag_stwyLkKtGfiNF3i9PDxcjkC */

#ifndef typedef_stwyLkKtGfiNF3i9PDxcjkC_NMPC__T
#define typedef_stwyLkKtGfiNF3i9PDxcjkC_NMPC__T

typedef struct tag_stwyLkKtGfiNF3i9PDxcjkC stwyLkKtGfiNF3i9PDxcjkC_NMPC__T;

#endif                             /* typedef_stwyLkKtGfiNF3i9PDxcjkC_NMPC__T */

#ifndef struct_emxArray_real_T_2002x600
#define struct_emxArray_real_T_2002x600

struct emxArray_real_T_2002x600
{
  real_T data[1201200];
  int32_T size[2];
};

#endif                                 /* struct_emxArray_real_T_2002x600 */

#ifndef typedef_emxArray_real_T_2002x600_NMPC_T
#define typedef_emxArray_real_T_2002x600_NMPC_T

typedef struct emxArray_real_T_2002x600 emxArray_real_T_2002x600_NMPC_T;

#endif                             /* typedef_emxArray_real_T_2002x600_NMPC_T */

#ifndef struct_emxArray_real_T_2002x300
#define struct_emxArray_real_T_2002x300

struct emxArray_real_T_2002x300
{
  real_T data[600600];
  int32_T size[2];
};

#endif                                 /* struct_emxArray_real_T_2002x300 */

#ifndef typedef_emxArray_real_T_2002x300_NMPC_T
#define typedef_emxArray_real_T_2002x300_NMPC_T

typedef struct emxArray_real_T_2002x300 emxArray_real_T_2002x300_NMPC_T;

#endif                             /* typedef_emxArray_real_T_2002x300_NMPC_T */

#ifndef struct_emxArray_real_T_3703x2002
#define struct_emxArray_real_T_3703x2002

struct emxArray_real_T_3703x2002
{
  real_T data[7413406];
  int32_T size[2];
};

#endif                                 /* struct_emxArray_real_T_3703x2002 */

#ifndef typedef_emxArray_real_T_3703x2002_NMP_T
#define typedef_emxArray_real_T_3703x2002_NMP_T

typedef struct emxArray_real_T_3703x2002 emxArray_real_T_3703x2002_NMP_T;

#endif                             /* typedef_emxArray_real_T_3703x2002_NMP_T */

#ifndef struct_emxArray_real_T_3703x3703
#define struct_emxArray_real_T_3703x3703

struct emxArray_real_T_3703x3703
{
  real_T data[13712209];
  int32_T size[2];
};

#endif                                 /* struct_emxArray_real_T_3703x3703 */

#ifndef typedef_emxArray_real_T_3703x3703_NMP_T
#define typedef_emxArray_real_T_3703x3703_NMP_T

typedef struct emxArray_real_T_3703x3703 emxArray_real_T_3703x3703_NMP_T;

#endif                             /* typedef_emxArray_real_T_3703x3703_NMP_T */

/* Custom Type definition for MATLAB Function: '<S6>/NLMPC' */
#ifndef struct_tag_LuoN3prQfsei9XMpifhsFF
#define struct_tag_LuoN3prQfsei9XMpifhsFF

struct tag_LuoN3prQfsei9XMpifhsFF
{
  emxArray_real_T_3703x3703_NMP_T FMat;
  int32_T ldm;
  int32_T ndims;
  int32_T info;
  real_T scaleFactor;
  boolean_T ConvexCheck;
  real_T regTol_;
  real_T workspace_;
  real_T workspace2_;
};

#endif                                 /* struct_tag_LuoN3prQfsei9XMpifhsFF */

#ifndef typedef_s_LuoN3prQfsei9XMpifhsFF_NMPC_T
#define typedef_s_LuoN3prQfsei9XMpifhsFF_NMPC_T

typedef struct tag_LuoN3prQfsei9XMpifhsFF s_LuoN3prQfsei9XMpifhsFF_NMPC_T;

#endif                             /* typedef_s_LuoN3prQfsei9XMpifhsFF_NMPC_T */

#ifndef struct_tag_MYIEghBYRH0ApjCG0mXJrB
#define struct_tag_MYIEghBYRH0ApjCG0mXJrB

struct tag_MYIEghBYRH0ApjCG0mXJrB
{
  real_T x[6];
  real_T lastMV[2];
  real_T ref[300];
  real_T OutputWeights[300];
  real_T MVWeights[100];
  real_T MVRateWeights[100];
  real_T ECRWeight;
  real_T OutputMin[300];
  real_T OutputMax[300];
  real_T StateMin[300];
  real_T StateMax[300];
  real_T MVMin[100];
  real_T MVMax[100];
  real_T MVRateMin[100];
  real_T MVRateMax[100];
  real_T MVScaledTarget[100];
};

#endif                                 /* struct_tag_MYIEghBYRH0ApjCG0mXJrB */

#ifndef typedef_s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T
#define typedef_s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T

typedef struct tag_MYIEghBYRH0ApjCG0mXJrB s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T;

#endif                             /* typedef_s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T */

#ifndef struct_tag_s02qEHDaRbOUcZF4Nctxs8G
#define struct_tag_s02qEHDaRbOUcZF4Nctxs8G

struct tag_s02qEHDaRbOUcZF4Nctxs8G
{
  real_T Ts;
  real_T CurrentStates[6];
  real_T LastMV[2];
  real_T References[300];
  real_T MVTarget[100];
  real_T PredictionHorizon;
  real_T NumOfStates;
  real_T NumOfOutputs;
  real_T NumOfInputs;
  real_T MVIndex[2];
  real_T InputPassivityIndex;
  real_T OutputPassivityIndex;
  boolean_T PassivityUsePredictedX;
};

#endif                                 /* struct_tag_s02qEHDaRbOUcZF4Nctxs8G */

#ifndef typedef_s02qEHDaRbOUcZF4Nctxs8G_NMPC__T
#define typedef_s02qEHDaRbOUcZF4Nctxs8G_NMPC__T

typedef struct tag_s02qEHDaRbOUcZF4Nctxs8G s02qEHDaRbOUcZF4Nctxs8G_NMPC__T;

#endif                             /* typedef_s02qEHDaRbOUcZF4Nctxs8G_NMPC__T */

#ifndef struct_tag_UduwGwOpHSPENrm1X2xchE
#define struct_tag_UduwGwOpHSPENrm1X2xchE

struct tag_UduwGwOpHSPENrm1X2xchE
{
  s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T runtimedata;
  s02qEHDaRbOUcZF4Nctxs8G_NMPC__T userdata;
};

#endif                                 /* struct_tag_UduwGwOpHSPENrm1X2xchE */

#ifndef typedef_s_UduwGwOpHSPENrm1X2xchE_NMPC_T
#define typedef_s_UduwGwOpHSPENrm1X2xchE_NMPC_T

typedef struct tag_UduwGwOpHSPENrm1X2xchE s_UduwGwOpHSPENrm1X2xchE_NMPC_T;

#endif                             /* typedef_s_UduwGwOpHSPENrm1X2xchE_NMPC_T */

#ifndef struct_tag_krpKLBYKsCU4AneyXttDo
#define struct_tag_krpKLBYKsCU4AneyXttDo

struct tag_krpKLBYKsCU4AneyXttDo
{
  s_UduwGwOpHSPENrm1X2xchE_NMPC_T workspace;
};

#endif                                 /* struct_tag_krpKLBYKsCU4AneyXttDo */

#ifndef typedef_anonymous_function_NMPC_Path__T
#define typedef_anonymous_function_NMPC_Path__T

typedef struct tag_krpKLBYKsCU4AneyXttDo anonymous_function_NMPC_Path__T;

#endif                             /* typedef_anonymous_function_NMPC_Path__T */

#ifndef struct_tag_uC6Qxg6ijYXtxNrMgtKYoG
#define struct_tag_uC6Qxg6ijYXtxNrMgtKYoG

struct tag_uC6Qxg6ijYXtxNrMgtKYoG
{
  anonymous_function_NMPC_Path__T b_value;
};

#endif                                 /* struct_tag_uC6Qxg6ijYXtxNrMgtKYoG */

#ifndef typedef_coder_internal_stickyStruct_1_T
#define typedef_coder_internal_stickyStruct_1_T

typedef struct tag_uC6Qxg6ijYXtxNrMgtKYoG coder_internal_stickyStruct_1_T;

#endif                             /* typedef_coder_internal_stickyStruct_1_T */

#ifndef struct_tag_IENion4fNpO89RbNTRAU2G
#define struct_tag_IENion4fNpO89RbNTRAU2G

struct tag_IENion4fNpO89RbNTRAU2G
{
  anonymous_function_NMPC_Path__T b_value;
  coder_internal_stickyStruct_1_T next;
};

#endif                                 /* struct_tag_IENion4fNpO89RbNTRAU2G */

#ifndef typedef_coder_internal_stickyStruct_h_T
#define typedef_coder_internal_stickyStruct_h_T

typedef struct tag_IENion4fNpO89RbNTRAU2G coder_internal_stickyStruct_h_T;

#endif                             /* typedef_coder_internal_stickyStruct_h_T */

#ifndef struct_tag_IKtoGGTynDjSkjUgncf4KD
#define struct_tag_IKtoGGTynDjSkjUgncf4KD

struct tag_IKtoGGTynDjSkjUgncf4KD
{
  coder_internal_stickyStruct_h_T next;
};

#endif                                 /* struct_tag_IKtoGGTynDjSkjUgncf4KD */

#ifndef typedef_coder_internal_stickyStruct_k_T
#define typedef_coder_internal_stickyStruct_k_T

typedef struct tag_IKtoGGTynDjSkjUgncf4KD coder_internal_stickyStruct_k_T;

#endif                             /* typedef_coder_internal_stickyStruct_k_T */

#ifndef struct_tag_YOcy6Tn0umupkW9YgPEq6E
#define struct_tag_YOcy6Tn0umupkW9YgPEq6E

struct tag_YOcy6Tn0umupkW9YgPEq6E
{
  int32_T b_value;
  coder_internal_stickyStruct_k_T next;
};

#endif                                 /* struct_tag_YOcy6Tn0umupkW9YgPEq6E */

#ifndef typedef_coder_internal_stickyStruct_g_T
#define typedef_coder_internal_stickyStruct_g_T

typedef struct tag_YOcy6Tn0umupkW9YgPEq6E coder_internal_stickyStruct_g_T;

#endif                             /* typedef_coder_internal_stickyStruct_g_T */

#ifndef struct_tag_fILNrFvViQD5FCTxrvl7HH
#define struct_tag_fILNrFvViQD5FCTxrvl7HH

struct tag_fILNrFvViQD5FCTxrvl7HH
{
  coder_internal_stickyStruct_g_T next;
};

#endif                                 /* struct_tag_fILNrFvViQD5FCTxrvl7HH */

#ifndef typedef_coder_internal_stickyStruct_a_T
#define typedef_coder_internal_stickyStruct_a_T

typedef struct tag_fILNrFvViQD5FCTxrvl7HH coder_internal_stickyStruct_a_T;

#endif                             /* typedef_coder_internal_stickyStruct_a_T */

#ifndef struct_tag_DH4nh91q0xLYcLJQRdgSHC
#define struct_tag_DH4nh91q0xLYcLJQRdgSHC

struct tag_DH4nh91q0xLYcLJQRdgSHC
{
  coder_internal_stickyStruct_a_T next;
};

#endif                                 /* struct_tag_DH4nh91q0xLYcLJQRdgSHC */

#ifndef typedef_coder_internal_stickyStruc_k4_T
#define typedef_coder_internal_stickyStruc_k4_T

typedef struct tag_DH4nh91q0xLYcLJQRdgSHC coder_internal_stickyStruc_k4_T;

#endif                             /* typedef_coder_internal_stickyStruc_k4_T */

#ifndef struct_tag_QSzhp4Kf06X6InPpaaIMSB
#define struct_tag_QSzhp4Kf06X6InPpaaIMSB

struct tag_QSzhp4Kf06X6InPpaaIMSB
{
  coder_internal_stickyStruc_k4_T next;
};

#endif                                 /* struct_tag_QSzhp4Kf06X6InPpaaIMSB */

#ifndef typedef_coder_internal_stickyStruct_p_T
#define typedef_coder_internal_stickyStruct_p_T

typedef struct tag_QSzhp4Kf06X6InPpaaIMSB coder_internal_stickyStruct_p_T;

#endif                             /* typedef_coder_internal_stickyStruct_p_T */

#ifndef struct_tag_6FUeOK17l2I6JK81D17TAB
#define struct_tag_6FUeOK17l2I6JK81D17TAB

struct tag_6FUeOK17l2I6JK81D17TAB
{
  coder_internal_stickyStruct_p_T next;
};

#endif                                 /* struct_tag_6FUeOK17l2I6JK81D17TAB */

#ifndef typedef_coder_internal_stickyStruct_l_T
#define typedef_coder_internal_stickyStruct_l_T

typedef struct tag_6FUeOK17l2I6JK81D17TAB coder_internal_stickyStruct_l_T;

#endif                             /* typedef_coder_internal_stickyStruct_l_T */

#ifndef struct_tag_gwd6y6A2I8SwNbw3npBi9
#define struct_tag_gwd6y6A2I8SwNbw3npBi9

struct tag_gwd6y6A2I8SwNbw3npBi9
{
  coder_internal_stickyStruct_l_T next;
};

#endif                                 /* struct_tag_gwd6y6A2I8SwNbw3npBi9 */

#ifndef typedef_coder_internal_stickyStruct_2_T
#define typedef_coder_internal_stickyStruct_2_T

typedef struct tag_gwd6y6A2I8SwNbw3npBi9 coder_internal_stickyStruct_2_T;

#endif                             /* typedef_coder_internal_stickyStruct_2_T */

#ifndef struct_emxArray_real_T_1000
#define struct_emxArray_real_T_1000

struct emxArray_real_T_1000
{
  real_T data[1000];
  int32_T size;
};

#endif                                 /* struct_emxArray_real_T_1000 */

#ifndef typedef_emxArray_real_T_1000_NMPC_Pat_T
#define typedef_emxArray_real_T_1000_NMPC_Pat_T

typedef struct emxArray_real_T_1000 emxArray_real_T_1000_NMPC_Pat_T;

#endif                             /* typedef_emxArray_real_T_1000_NMPC_Pat_T */

#ifndef struct_emxArray_real_T_2002
#define struct_emxArray_real_T_2002

struct emxArray_real_T_2002
{
  real_T data[2002];
  int32_T size;
};

#endif                                 /* struct_emxArray_real_T_2002 */

#ifndef typedef_emxArray_real_T_2002_NMPC_Pat_T
#define typedef_emxArray_real_T_2002_NMPC_Pat_T

typedef struct emxArray_real_T_2002 emxArray_real_T_2002_NMPC_Pat_T;

#endif                             /* typedef_emxArray_real_T_2002_NMPC_Pat_T */

#ifndef struct_emxArray_real_T_3703
#define struct_emxArray_real_T_3703

struct emxArray_real_T_3703
{
  real_T data[3703];
  int32_T size;
};

#endif                                 /* struct_emxArray_real_T_3703 */

#ifndef typedef_emxArray_real_T_3703_NMPC_Pat_T
#define typedef_emxArray_real_T_3703_NMPC_Pat_T

typedef struct emxArray_real_T_3703 emxArray_real_T_3703_NMPC_Pat_T;

#endif                             /* typedef_emxArray_real_T_3703_NMPC_Pat_T */

#ifndef struct_emxArray_int32_T_3703
#define struct_emxArray_int32_T_3703

struct emxArray_int32_T_3703
{
  int32_T data[3703];
  int32_T size;
};

#endif                                 /* struct_emxArray_int32_T_3703 */

#ifndef typedef_emxArray_int32_T_3703_NMPC_Pa_T
#define typedef_emxArray_int32_T_3703_NMPC_Pa_T

typedef struct emxArray_int32_T_3703 emxArray_int32_T_3703_NMPC_Pa_T;

#endif                             /* typedef_emxArray_int32_T_3703_NMPC_Pa_T */

/* Custom Type definition for MATLAB Function: '<S6>/NLMPC' */
#ifndef struct_tag_HcaNrKnqq2a7btHVCVfTFH
#define struct_tag_HcaNrKnqq2a7btHVCVfTFH

struct tag_HcaNrKnqq2a7btHVCVfTFH
{
  int32_T nVarMax;
  int32_T mNonlinIneq;
  int32_T mNonlinEq;
  int32_T mIneq;
  int32_T mEq;
  int32_T iNonIneq0;
  int32_T iNonEq0;
  real_T sqpFval;
  real_T sqpFval_old;
  real_T xstarsqp[401];
  real_T xstarsqp_old[401];
  emxArray_real_T_1000_NMPC_Pat_T cIneq;
  emxArray_real_T_1000_NMPC_Pat_T cIneq_old;
  real_T cEq[300];
  real_T cEq_old[300];
  emxArray_real_T_2002_NMPC_Pat_T grad;
  emxArray_real_T_2002_NMPC_Pat_T grad_old;
  int32_T FunctionEvaluations;
  int32_T sqpIterations;
  int32_T sqpExitFlag;
  emxArray_real_T_3703_NMPC_Pat_T lambdasqp;
  emxArray_real_T_3703_NMPC_Pat_T lambdaStopTest;
  emxArray_real_T_3703_NMPC_Pat_T lambdaStopTestPrev;
  real_T steplength;
  emxArray_real_T_3703_NMPC_Pat_T delta_x;
  emxArray_real_T_2002_NMPC_Pat_T socDirection;
  emxArray_int32_T_3703_NMPC_Pa_T workingset_old;
  emxArray_real_T_2002x600_NMPC_T JacCineqTrans_old;
  emxArray_real_T_2002x300_NMPC_T JacCeqTrans_old;
  emxArray_real_T_2002_NMPC_Pat_T gradLag;
  emxArray_real_T_2002_NMPC_Pat_T delta_gradLag;
  emxArray_real_T_3703_NMPC_Pat_T xstar;
  real_T fstar;
  real_T firstorderopt;
  emxArray_real_T_3703_NMPC_Pat_T lambda;
  int32_T state;
  real_T maxConstr;
  int32_T iterations;
  emxArray_real_T_3703_NMPC_Pat_T searchDir;
};

#endif                                 /* struct_tag_HcaNrKnqq2a7btHVCVfTFH */

#ifndef typedef_s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T
#define typedef_s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T

typedef struct tag_HcaNrKnqq2a7btHVCVfTFH s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T;

#endif                             /* typedef_s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T */

#ifndef struct_tag_BcTDS8pFolHdhtDibl2TnF
#define struct_tag_BcTDS8pFolHdhtDibl2TnF

struct tag_BcTDS8pFolHdhtDibl2TnF
{
  emxArray_real_T_3703x2002_NMP_T workspace_float;
  emxArray_int32_T_3703_NMPC_Pa_T workspace_int;
  emxArray_int32_T_3703_NMPC_Pa_T workspace_sort;
};

#endif                                 /* struct_tag_BcTDS8pFolHdhtDibl2TnF */

#ifndef typedef_s_BcTDS8pFolHdhtDibl2TnF_NMPC_T
#define typedef_s_BcTDS8pFolHdhtDibl2TnF_NMPC_T

typedef struct tag_BcTDS8pFolHdhtDibl2TnF s_BcTDS8pFolHdhtDibl2TnF_NMPC_T;

#endif                             /* typedef_s_BcTDS8pFolHdhtDibl2TnF_NMPC_T */

#ifndef struct_emxArray_real_T_2002000
#define struct_emxArray_real_T_2002000

struct emxArray_real_T_2002000
{
  real_T data[2002000];
  int32_T size;
};

#endif                                 /* struct_emxArray_real_T_2002000 */

#ifndef typedef_emxArray_real_T_2002000_NMPC__T
#define typedef_emxArray_real_T_2002000_NMPC__T

typedef struct emxArray_real_T_2002000 emxArray_real_T_2002000_NMPC__T;

#endif                             /* typedef_emxArray_real_T_2002000_NMPC__T */

#ifndef struct_emxArray_real_T_600600
#define struct_emxArray_real_T_600600

struct emxArray_real_T_600600
{
  real_T data[600600];
  int32_T size;
};

#endif                                 /* struct_emxArray_real_T_600600 */

#ifndef typedef_emxArray_real_T_600600_NMPC_P_T
#define typedef_emxArray_real_T_600600_NMPC_P_T

typedef struct emxArray_real_T_600600 emxArray_real_T_600600_NMPC_P_T;

#endif                             /* typedef_emxArray_real_T_600600_NMPC_P_T */

#ifndef struct_emxArray_int32_T_2002
#define struct_emxArray_int32_T_2002

struct emxArray_int32_T_2002
{
  int32_T data[2002];
  int32_T size;
};

#endif                                 /* struct_emxArray_int32_T_2002 */

#ifndef typedef_emxArray_int32_T_2002_NMPC_Pa_T
#define typedef_emxArray_int32_T_2002_NMPC_Pa_T

typedef struct emxArray_int32_T_2002 emxArray_int32_T_2002_NMPC_Pa_T;

#endif                             /* typedef_emxArray_int32_T_2002_NMPC_Pa_T */

#ifndef struct_emxArray_real_T_7413406
#define struct_emxArray_real_T_7413406

struct emxArray_real_T_7413406
{
  real_T data[7413406];
  int32_T size;
};

#endif                                 /* struct_emxArray_real_T_7413406 */

#ifndef typedef_emxArray_real_T_7413406_NMPC__T
#define typedef_emxArray_real_T_7413406_NMPC__T

typedef struct emxArray_real_T_7413406 emxArray_real_T_7413406_NMPC__T;

#endif                             /* typedef_emxArray_real_T_7413406_NMPC__T */

#ifndef struct_emxArray_boolean_T_3703
#define struct_emxArray_boolean_T_3703

struct emxArray_boolean_T_3703
{
  boolean_T data[3703];
  int32_T size;
};

#endif                                 /* struct_emxArray_boolean_T_3703 */

#ifndef typedef_emxArray_boolean_T_3703_NMPC__T
#define typedef_emxArray_boolean_T_3703_NMPC__T

typedef struct emxArray_boolean_T_3703 emxArray_boolean_T_3703_NMPC__T;

#endif                             /* typedef_emxArray_boolean_T_3703_NMPC__T */

/* Custom Type definition for MATLAB Function: '<S6>/NLMPC' */
#ifndef struct_tag_cNaZbIhx4NYzJeaWwvqMfG
#define struct_tag_cNaZbIhx4NYzJeaWwvqMfG

struct tag_cNaZbIhx4NYzJeaWwvqMfG
{
  int32_T mConstr;
  int32_T mConstrOrig;
  int32_T mConstrMax;
  int32_T nVar;
  int32_T nVarOrig;
  int32_T nVarMax;
  int32_T ldA;
  emxArray_real_T_2002000_NMPC__T Aineq;
  emxArray_real_T_1000_NMPC_Pat_T bineq;
  emxArray_real_T_600600_NMPC_P_T Aeq;
  real_T beq[300];
  emxArray_real_T_2002_NMPC_Pat_T lb;
  emxArray_real_T_2002_NMPC_Pat_T ub;
  emxArray_int32_T_2002_NMPC_Pa_T indexLB;
  emxArray_int32_T_2002_NMPC_Pa_T indexUB;
  emxArray_int32_T_2002_NMPC_Pa_T indexFixed;
  int32_T mEqRemoved;
  int32_T indexEqRemoved[300];
  emxArray_real_T_7413406_NMPC__T ATwset;
  emxArray_real_T_3703_NMPC_Pat_T bwset;
  int32_T nActiveConstr;
  emxArray_real_T_3703_NMPC_Pat_T maxConstrWorkspace;
  int32_T sizes[5];
  int32_T sizesNormal[5];
  int32_T sizesPhaseOne[5];
  int32_T sizesRegularized[5];
  int32_T sizesRegPhaseOne[5];
  int32_T isActiveIdx[6];
  int32_T isActiveIdxNormal[6];
  int32_T isActiveIdxPhaseOne[6];
  int32_T isActiveIdxRegularized[6];
  int32_T isActiveIdxRegPhaseOne[6];
  emxArray_boolean_T_3703_NMPC__T isActiveConstr;
  emxArray_int32_T_3703_NMPC_Pa_T Wid;
  emxArray_int32_T_3703_NMPC_Pa_T Wlocalidx;
  int32_T nWConstr[5];
  int32_T probType;
  real_T SLACK0;
};

#endif                                 /* struct_tag_cNaZbIhx4NYzJeaWwvqMfG */

#ifndef typedef_s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T
#define typedef_s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T

typedef struct tag_cNaZbIhx4NYzJeaWwvqMfG s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T;

#endif                             /* typedef_s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T */

#ifndef struct_emxArray_real_T_2001
#define struct_emxArray_real_T_2001

struct emxArray_real_T_2001
{
  real_T data[2001];
  int32_T size;
};

#endif                                 /* struct_emxArray_real_T_2001 */

#ifndef typedef_emxArray_real_T_2001_NMPC_Pat_T
#define typedef_emxArray_real_T_2001_NMPC_Pat_T

typedef struct emxArray_real_T_2001 emxArray_real_T_2001_NMPC_Pat_T;

#endif                             /* typedef_emxArray_real_T_2001_NMPC_Pat_T */

/* Custom Type definition for MATLAB Function: '<S6>/NLMPC' */
#ifndef struct_tag_Rv6h6Prjkty0LYc17X1sTH
#define struct_tag_Rv6h6Prjkty0LYc17X1sTH

struct tag_Rv6h6Prjkty0LYc17X1sTH
{
  emxArray_real_T_2002_NMPC_Pat_T grad;
  emxArray_real_T_2001_NMPC_Pat_T Hx;
  boolean_T hasLinear;
  int32_T nvar;
  int32_T maxVar;
  real_T beta;
  real_T rho;
  int32_T objtype;
  int32_T prev_objtype;
  int32_T prev_nvar;
  boolean_T prev_hasLinear;
  real_T gammaScalar;
};

#endif                                 /* struct_tag_Rv6h6Prjkty0LYc17X1sTH */

#ifndef typedef_s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T
#define typedef_s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T

typedef struct tag_Rv6h6Prjkty0LYc17X1sTH s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T;

#endif                             /* typedef_s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T */

#ifndef struct_tag_AqVMcqPkRdiuFOm62dRGmG
#define struct_tag_AqVMcqPkRdiuFOm62dRGmG

struct tag_AqVMcqPkRdiuFOm62dRGmG
{
  int32_T ldq;
  emxArray_real_T_3703x3703_NMP_T QR;
  emxArray_real_T_3703x3703_NMP_T Q;
  emxArray_int32_T_3703_NMPC_Pa_T jpvt;
  int32_T mrows;
  int32_T ncols;
  emxArray_real_T_3703_NMPC_Pat_T tau;
  int32_T minRowCol;
  boolean_T usedPivoting;
};

#endif                                 /* struct_tag_AqVMcqPkRdiuFOm62dRGmG */

#ifndef typedef_s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T
#define typedef_s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T

typedef struct tag_AqVMcqPkRdiuFOm62dRGmG s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T;

#endif                             /* typedef_s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T */

#ifndef struct_emxArray_real_T_600
#define struct_emxArray_real_T_600

struct emxArray_real_T_600
{
  real_T data[600];
  int32_T size;
};

#endif                                 /* struct_emxArray_real_T_600 */

#ifndef typedef_emxArray_real_T_600_NMPC_Path_T
#define typedef_emxArray_real_T_600_NMPC_Path_T

typedef struct emxArray_real_T_600 emxArray_real_T_600_NMPC_Path_T;

#endif                             /* typedef_emxArray_real_T_600_NMPC_Path_T */

/* Custom Type definition for MATLAB Function: '<S6>/NLMPC' */
#ifndef struct_tag_GHQAaZMt6EIZSPYdOF1p0
#define struct_tag_GHQAaZMt6EIZSPYdOF1p0

struct tag_GHQAaZMt6EIZSPYdOF1p0
{
  anonymous_function_NMPC_Path__T objfun;
  anonymous_function_NMPC_Path__T nonlin;
  real_T f_1;
  emxArray_real_T_600_NMPC_Path_T cIneq_1;
  real_T cEq_1[300];
  real_T f_2;
  emxArray_real_T_600_NMPC_Path_T cIneq_2;
  real_T cEq_2[300];
  int32_T nVar;
  int32_T mIneq;
  int32_T mEq;
  int32_T numEvals;
  boolean_T SpecifyObjectiveGradient;
  boolean_T SpecifyConstraintGradient;
  boolean_T isEmptyNonlcon;
  boolean_T hasLB[401];
  boolean_T hasUB[401];
  boolean_T hasBounds;
  int32_T FiniteDifferenceType;
};

#endif                                 /* struct_tag_GHQAaZMt6EIZSPYdOF1p0 */

#ifndef typedef_s_GHQAaZMt6EIZSPYdOF1p0_NMPC__T
#define typedef_s_GHQAaZMt6EIZSPYdOF1p0_NMPC__T

typedef struct tag_GHQAaZMt6EIZSPYdOF1p0 s_GHQAaZMt6EIZSPYdOF1p0_NMPC__T;

#endif                             /* typedef_s_GHQAaZMt6EIZSPYdOF1p0_NMPC__T */

/* Forward declaration for rtModel */
typedef struct tag_RTM_NMPC_Path_Tracking_T RT_MODEL_NMPC_Path_Tracking_T;

#endif                                 /* NMPC_Path_Tracking_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
