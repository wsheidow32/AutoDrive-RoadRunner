/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: NMPC_Path_Tracking.h
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

#ifndef NMPC_Path_Tracking_h_
#define NMPC_Path_Tracking_h_
#ifndef NMPC_Path_Tracking_COMMON_INCLUDES_
#define NMPC_Path_Tracking_COMMON_INCLUDES_
#include <stdio.h>
#include <string.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* NMPC_Path_Tracking_COMMON_INCLUDES_ */

#include "NMPC_Path_Tracking_types.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  s_AqVMcqPkRdiuFOm62dRGmG_NMPC_T QRManager;
  s_LuoN3prQfsei9XMpifhsFF_NMPC_T CholManager;
  real_T y_data[13712209];
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T WorkingSet;
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T b_obj;
  s_cNaZbIhx4NYzJeaWwvqMfG_NMPC_T c_WorkingSet;
  s_BcTDS8pFolHdhtDibl2TnF_NMPC_T memspace;
  real_T B_data[7413406];
  s_HcaNrKnqq2a7btHVCVfTFH_NMPC_T TrialState;
  real_T JacCineqTrans_data[240600];
  real_T b_varargin_1_data[240600];
  real_T a__4_data[240600];
  real_T Jx[180000];
  real_T Jx_data[180000];
  real_T varargin_1_data[180000];
  real_T Hessian[160801];
  real_T A_data[160400];
  real_T JacCeqTrans[120300];
  real_T JacEqTrans_tmp[120300];
  real_T Jx_m[90000];
  real_T tmp_data[60000];
  real_T tmp_data_c[60000];
  real_T varargin_2_data[60000];
  real_T tmp_data_k[60000];
  real_T Au[40000];
  real_T Auf_data[40000];
  real_T Jmv[30000];
  real_T Jmv_c[30000];
  coder_internal_stickyStruct_2_T FcnEvaluator;
  s_Rv6h6Prjkty0LYc17X1sTH_NMPC_T QPObjective;
  real_T y_data_b[3703];
  real_T y_data_p[3703];
  real_T work_data[3703];
  real_T work_data_c[3703];
  real_T vn1_data[3703];
  real_T vn2_data[3703];
  real_T work_data_f[3703];
  real_T b_data[3703];
  real_T y_data_g[3703];
  real_T y_data_g1[3703];
  s_MYIEghBYRH0ApjCG0mXJrB_NMPC_T runtimedata;
  stwyLkKtGfiNF3i9PDxcjkC_NMPC__T info;
  real_T Cineq_data[600];
  real_T varargin_1_data_m[600];
  real_T b_c[600];
  real_T Je_data[600];
  real_T varargin_1_data_n[600];
  real_T b_c_p[600];
  real_T a__3_data[600];
  real_T b_data_l[600];
} B_NMPC_Path_Tracking_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE[2];              /* '<S1>/Delay' */
  real_T mv_Delay_DSTATE[102];         /* '<S7>/mv_Delay' */
  real_T x_Delay_DSTATE[306];          /* '<S7>/x_Delay' */
  real_T slack_delay_DSTATE;           /* '<S7>/slack_delay' */
  boolean_T icLoad;                    /* '<S7>/mv_Delay' */
  boolean_T icLoad_c;                  /* '<S7>/x_Delay' */
  boolean_T icLoad_p;                  /* '<S7>/slack_delay' */
} DW_NMPC_Path_Tracking_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: min(3,PredictionHorizon+1):(PredictionHorizon+1)
   * Referenced by: '<S7>/Constant'
   */
  real_T Constant_Value[49];

  /* Expression: 2:max(2,PredictionHorizon)
   * Referenced by: '<S7>/Constant1'
   */
  real_T Constant1_Value[49];
} ConstP_NMPC_Path_Tracking_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T current_position[3];          /* '<Root>/current_position' */
  real_T yaw_rate;                     /* '<Root>/yaw_rate' */
  real_T psi;                          /* '<Root>/yaw' */
  real_T vx;                           /* '<Root>/vx' */
  real_T vy;                           /* '<Root>/vy' */
  real_T x_path[100];                  /* '<Root>/x_path' */
  real_T y_path[100];                  /* '<Root>/y_path' */
  real_T curvature[100];               /* '<Root>/curvature' */
  real_T v_max;                        /* '<Root>/v_max' */
  real_T PredictionHorizon;            /* '<Root>/PredictionHorizon' */
} ExtU_NMPC_Path_Tracking_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Out1;                         /* '<Root>/Out1' */
  real_T accel;                        /* '<Root>/accel' */
  real_T deccel;                       /* '<Root>/deccel' */
  real_T distances[100];               /* '<Root>/distances' */
  real_T Steer;                        /* '<Root>/Steer' */
} ExtY_NMPC_Path_Tracking_T;

/* Real-time Model Data Structure */
struct tag_RTM_NMPC_Path_Tracking_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_NMPC_Path_Tracking_T NMPC_Path_Tracking_B;

/* Block states (default storage) */
extern DW_NMPC_Path_Tracking_T NMPC_Path_Tracking_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_NMPC_Path_Tracking_T NMPC_Path_Tracking_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_NMPC_Path_Tracking_T NMPC_Path_Tracking_Y;

/* Constant parameters (default storage) */
extern const ConstP_NMPC_Path_Tracking_T NMPC_Path_Tracking_ConstP;

/* Model entry point functions */
extern void NMPC_Path_Tracking_initialize(void);
extern void NMPC_Path_Tracking_step(void);
extern void NMPC_Path_Tracking_terminate(void);

/* Real-time Model object */
extern RT_MODEL_NMPC_Path_Tracking_T *const NMPC_Path_Tracking_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S8>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S9>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S10>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S11>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S12>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S13>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S14>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S15>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S16>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S17>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S18>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S19>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S20>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S21>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S22>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S23>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S24>/Vector Dimension Check' : Unused code path elimination
 * Block '<S25>/Vector Dimension Check' : Unused code path elimination
 * Block '<S26>/Vector Dimension Check' : Unused code path elimination
 * Block '<S27>/Vector Dimension Check' : Unused code path elimination
 * Block '<S5>/mv.init_zero' : Unused code path elimination
 * Block '<S5>/x.init_zero' : Unused code path elimination
 * Block '<S6>/Reshape' : Reshape block reduction
 * Block '<S6>/Reshape1' : Reshape block reduction
 * Block '<S6>/mo or x Conversion' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion1' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion10' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion11' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion12' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion13' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion14' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion15' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion16' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion17' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion18' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion19' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion2' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion3' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion4' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion5' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion6' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion7' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion8' : Eliminate redundant data type conversion
 * Block '<S6>/mo or x Conversion9' : Eliminate redundant data type conversion
 * Block '<S7>/reshape_mv' : Reshape block reduction
 * Block '<S7>/reshape_x' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking')    - opens subsystem vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking
 * hilite_system('vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'vehicle_hostmodel_2022b_v12'
 * '<S1>'   : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking'
 * '<S2>'   : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/MATLAB Function2'
 * '<S3>'   : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/MATLAB Function6'
 * '<S4>'   : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/MATLAB Function7'
 * '<S5>'   : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller'
 * '<S6>'   : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC'
 * '<S7>'   : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/xmvs_router'
 * '<S8>'   : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check'
 * '<S9>'   : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check1'
 * '<S10>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check10'
 * '<S11>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check11'
 * '<S12>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check12'
 * '<S13>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check13'
 * '<S14>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check14'
 * '<S15>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check15'
 * '<S16>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check16'
 * '<S17>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check3'
 * '<S18>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check4'
 * '<S19>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check5'
 * '<S20>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check6'
 * '<S21>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check7'
 * '<S22>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check8'
 * '<S23>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Preview Signal Check9'
 * '<S24>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Scalar Signal Check1'
 * '<S25>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Scalar Signal Check2'
 * '<S26>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Vector Signal Check1'
 * '<S27>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/MPC Vector Signal Check11'
 * '<S28>'  : 'vehicle_hostmodel_2022b_v12/NMPC_Path_Tracking/Nonlinear MPC Controller/MPC/NLMPC'
 */
#endif                                 /* NMPC_Path_Tracking_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
