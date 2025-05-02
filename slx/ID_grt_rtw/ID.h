/*
 * ID.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "ID".
 *
 * Model version              : 1.2
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Fri May  2 14:17:03 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef ID_h_
#define ID_h_
#ifndef ID_COMMON_INCLUDES_
#define ID_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "collisioncodegen_api.hpp"
#endif                                 /* ID_COMMON_INCLUDES_ */

#include "ID_types.h"
#include "rtGetNaN.h"
#include <float.h>
#include <string.h>
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block signals (default storage) */
typedef struct {
  h_cell_wrap_ID_T Ic_data[17];
  h_cell_wrap_ID_T X_data[17];
  g_cell_wrap_ID_T X_data_m[17];
  g_cell_wrap_ID_T Xtree_data[17];
  g_cell_wrap_ID_T X_data_c[17];
  g_cell_wrap_ID_T Xtree_data_k[17];
} B_ID_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  robotics_slmanip_internal_blo_T obj; /* '<S2>/MATLAB System' */
  robotics_slmanip_internal__hy_T obj_h;/* '<S4>/MATLAB System' */
  robotics_slmanip_internal_b_h_T obj_g;/* '<S3>/MATLAB System' */
  uint32_T method;                     /* '<S4>/MATLAB System' */
  uint32_T state;                      /* '<S4>/MATLAB System' */
  uint32_T state_p[2];                 /* '<S4>/MATLAB System' */
  uint32_T state_m[625];               /* '<S4>/MATLAB System' */
  uint32_T method_k;                   /* '<S3>/MATLAB System' */
  uint32_T state_k;                    /* '<S3>/MATLAB System' */
  uint32_T state_o[2];                 /* '<S3>/MATLAB System' */
  uint32_T state_l[625];               /* '<S3>/MATLAB System' */
  uint32_T method_h;                   /* '<S2>/MATLAB System' */
  uint32_T state_b;                    /* '<S2>/MATLAB System' */
  uint32_T state_n[2];                 /* '<S2>/MATLAB System' */
  uint32_T state_d[625];               /* '<S2>/MATLAB System' */
  boolean_T objisempty;                /* '<S4>/MATLAB System' */
  boolean_T method_not_empty;          /* '<S4>/MATLAB System' */
  boolean_T state_not_empty;           /* '<S4>/MATLAB System' */
  boolean_T state_not_empty_l;         /* '<S4>/MATLAB System' */
  boolean_T state_not_empty_i;         /* '<S4>/MATLAB System' */
  boolean_T objisempty_h;              /* '<S3>/MATLAB System' */
  boolean_T method_not_empty_j;        /* '<S3>/MATLAB System' */
  boolean_T state_not_empty_p;         /* '<S3>/MATLAB System' */
  boolean_T state_not_empty_d;         /* '<S3>/MATLAB System' */
  boolean_T state_not_empty_g;         /* '<S3>/MATLAB System' */
  boolean_T objisempty_n;              /* '<S2>/MATLAB System' */
  boolean_T method_not_empty_o;        /* '<S2>/MATLAB System' */
  boolean_T state_not_empty_gu;        /* '<S2>/MATLAB System' */
  boolean_T state_not_empty_k;         /* '<S2>/MATLAB System' */
  boolean_T state_not_empty_d1;        /* '<S2>/MATLAB System' */
} DW_ID_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T q[12];                        /* '<Root>/q' */
  real_T qdot[12];                     /* '<Root>/qdot' */
} ExtU_ID_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T M[144];                       /* '<Root>/M' */
  real_T c[12];                        /* '<Root>/c' */
  real_T g[12];                        /* '<Root>/g' */
} ExtY_ID_T;

/* Real-time Model Data Structure */
struct tag_RTM_ID_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block signals (default storage) */
extern B_ID_T ID_B;

/* Block states (default storage) */
extern DW_ID_T ID_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_ID_T ID_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_ID_T ID_Y;

/* Model entry point functions */
extern void ID_initialize(void);
extern void ID_step(void);
extern void ID_terminate(void);

/* Real-time Model object */
extern RT_MODEL_ID_T *const ID_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'ID'
 * '<S1>'   : 'ID/InverseDynamics'
 * '<S2>'   : 'ID/InverseDynamics/Gravity Torque'
 * '<S3>'   : 'ID/InverseDynamics/Joint Space Mass Matrix'
 * '<S4>'   : 'ID/InverseDynamics/Velocity Product Torque'
 */
#endif                                 /* ID_h_ */
