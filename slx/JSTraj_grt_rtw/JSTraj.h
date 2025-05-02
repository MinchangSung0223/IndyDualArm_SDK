/*
 * JSTraj.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "JSTraj".
 *
 * Model version              : 1.10
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Fri May  2 14:17:19 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef JSTraj_h_
#define JSTraj_h_
#ifndef JSTraj_COMMON_INCLUDES_
#define JSTraj_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* JSTraj_COMMON_INCLUDES_ */

#include "JSTraj_types.h"
#include "rtGetInf.h"
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

/* Block states (default storage) for system '<Root>' */
typedef struct {
  robotics_slcore_internal_bloc_T obj; /* '<Root>/Polynomial Trajectory' */
  boolean_T objisempty;                /* '<Root>/Polynomial Trajectory' */
} DW_JSTraj_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T gt;                           /* '<Root>/gt' */
  real_T q_start[6];                   /* '<Root>/q_start' */
  real_T q_end[6];                     /* '<Root>/q_end' */
  real_T T0;                           /* '<Root>/T0' */
  real_T Tf;                           /* '<Root>/Tf' */
} ExtU_JSTraj_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T q_des[6];                     /* '<Root>/q_des' */
  real_T qdot_des[6];                  /* '<Root>/qdot_des' */
  real_T qddot_des[6];                 /* '<Root>/qddot_des' */
} ExtY_JSTraj_T;

/* Parameters (default storage) */
struct P_JSTraj_T_ {
  real_T Constant_Value;               /* Expression: eps
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<Root>/Switch'
                                        */
  real_T Gain_Gain;                    /* Expression: 0
                                        * Referenced by: '<Root>/Gain'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_JSTraj_T {
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

/* Block parameters (default storage) */
extern P_JSTraj_T JSTraj_P;

/* Block states (default storage) */
extern DW_JSTraj_T JSTraj_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_JSTraj_T JSTraj_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_JSTraj_T JSTraj_Y;

/* Model entry point functions */
extern void JSTraj_initialize(void);
extern void JSTraj_step(void);
extern void JSTraj_terminate(void);

/* Real-time Model object */
extern RT_MODEL_JSTraj_T *const JSTraj_M;

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
 * '<Root>' : 'JSTraj'
 */
#endif                                 /* JSTraj_h_ */
