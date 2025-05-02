/*
 * HinfController.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "HinfController".
 *
 * Model version              : 1.15
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Fri May  2 19:03:22 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef HinfController_h_
#define HinfController_h_
#ifndef HinfController_COMMON_INCLUDES_
#define HinfController_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* HinfController_COMMON_INCLUDES_ */

#include "HinfController_types.h"
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
  real_T Delay_DSTATE[12];             /* '<Root>/Delay' */
} DW_HinfController_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T M[144];                       /* '<Root>/M' */
  real_T g[12];                        /* '<Root>/g' */
  real_T c[12];                        /* '<Root>/c' */
  real_T q[12];                        /* '<Root>/q' */
  real_T qdot[12];                     /* '<Root>/qdot' */
  real_T q_des[12];                    /* '<Root>/q_des' */
  real_T qdot_des[12];                 /* '<Root>/qdot_des' */
  real_T qddot_des[12];                /* '<Root>/qddot_des' */
  real_T HinfK[12];                    /* '<Root>/HinfK' */
  real_T dt[12];                       /* '<Root>/dt' */
} ExtU_HinfController_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T tau[12];                      /* '<Root>/tau' */
} ExtY_HinfController_T;

/* Parameters (default storage) */
struct P_HinfController_T_ {
  real_T Delay_InitialCondition;       /* Expression: 0.0
                                        * Referenced by: '<Root>/Delay'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_HinfController_T {
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
extern P_HinfController_T HinfController_P;

/* Block states (default storage) */
extern DW_HinfController_T HinfController_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_HinfController_T HinfController_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_HinfController_T HinfController_Y;

/* Model entry point functions */

#ifdef __cplusplus
extern "C" {
#endif
 void HinfController_initialize(void);
 void HinfController_step(void);
 void HinfController_terminate(void);
 #ifdef __cplusplus
}
#endif
/* Real-time Model object */
extern RT_MODEL_HinfController_T *const HinfController_M;

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
 * '<Root>' : 'HinfController'
 * '<S1>'   : 'HinfController/HinfController'
 */
#endif                                 /* HinfController_h_ */
