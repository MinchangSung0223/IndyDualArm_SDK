/*
 * FK.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "FK".
 *
 * Model version              : 2.4
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Sat May  3 16:25:47 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef FK_h_
#define FK_h_
#ifndef FK_COMMON_INCLUDES_
#define FK_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* FK_COMMON_INCLUDES_ */

#include "FK_types.h"
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

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T q[12];                        /* '<Root>/q' */
  real_T qdot[12];                     /* '<Root>/qdot' */
  real_T lambda_lr[78];                /* '<Root>/lambda_lr' */
  real_T lambda_l[42];                 /* '<Root>/lambda_l' */
  real_T lambda_r[42];                 /* '<Root>/lambda_r' */
} ExtU_FK_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T T_lr[16];                     /* '<Root>/T_lr' */
  real_T T_r[16];                      /* '<Root>/T_r' */
  real_T T_l[16];                      /* '<Root>/T_l' */
  real_T Jb_l[36];                     /* '<Root>/Jb_l' */
  real_T Jb_r[36];                     /* '<Root>/Jb_r' */
  real_T Jbdot_l[36];                  /* '<Root>/Jbdot_l' */
  real_T Jbdot_r[36];                  /* '<Root>/Jbdot_r' */
  real_T Jb_lr[72];                    /* '<Root>/Jb_lr' */
  real_T Jbdot_lr[72];                 /* '<Root>/Jbdot_lr' */
  real_T V_r[6];                       /* '<Root>/V_r' */
  real_T V_l[6];                       /* '<Root>/V_l' */
  real_T V_lr[6];                      /* '<Root>/V_lr' */
} ExtY_FK_T;

/* Parameters (default storage) */
struct P_FK_T_ {
  real_T Constant20_Value;             /* Expression: pi
                                        * Referenced by: '<S2>/Constant20'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_FK_T {
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
extern P_FK_T FK_P;

/* External inputs (root inport signals with default storage) */
extern ExtU_FK_T FK_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_FK_T FK_Y;

/* Model entry point functions */

#ifdef __cplusplus
extern "C" {
#endif
 void FK_initialize(void);
 void FK_step(void);
 void FK_terminate(void);
 #ifdef __cplusplus
}
#endif
/* Real-time Model object */
extern RT_MODEL_FK_T *const FK_M;

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
 * '<Root>' : 'FK'
 * '<S1>'   : 'FK/Kinematics'
 * '<S2>'   : 'FK/Subsystem2'
 * '<S3>'   : 'FK/Kinematics/MATLAB Function'
 * '<S4>'   : 'FK/Kinematics/MATLAB Function1'
 * '<S5>'   : 'FK/Kinematics/MATLAB Function2'
 * '<S6>'   : 'FK/Kinematics/MATLAB Function7'
 * '<S7>'   : 'FK/Kinematics/MATLAB Function8'
 */
#endif                                 /* FK_h_ */
