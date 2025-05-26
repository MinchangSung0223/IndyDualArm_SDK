/*
 * LR.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "LR".
 *
 * Model version              : 1.12
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Sat May 17 22:25:22 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef LR_h_
#define LR_h_
#ifndef LR_COMMON_INCLUDES_
#define LR_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* LR_COMMON_INCLUDES_ */

#include "LR_types.h"
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

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T log6_T[16];                   /* '<Root>/log6_T' */
  real_T exp6_lambda[6];               /* '<Root>/exp6_lambda' */
  real_T dexp6_lambda[6];              /* '<Root>/dexp6_lambda' */
  real_T ddexp6_lambda[6];             /* '<Root>/ddexp6_lambda' */
  real_T ddexp6_lambdadot[6];          /* '<Root>/ddexp6_lambdadot' */
  real_T dddexp6_lambda[6];            /* '<Root>/dddexp6_lambda' */
  real_T dddexp6_lambdadot[6];         /* '<Root>/dddexp6_lambdadot' */
  real_T dddexp6_lambdaddot[6];        /* '<Root>/dddexp6_lambdaddot' */
  real_T dexp3_xi[3];                  /* '<Root>/dexp3_xi' */
  real_T ddexp3_xi[3];                 /* '<Root>/ddexp3_xi' */
  real_T ddexp3_xidot[3];              /* '<Root>/ddexp3_xidot' */
  real_T dddexp3_xi[3];                /* '<Root>/dddexp3_xi' */
  real_T dddexp3_xidot[3];             /* '<Root>/dddexp3_xidot' */
  real_T dddexp3_xiddot[3];            /* '<Root>/dddexp3_xiddot' */
  real_T dexp3inv_xi[6];               /* '<Root>/dexp3inv_xi' */
  real_T ddexp3inv_xi[3];              /* '<Root>/ddexp3inv_xi' */
  real_T ddexp3inv_xidot[3];           /* '<Root>/ddexp3inv_xidot' */
  real_T dddexp3inv_xi[3];             /* '<Root>/dddexp3inv_xi' */
  real_T dddexp3inv_xidot[3];          /* '<Root>/dddexp3inv_xidot' */
  real_T dddexp3inv_xiddot[3];         /* '<Root>/dddexp3inv_xiddot' */
  real_T dexp6inv_lambda[6];           /* '<Root>/dexp6inv_lambda' */
  real_T ddexp6inv_lambda[6];          /* '<Root>/ddexp6inv_lambda' */
  real_T ddexp6inv_lambdadot[6];       /* '<Root>/ddexp6inv_lambdadot' */
  real_T dddexp6inv_lambda[6];         /* '<Root>/dddexp6inv_lambda' */
  real_T dddexp6inv_lambdadot[6];      /* '<Root>/dddexp6inv_lambdadot' */
  real_T dddexp6inv_lambdaddot[6];     /* '<Root>/dddexp6inv_lambdaddot' */
} ExtU_LR_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T exp6_T[16];                   /* '<Root>/exp6_T' */
  real_T dexp6[36];                    /* '<Root>/dexp6' */
  real_T ddexp6[36];                   /* '<Root>/ddexp6' */
  real_T dddexp6[36];                  /* '<Root>/dddexp6' */
  real_T dexp3[9];                     /* '<Root>/dexp3' */
  real_T ddexp3[9];                    /* '<Root>/ddexp3' */
  real_T dddexp3[9];                   /* '<Root>/dddexp3' */
  real_T dexp3inv[9];                  /* '<Root>/dexp3inv' */
  real_T ddexp3inv[9];                 /* '<Root>/ddexp3inv' */
  real_T dddexp3inv[9];                /* '<Root>/dddexp3inv' */
  real_T dexp6inv[36];                 /* '<Root>/dexp6inv' */
  real_T ddexp6inv[36];                /* '<Root>/ddexp6inv' */
  real_T dddexp6inv[36];               /* '<Root>/dddexp6inv' */
  creal_T lambda[6];                   /* '<Root>/lambda' */
} ExtY_LR_T;

/* Real-time Model Data Structure */
struct tag_RTM_LR_T {
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

/* External inputs (root inport signals with default storage) */
extern ExtU_LR_T LR_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_LR_T LR_Y;

/* Model entry point functions */
#ifdef __cplusplus
extern "C" {
#endif
void LR_initialize(void);
void LR_step(void);
void LR_terminate(void);
#ifdef __cplusplus
}
#endif

/* Real-time Model object */
extern RT_MODEL_LR_T *const LR_M;

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
 * '<Root>' : 'LR'
 * '<S1>'   : 'LR/MATLAB Function'
 * '<S2>'   : 'LR/MATLAB Function1'
 * '<S3>'   : 'LR/MATLAB Function10'
 * '<S4>'   : 'LR/MATLAB Function11'
 * '<S5>'   : 'LR/MATLAB Function12'
 * '<S6>'   : 'LR/MATLAB Function13'
 * '<S7>'   : 'LR/MATLAB Function2'
 * '<S8>'   : 'LR/MATLAB Function3'
 * '<S9>'   : 'LR/MATLAB Function4'
 * '<S10>'  : 'LR/MATLAB Function5'
 * '<S11>'  : 'LR/MATLAB Function6'
 * '<S12>'  : 'LR/MATLAB Function7'
 * '<S13>'  : 'LR/MATLAB Function8'
 * '<S14>'  : 'LR/MATLAB Function9'
 */
#endif                                 /* LR_h_ */
