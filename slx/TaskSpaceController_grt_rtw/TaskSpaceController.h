/*
 * TaskSpaceController.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "TaskSpaceController".
 *
 * Model version              : 1.25
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Fri May  2 23:33:07 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef TaskSpaceController_h_
#define TaskSpaceController_h_
#ifndef TaskSpaceController_COMMON_INCLUDES_
#define TaskSpaceController_COMMON_INCLUDES_
#include <stdio.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                /* TaskSpaceController_COMMON_INCLUDES_ */

#include "TaskSpaceController_types.h"
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

/* Block signals (default storage) */
typedef struct {
  s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T CholRegManager;
  s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T CholRegManager_m;
} B_TaskSpaceController_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE[12];             /* '<S1>/Delay' */
} DW_TaskSpaceController_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T M[144];                       /* '<Root>/M' */
  real_T c[12];                        /* '<Root>/c' */
  real_T g[12];                        /* '<Root>/g' */
  real_T q[12];                        /* '<Root>/q' */
  real_T qdot[12];                     /* '<Root>/qdot' */
  real_T T_l[16];                      /* '<Root>/T_l' */
  real_T V_l[6];                       /* '<Root>/V_l' */
  real_T Jb_l[36];                     /* '<Root>/Jb_l' */
  real_T Jbdot_l[36];                  /* '<Root>/Jbdot_l' */
  real_T T_r[16];                      /* '<Root>/T_r' */
  real_T V_r[6];                       /* '<Root>/V_r' */
  real_T Jb_r[36];                     /* '<Root>/Jb_r' */
  real_T Jbdot_r[36];                  /* '<Root>/Jbdot_r' */
  real_T T_des_l[16];                  /* '<Root>/T_des_l' */
  real_T V_des_l[6];                   /* '<Root>/V_des_l' */
  real_T Vdot_des_l[6];                /* '<Root>/Vdot_des_l' */
  real_T T_des_r[16];                  /* '<Root>/T_des_r' */
  real_T V_des_r[6];                   /* '<Root>/V_des_r' */
  real_T Vdot_des_r[6];                /* '<Root>/Vdot_des_r' */
  real_T dt;                           /* '<Root>/dt' */
  real_T q_init[12];                   /* '<Root>/q_init' */
  real_T q_max[12];                    /* '<Root>/q_max' */
  real_T q_min[12];                    /* '<Root>/q_min' */
  real_T qdot_max[12];                 /* '<Root>/qdot_max' */
  real_T qdot_min[12];                 /* '<Root>/qdot_min' */
  real_T TaskKp[12];                   /* '<Root>/TaskKp' */
  real_T TaskKv[12];                   /* '<Root>/TaskKv' */
  real_T b0[2];                        /* '<Root>/b0' */
  real_T a[2];                         /* '<Root>/a' */
  real_T HinfK[12];                    /* '<Root>/HinfK' */
} ExtU_TaskSpaceController_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T tau[12];                      /* '<Root>/tau' */
} ExtY_TaskSpaceController_T;

/* Parameters (default storage) */
struct P_TaskSpaceController_T_ {
  real_T Delay_InitialCondition;       /* Expression: 0.0
                                        * Referenced by: '<S1>/Delay'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_TaskSpaceController_T {
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
extern P_TaskSpaceController_T TaskSpaceController_P;

/* Block signals (default storage) */
extern B_TaskSpaceController_T TaskSpaceController_B;

/* Block states (default storage) */
extern DW_TaskSpaceController_T TaskSpaceController_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_TaskSpaceController_T TaskSpaceController_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_TaskSpaceController_T TaskSpaceController_Y;

/* Model entry point functions */
extern void TaskSpaceController_initialize(void);
extern void TaskSpaceController_step(void);
extern void TaskSpaceController_terminate(void);

/* Real-time Model object */
extern RT_MODEL_TaskSpaceController_T *const TaskSpaceController_M;

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
 * '<Root>' : 'TaskSpaceController'
 * '<S1>'   : 'TaskSpaceController/DualArmTaskSpaceController'
 * '<S2>'   : 'TaskSpaceController/DualArmTaskSpaceController/HinfController'
 * '<S3>'   : 'TaskSpaceController/DualArmTaskSpaceController/SPDControllerWithQP_Left1'
 * '<S4>'   : 'TaskSpaceController/DualArmTaskSpaceController/SPDControllerWithQP_Right1'
 */
#endif                                 /* TaskSpaceController_h_ */
