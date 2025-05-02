/*
 * TSTraj.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "TSTraj".
 *
 * Model version              : 1.10
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Fri May  2 22:41:20 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef TSTraj_h_
#define TSTraj_h_
#ifndef TSTraj_COMMON_INCLUDES_
#define TSTraj_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* TSTraj_COMMON_INCLUDES_ */

#include "TSTraj_types.h"
#include "rtGetNaN.h"
#include "rtGetInf.h"
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

/* Block signals for system '<S1>/TransToRp' */
typedef struct {
  real_T R[9];                         /* '<S1>/TransToRp' */
} B_TransToRp_TSTraj_T;

/* Block signals for system '<S1>/Trapezoidal Velocity Profile Trajectory' */
typedef struct {
  real_T TrapezoidalVelocityProfileTra_j[3];
                            /* '<S1>/Trapezoidal Velocity Profile Trajectory' */
  real_T TrapezoidalVelocityProfileTra_l[3];
                            /* '<S1>/Trapezoidal Velocity Profile Trajectory' */
  real_T TrapezoidalVelocityProfileTra_f[3];
                            /* '<S1>/Trapezoidal Velocity Profile Trajectory' */
} B_TrapezoidalVelocityProfileT_T;

/* Block states (default storage) for system '<S1>/Trapezoidal Velocity Profile Trajectory' */
typedef struct {
  robotics_slcore_internal_bloc_T obj;
                            /* '<S1>/Trapezoidal Velocity Profile Trajectory' */
  boolean_T objisempty;     /* '<S1>/Trapezoidal Velocity Profile Trajectory' */
} DW_TrapezoidalVelocityProfile_T;

/* Block signals (default storage) */
typedef struct {
  B_TrapezoidalVelocityProfileT_T TrapezoidalVelocityProfileTr_pn;
                            /* '<S1>/Trapezoidal Velocity Profile Trajectory' */
  B_TrapezoidalVelocityProfileT_T TrapezoidalVelocityProfileTra_p;
                            /* '<S1>/Trapezoidal Velocity Profile Trajectory' */
  B_TransToRp_TSTraj_T sf_TransToRp1;  /* '<S1>/TransToRp1' */
  B_TransToRp_TSTraj_T sf_TransToRp;   /* '<S1>/TransToRp' */
} B_TSTraj_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  DW_TrapezoidalVelocityProfile_T TrapezoidalVelocityProfileTr_pn;
                            /* '<S1>/Trapezoidal Velocity Profile Trajectory' */
  DW_TrapezoidalVelocityProfile_T TrapezoidalVelocityProfileTra_p;
                            /* '<S1>/Trapezoidal Velocity Profile Trajectory' */
} DW_TSTraj_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T T_start[16];                  /* '<Root>/T_start' */
  real_T T_end[16];                    /* '<Root>/T_end' */
  real_T gt;                           /* '<Root>/gt' */
  real_T Tf;                           /* '<Root>/Tf' */
  real_T T0;                           /* '<Root>/T0' */
} ExtU_TSTraj_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T T_t[16];                      /* '<Root>/T_t' */
  real_T V_t[6];                       /* '<Root>/V_t' */
  real_T Vdot_t[6];                    /* '<Root>/Vdot_t' */
  real_T p_t[3];                       /* '<Root>/p_t' */
  real_T pdot_t[3];                    /* '<Root>/pdot_t' */
  real_T pddot_t[3];                   /* '<Root>/pddot_t' */
  real_T xi_t[3];                      /* '<Root>/xi_t' */
  real_T xidot_t[3];                   /* '<Root>/xidot_t' */
  real_T xiddot_t[3];                  /* '<Root>/xiddot_t' */
} ExtY_TSTraj_T;

/* Parameters (default storage) */
struct P_TSTraj_T_ {
  real_T Constant2_Value;              /* Expression: eps
                                        * Referenced by: '<S1>/Constant2'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S1>/Constant'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<S1>/Switch'
                                        */
  real_T Switch2_Threshold;            /* Expression: 0
                                        * Referenced by: '<S1>/Switch2'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_TSTraj_T {
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
extern P_TSTraj_T TSTraj_P;

/* Block signals (default storage) */
extern B_TSTraj_T TSTraj_B;

/* Block states (default storage) */
extern DW_TSTraj_T TSTraj_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_TSTraj_T TSTraj_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_TSTraj_T TSTraj_Y;

/* Model entry point functions */
extern void TSTraj_initialize(void);
extern void TSTraj_step(void);
extern void TSTraj_terminate(void);

/* Real-time Model object */
extern RT_MODEL_TSTraj_T *const TSTraj_M;

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
 * '<Root>' : 'TSTraj'
 * '<S1>'   : 'TSTraj/Subsystem'
 * '<S2>'   : 'TSTraj/Subsystem/GetT,V,Vdot'
 * '<S3>'   : 'TSTraj/Subsystem/MATLAB Function'
 * '<S4>'   : 'TSTraj/Subsystem/MATLAB Function1'
 * '<S5>'   : 'TSTraj/Subsystem/TransToRp'
 * '<S6>'   : 'TSTraj/Subsystem/TransToRp1'
 */
#endif                                 /* TSTraj_h_ */
