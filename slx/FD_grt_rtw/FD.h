/*
 * FD.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "FD".
 *
 * Model version              : 1.10
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Fri May  2 15:30:14 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef FD_h_
#define FD_h_
#ifndef FD_COMMON_INCLUDES_
#define FD_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "FD_cd27307_1_gateway.h"
#include "sm_discr_RtwAdvancerData.h"
#endif                                 /* FD_COMMON_INCLUDES_ */

#include "FD_types.h"
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

/* Block signals for system '<S1>/MATLAB Function' */
typedef struct {
  real_T T[16];                        /* '<S1>/MATLAB Function' */
} B_MATLABFunction_FD_T;

/* Block signals (default storage) */
typedef struct {
  real_T TmpSignalConversionAtOUTPUT_1_0[12];/* '<S11>/RTP_1' */
  real_T STATE_1[24];                  /* '<S114>/STATE_1' */
  real_T INPUT_1_1_1[4];               /* '<S114>/INPUT_1_1_1' */
  real_T INPUT_2_1_1[4];               /* '<S114>/INPUT_2_1_1' */
  real_T INPUT_3_1_1[4];               /* '<S114>/INPUT_3_1_1' */
  real_T INPUT_4_1_1[4];               /* '<S114>/INPUT_4_1_1' */
  real_T INPUT_5_1_1[4];               /* '<S114>/INPUT_5_1_1' */
  real_T INPUT_6_1_1[4];               /* '<S114>/INPUT_6_1_1' */
  real_T INPUT_9_1_1[4];               /* '<S114>/INPUT_9_1_1' */
  real_T INPUT_10_1_1[4];              /* '<S114>/INPUT_10_1_1' */
  real_T INPUT_11_1_1[4];              /* '<S114>/INPUT_11_1_1' */
  real_T INPUT_12_1_1[4];              /* '<S114>/INPUT_12_1_1' */
  real_T INPUT_13_1_1[4];              /* '<S114>/INPUT_13_1_1' */
  real_T INPUT_14_1_1[4];              /* '<S114>/INPUT_14_1_1' */
  real_T INPUT_7_1_1[4];               /* '<S114>/INPUT_7_1_1' */
  real_T INPUT_7_1_2[4];               /* '<S114>/INPUT_7_1_2' */
  real_T INPUT_7_1_3[4];               /* '<S114>/INPUT_7_1_3' */
  real_T INPUT_8_1_1[4];               /* '<S114>/INPUT_8_1_1' */
  real_T INPUT_8_1_2[4];               /* '<S114>/INPUT_8_1_2' */
  real_T INPUT_8_1_3[4];               /* '<S114>/INPUT_8_1_3' */
  real_T INPUT_15_1_1[4];              /* '<S114>/INPUT_15_1_1' */
  real_T INPUT_15_1_2[4];              /* '<S114>/INPUT_15_1_2' */
  real_T INPUT_15_1_3[4];              /* '<S114>/INPUT_15_1_3' */
  real_T INPUT_16_1_1[4];              /* '<S114>/INPUT_16_1_1' */
  real_T INPUT_16_1_2[4];              /* '<S114>/INPUT_16_1_2' */
  real_T INPUT_16_1_3[4];              /* '<S114>/INPUT_16_1_3' */
  B_MATLABFunction_FD_T sf_MATLABFunction1;/* '<S1>/MATLAB Function1' */
  B_MATLABFunction_FD_T sf_MATLABFunction;/* '<S1>/MATLAB Function' */
} B_FD_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T STATE_1_Discrete[24];         /* '<S114>/STATE_1' */
  real_T STATE_1_FirstOutput;          /* '<S114>/STATE_1' */
  real_T OUTPUT_1_0_FirstOutput;       /* '<S114>/OUTPUT_1_0' */
  real_T INPUT_1_1_1_Discrete[2];      /* '<S114>/INPUT_1_1_1' */
  real_T INPUT_2_1_1_Discrete[2];      /* '<S114>/INPUT_2_1_1' */
  real_T INPUT_3_1_1_Discrete[2];      /* '<S114>/INPUT_3_1_1' */
  real_T INPUT_4_1_1_Discrete[2];      /* '<S114>/INPUT_4_1_1' */
  real_T INPUT_5_1_1_Discrete[2];      /* '<S114>/INPUT_5_1_1' */
  real_T INPUT_6_1_1_Discrete[2];      /* '<S114>/INPUT_6_1_1' */
  real_T INPUT_9_1_1_Discrete[2];      /* '<S114>/INPUT_9_1_1' */
  real_T INPUT_10_1_1_Discrete[2];     /* '<S114>/INPUT_10_1_1' */
  real_T INPUT_11_1_1_Discrete[2];     /* '<S114>/INPUT_11_1_1' */
  real_T INPUT_12_1_1_Discrete[2];     /* '<S114>/INPUT_12_1_1' */
  real_T INPUT_13_1_1_Discrete[2];     /* '<S114>/INPUT_13_1_1' */
  real_T INPUT_14_1_1_Discrete[2];     /* '<S114>/INPUT_14_1_1' */
  real_T INPUT_7_1_1_Discrete[2];      /* '<S114>/INPUT_7_1_1' */
  real_T INPUT_7_1_2_Discrete[2];      /* '<S114>/INPUT_7_1_2' */
  real_T INPUT_7_1_3_Discrete[2];      /* '<S114>/INPUT_7_1_3' */
  real_T INPUT_8_1_1_Discrete[2];      /* '<S114>/INPUT_8_1_1' */
  real_T INPUT_8_1_2_Discrete[2];      /* '<S114>/INPUT_8_1_2' */
  real_T INPUT_8_1_3_Discrete[2];      /* '<S114>/INPUT_8_1_3' */
  real_T INPUT_15_1_1_Discrete[2];     /* '<S114>/INPUT_15_1_1' */
  real_T INPUT_15_1_2_Discrete[2];     /* '<S114>/INPUT_15_1_2' */
  real_T INPUT_15_1_3_Discrete[2];     /* '<S114>/INPUT_15_1_3' */
  real_T INPUT_16_1_1_Discrete[2];     /* '<S114>/INPUT_16_1_1' */
  real_T INPUT_16_1_2_Discrete[2];     /* '<S114>/INPUT_16_1_2' */
  real_T INPUT_16_1_3_Discrete[2];     /* '<S114>/INPUT_16_1_3' */
  real_T SINK_1_FirstOutput;           /* '<S114>/SINK_1' */
  real_T STATE_1_RTP[12];              /* '<S114>/STATE_1' */
  real_T STATE_1_T0;                   /* '<S114>/STATE_1' */
  real_T STATE_1_U0[24];               /* '<S114>/STATE_1' */
  real_T STATE_1_V0[24];               /* '<S114>/STATE_1' */
  real_T STATE_1_X0[24];               /* '<S114>/STATE_1' */
  real_T STATE_1_D0;                   /* '<S114>/STATE_1' */
  real_T STATE_1_T1;                   /* '<S114>/STATE_1' */
  real_T STATE_1_U1[24];               /* '<S114>/STATE_1' */
  real_T STATE_1_V1[24];               /* '<S114>/STATE_1' */
  real_T STATE_1_X1[24];               /* '<S114>/STATE_1' */
  real_T STATE_1_D1;                   /* '<S114>/STATE_1' */
  real_T STATE_1_Work[24];             /* '<S114>/STATE_1' */
  real_T OUTPUT_1_0_RTP[12];           /* '<S114>/OUTPUT_1_0' */
  real_T OUTPUT_1_0_T;                 /* '<S114>/OUTPUT_1_0' */
  real_T OUTPUT_1_0_U[24];             /* '<S114>/OUTPUT_1_0' */
  real_T OUTPUT_1_0_V[24];             /* '<S114>/OUTPUT_1_0' */
  real_T OUTPUT_1_0_X[24];             /* '<S114>/OUTPUT_1_0' */
  real_T OUTPUT_1_0_D;                 /* '<S114>/OUTPUT_1_0' */
  real_T OUTPUT_1_0_Y[48];             /* '<S114>/OUTPUT_1_0' */
  void* STATE_1_RtwData;               /* '<S114>/STATE_1' */
  void* OUTPUT_1_0_RtwData;            /* '<S114>/OUTPUT_1_0' */
  void* SINK_1_RtwLogger;              /* '<S114>/SINK_1' */
  void* SINK_1_RtwLogBuffer;           /* '<S114>/SINK_1' */
  void* SINK_1_RtwLogFcnManager;       /* '<S114>/SINK_1' */
  void* SINK_1_InstRtwLogger;          /* '<S114>/SINK_1' */
  void* SINK_1_InstRtwLogBuffer;       /* '<S114>/SINK_1' */
  int_T STATE_1_M0;                    /* '<S114>/STATE_1' */
  int_T STATE_1_M1;                    /* '<S114>/STATE_1' */
  int_T OUTPUT_1_0_M;                  /* '<S114>/OUTPUT_1_0' */
} DW_FD_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T tau[12];                      /* '<Root>/tau' */
  real_T Fext_r[6];                    /* '<Root>/Fext_r' */
  real_T Fext_l[6];                    /* '<Root>/Fext_l' */
} ExtU_FD_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T q[12];                        /* '<Root>/q' */
  real_T qdot[12];                     /* '<Root>/qdot' */
} ExtY_FD_T;

/* Real-time Model Data Structure */
struct tag_RTM_FD_T {
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
extern B_FD_T FD_B;

/* Block states (default storage) */
extern DW_FD_T FD_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_FD_T FD_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_FD_T FD_Y;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real_T q_init_l1;               /* Variable: q_init_l1
                                        * Referenced by: '<S2>/Subsystem_around_RTP_9E3B9D80_PositionTargetValue'
                                        */
extern real_T q_init_l2;               /* Variable: q_init_l2
                                        * Referenced by: '<S2>/Subsystem_around_RTP_E93CAD16_PositionTargetValue'
                                        */
extern real_T q_init_l3;               /* Variable: q_init_l3
                                        * Referenced by: '<S2>/Subsystem_around_RTP_91227F85_PositionTargetValue'
                                        */
extern real_T q_init_l4;               /* Variable: q_init_l4
                                        * Referenced by: '<S2>/Subsystem_around_RTP_E6254F13_PositionTargetValue'
                                        */
extern real_T q_init_l5;               /* Variable: q_init_l5
                                        * Referenced by: '<S2>/Subsystem_around_RTP_7F2C1EA9_PositionTargetValue'
                                        */
extern real_T q_init_l6;               /* Variable: q_init_l6
                                        * Referenced by: '<S2>/Subsystem_around_RTP_082B2E3F_PositionTargetValue'
                                        */
extern real_T q_init_r1;               /* Variable: q_init_r1
                                        * Referenced by: '<S10>/Subsystem_around_RTP_959B89D2_PositionTargetValue'
                                        */
extern real_T q_init_r2;               /* Variable: q_init_r2
                                        * Referenced by: '<S10>/Subsystem_around_RTP_0C92D868_PositionTargetValue'
                                        */
extern real_T q_init_r3;               /* Variable: q_init_r3
                                        * Referenced by: '<S10>/Subsystem_around_RTP_7B95E8FE_PositionTargetValue'
                                        */
extern real_T q_init_r4;               /* Variable: q_init_r4
                                        * Referenced by: '<S10>/Subsystem_around_RTP_EB2AF56F_PositionTargetValue'
                                        */
extern real_T q_init_r5;               /* Variable: q_init_r5
                                        * Referenced by: '<S10>/Subsystem_around_RTP_9C2DC5F9_PositionTargetValue'
                                        */
extern real_T q_init_r6;               /* Variable: q_init_r6
                                        * Referenced by: '<S10>/Subsystem_around_RTP_E7B7C304_PositionTargetValue'
                                        */

/* Model entry point functions */
extern void FD_initialize(void);
extern void FD_step(void);
extern void FD_terminate(void);

/* Real-time Model object */
extern RT_MODEL_FD_T *const FD_M;

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
 * '<Root>' : 'FD'
 * '<S1>'   : 'FD/DualArm'
 * '<S2>'   : 'FD/DualArm/Left'
 * '<S3>'   : 'FD/DualArm/MATLAB Function'
 * '<S4>'   : 'FD/DualArm/MATLAB Function1'
 * '<S5>'   : 'FD/DualArm/MATLAB Function2'
 * '<S6>'   : 'FD/DualArm/PS-Simulink Converter'
 * '<S7>'   : 'FD/DualArm/PS-Simulink Converter1'
 * '<S8>'   : 'FD/DualArm/PS-Simulink Converter2'
 * '<S9>'   : 'FD/DualArm/PS-Simulink Converter3'
 * '<S10>'  : 'FD/DualArm/Right'
 * '<S11>'  : 'FD/DualArm/Solver Configuration'
 * '<S12>'  : 'FD/DualArm/Subsystem'
 * '<S13>'  : 'FD/DualArm/world'
 * '<S14>'  : 'FD/DualArm/Left/PS-Simulink Converter1'
 * '<S15>'  : 'FD/DualArm/Left/PS-Simulink Converter10'
 * '<S16>'  : 'FD/DualArm/Left/PS-Simulink Converter11'
 * '<S17>'  : 'FD/DualArm/Left/PS-Simulink Converter12'
 * '<S18>'  : 'FD/DualArm/Left/PS-Simulink Converter2'
 * '<S19>'  : 'FD/DualArm/Left/PS-Simulink Converter3'
 * '<S20>'  : 'FD/DualArm/Left/PS-Simulink Converter4'
 * '<S21>'  : 'FD/DualArm/Left/PS-Simulink Converter5'
 * '<S22>'  : 'FD/DualArm/Left/PS-Simulink Converter6'
 * '<S23>'  : 'FD/DualArm/Left/PS-Simulink Converter7'
 * '<S24>'  : 'FD/DualArm/Left/PS-Simulink Converter8'
 * '<S25>'  : 'FD/DualArm/Left/PS-Simulink Converter9'
 * '<S26>'  : 'FD/DualArm/Left/Simulink-PS Converter'
 * '<S27>'  : 'FD/DualArm/Left/Simulink-PS Converter1'
 * '<S28>'  : 'FD/DualArm/Left/Simulink-PS Converter2'
 * '<S29>'  : 'FD/DualArm/Left/Simulink-PS Converter3'
 * '<S30>'  : 'FD/DualArm/Left/Simulink-PS Converter4'
 * '<S31>'  : 'FD/DualArm/Left/Simulink-PS Converter5'
 * '<S32>'  : 'FD/DualArm/Left/Simulink-PS Converter6'
 * '<S33>'  : 'FD/DualArm/Left/Simulink-PS Converter7'
 * '<S34>'  : 'FD/DualArm/Left/l_0'
 * '<S35>'  : 'FD/DualArm/Left/l_1'
 * '<S36>'  : 'FD/DualArm/Left/l_2'
 * '<S37>'  : 'FD/DualArm/Left/l_3'
 * '<S38>'  : 'FD/DualArm/Left/l_4'
 * '<S39>'  : 'FD/DualArm/Left/l_5'
 * '<S40>'  : 'FD/DualArm/Left/l_6'
 * '<S41>'  : 'FD/DualArm/Left/l_tcp'
 * '<S42>'  : 'FD/DualArm/Left/PS-Simulink Converter1/EVAL_KEY'
 * '<S43>'  : 'FD/DualArm/Left/PS-Simulink Converter10/EVAL_KEY'
 * '<S44>'  : 'FD/DualArm/Left/PS-Simulink Converter11/EVAL_KEY'
 * '<S45>'  : 'FD/DualArm/Left/PS-Simulink Converter12/EVAL_KEY'
 * '<S46>'  : 'FD/DualArm/Left/PS-Simulink Converter2/EVAL_KEY'
 * '<S47>'  : 'FD/DualArm/Left/PS-Simulink Converter3/EVAL_KEY'
 * '<S48>'  : 'FD/DualArm/Left/PS-Simulink Converter4/EVAL_KEY'
 * '<S49>'  : 'FD/DualArm/Left/PS-Simulink Converter5/EVAL_KEY'
 * '<S50>'  : 'FD/DualArm/Left/PS-Simulink Converter6/EVAL_KEY'
 * '<S51>'  : 'FD/DualArm/Left/PS-Simulink Converter7/EVAL_KEY'
 * '<S52>'  : 'FD/DualArm/Left/PS-Simulink Converter8/EVAL_KEY'
 * '<S53>'  : 'FD/DualArm/Left/PS-Simulink Converter9/EVAL_KEY'
 * '<S54>'  : 'FD/DualArm/Left/Simulink-PS Converter/EVAL_KEY'
 * '<S55>'  : 'FD/DualArm/Left/Simulink-PS Converter1/EVAL_KEY'
 * '<S56>'  : 'FD/DualArm/Left/Simulink-PS Converter2/EVAL_KEY'
 * '<S57>'  : 'FD/DualArm/Left/Simulink-PS Converter3/EVAL_KEY'
 * '<S58>'  : 'FD/DualArm/Left/Simulink-PS Converter4/EVAL_KEY'
 * '<S59>'  : 'FD/DualArm/Left/Simulink-PS Converter5/EVAL_KEY'
 * '<S60>'  : 'FD/DualArm/Left/Simulink-PS Converter6/EVAL_KEY'
 * '<S61>'  : 'FD/DualArm/Left/Simulink-PS Converter7/EVAL_KEY'
 * '<S62>'  : 'FD/DualArm/PS-Simulink Converter/EVAL_KEY'
 * '<S63>'  : 'FD/DualArm/PS-Simulink Converter1/EVAL_KEY'
 * '<S64>'  : 'FD/DualArm/PS-Simulink Converter2/EVAL_KEY'
 * '<S65>'  : 'FD/DualArm/PS-Simulink Converter3/EVAL_KEY'
 * '<S66>'  : 'FD/DualArm/Right/PS-Simulink Converter'
 * '<S67>'  : 'FD/DualArm/Right/PS-Simulink Converter1'
 * '<S68>'  : 'FD/DualArm/Right/PS-Simulink Converter10'
 * '<S69>'  : 'FD/DualArm/Right/PS-Simulink Converter11'
 * '<S70>'  : 'FD/DualArm/Right/PS-Simulink Converter2'
 * '<S71>'  : 'FD/DualArm/Right/PS-Simulink Converter3'
 * '<S72>'  : 'FD/DualArm/Right/PS-Simulink Converter4'
 * '<S73>'  : 'FD/DualArm/Right/PS-Simulink Converter5'
 * '<S74>'  : 'FD/DualArm/Right/PS-Simulink Converter6'
 * '<S75>'  : 'FD/DualArm/Right/PS-Simulink Converter7'
 * '<S76>'  : 'FD/DualArm/Right/PS-Simulink Converter8'
 * '<S77>'  : 'FD/DualArm/Right/PS-Simulink Converter9'
 * '<S78>'  : 'FD/DualArm/Right/Simulink-PS Converter'
 * '<S79>'  : 'FD/DualArm/Right/Simulink-PS Converter1'
 * '<S80>'  : 'FD/DualArm/Right/Simulink-PS Converter2'
 * '<S81>'  : 'FD/DualArm/Right/Simulink-PS Converter3'
 * '<S82>'  : 'FD/DualArm/Right/Simulink-PS Converter4'
 * '<S83>'  : 'FD/DualArm/Right/Simulink-PS Converter5'
 * '<S84>'  : 'FD/DualArm/Right/Simulink-PS Converter6'
 * '<S85>'  : 'FD/DualArm/Right/Simulink-PS Converter7'
 * '<S86>'  : 'FD/DualArm/Right/r_0'
 * '<S87>'  : 'FD/DualArm/Right/r_1'
 * '<S88>'  : 'FD/DualArm/Right/r_2'
 * '<S89>'  : 'FD/DualArm/Right/r_3'
 * '<S90>'  : 'FD/DualArm/Right/r_4'
 * '<S91>'  : 'FD/DualArm/Right/r_5'
 * '<S92>'  : 'FD/DualArm/Right/r_6'
 * '<S93>'  : 'FD/DualArm/Right/r_tcp'
 * '<S94>'  : 'FD/DualArm/Right/PS-Simulink Converter/EVAL_KEY'
 * '<S95>'  : 'FD/DualArm/Right/PS-Simulink Converter1/EVAL_KEY'
 * '<S96>'  : 'FD/DualArm/Right/PS-Simulink Converter10/EVAL_KEY'
 * '<S97>'  : 'FD/DualArm/Right/PS-Simulink Converter11/EVAL_KEY'
 * '<S98>'  : 'FD/DualArm/Right/PS-Simulink Converter2/EVAL_KEY'
 * '<S99>'  : 'FD/DualArm/Right/PS-Simulink Converter3/EVAL_KEY'
 * '<S100>' : 'FD/DualArm/Right/PS-Simulink Converter4/EVAL_KEY'
 * '<S101>' : 'FD/DualArm/Right/PS-Simulink Converter5/EVAL_KEY'
 * '<S102>' : 'FD/DualArm/Right/PS-Simulink Converter6/EVAL_KEY'
 * '<S103>' : 'FD/DualArm/Right/PS-Simulink Converter7/EVAL_KEY'
 * '<S104>' : 'FD/DualArm/Right/PS-Simulink Converter8/EVAL_KEY'
 * '<S105>' : 'FD/DualArm/Right/PS-Simulink Converter9/EVAL_KEY'
 * '<S106>' : 'FD/DualArm/Right/Simulink-PS Converter/EVAL_KEY'
 * '<S107>' : 'FD/DualArm/Right/Simulink-PS Converter1/EVAL_KEY'
 * '<S108>' : 'FD/DualArm/Right/Simulink-PS Converter2/EVAL_KEY'
 * '<S109>' : 'FD/DualArm/Right/Simulink-PS Converter3/EVAL_KEY'
 * '<S110>' : 'FD/DualArm/Right/Simulink-PS Converter4/EVAL_KEY'
 * '<S111>' : 'FD/DualArm/Right/Simulink-PS Converter5/EVAL_KEY'
 * '<S112>' : 'FD/DualArm/Right/Simulink-PS Converter6/EVAL_KEY'
 * '<S113>' : 'FD/DualArm/Right/Simulink-PS Converter7/EVAL_KEY'
 * '<S114>' : 'FD/DualArm/Solver Configuration/EVAL_KEY'
 * '<S115>' : 'FD/DualArm/Subsystem/body'
 */
#endif                                 /* FD_h_ */
