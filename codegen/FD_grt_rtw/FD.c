/*
 * FD.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "FD".
 *
 * Model version              : 1.15
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Sat May  3 16:25:30 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "FD.h"
#include "rtwtypes.h"
#include "FD_private.h"
#include <string.h>
#include <stddef.h>

/* Exported block parameters */
real_T q_init_l1 = 0.165747;           /* Variable: q_init_l1
                                        * Referenced by: '<S2>/Subsystem_around_RTP_646A00CF_PositionTargetValue'
                                        */
real_T q_init_l2 = 0.819274;           /* Variable: q_init_l2
                                        * Referenced by: '<S2>/Subsystem_around_RTP_136D3059_PositionTargetValue'
                                        */
real_T q_init_l3 = 1.26369;            /* Variable: q_init_l3
                                        * Referenced by: '<S2>/Subsystem_around_RTP_68F736A4_PositionTargetValue'
                                        */
real_T q_init_l4 = 0.604148;           /* Variable: q_init_l4
                                        * Referenced by: '<S2>/Subsystem_around_RTP_1FF00632_PositionTargetValue'
                                        */
real_T q_init_l5 = 1.0515;             /* Variable: q_init_l5
                                        * Referenced by: '<S2>/Subsystem_around_RTP_86F95788_PositionTargetValue'
                                        */
real_T q_init_l6 = -0.139217;          /* Variable: q_init_l6
                                        * Referenced by: '<S2>/Subsystem_around_RTP_F1FE671E_PositionTargetValue'
                                        */
real_T q_init_r1 = -0.165747;          /* Variable: q_init_r1
                                        * Referenced by: '<S10>/Subsystem_around_RTP_2A185AF1_PositionTargetValue'
                                        */
real_T q_init_r2 = -0.819274;          /* Variable: q_init_r2
                                        * Referenced by: '<S10>/Subsystem_around_RTP_5D1F6A67_PositionTargetValue'
                                        */
real_T q_init_r3 = -1.26369;           /* Variable: q_init_r3
                                        * Referenced by: '<S10>/Subsystem_around_RTP_C4163BDD_PositionTargetValue'
                                        */
real_T q_init_r4 = -0.604148;          /* Variable: q_init_r4
                                        * Referenced by: '<S10>/Subsystem_around_RTP_B3110B4B_PositionTargetValue'
                                        */
real_T q_init_r5 = -1.0515;            /* Variable: q_init_r5
                                        * Referenced by: '<S10>/Subsystem_around_RTP_2D759EE8_PositionTargetValue'
                                        */
real_T q_init_r6 = 0.139217;           /* Variable: q_init_r6
                                        * Referenced by: '<S10>/Subsystem_around_RTP_5A72AE7E_PositionTargetValue'
                                        */

/* Block signals (default storage) */
B_FD_T FD_B;

/* Block states (default storage) */
DW_FD_T FD_DW;

/* External inputs (root inport signals with default storage) */
ExtU_FD_T FD_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_FD_T FD_Y;

/* Real-time model */
static RT_MODEL_FD_T FD_M_;
RT_MODEL_FD_T *const FD_M = &FD_M_;

/*
 * Output and update for atomic system:
 *    '<S1>/MATLAB Function'
 *    '<S1>/MATLAB Function1'
 */
void FD_MATLABFunction(const real_T rtu_R[9], const real_T rtu_p[3],
  B_MATLABFunction_FD_T *localB)
{
  int32_T i;
  for (i = 0; i < 3; i++) {
    int32_T T_tmp;
    T_tmp = i << 2;
    localB->T[T_tmp] = rtu_R[3 * i];
    localB->T[T_tmp + 1] = rtu_R[3 * i + 1];
    localB->T[T_tmp + 2] = rtu_R[3 * i + 2];
    localB->T[i + 12] = rtu_p[i];
  }

  localB->T[3] = 0.0;
  localB->T[7] = 0.0;
  localB->T[11] = 0.0;
  localB->T[15] = 1.0;
}

/* Model step function */
void FD_step(void)
{
  real_T STATE_1_T1_tmp;

  /* SignalConversion generated from: '<S114>/OUTPUT_1_0' incorporates:
   *  Constant: '<S10>/Subsystem_around_RTP_2A185AF1_PositionTargetValue'
   *  Constant: '<S10>/Subsystem_around_RTP_2D759EE8_PositionTargetValue'
   *  Constant: '<S10>/Subsystem_around_RTP_5A72AE7E_PositionTargetValue'
   *  Constant: '<S10>/Subsystem_around_RTP_5D1F6A67_PositionTargetValue'
   *  Constant: '<S10>/Subsystem_around_RTP_B3110B4B_PositionTargetValue'
   *  Constant: '<S10>/Subsystem_around_RTP_C4163BDD_PositionTargetValue'
   *  Constant: '<S2>/Subsystem_around_RTP_136D3059_PositionTargetValue'
   *  Constant: '<S2>/Subsystem_around_RTP_1FF00632_PositionTargetValue'
   *  Constant: '<S2>/Subsystem_around_RTP_646A00CF_PositionTargetValue'
   *  Constant: '<S2>/Subsystem_around_RTP_68F736A4_PositionTargetValue'
   *  Constant: '<S2>/Subsystem_around_RTP_86F95788_PositionTargetValue'
   *  Constant: '<S2>/Subsystem_around_RTP_F1FE671E_PositionTargetValue'
   */
  FD_B.TmpSignalConversionAtOUTPUT_1_0[0] = q_init_l2;
  FD_B.TmpSignalConversionAtOUTPUT_1_0[1] = q_init_l4;
  FD_B.TmpSignalConversionAtOUTPUT_1_0[2] = q_init_r1;
  FD_B.TmpSignalConversionAtOUTPUT_1_0[3] = q_init_r5;
  FD_B.TmpSignalConversionAtOUTPUT_1_0[4] = q_init_r6;
  FD_B.TmpSignalConversionAtOUTPUT_1_0[5] = q_init_r2;
  FD_B.TmpSignalConversionAtOUTPUT_1_0[6] = q_init_l1 + 3.1415926535897931;
  FD_B.TmpSignalConversionAtOUTPUT_1_0[7] = q_init_l3;
  FD_B.TmpSignalConversionAtOUTPUT_1_0[8] = q_init_l5;
  FD_B.TmpSignalConversionAtOUTPUT_1_0[9] = q_init_r4;
  FD_B.TmpSignalConversionAtOUTPUT_1_0[10] = q_init_r3;
  FD_B.TmpSignalConversionAtOUTPUT_1_0[11] = q_init_l6;

  /* MultibodyStateDiscrete: '<S114>/STATE_1' incorporates:
   *  MultibodyOutputDiscrete: '<S114>/OUTPUT_1_0'
   */
  STATE_1_T1_tmp = FD_M->Timing.taskTime0;
  FD_DW.STATE_1_T1 = STATE_1_T1_tmp;
  FD_DW.STATE_1_U1[0] = (rtNaN);
  FD_DW.STATE_1_V1[0] = (rtNaN);
  FD_DW.STATE_1_U1[1] = (rtNaN);
  FD_DW.STATE_1_V1[1] = (rtNaN);
  FD_DW.STATE_1_U1[2] = (rtNaN);
  FD_DW.STATE_1_V1[2] = (rtNaN);
  FD_DW.STATE_1_U1[3] = (rtNaN);
  FD_DW.STATE_1_V1[3] = (rtNaN);
  FD_DW.STATE_1_U1[4] = (rtNaN);
  FD_DW.STATE_1_V1[4] = (rtNaN);
  FD_DW.STATE_1_U1[5] = (rtNaN);
  FD_DW.STATE_1_V1[5] = (rtNaN);
  FD_DW.STATE_1_U1[6] = (rtNaN);
  FD_DW.STATE_1_V1[6] = (rtNaN);
  FD_DW.STATE_1_U1[7] = (rtNaN);
  FD_DW.STATE_1_V1[7] = (rtNaN);
  FD_DW.STATE_1_U1[8] = (rtNaN);
  FD_DW.STATE_1_V1[8] = (rtNaN);
  FD_DW.STATE_1_U1[9] = (rtNaN);
  FD_DW.STATE_1_V1[9] = (rtNaN);
  FD_DW.STATE_1_U1[10] = (rtNaN);
  FD_DW.STATE_1_V1[10] = (rtNaN);
  FD_DW.STATE_1_U1[11] = (rtNaN);
  FD_DW.STATE_1_V1[11] = (rtNaN);
  FD_DW.STATE_1_U1[12] = (rtNaN);
  FD_DW.STATE_1_V1[12] = (rtNaN);
  FD_DW.STATE_1_U1[13] = (rtNaN);
  FD_DW.STATE_1_V1[13] = (rtNaN);
  FD_DW.STATE_1_U1[14] = (rtNaN);
  FD_DW.STATE_1_V1[14] = (rtNaN);
  FD_DW.STATE_1_U1[15] = (rtNaN);
  FD_DW.STATE_1_V1[15] = (rtNaN);
  FD_DW.STATE_1_U1[16] = (rtNaN);
  FD_DW.STATE_1_V1[16] = (rtNaN);
  FD_DW.STATE_1_U1[17] = (rtNaN);
  FD_DW.STATE_1_V1[17] = (rtNaN);
  FD_DW.STATE_1_U1[18] = (rtNaN);
  FD_DW.STATE_1_V1[18] = (rtNaN);
  FD_DW.STATE_1_U1[19] = (rtNaN);
  FD_DW.STATE_1_V1[19] = (rtNaN);
  FD_DW.STATE_1_U1[20] = (rtNaN);
  FD_DW.STATE_1_V1[20] = (rtNaN);
  FD_DW.STATE_1_U1[21] = (rtNaN);
  FD_DW.STATE_1_V1[21] = (rtNaN);
  FD_DW.STATE_1_U1[22] = (rtNaN);
  FD_DW.STATE_1_V1[22] = (rtNaN);
  FD_DW.STATE_1_U1[23] = (rtNaN);
  FD_DW.STATE_1_V1[23] = (rtNaN);
  if (FD_DW.STATE_1_FirstOutput == 0.0) {
    FD_DW.STATE_1_FirstOutput = 1.0;
    memcpy(&FD_DW.STATE_1_RTP[0], &FD_B.TmpSignalConversionAtOUTPUT_1_0[0], 12U *
           sizeof(real_T));
    if (rtmGetErrorStatus(FD_M) == NULL) {
      rtmSetErrorStatus(FD_M, sm_discr_RtwAdvancerData_set_parameters
                        (FD_DW.STATE_1_RtwData, FD_M->Timing.taskTime0,
                         FD_DW.STATE_1_RTP, true));
      rtmSetErrorStatus(FD_M, sm_discr_RtwAdvancerData_initialize_states
                        (FD_DW.STATE_1_RtwData, &FD_DW.STATE_1_T1,
                         FD_DW.STATE_1_U1, FD_DW.STATE_1_V1, FD_DW.STATE_1_X1,
                         &FD_DW.STATE_1_D1, &FD_DW.STATE_1_M1));
    }
  } else if (rtmGetErrorStatus(FD_M) == NULL) {
    rtmSetErrorStatus(FD_M, sm_discr_RtwAdvancerData_advance_states
                      (FD_DW.STATE_1_RtwData, &FD_DW.STATE_1_T0,
                       FD_DW.STATE_1_U0, FD_DW.STATE_1_V0, FD_DW.STATE_1_X0,
                       &FD_DW.STATE_1_D0, &FD_DW.STATE_1_M0, &FD_DW.STATE_1_T1,
                       FD_DW.STATE_1_U1, FD_DW.STATE_1_V1, FD_DW.STATE_1_X1,
                       &FD_DW.STATE_1_D1, &FD_DW.STATE_1_M1, FD_DW.STATE_1_Work));
  }

  memcpy(&FD_B.STATE_1[0], &FD_DW.STATE_1_X1[0], 24U * sizeof(real_T));
  memcpy(&FD_DW.STATE_1_Discrete[0], &FD_DW.STATE_1_X1[0], 24U * sizeof(real_T));

  /* End of MultibodyStateDiscrete: '<S114>/STATE_1' */

  /* MultibodyOutputDiscrete: '<S114>/OUTPUT_1_0' */
  if (FD_DW.OUTPUT_1_0_FirstOutput == 0.0) {
    FD_DW.OUTPUT_1_0_FirstOutput = 1.0;
    memcpy(&FD_DW.OUTPUT_1_0_RTP[0], &FD_B.TmpSignalConversionAtOUTPUT_1_0[0],
           12U * sizeof(real_T));
    if (rtmGetErrorStatus(FD_M) == NULL) {
      rtmSetErrorStatus(FD_M, sm_discr_RtwAdvancerData_set_parameters
                        (FD_DW.OUTPUT_1_0_RtwData, FD_M->Timing.taskTime0,
                         FD_DW.OUTPUT_1_0_RTP, false));
    }
  }

  FD_DW.OUTPUT_1_0_T = STATE_1_T1_tmp;
  FD_DW.OUTPUT_1_0_U[0] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[0] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[1] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[1] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[2] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[2] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[3] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[3] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[4] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[4] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[5] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[5] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[6] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[6] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[7] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[7] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[8] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[8] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[9] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[9] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[10] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[10] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[11] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[11] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[12] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[12] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[13] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[13] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[14] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[14] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[15] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[15] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[16] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[16] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[17] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[17] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[18] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[18] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[19] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[19] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[20] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[20] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[21] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[21] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[22] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[22] = (rtNaN);
  FD_DW.OUTPUT_1_0_U[23] = (rtNaN);
  FD_DW.OUTPUT_1_0_V[23] = (rtNaN);
  memcpy(&FD_DW.OUTPUT_1_0_X[0], &FD_B.STATE_1[0], 24U * sizeof(real_T));
  if (rtmGetErrorStatus(FD_M) == NULL) {
    rtmSetErrorStatus(FD_M, sm_discr_RtwAdvancerData_advance_outputs
                      (FD_DW.OUTPUT_1_0_RtwData, 0, &FD_DW.OUTPUT_1_0_T,
                       FD_DW.OUTPUT_1_0_U, FD_DW.OUTPUT_1_0_V,
                       FD_DW.OUTPUT_1_0_X, &FD_DW.OUTPUT_1_0_D,
                       &FD_DW.OUTPUT_1_0_M, FD_DW.OUTPUT_1_0_Y));
  }

  /* Outport: '<Root>/q' incorporates:
   *  Constant: '<S2>/Constant'
   *  MultibodyOutputDiscrete: '<S114>/OUTPUT_1_0'
   *  Sum: '<S2>/Sum'
   */
  FD_Y.q[0] = FD_DW.OUTPUT_1_0_Y[0] - FD_P.Constant_Value;
  FD_Y.q[1] = FD_DW.OUTPUT_1_0_Y[2];
  FD_Y.q[2] = FD_DW.OUTPUT_1_0_Y[4];
  FD_Y.q[3] = FD_DW.OUTPUT_1_0_Y[6];
  FD_Y.q[4] = FD_DW.OUTPUT_1_0_Y[8];
  FD_Y.q[5] = FD_DW.OUTPUT_1_0_Y[10];
  FD_Y.q[6] = FD_DW.OUTPUT_1_0_Y[12];
  FD_Y.q[7] = FD_DW.OUTPUT_1_0_Y[14];
  FD_Y.q[8] = FD_DW.OUTPUT_1_0_Y[16];
  FD_Y.q[9] = FD_DW.OUTPUT_1_0_Y[18];
  FD_Y.q[10] = FD_DW.OUTPUT_1_0_Y[20];
  FD_Y.q[11] = FD_DW.OUTPUT_1_0_Y[22];

  /* Outport: '<Root>/qdot' incorporates:
   *  MultibodyOutputDiscrete: '<S114>/OUTPUT_1_0'
   */
  FD_Y.qdot[0] = FD_DW.OUTPUT_1_0_Y[1];
  FD_Y.qdot[1] = FD_DW.OUTPUT_1_0_Y[3];
  FD_Y.qdot[2] = FD_DW.OUTPUT_1_0_Y[5];
  FD_Y.qdot[3] = FD_DW.OUTPUT_1_0_Y[7];
  FD_Y.qdot[4] = FD_DW.OUTPUT_1_0_Y[9];
  FD_Y.qdot[5] = FD_DW.OUTPUT_1_0_Y[11];
  FD_Y.qdot[6] = FD_DW.OUTPUT_1_0_Y[13];
  FD_Y.qdot[7] = FD_DW.OUTPUT_1_0_Y[15];
  FD_Y.qdot[8] = FD_DW.OUTPUT_1_0_Y[17];
  FD_Y.qdot[9] = FD_DW.OUTPUT_1_0_Y[19];
  FD_Y.qdot[10] = FD_DW.OUTPUT_1_0_Y[21];
  FD_Y.qdot[11] = FD_DW.OUTPUT_1_0_Y[23];

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  MultibodyOutputDiscrete: '<S114>/OUTPUT_1_0'
   */
  FD_MATLABFunction(&(&FD_DW.OUTPUT_1_0_Y[0])[24], &(&FD_DW.OUTPUT_1_0_Y[0])[33],
                    &FD_B.sf_MATLABFunction);

  /* MATLAB Function: '<S1>/MATLAB Function1' incorporates:
   *  MultibodyOutputDiscrete: '<S114>/OUTPUT_1_0'
   */
  FD_MATLABFunction(&(&FD_DW.OUTPUT_1_0_Y[0])[36], &(&FD_DW.OUTPUT_1_0_Y[0])[45],
                    &FD_B.sf_MATLABFunction1);

  /* SimscapeInputBlock: '<S114>/INPUT_1_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_1_1_1[0] = FD_U.tau[0];
  FD_B.INPUT_1_1_1[1] = 0.0;
  FD_B.INPUT_1_1_1[2] = 0.0;
  FD_DW.INPUT_1_1_1_Discrete[0] = !(FD_B.INPUT_1_1_1[0] ==
    FD_DW.INPUT_1_1_1_Discrete[1]);
  FD_DW.INPUT_1_1_1_Discrete[1] = FD_B.INPUT_1_1_1[0];
  FD_B.INPUT_1_1_1[0] = FD_DW.INPUT_1_1_1_Discrete[1];
  FD_B.INPUT_1_1_1[3] = FD_DW.INPUT_1_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_2_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_2_1_1[0] = FD_U.tau[1];
  FD_B.INPUT_2_1_1[1] = 0.0;
  FD_B.INPUT_2_1_1[2] = 0.0;
  FD_DW.INPUT_2_1_1_Discrete[0] = !(FD_B.INPUT_2_1_1[0] ==
    FD_DW.INPUT_2_1_1_Discrete[1]);
  FD_DW.INPUT_2_1_1_Discrete[1] = FD_B.INPUT_2_1_1[0];
  FD_B.INPUT_2_1_1[0] = FD_DW.INPUT_2_1_1_Discrete[1];
  FD_B.INPUT_2_1_1[3] = FD_DW.INPUT_2_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_3_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_3_1_1[0] = FD_U.tau[2];
  FD_B.INPUT_3_1_1[1] = 0.0;
  FD_B.INPUT_3_1_1[2] = 0.0;
  FD_DW.INPUT_3_1_1_Discrete[0] = !(FD_B.INPUT_3_1_1[0] ==
    FD_DW.INPUT_3_1_1_Discrete[1]);
  FD_DW.INPUT_3_1_1_Discrete[1] = FD_B.INPUT_3_1_1[0];
  FD_B.INPUT_3_1_1[0] = FD_DW.INPUT_3_1_1_Discrete[1];
  FD_B.INPUT_3_1_1[3] = FD_DW.INPUT_3_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_4_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_4_1_1[0] = FD_U.tau[3];
  FD_B.INPUT_4_1_1[1] = 0.0;
  FD_B.INPUT_4_1_1[2] = 0.0;
  FD_DW.INPUT_4_1_1_Discrete[0] = !(FD_B.INPUT_4_1_1[0] ==
    FD_DW.INPUT_4_1_1_Discrete[1]);
  FD_DW.INPUT_4_1_1_Discrete[1] = FD_B.INPUT_4_1_1[0];
  FD_B.INPUT_4_1_1[0] = FD_DW.INPUT_4_1_1_Discrete[1];
  FD_B.INPUT_4_1_1[3] = FD_DW.INPUT_4_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_5_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_5_1_1[0] = FD_U.tau[4];
  FD_B.INPUT_5_1_1[1] = 0.0;
  FD_B.INPUT_5_1_1[2] = 0.0;
  FD_DW.INPUT_5_1_1_Discrete[0] = !(FD_B.INPUT_5_1_1[0] ==
    FD_DW.INPUT_5_1_1_Discrete[1]);
  FD_DW.INPUT_5_1_1_Discrete[1] = FD_B.INPUT_5_1_1[0];
  FD_B.INPUT_5_1_1[0] = FD_DW.INPUT_5_1_1_Discrete[1];
  FD_B.INPUT_5_1_1[3] = FD_DW.INPUT_5_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_6_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_6_1_1[0] = FD_U.tau[5];
  FD_B.INPUT_6_1_1[1] = 0.0;
  FD_B.INPUT_6_1_1[2] = 0.0;
  FD_DW.INPUT_6_1_1_Discrete[0] = !(FD_B.INPUT_6_1_1[0] ==
    FD_DW.INPUT_6_1_1_Discrete[1]);
  FD_DW.INPUT_6_1_1_Discrete[1] = FD_B.INPUT_6_1_1[0];
  FD_B.INPUT_6_1_1[0] = FD_DW.INPUT_6_1_1_Discrete[1];
  FD_B.INPUT_6_1_1[3] = FD_DW.INPUT_6_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_9_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_9_1_1[0] = FD_U.tau[6];
  FD_B.INPUT_9_1_1[1] = 0.0;
  FD_B.INPUT_9_1_1[2] = 0.0;
  FD_DW.INPUT_9_1_1_Discrete[0] = !(FD_B.INPUT_9_1_1[0] ==
    FD_DW.INPUT_9_1_1_Discrete[1]);
  FD_DW.INPUT_9_1_1_Discrete[1] = FD_B.INPUT_9_1_1[0];
  FD_B.INPUT_9_1_1[0] = FD_DW.INPUT_9_1_1_Discrete[1];
  FD_B.INPUT_9_1_1[3] = FD_DW.INPUT_9_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_10_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_10_1_1[0] = FD_U.tau[7];
  FD_B.INPUT_10_1_1[1] = 0.0;
  FD_B.INPUT_10_1_1[2] = 0.0;
  FD_DW.INPUT_10_1_1_Discrete[0] = !(FD_B.INPUT_10_1_1[0] ==
    FD_DW.INPUT_10_1_1_Discrete[1]);
  FD_DW.INPUT_10_1_1_Discrete[1] = FD_B.INPUT_10_1_1[0];
  FD_B.INPUT_10_1_1[0] = FD_DW.INPUT_10_1_1_Discrete[1];
  FD_B.INPUT_10_1_1[3] = FD_DW.INPUT_10_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_11_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_11_1_1[0] = FD_U.tau[8];
  FD_B.INPUT_11_1_1[1] = 0.0;
  FD_B.INPUT_11_1_1[2] = 0.0;
  FD_DW.INPUT_11_1_1_Discrete[0] = !(FD_B.INPUT_11_1_1[0] ==
    FD_DW.INPUT_11_1_1_Discrete[1]);
  FD_DW.INPUT_11_1_1_Discrete[1] = FD_B.INPUT_11_1_1[0];
  FD_B.INPUT_11_1_1[0] = FD_DW.INPUT_11_1_1_Discrete[1];
  FD_B.INPUT_11_1_1[3] = FD_DW.INPUT_11_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_12_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_12_1_1[0] = FD_U.tau[9];
  FD_B.INPUT_12_1_1[1] = 0.0;
  FD_B.INPUT_12_1_1[2] = 0.0;
  FD_DW.INPUT_12_1_1_Discrete[0] = !(FD_B.INPUT_12_1_1[0] ==
    FD_DW.INPUT_12_1_1_Discrete[1]);
  FD_DW.INPUT_12_1_1_Discrete[1] = FD_B.INPUT_12_1_1[0];
  FD_B.INPUT_12_1_1[0] = FD_DW.INPUT_12_1_1_Discrete[1];
  FD_B.INPUT_12_1_1[3] = FD_DW.INPUT_12_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_13_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_13_1_1[0] = FD_U.tau[10];
  FD_B.INPUT_13_1_1[1] = 0.0;
  FD_B.INPUT_13_1_1[2] = 0.0;
  FD_DW.INPUT_13_1_1_Discrete[0] = !(FD_B.INPUT_13_1_1[0] ==
    FD_DW.INPUT_13_1_1_Discrete[1]);
  FD_DW.INPUT_13_1_1_Discrete[1] = FD_B.INPUT_13_1_1[0];
  FD_B.INPUT_13_1_1[0] = FD_DW.INPUT_13_1_1_Discrete[1];
  FD_B.INPUT_13_1_1[3] = FD_DW.INPUT_13_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_14_1_1' incorporates:
   *  Inport: '<Root>/tau'
   */
  FD_B.INPUT_14_1_1[0] = FD_U.tau[11];
  FD_B.INPUT_14_1_1[1] = 0.0;
  FD_B.INPUT_14_1_1[2] = 0.0;
  FD_DW.INPUT_14_1_1_Discrete[0] = !(FD_B.INPUT_14_1_1[0] ==
    FD_DW.INPUT_14_1_1_Discrete[1]);
  FD_DW.INPUT_14_1_1_Discrete[1] = FD_B.INPUT_14_1_1[0];
  FD_B.INPUT_14_1_1[0] = FD_DW.INPUT_14_1_1_Discrete[1];
  FD_B.INPUT_14_1_1[3] = FD_DW.INPUT_14_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_7_1_1' incorporates:
   *  Inport: '<Root>/Fext_l'
   */
  FD_B.INPUT_7_1_1[0] = FD_U.Fext_l[0];
  FD_B.INPUT_7_1_1[1] = 0.0;
  FD_B.INPUT_7_1_1[2] = 0.0;
  FD_DW.INPUT_7_1_1_Discrete[0] = !(FD_B.INPUT_7_1_1[0] ==
    FD_DW.INPUT_7_1_1_Discrete[1]);
  FD_DW.INPUT_7_1_1_Discrete[1] = FD_B.INPUT_7_1_1[0];
  FD_B.INPUT_7_1_1[0] = FD_DW.INPUT_7_1_1_Discrete[1];
  FD_B.INPUT_7_1_1[3] = FD_DW.INPUT_7_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_7_1_2' incorporates:
   *  Inport: '<Root>/Fext_l'
   */
  FD_B.INPUT_7_1_2[0] = FD_U.Fext_l[1];
  FD_B.INPUT_7_1_2[1] = 0.0;
  FD_B.INPUT_7_1_2[2] = 0.0;
  FD_DW.INPUT_7_1_2_Discrete[0] = !(FD_B.INPUT_7_1_2[0] ==
    FD_DW.INPUT_7_1_2_Discrete[1]);
  FD_DW.INPUT_7_1_2_Discrete[1] = FD_B.INPUT_7_1_2[0];
  FD_B.INPUT_7_1_2[0] = FD_DW.INPUT_7_1_2_Discrete[1];
  FD_B.INPUT_7_1_2[3] = FD_DW.INPUT_7_1_2_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_7_1_3' incorporates:
   *  Inport: '<Root>/Fext_l'
   */
  FD_B.INPUT_7_1_3[0] = FD_U.Fext_l[2];
  FD_B.INPUT_7_1_3[1] = 0.0;
  FD_B.INPUT_7_1_3[2] = 0.0;
  FD_DW.INPUT_7_1_3_Discrete[0] = !(FD_B.INPUT_7_1_3[0] ==
    FD_DW.INPUT_7_1_3_Discrete[1]);
  FD_DW.INPUT_7_1_3_Discrete[1] = FD_B.INPUT_7_1_3[0];
  FD_B.INPUT_7_1_3[0] = FD_DW.INPUT_7_1_3_Discrete[1];
  FD_B.INPUT_7_1_3[3] = FD_DW.INPUT_7_1_3_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_8_1_1' incorporates:
   *  Inport: '<Root>/Fext_l'
   */
  FD_B.INPUT_8_1_1[0] = FD_U.Fext_l[3];
  FD_B.INPUT_8_1_1[1] = 0.0;
  FD_B.INPUT_8_1_1[2] = 0.0;
  FD_DW.INPUT_8_1_1_Discrete[0] = !(FD_B.INPUT_8_1_1[0] ==
    FD_DW.INPUT_8_1_1_Discrete[1]);
  FD_DW.INPUT_8_1_1_Discrete[1] = FD_B.INPUT_8_1_1[0];
  FD_B.INPUT_8_1_1[0] = FD_DW.INPUT_8_1_1_Discrete[1];
  FD_B.INPUT_8_1_1[3] = FD_DW.INPUT_8_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_8_1_2' incorporates:
   *  Inport: '<Root>/Fext_l'
   */
  FD_B.INPUT_8_1_2[0] = FD_U.Fext_l[4];
  FD_B.INPUT_8_1_2[1] = 0.0;
  FD_B.INPUT_8_1_2[2] = 0.0;
  FD_DW.INPUT_8_1_2_Discrete[0] = !(FD_B.INPUT_8_1_2[0] ==
    FD_DW.INPUT_8_1_2_Discrete[1]);
  FD_DW.INPUT_8_1_2_Discrete[1] = FD_B.INPUT_8_1_2[0];
  FD_B.INPUT_8_1_2[0] = FD_DW.INPUT_8_1_2_Discrete[1];
  FD_B.INPUT_8_1_2[3] = FD_DW.INPUT_8_1_2_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_8_1_3' incorporates:
   *  Inport: '<Root>/Fext_l'
   */
  FD_B.INPUT_8_1_3[0] = FD_U.Fext_l[5];
  FD_B.INPUT_8_1_3[1] = 0.0;
  FD_B.INPUT_8_1_3[2] = 0.0;
  FD_DW.INPUT_8_1_3_Discrete[0] = !(FD_B.INPUT_8_1_3[0] ==
    FD_DW.INPUT_8_1_3_Discrete[1]);
  FD_DW.INPUT_8_1_3_Discrete[1] = FD_B.INPUT_8_1_3[0];
  FD_B.INPUT_8_1_3[0] = FD_DW.INPUT_8_1_3_Discrete[1];
  FD_B.INPUT_8_1_3[3] = FD_DW.INPUT_8_1_3_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_15_1_1' incorporates:
   *  Inport: '<Root>/Fext_r'
   */
  FD_B.INPUT_15_1_1[0] = FD_U.Fext_r[0];
  FD_B.INPUT_15_1_1[1] = 0.0;
  FD_B.INPUT_15_1_1[2] = 0.0;
  FD_DW.INPUT_15_1_1_Discrete[0] = !(FD_B.INPUT_15_1_1[0] ==
    FD_DW.INPUT_15_1_1_Discrete[1]);
  FD_DW.INPUT_15_1_1_Discrete[1] = FD_B.INPUT_15_1_1[0];
  FD_B.INPUT_15_1_1[0] = FD_DW.INPUT_15_1_1_Discrete[1];
  FD_B.INPUT_15_1_1[3] = FD_DW.INPUT_15_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_15_1_2' incorporates:
   *  Inport: '<Root>/Fext_r'
   */
  FD_B.INPUT_15_1_2[0] = FD_U.Fext_r[1];
  FD_B.INPUT_15_1_2[1] = 0.0;
  FD_B.INPUT_15_1_2[2] = 0.0;
  FD_DW.INPUT_15_1_2_Discrete[0] = !(FD_B.INPUT_15_1_2[0] ==
    FD_DW.INPUT_15_1_2_Discrete[1]);
  FD_DW.INPUT_15_1_2_Discrete[1] = FD_B.INPUT_15_1_2[0];
  FD_B.INPUT_15_1_2[0] = FD_DW.INPUT_15_1_2_Discrete[1];
  FD_B.INPUT_15_1_2[3] = FD_DW.INPUT_15_1_2_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_15_1_3' incorporates:
   *  Inport: '<Root>/Fext_r'
   */
  FD_B.INPUT_15_1_3[0] = FD_U.Fext_r[2];
  FD_B.INPUT_15_1_3[1] = 0.0;
  FD_B.INPUT_15_1_3[2] = 0.0;
  FD_DW.INPUT_15_1_3_Discrete[0] = !(FD_B.INPUT_15_1_3[0] ==
    FD_DW.INPUT_15_1_3_Discrete[1]);
  FD_DW.INPUT_15_1_3_Discrete[1] = FD_B.INPUT_15_1_3[0];
  FD_B.INPUT_15_1_3[0] = FD_DW.INPUT_15_1_3_Discrete[1];
  FD_B.INPUT_15_1_3[3] = FD_DW.INPUT_15_1_3_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_16_1_1' incorporates:
   *  Inport: '<Root>/Fext_r'
   */
  FD_B.INPUT_16_1_1[0] = FD_U.Fext_r[3];
  FD_B.INPUT_16_1_1[1] = 0.0;
  FD_B.INPUT_16_1_1[2] = 0.0;
  FD_DW.INPUT_16_1_1_Discrete[0] = !(FD_B.INPUT_16_1_1[0] ==
    FD_DW.INPUT_16_1_1_Discrete[1]);
  FD_DW.INPUT_16_1_1_Discrete[1] = FD_B.INPUT_16_1_1[0];
  FD_B.INPUT_16_1_1[0] = FD_DW.INPUT_16_1_1_Discrete[1];
  FD_B.INPUT_16_1_1[3] = FD_DW.INPUT_16_1_1_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_16_1_2' incorporates:
   *  Inport: '<Root>/Fext_r'
   */
  FD_B.INPUT_16_1_2[0] = FD_U.Fext_r[4];
  FD_B.INPUT_16_1_2[1] = 0.0;
  FD_B.INPUT_16_1_2[2] = 0.0;
  FD_DW.INPUT_16_1_2_Discrete[0] = !(FD_B.INPUT_16_1_2[0] ==
    FD_DW.INPUT_16_1_2_Discrete[1]);
  FD_DW.INPUT_16_1_2_Discrete[1] = FD_B.INPUT_16_1_2[0];
  FD_B.INPUT_16_1_2[0] = FD_DW.INPUT_16_1_2_Discrete[1];
  FD_B.INPUT_16_1_2[3] = FD_DW.INPUT_16_1_2_Discrete[0];

  /* SimscapeInputBlock: '<S114>/INPUT_16_1_3' incorporates:
   *  Inport: '<Root>/Fext_r'
   */
  FD_B.INPUT_16_1_3[0] = FD_U.Fext_r[5];
  FD_B.INPUT_16_1_3[1] = 0.0;
  FD_B.INPUT_16_1_3[2] = 0.0;
  FD_DW.INPUT_16_1_3_Discrete[0] = !(FD_B.INPUT_16_1_3[0] ==
    FD_DW.INPUT_16_1_3_Discrete[1]);
  FD_DW.INPUT_16_1_3_Discrete[1] = FD_B.INPUT_16_1_3[0];
  FD_B.INPUT_16_1_3[0] = FD_DW.INPUT_16_1_3_Discrete[1];
  FD_B.INPUT_16_1_3[3] = FD_DW.INPUT_16_1_3_Discrete[0];

  /* Update for MultibodyStateDiscrete: '<S114>/STATE_1' */
  FD_DW.STATE_1_U1[0] = FD_B.INPUT_1_1_1[0];
  FD_DW.STATE_1_V1[0] = FD_B.INPUT_1_1_1[1];
  FD_DW.STATE_1_U1[1] = FD_B.INPUT_2_1_1[0];
  FD_DW.STATE_1_V1[1] = FD_B.INPUT_2_1_1[1];
  FD_DW.STATE_1_U1[2] = FD_B.INPUT_3_1_1[0];
  FD_DW.STATE_1_V1[2] = FD_B.INPUT_3_1_1[1];
  FD_DW.STATE_1_U1[3] = FD_B.INPUT_4_1_1[0];
  FD_DW.STATE_1_V1[3] = FD_B.INPUT_4_1_1[1];
  FD_DW.STATE_1_U1[4] = FD_B.INPUT_5_1_1[0];
  FD_DW.STATE_1_V1[4] = FD_B.INPUT_5_1_1[1];
  FD_DW.STATE_1_U1[5] = FD_B.INPUT_6_1_1[0];
  FD_DW.STATE_1_V1[5] = FD_B.INPUT_6_1_1[1];
  FD_DW.STATE_1_U1[6] = FD_B.INPUT_9_1_1[0];
  FD_DW.STATE_1_V1[6] = FD_B.INPUT_9_1_1[1];
  FD_DW.STATE_1_U1[7] = FD_B.INPUT_10_1_1[0];
  FD_DW.STATE_1_V1[7] = FD_B.INPUT_10_1_1[1];
  FD_DW.STATE_1_U1[8] = FD_B.INPUT_11_1_1[0];
  FD_DW.STATE_1_V1[8] = FD_B.INPUT_11_1_1[1];
  FD_DW.STATE_1_U1[9] = FD_B.INPUT_12_1_1[0];
  FD_DW.STATE_1_V1[9] = FD_B.INPUT_12_1_1[1];
  FD_DW.STATE_1_U1[10] = FD_B.INPUT_13_1_1[0];
  FD_DW.STATE_1_V1[10] = FD_B.INPUT_13_1_1[1];
  FD_DW.STATE_1_U1[11] = FD_B.INPUT_14_1_1[0];
  FD_DW.STATE_1_V1[11] = FD_B.INPUT_14_1_1[1];
  FD_DW.STATE_1_U1[12] = FD_B.INPUT_7_1_1[0];
  FD_DW.STATE_1_V1[12] = FD_B.INPUT_7_1_1[1];
  FD_DW.STATE_1_U1[13] = FD_B.INPUT_7_1_2[0];
  FD_DW.STATE_1_V1[13] = FD_B.INPUT_7_1_2[1];
  FD_DW.STATE_1_U1[14] = FD_B.INPUT_7_1_3[0];
  FD_DW.STATE_1_V1[14] = FD_B.INPUT_7_1_3[1];
  FD_DW.STATE_1_U1[15] = FD_B.INPUT_8_1_1[0];
  FD_DW.STATE_1_V1[15] = FD_B.INPUT_8_1_1[1];
  FD_DW.STATE_1_U1[16] = FD_B.INPUT_8_1_2[0];
  FD_DW.STATE_1_V1[16] = FD_B.INPUT_8_1_2[1];
  FD_DW.STATE_1_U1[17] = FD_B.INPUT_8_1_3[0];
  FD_DW.STATE_1_V1[17] = FD_B.INPUT_8_1_3[1];
  FD_DW.STATE_1_U1[18] = FD_B.INPUT_15_1_1[0];
  FD_DW.STATE_1_V1[18] = FD_B.INPUT_15_1_1[1];
  FD_DW.STATE_1_U1[19] = FD_B.INPUT_15_1_2[0];
  FD_DW.STATE_1_V1[19] = FD_B.INPUT_15_1_2[1];
  FD_DW.STATE_1_U1[20] = FD_B.INPUT_15_1_3[0];
  FD_DW.STATE_1_V1[20] = FD_B.INPUT_15_1_3[1];
  FD_DW.STATE_1_U1[21] = FD_B.INPUT_16_1_1[0];
  FD_DW.STATE_1_V1[21] = FD_B.INPUT_16_1_1[1];
  FD_DW.STATE_1_U1[22] = FD_B.INPUT_16_1_2[0];
  FD_DW.STATE_1_V1[22] = FD_B.INPUT_16_1_2[1];
  FD_DW.STATE_1_U1[23] = FD_B.INPUT_16_1_3[0];
  FD_DW.STATE_1_V1[23] = FD_B.INPUT_16_1_3[1];
  FD_DW.STATE_1_T0 = FD_DW.STATE_1_T1;
  memcpy(&FD_DW.STATE_1_U0[0], &FD_DW.STATE_1_U1[0], 24U * sizeof(real_T));
  memcpy(&FD_DW.STATE_1_V0[0], &FD_DW.STATE_1_V1[0], 24U * sizeof(real_T));
  memcpy(&FD_DW.STATE_1_X0[0], &FD_DW.STATE_1_X1[0], 24U * sizeof(real_T));

  /* Matfile logging */
  rt_UpdateTXYLogVars(FD_M->rtwLogInfo, (&FD_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.001s, 0.0s] */
    if ((rtmGetTFinal(FD_M)!=-1) &&
        !((rtmGetTFinal(FD_M)-FD_M->Timing.taskTime0) > FD_M->Timing.taskTime0 *
          (DBL_EPSILON))) {
      rtmSetErrorStatus(FD_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++FD_M->Timing.clockTick0)) {
    ++FD_M->Timing.clockTickH0;
  }

  FD_M->Timing.taskTime0 = FD_M->Timing.clockTick0 * FD_M->Timing.stepSize0 +
    FD_M->Timing.clockTickH0 * FD_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void FD_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)FD_M, 0,
                sizeof(RT_MODEL_FD_T));
  rtmSetTFinal(FD_M, -1);
  FD_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    FD_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(FD_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(FD_M->rtwLogInfo, (NULL));
    rtliSetLogT(FD_M->rtwLogInfo, "tout");
    rtliSetLogX(FD_M->rtwLogInfo, "");
    rtliSetLogXFinal(FD_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(FD_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(FD_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(FD_M->rtwLogInfo, 0);
    rtliSetLogDecimation(FD_M->rtwLogInfo, 1);
    rtliSetLogY(FD_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(FD_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(FD_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &FD_B), 0,
                sizeof(B_FD_T));

  /* states (dwork) */
  (void) memset((void *)&FD_DW, 0,
                sizeof(DW_FD_T));

  /* external inputs */
  (void)memset(&FD_U, 0, sizeof(ExtU_FD_T));

  /* external outputs */
  (void)memset(&FD_Y, 0, sizeof(ExtY_FD_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(FD_M->rtwLogInfo, 0.0, rtmGetTFinal(FD_M),
    FD_M->Timing.stepSize0, (&rtmGetErrorStatus(FD_M)));

  /* Start for MultibodyStateDiscrete: '<S114>/STATE_1' */
  FD_DW.STATE_1_RtwData = sm_discr_RtwAdvancerData_create(FD_552be714_1_gateway(),
    0);

  /* Start for MultibodyOutputDiscrete: '<S114>/OUTPUT_1_0' */
  FD_DW.OUTPUT_1_0_RtwData = sm_discr_RtwAdvancerData_create
    (FD_552be714_1_gateway(), 0);
}

/* Model terminate function */
void FD_terminate(void)
{
  /* Terminate for MultibodyStateDiscrete: '<S114>/STATE_1' */
  sm_discr_RtwAdvancerData_destroy(FD_DW.STATE_1_RtwData);

  /* Terminate for MultibodyOutputDiscrete: '<S114>/OUTPUT_1_0' */
  sm_discr_RtwAdvancerData_destroy(FD_DW.OUTPUT_1_0_RtwData);
}
