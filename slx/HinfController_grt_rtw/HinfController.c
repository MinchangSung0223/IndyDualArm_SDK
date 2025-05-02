/*
 * HinfController.c
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

#include "HinfController.h"
#include "rtwtypes.h"
#include <string.h>
#include <emmintrin.h>
#include "HinfController_private.h"

/* Block states (default storage) */
DW_HinfController_T HinfController_DW;

/* External inputs (root inport signals with default storage) */
ExtU_HinfController_T HinfController_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_HinfController_T HinfController_Y;

/* Real-time model */
static RT_MODEL_HinfController_T HinfController_M_;
RT_MODEL_HinfController_T *const HinfController_M = &HinfController_M_;

/* Model step function */
void HinfController_step(void)
{
  __m128d tmp_1;
  __m128d tmp_2;
  real_T a[144];
  real_T e[12];
  real_T edot[12];
  real_T edot_0[12];
  real_T rtb_eint[12];
  real_T tmp[12];
  real_T tmp_3[2];
  real_T a_0;
  real_T tmp_0;
  int32_T i;
  int32_T i_0;
  int32_T tmp_4;

  /* MATLAB Function: '<Root>/HinfController' incorporates:
   *  Delay: '<Root>/Delay'
   *  Inport: '<Root>/HinfK'
   *  Inport: '<Root>/M'
   *  Inport: '<Root>/dt'
   *  Inport: '<Root>/q'
   *  Inport: '<Root>/q_des'
   *  Inport: '<Root>/qddot_des'
   *  Inport: '<Root>/qdot'
   *  Inport: '<Root>/qdot_des'
   */
  for (i = 0; i <= 10; i += 2) {
    tmp_1 = _mm_sub_pd(_mm_loadu_pd(&HinfController_U.q_des[i]), _mm_loadu_pd
                       (&HinfController_U.q[i]));
    _mm_storeu_pd(&e[i], tmp_1);
    _mm_storeu_pd(&edot[i], _mm_sub_pd(_mm_loadu_pd(&HinfController_U.qdot_des[i]),
      _mm_loadu_pd(&HinfController_U.qdot[i])));
    tmp_2 = _mm_loadu_pd(&HinfController_DW.Delay_DSTATE[i]);
    _mm_storeu_pd(&rtb_eint[i], _mm_add_pd(_mm_mul_pd(tmp_1, _mm_loadu_pd
      (&HinfController_U.HinfK[i])), tmp_2));
  }

  memset(&a[0], 0, 144U * sizeof(real_T));
  for (i = 0; i < 12; i++) {
    a[i + 12 * i] = HinfController_U.dt[i];
    _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd(20.0,
      100.0), _mm_set1_pd(e[i])), _mm_set_pd(edot[i],
      HinfController_U.qddot_des[i])), _mm_mul_pd(_mm_set_pd(100.0, 20.0),
      _mm_set_pd(rtb_eint[i], edot[i]))));
    tmp[i] = tmp_3[0];
    edot_0[i] = tmp_3[1];
  }

  for (i = 0; i < 12; i++) {
    tmp_0 = 0.0;
    a_0 = 0.0;
    for (i_0 = 0; i_0 < 12; i_0++) {
      tmp_4 = 12 * i_0 + i;
      _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(a[tmp_4],
        HinfController_U.M[tmp_4]), _mm_set_pd(edot_0[i_0], tmp[i_0])),
        _mm_set_pd(a_0, tmp_0)));
      tmp_0 = tmp_3[0];
      a_0 = tmp_3[1];
    }

    /* Outport: '<Root>/tau' incorporates:
     *  Inport: '<Root>/M'
     *  Inport: '<Root>/c'
     *  Inport: '<Root>/g'
     */
    HinfController_Y.tau[i] = ((tmp_0 + HinfController_U.c[i]) +
      HinfController_U.g[i]) + a_0;

    /* Update for Delay: '<Root>/Delay' */
    HinfController_DW.Delay_DSTATE[i] = rtb_eint[i];
  }

  /* End of MATLAB Function: '<Root>/HinfController' */

  /* Matfile logging */
  rt_UpdateTXYLogVars(HinfController_M->rtwLogInfo,
                      (&HinfController_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.001s, 0.0s] */
    if ((rtmGetTFinal(HinfController_M)!=-1) &&
        !((rtmGetTFinal(HinfController_M)-HinfController_M->Timing.taskTime0) >
          HinfController_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(HinfController_M, "Simulation finished");
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
  if (!(++HinfController_M->Timing.clockTick0)) {
    ++HinfController_M->Timing.clockTickH0;
  }

  HinfController_M->Timing.taskTime0 = HinfController_M->Timing.clockTick0 *
    HinfController_M->Timing.stepSize0 + HinfController_M->Timing.clockTickH0 *
    HinfController_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void HinfController_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)HinfController_M, 0,
                sizeof(RT_MODEL_HinfController_T));
  rtmSetTFinal(HinfController_M, -1);
  HinfController_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    HinfController_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(HinfController_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(HinfController_M->rtwLogInfo, (NULL));
    rtliSetLogT(HinfController_M->rtwLogInfo, "tout");
    rtliSetLogX(HinfController_M->rtwLogInfo, "");
    rtliSetLogXFinal(HinfController_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(HinfController_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(HinfController_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(HinfController_M->rtwLogInfo, 0);
    rtliSetLogDecimation(HinfController_M->rtwLogInfo, 1);
    rtliSetLogY(HinfController_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(HinfController_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(HinfController_M->rtwLogInfo, (NULL));
  }

  /* states (dwork) */
  (void) memset((void *)&HinfController_DW, 0,
                sizeof(DW_HinfController_T));

  /* external inputs */
  (void)memset(&HinfController_U, 0, sizeof(ExtU_HinfController_T));

  /* external outputs */
  (void)memset(&HinfController_Y, 0, sizeof(ExtY_HinfController_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(HinfController_M->rtwLogInfo, 0.0,
    rtmGetTFinal(HinfController_M), HinfController_M->Timing.stepSize0,
    (&rtmGetErrorStatus(HinfController_M)));

  {
    int32_T i;

    /* InitializeConditions for Delay: '<Root>/Delay' */
    for (i = 0; i < 12; i++) {
      HinfController_DW.Delay_DSTATE[i] =
        HinfController_P.Delay_InitialCondition;
    }

    /* End of InitializeConditions for Delay: '<Root>/Delay' */
  }
}

/* Model terminate function */
void HinfController_terminate(void)
{
  /* (no terminate code required) */
}
