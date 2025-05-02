/*
 * JSTraj.c
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

#include "JSTraj.h"
#include "JSTraj_types.h"
#include "rtwtypes.h"
#include <emmintrin.h>
#include <string.h>
#include "JSTraj_private.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Block states (default storage) */
DW_JSTraj_T JSTraj_DW;

/* External inputs (root inport signals with default storage) */
ExtU_JSTraj_T JSTraj_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_JSTraj_T JSTraj_Y;

/* Real-time model */
static RT_MODEL_JSTraj_T JSTraj_M_;
RT_MODEL_JSTraj_T *const JSTraj_M = &JSTraj_M_;

/* Forward declaration for local functions */
static void JSTraj_generateQuinticCoeffs(const real_T posPts[2], const real_T
  velPts[2], const real_T accPts[2], real_T finalTime, real_T coeffVec[6]);
static void JS_addFlatSegmentsToPPFormParts(const real_T oldbreaks[2], const
  real_T oldCoeffs[36], real_T newBreaks[4], real_T newCoefs[108]);
static void PolyTrajSys_updateStoredPPForms(robotics_slcore_internal_bloc_T *obj,
  const real_T pp_breaks[4], const real_T pp_coefs[108]);
static void JSTraj_PolyTrajSys_setupImpl(robotics_slcore_internal_bloc_T *obj);
static void JSTraj_ppval(const real_T pp_breaks[4], const real_T pp_coefs[108],
  real_T x, real_T v[6]);
real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

static void JSTraj_generateQuinticCoeffs(const real_T posPts[2], const real_T
  velPts[2], const real_T accPts[2], real_T finalTime, real_T coeffVec[6])
{
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  __m128d tmp_4;
  real_T tmp[9];
  real_T tmp_0[9];
  real_T posPts_0[3];
  real_T posPts_1[3];
  real_T coeffVec_0;
  real_T coeffVec_1;
  real_T xtmp;
  int32_T i;

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  coeffVec[0] = posPts[0];
  coeffVec[1] = velPts[0];
  coeffVec[2] = accPts[0] / 2.0;
  coeffVec[3] = 0.0;
  coeffVec[4] = 0.0;
  coeffVec[5] = 0.0;
  xtmp = rt_powd_snf(finalTime, 3.0);
  coeffVec_0 = rt_powd_snf(finalTime, 4.0);
  tmp[0] = 1.0;
  tmp[3] = finalTime;
  coeffVec_1 = finalTime * finalTime;
  tmp[6] = coeffVec_1;
  tmp[1] = 0.0;
  tmp[4] = 1.0;
  tmp[7] = 2.0 * finalTime;
  tmp[2] = 0.0;
  tmp[5] = 0.0;
  tmp[8] = 2.0;
  posPts_0[0] = posPts[1];
  posPts_0[1] = velPts[1];
  posPts_0[2] = accPts[1];
  tmp_0[0] = 10.0 / xtmp;
  tmp_0[3] = -4.0 / coeffVec_1;
  tmp_0[6] = 1.0 / (2.0 * finalTime);
  tmp_0[1] = -15.0 / coeffVec_0;
  tmp_0[4] = 7.0 / xtmp;
  tmp_0[7] = -1.0 / coeffVec_1;
  tmp_0[2] = 6.0 / rt_powd_snf(finalTime, 5.0);
  tmp_0[5] = -3.0 / coeffVec_0;
  tmp_0[8] = 1.0 / (2.0 * xtmp);
  xtmp = velPts[0];
  coeffVec_0 = posPts[0];
  coeffVec_1 = coeffVec[2];
  for (i = 0; i <= 0; i += 2) {
    tmp_1 = _mm_loadu_pd(&tmp[i + 3]);
    tmp_2 = _mm_loadu_pd(&tmp[i]);
    tmp_3 = _mm_loadu_pd(&tmp[i + 6]);
    tmp_4 = _mm_loadu_pd(&posPts_0[i]);
    _mm_storeu_pd(&posPts_1[i], _mm_sub_pd(tmp_4, _mm_add_pd(_mm_add_pd
      (_mm_mul_pd(tmp_1, _mm_set1_pd(xtmp)), _mm_mul_pd(tmp_2, _mm_set1_pd
      (coeffVec_0))), _mm_mul_pd(tmp_3, _mm_set1_pd(coeffVec_1)))));
  }

  for (i = 2; i < 3; i++) {
    posPts_1[i] = posPts_0[i] - ((tmp[i + 3] * xtmp + tmp[i] * coeffVec_0) +
      tmp[i + 6] * coeffVec_1);
  }

  xtmp = posPts_1[1];
  coeffVec_0 = posPts_1[0];
  coeffVec_1 = posPts_1[2];
  for (i = 0; i <= 0; i += 2) {
    tmp_1 = _mm_loadu_pd(&tmp_0[i + 3]);
    tmp_2 = _mm_loadu_pd(&tmp_0[i]);
    tmp_3 = _mm_loadu_pd(&tmp_0[i + 6]);
    _mm_storeu_pd(&coeffVec[i + 3], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_1,
      _mm_set1_pd(xtmp)), _mm_mul_pd(tmp_2, _mm_set1_pd(coeffVec_0))),
      _mm_mul_pd(tmp_3, _mm_set1_pd(coeffVec_1))));
  }

  for (i = 2; i < 3; i++) {
    coeffVec[i + 3] = (tmp_0[i + 3] * xtmp + tmp_0[i] * coeffVec_0) + tmp_0[i +
      6] * coeffVec_1;
  }

  xtmp = coeffVec[0];

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  coeffVec[0] = coeffVec[5];
  coeffVec[5] = xtmp;
  xtmp = coeffVec[1];

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  coeffVec[1] = coeffVec[4];
  coeffVec[4] = xtmp;
  xtmp = coeffVec[2];

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  coeffVec[2] = coeffVec[3];
  coeffVec[3] = xtmp;
}

static void JS_addFlatSegmentsToPPFormParts(const real_T oldbreaks[2], const
  real_T oldCoeffs[36], real_T newBreaks[4], real_T newCoefs[108])
{
  real_T coefsWithFlatStart[72];
  real_T newSegmentCoeffs[36];
  real_T evalPointVector[6];
  real_T holdPoint;
  int32_T b_i;
  int32_T coefsWithFlatStart_tmp;
  int32_T coefsWithFlatStart_tmp_0;
  int32_T i;
  static const int8_T tmp[6] = { 0, 0, 0, 0, 0, 1 };

  memset(&newSegmentCoeffs[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
    holdPoint = 0.0;
    for (b_i = 0; b_i < 6; b_i++) {
      /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
      holdPoint += oldCoeffs[6 * b_i + i] * (real_T)tmp[b_i];
    }

    /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
    newSegmentCoeffs[i + 30] = holdPoint;
  }

  memset(&coefsWithFlatStart[0], 0, 72U * sizeof(real_T));

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  holdPoint = oldbreaks[1] - oldbreaks[0];
  for (b_i = 0; b_i < 6; b_i++) {
    for (i = 0; i < 6; i++) {
      /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
      coefsWithFlatStart_tmp = 6 * b_i + i;
      coefsWithFlatStart_tmp_0 = 12 * b_i + i;
      coefsWithFlatStart[coefsWithFlatStart_tmp_0] =
        newSegmentCoeffs[coefsWithFlatStart_tmp];

      /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
      coefsWithFlatStart[coefsWithFlatStart_tmp_0 + 6] =
        oldCoeffs[coefsWithFlatStart_tmp];
    }

    /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
    evalPointVector[b_i] = rt_powd_snf(holdPoint, 6.0 - ((real_T)b_i + 1.0));
  }

  memset(&newSegmentCoeffs[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
    holdPoint = 0.0;
    for (b_i = 0; b_i < 6; b_i++) {
      /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
      holdPoint += coefsWithFlatStart[(12 * b_i + i) + 6] * evalPointVector[b_i];
    }

    /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
    newSegmentCoeffs[i + 30] = holdPoint;
  }

  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
    memcpy(&newCoefs[i * 18], &coefsWithFlatStart[i * 12], 12U * sizeof(real_T));
    for (b_i = 0; b_i < 6; b_i++) {
      newCoefs[(b_i + 18 * i) + 12] = newSegmentCoeffs[6 * i + b_i];
    }
  }

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  newBreaks[0] = oldbreaks[0] - 1.0;
  newBreaks[1] = oldbreaks[0];
  newBreaks[2] = oldbreaks[1];
  newBreaks[3] = oldbreaks[1] + 1.0;
}

static void PolyTrajSys_updateStoredPPForms(robotics_slcore_internal_bloc_T *obj,
  const real_T pp_breaks[4], const real_T pp_coefs[108])
{
  int32_T b_i;
  int32_T i;
  memset(&obj->PPDStruct.coefs[0], 0, 108U * sizeof(real_T));
  for (b_i = 0; b_i < 5; b_i++) {
    for (i = 0; i <= 16; i += 2) {
      /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
      _mm_storeu_pd(&obj->PPDStruct.coefs[i + 18 * (b_i + 1)], _mm_mul_pd
                    (_mm_loadu_pd(&pp_coefs[b_i * 18 + i]), _mm_set1_pd(5.0 -
        (real_T)b_i)));
    }
  }

  memset(&obj->PPDDStruct.coefs[0], 0, 108U * sizeof(real_T));
  for (b_i = 0; b_i < 5; b_i++) {
    for (i = 0; i <= 16; i += 2) {
      __m128d tmp;

      /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
      tmp = _mm_loadu_pd(&obj->PPDStruct.coefs[b_i * 18 + i]);
      _mm_storeu_pd(&obj->PPDDStruct.coefs[i + 18 * (b_i + 1)], _mm_mul_pd(tmp,
        _mm_set1_pd(5.0 - (real_T)b_i)));
    }
  }

  obj->PPStruct.breaks[0] = pp_breaks[0];
  obj->PPStruct.breaks[1] = pp_breaks[1];
  obj->PPStruct.breaks[2] = pp_breaks[2];
  obj->PPStruct.breaks[3] = pp_breaks[3];
  memcpy(&obj->PPStruct.coefs[0], &pp_coefs[0], 108U * sizeof(real_T));

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  obj->PPDStruct.breaks[0] = pp_breaks[0];
  obj->PPDDStruct.breaks[0] = pp_breaks[0];
  obj->PPDStruct.breaks[1] = pp_breaks[1];
  obj->PPDDStruct.breaks[1] = pp_breaks[1];
  obj->PPDStruct.breaks[2] = pp_breaks[2];
  obj->PPDDStruct.breaks[2] = pp_breaks[2];
  obj->PPDStruct.breaks[3] = pp_breaks[3];
  obj->PPDDStruct.breaks[3] = pp_breaks[3];
}

static void JSTraj_PolyTrajSys_setupImpl(robotics_slcore_internal_bloc_T *obj)
{
  real_T modCoeffs[108];
  real_T coefMat[36];
  real_T tmp_0[6];
  real_T modBreaks[4];
  real_T parser_Defaults[2];
  real_T parser_Defaults_0[2];
  real_T tmp[2];
  int32_T b_j;
  int32_T i;

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  obj->PrevOptInputs.f2[0] = 1.0;
  obj->PrevOptInputs.f2[1] = 1.0;
  for (i = 0; i < 12; i++) {
    /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
    obj->PrevOptInputs.f1[i] = 1.0;
    obj->PrevOptInputs.f3[i] = 1.0;
    obj->PrevOptInputs.f4[i] = 1.0;
  }

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  tmp[0] = 1.0;
  parser_Defaults[0] = 0.0;
  parser_Defaults_0[0] = 0.0;
  tmp[1] = 1.0;
  parser_Defaults[1] = 0.0;
  parser_Defaults_0[1] = 0.0;
  JSTraj_generateQuinticCoeffs(tmp, parser_Defaults, parser_Defaults_0, 1.0,
    tmp_0);
  for (b_j = 0; b_j < 6; b_j++) {
    for (i = 0; i < 6; i++) {
      /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
      coefMat[b_j + 6 * i] = tmp_0[i];
    }
  }

  tmp[0] = 1.0;
  tmp[1] = 2.0;
  JS_addFlatSegmentsToPPFormParts(tmp, coefMat, modBreaks, modCoeffs);

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  PolyTrajSys_updateStoredPPForms(obj, modBreaks, modCoeffs);
}

static void JSTraj_ppval(const real_T pp_breaks[4], const real_T pp_coefs[108],
  real_T x, real_T v[6])
{
  int32_T high_i;
  int32_T low_i;
  int32_T low_ip1;

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  if (rtIsNaN(x)) {
    for (low_ip1 = 0; low_ip1 < 6; low_ip1++) {
      v[low_ip1] = (rtNaN);
    }
  } else {
    real_T xloc;
    int32_T ic0;
    low_i = 0;
    low_ip1 = 1;
    high_i = 4;
    while (high_i > low_ip1 + 1) {
      ic0 = ((low_i + high_i) + 1) >> 1;
      if (x >= pp_breaks[ic0 - 1]) {
        low_i = ic0 - 1;
        low_ip1 = ic0;
      } else {
        high_i = ic0;
      }
    }

    low_ip1 = low_i * 6;
    xloc = x - pp_breaks[low_i];
    for (high_i = 0; high_i < 6; high_i++) {
      v[high_i] = pp_coefs[low_ip1 + high_i];
    }

    for (high_i = 0; high_i < 5; high_i++) {
      ic0 = ((high_i + 1) * 18 + low_ip1) - 1;
      for (low_i = 0; low_i <= 4; low_i += 2) {
        __m128d tmp;
        tmp = _mm_loadu_pd(&v[low_i]);
        _mm_storeu_pd(&v[low_i], _mm_add_pd(_mm_loadu_pd(&pp_coefs[(ic0 + low_i)
          + 1]), _mm_mul_pd(_mm_set1_pd(xloc), tmp)));
      }
    }
  }

  /* End of Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
}

/* Model step function */
void JSTraj_step(void)
{
  __m128d tmp_0;
  e_cell_wrap_JSTraj_T c;
  real_T modCoeffs[108];
  real_T coefMat[36];
  real_T rtb_MatrixConcatenate[12];
  real_T tmp[6];
  real_T modBreaks[4];
  real_T paramInputArgs[2];
  real_T paramInputArgs_0[2];
  real_T rtb_MatrixConcatenate1[2];
  real_T rtb_MatrixConcatenate_0[2];
  real_T c_f1;
  real_T finalTime;
  int32_T b_k;
  int32_T i;
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T out;
  boolean_T p;
  for (i = 0; i < 6; i++) {
    /* SignalConversion generated from: '<Root>/Matrix Concatenate' incorporates:
     *  Inport: '<Root>/q_start'
     */
    rtb_MatrixConcatenate[i] = JSTraj_U.q_start[i];

    /* SignalConversion generated from: '<Root>/Matrix Concatenate' incorporates:
     *  Inport: '<Root>/q_end'
     */
    rtb_MatrixConcatenate[i + 6] = JSTraj_U.q_end[i];
  }

  /* SignalConversion generated from: '<Root>/Matrix Concatenate1' incorporates:
   *  Inport: '<Root>/T0'
   */
  rtb_MatrixConcatenate1[0] = JSTraj_U.T0;

  /* Switch: '<Root>/Switch' incorporates:
   *  Constant: '<Root>/Constant'
   *  Inport: '<Root>/T0'
   *  Inport: '<Root>/Tf'
   *  Sum: '<Root>/Sum'
   */
  if (JSTraj_U.Tf > JSTraj_P.Switch_Threshold) {
    rtb_MatrixConcatenate1[1] = JSTraj_U.Tf;
  } else {
    rtb_MatrixConcatenate1[1] = JSTraj_U.T0 + JSTraj_P.Constant_Value;
  }

  /* End of Switch: '<Root>/Switch' */

  /* Gain: '<Root>/Gain' incorporates:
   *  Concatenate: '<Root>/Matrix Concatenate'
   */
  for (i = 0; i <= 10; i += 2) {
    tmp_0 = _mm_loadu_pd(&rtb_MatrixConcatenate[i]);
    _mm_storeu_pd(&c.f1[i], _mm_mul_pd(_mm_set1_pd(JSTraj_P.Gain_Gain), tmp_0));
  }

  /* End of Gain: '<Root>/Gain' */

  /* MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
   *  Concatenate: '<Root>/Matrix Concatenate'
   *  Concatenate: '<Root>/Matrix Concatenate1'
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/T0'
   *  Inport: '<Root>/gt'
   */
  if (JSTraj_DW.obj.TunablePropsChanged) {
    JSTraj_DW.obj.TunablePropsChanged = false;
    JSTraj_DW.obj.PPFormUpdatedNeeded = false;
  }

  guard1 = false;
  if (JSTraj_DW.obj.PPFormUpdatedNeeded) {
    guard1 = true;
  } else {
    p = false;
    out = true;
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k < 12)) {
      if (!(rtb_MatrixConcatenate[b_k] == JSTraj_DW.obj.PrevOptInputs.f1[b_k]))
      {
        out = false;
        exitg1 = true;
      } else {
        b_k++;
      }
    }

    if (out) {
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k < 2)) {
        if (!(rtb_MatrixConcatenate1[b_k] == JSTraj_DW.obj.PrevOptInputs.f2[b_k]))
        {
          out = false;
          exitg1 = true;
        } else {
          b_k++;
        }
      }

      if (out) {
        b_k = 0;
        exitg1 = false;
        while ((!exitg1) && (b_k < 12)) {
          if (!(c.f1[b_k] == JSTraj_DW.obj.PrevOptInputs.f3[b_k])) {
            out = false;
            exitg1 = true;
          } else {
            b_k++;
          }
        }

        if (out) {
          b_k = 0;
          exitg1 = false;
          while ((!exitg1) && (b_k < 12)) {
            if (!(c.f1[b_k] == JSTraj_DW.obj.PrevOptInputs.f4[b_k])) {
              out = false;
              exitg1 = true;
            } else {
              b_k++;
            }
          }
        }
      }
    }

    if (out) {
      p = true;
    }

    JSTraj_DW.obj.PrevOptInputs.f2[0] = JSTraj_U.T0;
    JSTraj_DW.obj.PrevOptInputs.f2[1] = rtb_MatrixConcatenate1[1];
    for (i = 0; i < 12; i++) {
      JSTraj_DW.obj.PrevOptInputs.f1[i] = rtb_MatrixConcatenate[i];

      /* Start for Gain: '<Root>/Gain' incorporates:
       *  Concatenate: '<Root>/Matrix Concatenate'
       */
      c_f1 = c.f1[i];
      JSTraj_DW.obj.PrevOptInputs.f3[i] = c_f1;
      JSTraj_DW.obj.PrevOptInputs.f4[i] = c_f1;
    }

    if (!p) {
      guard1 = true;
    }
  }

  if (guard1) {
    finalTime = rtb_MatrixConcatenate1[1] - JSTraj_U.T0;
    for (b_k = 0; b_k < 6; b_k++) {
      rtb_MatrixConcatenate_0[0] = rtb_MatrixConcatenate[b_k];
      c_f1 = c.f1[b_k];
      paramInputArgs[0] = c_f1;
      paramInputArgs_0[0] = c_f1;
      rtb_MatrixConcatenate_0[1] = rtb_MatrixConcatenate[b_k + 6];
      c_f1 = c.f1[b_k + 6];
      paramInputArgs[1] = c_f1;
      paramInputArgs_0[1] = c_f1;
      JSTraj_generateQuinticCoeffs(rtb_MatrixConcatenate_0, paramInputArgs,
        paramInputArgs_0, finalTime, tmp);
      for (i = 0; i < 6; i++) {
        coefMat[b_k + 6 * i] = tmp[i];
      }
    }

    JS_addFlatSegmentsToPPFormParts(rtb_MatrixConcatenate1, coefMat, modBreaks,
      modCoeffs);
    PolyTrajSys_updateStoredPPForms(&JSTraj_DW.obj, modBreaks, modCoeffs);
    JSTraj_DW.obj.PPFormUpdatedNeeded = false;
  }

  c_f1 = JSTraj_U.gt;

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
   *  Inport: '<Root>/gt'
   */
  if (JSTraj_U.gt == JSTraj_DW.obj.PPStruct.breaks[2]) {
    for (i = 0; i < 1; i++) {
      c_f1 = JSTraj_DW.obj.PPStruct.breaks[2] - 2.2204460492503131E-15;
    }
  }

  /* MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
   *  Inport: '<Root>/gt'
   *  Outport: '<Root>/q_des'
   *  Outport: '<Root>/qddot_des'
   *  Outport: '<Root>/qdot_des'
   */
  JSTraj_ppval(JSTraj_DW.obj.PPStruct.breaks, JSTraj_DW.obj.PPStruct.coefs,
               JSTraj_U.gt, JSTraj_Y.q_des);
  JSTraj_ppval(JSTraj_DW.obj.PPDStruct.breaks, JSTraj_DW.obj.PPDStruct.coefs,
               c_f1, JSTraj_Y.qdot_des);
  JSTraj_ppval(JSTraj_DW.obj.PPDDStruct.breaks, JSTraj_DW.obj.PPDDStruct.coefs,
               c_f1, JSTraj_Y.qddot_des);

  /* Matfile logging */
  rt_UpdateTXYLogVars(JSTraj_M->rtwLogInfo, (&JSTraj_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.001s, 0.0s] */
    if ((rtmGetTFinal(JSTraj_M)!=-1) &&
        !((rtmGetTFinal(JSTraj_M)-JSTraj_M->Timing.taskTime0) >
          JSTraj_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(JSTraj_M, "Simulation finished");
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
  if (!(++JSTraj_M->Timing.clockTick0)) {
    ++JSTraj_M->Timing.clockTickH0;
  }

  JSTraj_M->Timing.taskTime0 = JSTraj_M->Timing.clockTick0 *
    JSTraj_M->Timing.stepSize0 + JSTraj_M->Timing.clockTickH0 *
    JSTraj_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void JSTraj_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)JSTraj_M, 0,
                sizeof(RT_MODEL_JSTraj_T));
  rtmSetTFinal(JSTraj_M, -1);
  JSTraj_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    JSTraj_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(JSTraj_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(JSTraj_M->rtwLogInfo, (NULL));
    rtliSetLogT(JSTraj_M->rtwLogInfo, "tout");
    rtliSetLogX(JSTraj_M->rtwLogInfo, "");
    rtliSetLogXFinal(JSTraj_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(JSTraj_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(JSTraj_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(JSTraj_M->rtwLogInfo, 0);
    rtliSetLogDecimation(JSTraj_M->rtwLogInfo, 1);
    rtliSetLogY(JSTraj_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(JSTraj_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(JSTraj_M->rtwLogInfo, (NULL));
  }

  /* states (dwork) */
  (void) memset((void *)&JSTraj_DW, 0,
                sizeof(DW_JSTraj_T));

  /* external inputs */
  (void)memset(&JSTraj_U, 0, sizeof(ExtU_JSTraj_T));

  /* external outputs */
  (void)memset(&JSTraj_Y, 0, sizeof(ExtY_JSTraj_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(JSTraj_M->rtwLogInfo, 0.0, rtmGetTFinal
    (JSTraj_M), JSTraj_M->Timing.stepSize0, (&rtmGetErrorStatus(JSTraj_M)));

  /* Start for MATLABSystem: '<Root>/Polynomial Trajectory' */
  JSTraj_DW.obj.PPFormUpdatedNeeded = false;
  JSTraj_DW.objisempty = true;
  JSTraj_DW.obj.isInitialized = 1;
  JSTraj_PolyTrajSys_setupImpl(&JSTraj_DW.obj);
  JSTraj_DW.obj.TunablePropsChanged = false;
}

/* Model terminate function */
void JSTraj_terminate(void)
{
  /* (no terminate code required) */
}
