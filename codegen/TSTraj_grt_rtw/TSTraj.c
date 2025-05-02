/*
 * TSTraj.c
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

#include "TSTraj.h"
#include "rtwtypes.h"
#include "TSTraj_types.h"
#include "TSTraj_private.h"
#include <string.h>
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>

/* Block signals (default storage) */
B_TSTraj_T TSTraj_B;

/* Block states (default storage) */
DW_TSTraj_T TSTraj_DW;

/* External inputs (root inport signals with default storage) */
ExtU_TSTraj_T TSTraj_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_TSTraj_T TSTraj_Y;

/* Real-time model */
static RT_MODEL_TSTraj_T TSTraj_M_;
RT_MODEL_TSTraj_T *const TSTraj_M = &TSTraj_M_;

/* Forward declaration for local functions */
static void TSTraj_TrapVelTrajSys_setupImpl(robotics_slcore_internal_bloc_T *obj);
static boolean_T TSTr_checkPolyForMultipleBreaks(const real_T breakMat[12]);
static void TSTraj_processPolynomialResults(const real_T breakMat[12], const
  real_T coeffMat[27], boolean_T hasMultipleBreaks, g_cell_wrap_TSTraj_T
  breaksCell[3], i_cell_wrap_TSTraj_T coeffCell[3]);
static void TSTraj_ppval(const real_T pp_breaks[6], const real_T pp_coefs_data[],
  const int32_T pp_coefs_size[3], const real_T x[2], real_T v_data[], int32_T
  v_size[2]);
static void T_generateTrajectoriesFromCoefs(const real_T breaks[4], const real_T
  coeffs_data[], const int32_T coeffs_size[2], real_T dim, const real_T t[2],
  real_T q_data[], int32_T q_size[2], real_T qd_data[], int32_T qd_size[2],
  real_T qdd_data[], int32_T qdd_size[2], real_T pp_breaks[6], real_T
  pp_coefs_data[], int32_T pp_coefs_size[3]);
static void TSTraj_trapveltraj(const real_T wayPoints[6], real_T varargin_2,
  real_T q[6], real_T qd[6], real_T qdd[6], real_T t[2],
  s_vjEZ2dxatR8VOmLd9oOqoD_TSTr_T ppCell_data[], int32_T *ppCell_size);
static real_T TSTraj_ppval_o(const real_T pp_breaks[6], const real_T pp_coefs[15],
  real_T x);
static void TrapVelTrajSys_generate1DPVAP_d(const real_T breaks[6], const real_T
  coefs_data[], const int32_T coefs_size[2], real_T pp_breaks[6], real_T
  pp_coefs[15], real_T ppd_breaks[6], real_T ppd_coefs[15], real_T ppdd_coefs[15]);

/* Forward declaration for local functions */
static real_T TSTraj_norm(const real_T x[3]);

/*
 * Output and update for atomic system:
 *    '<S1>/TransToRp'
 *    '<S1>/TransToRp1'
 */
void TSTraj_TransToRp(const real_T rtu_T[16], real_T rty_p[3],
                      B_TransToRp_TSTraj_T *localB)
{
  int32_T i;
  for (i = 0; i < 3; i++) {
    int32_T R_tmp;
    R_tmp = i << 2;
    localB->R[3 * i] = rtu_T[R_tmp];
    localB->R[3 * i + 1] = rtu_T[R_tmp + 1];
    localB->R[3 * i + 2] = rtu_T[R_tmp + 2];
    rty_p[i] = rtu_T[i + 12];
  }
}

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

static void TSTraj_TrapVelTrajSys_setupImpl(robotics_slcore_internal_bloc_T *obj)
{
  siswYcTR8LLamuD4YWmtXHC_TSTra_T expl_temp;
  real_T newSegmentCoeffs[3];
  real_T coefsWithFlatStart_0;
  int32_T i;
  int32_T trueCount;
  int8_T coefsWithFlatStart[12];
  int8_T coeffMat[9];
  int8_T coefs[9];
  int8_T tmp_data[3];
  int8_T coefsWithFlatStart_tmp;
  static const real_T f_breaks[6] = { -1.0, 0.0, 0.33333333333333331,
    0.66666666666666674, 1.0, 2.0 };

  int32_T tmp_size_idx_0;
  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    obj->PrevOptInputs.f1[i] = 1.0;
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  obj->PrevOptInputs.f2 = 1.0;
  for (i = 0; i < 9; i++) {
    coeffMat[i] = 0;
    coefs[i] = 0;
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  coefs[6] = 1;
  coefs[7] = 1;
  coefs[8] = 1;
  trueCount = 0;
  for (i = 0; i < 3; i++) {
    trueCount++;
  }

  tmp_size_idx_0 = trueCount;
  trueCount = 0;
  for (i = 0; i < 3; i++) {
    tmp_data[trueCount] = (int8_T)i;
    trueCount++;
  }

  for (i = 0; i < 3; i++) {
    for (trueCount = 0; trueCount < tmp_size_idx_0; trueCount++) {
      coeffMat[tmp_data[trueCount] + 3 * i] = coefs[3 * i + trueCount];
    }

    newSegmentCoeffs[i] = 0.0;
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  newSegmentCoeffs[2] = coeffMat[6];
  coefsWithFlatStart_0 = 0.0;
  for (i = 0; i < 3; i++) {
    trueCount = i << 2;
    coefsWithFlatStart[trueCount] = (int8_T)newSegmentCoeffs[i];
    coefsWithFlatStart[trueCount + 1] = coeffMat[3 * i];
    coefsWithFlatStart[trueCount + 2] = coeffMat[3 * i + 1];
    coefsWithFlatStart_tmp = coeffMat[3 * i + 2];
    coefsWithFlatStart[trueCount + 3] = coefsWithFlatStart_tmp;
    newSegmentCoeffs[i] = 0.0;
    coefsWithFlatStart_0 += rt_powd_snf(0.33333333333333326, 3.0 - ((real_T)i +
      1.0)) * (real_T)coefsWithFlatStart_tmp;
  }

  newSegmentCoeffs[2] = coefsWithFlatStart_0;
  memset(&expl_temp.coefs[0], 0, 15U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    trueCount = i << 2;
    expl_temp.coefs[5 * i] = coefsWithFlatStart[trueCount];
    expl_temp.coefs[5 * i + 1] = coefsWithFlatStart[trueCount + 1];
    expl_temp.coefs[5 * i + 2] = coefsWithFlatStart[trueCount + 2];
    expl_temp.coefs[5 * i + 3] = coefsWithFlatStart[trueCount + 3];
    expl_temp.coefs[5 * i + 4] = newSegmentCoeffs[i];
  }

  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    expl_temp.breaks[i] = f_breaks[i];
  }

  for (i = 0; i < 3; i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    obj->PPDDCell[i] = expl_temp;
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  obj->PPCell[0] = obj->PPDDCell[0];
  obj->PPCell[1] = obj->PPDDCell[1];
  obj->PPCell[2] = obj->PPDDCell[2];
  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    expl_temp.breaks[i] = f_breaks[i];
  }

  for (i = 0; i < 3; i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    obj->PPDDCell[i] = expl_temp;
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  obj->PPDCell[0] = obj->PPDDCell[0];
  obj->PPDCell[1] = obj->PPDDCell[1];
  obj->PPDCell[2] = obj->PPDDCell[2];
  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    expl_temp.breaks[i] = f_breaks[i];
  }

  for (i = 0; i < 3; i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    obj->PPDDCell[i] = expl_temp;
  }
}

static boolean_T TSTr_checkPolyForMultipleBreaks(const real_T breakMat[12])
{
  real_T y[4];
  int32_T b_i;
  boolean_T hasMultipleBreaks;
  hasMultipleBreaks = false;
  for (b_i = 0; b_i < 2; b_i++) {
    int32_T b_k;
    boolean_T exitg1;
    boolean_T y_0;

    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    y[0] = fabs(breakMat[b_i] - breakMat[b_i + 1]);
    y[1] = fabs(breakMat[b_i + 3] - breakMat[b_i + 4]);
    y[2] = fabs(breakMat[b_i + 6] - breakMat[b_i + 7]);
    y[3] = fabs(breakMat[b_i + 9] - breakMat[b_i + 10]);
    y_0 = false;
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k < 4)) {
      if (y[b_k] > 2.2204460492503131E-16) {
        y_0 = true;
        exitg1 = true;
      } else {
        b_k++;
      }
    }

    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    hasMultipleBreaks = (y_0 || hasMultipleBreaks);
  }

  return hasMultipleBreaks;
}

static void TSTraj_processPolynomialResults(const real_T breakMat[12], const
  real_T coeffMat[27], boolean_T hasMultipleBreaks, g_cell_wrap_TSTraj_T
  breaksCell[3], i_cell_wrap_TSTraj_T coeffCell[3])
{
  int32_T i;
  if (hasMultipleBreaks) {
    coeffCell[0].f1.size[0] = 3;
    coeffCell[0].f1.size[1] = 3;
    breaksCell[0].f1[0] = breakMat[0];
    breaksCell[0].f1[1] = breakMat[3];
    breaksCell[0].f1[2] = breakMat[6];
    breaksCell[0].f1[3] = breakMat[9];
    coeffCell[1].f1.size[0] = 3;
    coeffCell[1].f1.size[1] = 3;
    breaksCell[1].f1[0] = breakMat[1];
    breaksCell[1].f1[1] = breakMat[4];
    breaksCell[1].f1[2] = breakMat[7];
    breaksCell[1].f1[3] = breakMat[10];
    coeffCell[2].f1.size[0] = 3;
    coeffCell[2].f1.size[1] = 3;
    for (i = 0; i < 3; i++) {
      coeffCell[0].f1.data[coeffCell[0].f1.size[0] * i] = coeffMat[9 * i];

      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      coeffCell[0].f1.data[1 + coeffCell[0].f1.size[0] * i] = coeffMat[9 * i + 3];
      coeffCell[0].f1.data[2 + coeffCell[0].f1.size[0] * i] = coeffMat[9 * i + 6];
      coeffCell[1].f1.data[coeffCell[1].f1.size[0] * i] = coeffMat[9 * i + 1];
      coeffCell[1].f1.data[1 + coeffCell[1].f1.size[0] * i] = coeffMat[9 * i + 4];
      coeffCell[1].f1.data[2 + coeffCell[1].f1.size[0] * i] = coeffMat[9 * i + 7];
      coeffCell[2].f1.data[coeffCell[2].f1.size[0] * i] = coeffMat[9 * i + 2];
      coeffCell[2].f1.data[1 + coeffCell[2].f1.size[0] * i] = coeffMat[9 * i + 5];
      coeffCell[2].f1.data[2 + coeffCell[2].f1.size[0] * i] = coeffMat[9 * i + 8];
    }

    breaksCell[2].f1[0] = breakMat[2];
    breaksCell[2].f1[1] = breakMat[5];
    breaksCell[2].f1[2] = breakMat[8];
    breaksCell[2].f1[3] = breakMat[11];
  } else {
    coeffCell[0].f1.size[0] = 9;
    coeffCell[0].f1.size[1] = 3;
    breaksCell[0].f1[0] = breakMat[0];
    breaksCell[0].f1[1] = breakMat[3];
    breaksCell[0].f1[2] = breakMat[6];
    breaksCell[0].f1[3] = breakMat[9];
    coeffCell[1].f1.size[0] = 9;
    coeffCell[1].f1.size[1] = 3;
    breaksCell[1].f1[0] = breakMat[0];
    breaksCell[1].f1[1] = breakMat[3];
    breaksCell[1].f1[2] = breakMat[6];
    breaksCell[1].f1[3] = breakMat[9];
    coeffCell[2].f1.size[0] = 9;
    coeffCell[2].f1.size[1] = 3;
    memcpy(&coeffCell[0].f1.data[0], &coeffMat[0], 27U * sizeof(real_T));
    memcpy(&coeffCell[1].f1.data[0], &coeffMat[0], 27U * sizeof(real_T));
    memcpy(&coeffCell[2].f1.data[0], &coeffMat[0], 27U * sizeof(real_T));
    breaksCell[2].f1[0] = breakMat[0];
    breaksCell[2].f1[1] = breakMat[3];
    breaksCell[2].f1[2] = breakMat[6];
    breaksCell[2].f1[3] = breakMat[9];
  }
}

static void TSTraj_ppval(const real_T pp_breaks[6], const real_T pp_coefs_data[],
  const int32_T pp_coefs_size[3], const real_T x[2], real_T v_data[], int32_T
  v_size[2])
{
  int32_T c_ix;
  int32_T coefStride_tmp;
  int32_T d_j;
  int32_T elementsPerPage_tmp;
  int32_T high_i;
  int32_T low_i;
  int32_T mid_i;
  elementsPerPage_tmp = pp_coefs_size[0];

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  coefStride_tmp = pp_coefs_size[0] * 5;
  v_size[0] = pp_coefs_size[0];
  v_size[1] = 2;

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  if (pp_coefs_size[0] == 1) {
    real_T xloc;
    int32_T low_ip1;
    if (rtIsNaN(x[0])) {
      v_data[0] = (rtNaN);
    } else {
      low_i = 0;
      low_ip1 = 1;
      high_i = 6;
      while (high_i > low_ip1 + 1) {
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (x[0] >= pp_breaks[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i;
        } else {
          high_i = mid_i;
        }
      }

      xloc = x[0] - pp_breaks[low_i];
      v_data[0] = (pp_coefs_data[low_i] * xloc + pp_coefs_data[low_i +
                   coefStride_tmp]) * xloc + pp_coefs_data[(coefStride_tmp << 1)
        + low_i];
    }

    if (rtIsNaN(x[1])) {
      v_data[1] = (rtNaN);
    } else {
      low_i = 0;
      low_ip1 = 1;
      high_i = 6;
      while (high_i > low_ip1 + 1) {
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (x[1] >= pp_breaks[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i;
        } else {
          high_i = mid_i;
        }
      }

      xloc = x[1] - pp_breaks[low_i];
      v_data[1] = (pp_coefs_data[low_i] * xloc + pp_coefs_data[low_i +
                   coefStride_tmp]) * xloc + pp_coefs_data[(coefStride_tmp << 1)
        + low_i];
    }
  } else {
    for (c_ix = 0; c_ix < 2; c_ix++) {
      int32_T iv0;
      iv0 = c_ix * elementsPerPage_tmp - 1;
      if (rtIsNaN(x[c_ix])) {
        for (high_i = 0; high_i < elementsPerPage_tmp; high_i++) {
          v_data[(iv0 + high_i) + 1] = x[c_ix];
        }
      } else {
        real_T xloc;
        int32_T low_ip1;
        low_i = 0;
        low_ip1 = 1;
        high_i = 6;
        while (high_i > low_ip1 + 1) {
          mid_i = ((low_i + high_i) + 1) >> 1;
          if (x[c_ix] >= pp_breaks[mid_i - 1]) {
            low_i = mid_i - 1;
            low_ip1 = mid_i;
          } else {
            high_i = mid_i;
          }
        }

        high_i = low_i * elementsPerPage_tmp;
        xloc = x[c_ix] - pp_breaks[low_i];
        for (low_i = 0; low_i < elementsPerPage_tmp; low_i++) {
          v_data[(iv0 + low_i) + 1] = pp_coefs_data[high_i + low_i];
        }

        for (mid_i = 0; mid_i < 2; mid_i++) {
          int32_T tmp_0;
          int32_T vectorUB;
          low_i = ((mid_i + 1) * coefStride_tmp + high_i) - 1;
          low_ip1 = (elementsPerPage_tmp / 2) << 1;
          vectorUB = low_ip1 - 2;
          for (d_j = 0; d_j <= vectorUB; d_j += 2) {
            __m128d tmp;
            tmp_0 = (d_j + iv0) + 1;
            tmp = _mm_loadu_pd(&v_data[tmp_0]);
            _mm_storeu_pd(&v_data[tmp_0], _mm_add_pd(_mm_mul_pd(tmp, _mm_set1_pd
              (xloc)), _mm_loadu_pd(&pp_coefs_data[(d_j + low_i) + 1])));
          }

          for (d_j = low_ip1; d_j < elementsPerPage_tmp; d_j++) {
            tmp_0 = (d_j + iv0) + 1;
            v_data[tmp_0] = pp_coefs_data[(d_j + low_i) + 1] + v_data[tmp_0] *
              xloc;
          }
        }
      }
    }
  }
}

static void T_generateTrajectoriesFromCoefs(const real_T breaks[4], const real_T
  coeffs_data[], const int32_T coeffs_size[2], real_T dim, const real_T t[2],
  real_T q_data[], int32_T q_size[2], real_T qd_data[], int32_T qd_size[2],
  real_T qdd_data[], int32_T qdd_size[2], real_T pp_breaks[6], real_T
  pp_coefs_data[], int32_T pp_coefs_size[3])
{
  __m128d tmp;
  real_T b_newCoefs_data[45];
  real_T dCoeffs_data[45];
  real_T ddCoeffs_data[45];
  real_T coefsWithFlatStart_data[36];
  real_T valueAtEnd_data[12];
  real_T newSegmentCoeffs_data[9];
  real_T breaksWithFlatStart[5];
  real_T valueAtStart_data[3];
  real_T evalPointVector_idx_0;
  real_T evalPointVector_idx_1;
  real_T evalPointVector_idx_2;
  real_T s;
  int32_T dim_0[3];
  int32_T b;
  int32_T b_i;
  int32_T coefsWithFlatStart;
  int32_T d;
  int32_T dim_idx_0;
  int32_T dim_idx_0_tmp;
  int32_T e;
  int32_T loop_ub_tmp;

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  dim_idx_0_tmp = (int32_T)dim;
  b = (uint8_T)(int32_T)dim;
  for (b_i = 0; b_i < b; b_i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    d = (int32_T)dim + b_i;
    s = coeffs_data[b_i / (int32_T)dim * coeffs_size[0] + b_i % (int32_T)dim] *
      0.0 + coeffs_data[d / (int32_T)dim * coeffs_size[0] + d % (int32_T)dim] *
      0.0;
    d = ((int32_T)dim << 1) + b_i;
    valueAtStart_data[b_i] = coeffs_data[d / (int32_T)dim * coeffs_size[0] + d %
      (int32_T)dim] + s;
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  loop_ub_tmp = (int32_T)dim * 3;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&newSegmentCoeffs_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  for (d = 0; d < dim_idx_0_tmp; d++) {
    newSegmentCoeffs_data[d + ((int32_T)dim << 1)] = valueAtStart_data[d];
  }

  coefsWithFlatStart = (int32_T)((real_T)coeffs_size[0] + dim);
  b_i = coefsWithFlatStart * 3;
  if (b_i - 1 >= 0) {
    memset(&coefsWithFlatStart_data[0], 0, (uint32_T)b_i * sizeof(real_T));
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  b_i = (coefsWithFlatStart - (int32_T)(dim + 1.0)) + 1;
  for (d = 0; d < 3; d++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    for (dim_idx_0 = 0; dim_idx_0 < dim_idx_0_tmp; dim_idx_0++) {
      coefsWithFlatStart_data[dim_idx_0 + coefsWithFlatStart * d] =
        newSegmentCoeffs_data[(int32_T)dim * d + dim_idx_0];
    }

    for (dim_idx_0 = 0; dim_idx_0 < b_i; dim_idx_0++) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      coefsWithFlatStart_data[(((int32_T)(dim + 1.0) + dim_idx_0) +
        coefsWithFlatStart * d) - 1] = coeffs_data[coeffs_size[0] * d +
        dim_idx_0];
    }
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  breaksWithFlatStart[0] = breaks[0] - 1.0;
  breaksWithFlatStart[1] = breaks[0];
  breaksWithFlatStart[2] = breaks[1];
  breaksWithFlatStart[3] = breaks[2];
  breaksWithFlatStart[4] = breaks[3];
  s = breaks[3] - breaks[2];
  evalPointVector_idx_0 = rt_powd_snf(s, 2.0);
  evalPointVector_idx_1 = rt_powd_snf(s, 1.0);
  evalPointVector_idx_2 = rt_powd_snf(s, 0.0);
  s = ((real_T)coefsWithFlatStart - dim) + 1.0;
  if (s > coefsWithFlatStart) {
    e = 0;
    d = 0;
  } else {
    e = (int32_T)s - 1;
    d = coefsWithFlatStart;
  }

  dim_idx_0 = d - e;
  b = (uint8_T)dim_idx_0;
  for (b_i = 0; b_i < b; b_i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    d = dim_idx_0 + b_i;
    s = coefsWithFlatStart_data[(b_i % dim_idx_0 + e) + b_i / dim_idx_0 *
      coefsWithFlatStart] * evalPointVector_idx_0 + coefsWithFlatStart_data[(d %
      dim_idx_0 + e) + d / dim_idx_0 * coefsWithFlatStart] *
      evalPointVector_idx_1;
    d = (dim_idx_0 << 1) + b_i;
    valueAtEnd_data[b_i] = coefsWithFlatStart_data[(d % dim_idx_0 + e) + d /
      dim_idx_0 * coefsWithFlatStart] * evalPointVector_idx_2 + s;
  }

  if (loop_ub_tmp - 1 >= 0) {
    memset(&newSegmentCoeffs_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  for (d = 0; d < dim_idx_0_tmp; d++) {
    newSegmentCoeffs_data[d + ((int32_T)dim << 1)] = valueAtEnd_data[d];
  }

  s = (real_T)coefsWithFlatStart + dim;
  dim_idx_0_tmp = (int32_T)s;
  loop_ub_tmp = (int32_T)s * 3;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&b_newCoefs_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
  }

  for (d = 0; d < 3; d++) {
    for (dim_idx_0 = 0; dim_idx_0 < coefsWithFlatStart; dim_idx_0++) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      b_newCoefs_data[dim_idx_0 + (int32_T)s * d] =
        coefsWithFlatStart_data[coefsWithFlatStart * d + dim_idx_0];
    }
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  if ((real_T)coefsWithFlatStart + 1.0 > s) {
    b = 0;
    d = 0;
  } else {
    b = coefsWithFlatStart;
    d = (int32_T)s;
  }

  b_i = d - b;
  for (d = 0; d < 3; d++) {
    for (dim_idx_0 = 0; dim_idx_0 < b_i; dim_idx_0++) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      b_newCoefs_data[(b + dim_idx_0) + (int32_T)s * d] = newSegmentCoeffs_data
        [(int32_T)dim * d + dim_idx_0];
    }
  }

  for (d = 0; d < 5; d++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    pp_breaks[d] = breaksWithFlatStart[d];
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  pp_breaks[5] = breaks[3] + 1.0;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&dCoeffs_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
  }

  for (b_i = 0; b_i < 2; b_i++) {
    loop_ub_tmp = ((int32_T)s / 2) << 1;
    dim_idx_0 = loop_ub_tmp - 2;
    for (d = 0; d <= dim_idx_0; d += 2) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      tmp = _mm_loadu_pd(&b_newCoefs_data[(int32_T)s * b_i + d]);
      _mm_storeu_pd(&dCoeffs_data[d + (int32_T)s * (b_i + 1)], _mm_mul_pd(tmp,
        _mm_set1_pd(2.0 - (real_T)b_i)));
    }

    for (d = loop_ub_tmp; d < dim_idx_0_tmp; d++) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      dCoeffs_data[d + (int32_T)s * (b_i + 1)] = b_newCoefs_data[(int32_T)s *
        b_i + d] * (2.0 - (real_T)b_i);
    }
  }

  b_i = (int32_T)s * 3;
  if (b_i - 1 >= 0) {
    memset(&ddCoeffs_data[0], 0, (uint32_T)b_i * sizeof(real_T));
  }

  for (b_i = 0; b_i < 2; b_i++) {
    loop_ub_tmp = ((int32_T)s / 2) << 1;
    dim_idx_0 = loop_ub_tmp - 2;
    for (d = 0; d <= dim_idx_0; d += 2) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      tmp = _mm_loadu_pd(&dCoeffs_data[(int32_T)s * b_i + d]);
      _mm_storeu_pd(&ddCoeffs_data[d + (int32_T)s * (b_i + 1)], _mm_mul_pd(tmp,
        _mm_set1_pd(2.0 - (real_T)b_i)));
    }

    for (d = loop_ub_tmp; d < dim_idx_0_tmp; d++) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      ddCoeffs_data[d + (int32_T)s * (b_i + 1)] = dCoeffs_data[(int32_T)s * b_i
        + d] * (2.0 - (real_T)b_i);
    }
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  pp_coefs_size[0] = (int32_T)dim;
  pp_coefs_size[1] = 5;
  pp_coefs_size[2] = 3;
  loop_ub_tmp = (int32_T)dim * 5 * 3;
  if (loop_ub_tmp - 1 >= 0) {
    memcpy(&pp_coefs_data[0], &b_newCoefs_data[0], (uint32_T)loop_ub_tmp *
           sizeof(real_T));
  }

  dim_0[0] = (int32_T)dim;
  dim_0[1] = 5;
  dim_0[2] = 3;
  TSTraj_ppval(pp_breaks, b_newCoefs_data, dim_0, t, q_data, q_size);
  dim_0[0] = (int32_T)dim;
  dim_0[1] = 5;
  dim_0[2] = 3;
  TSTraj_ppval(pp_breaks, dCoeffs_data, dim_0, t, qd_data, qd_size);
  dim_0[0] = (int32_T)dim;
  dim_0[1] = 5;
  dim_0[2] = 3;
  TSTraj_ppval(pp_breaks, ddCoeffs_data, dim_0, t, qdd_data, qdd_size);
}

static void TSTraj_trapveltraj(const real_T wayPoints[6], real_T varargin_2,
  real_T q[6], real_T qd[6], real_T qdd[6], real_T t[2],
  s_vjEZ2dxatR8VOmLd9oOqoD_TSTr_T ppCell_data[], int32_T *ppCell_size)
{
  __m128i tmp_0;
  g_cell_wrap_TSTraj_T breaksCell[3];
  i_cell_wrap_TSTraj_T coeffsCell[3];
  real_T coeffMat[27];
  real_T parameterMat[18];
  real_T breakMat[12];
  real_T coefs[9];
  real_T b_data[6];
  real_T c_data[6];
  real_T d_data[6];
  real_T varargin_1[3];
  real_T indivPolyDim;
  real_T numComputedPolynomials;
  real_T s0;
  real_T sF;
  real_T segAcc;
  real_T segVel;
  real_T wayPoints_0;
  real_T wayPoints_1;
  int32_T b_size[2];
  int32_T c_size[2];
  int32_T d_size[2];
  int32_T b_i;
  int32_T b_idx;
  int32_T deltaSign;
  int32_T rowSelection_size_idx_1;
  int32_T tmp_size_idx_0;
  int32_T vectorUB;
  int8_T tmp_data[9];
  int8_T f_data[4];
  int8_T lspbSegIndices_data[4];
  int8_T k_tmp_data[3];
  int8_T rowSelection_data[3];
  int8_T tmp[2];
  boolean_T coefIndex[9];
  boolean_T exitg1;
  boolean_T hasMultipleBreaks;
  for (deltaSign = 0; deltaSign < 6; deltaSign++) {
    q[deltaSign] = 0.0;
    qd[deltaSign] = 0.0;
    qdd[deltaSign] = 0.0;
  }

  memset(&coeffMat[0], 0, 27U * sizeof(real_T));
  memset(&breakMat[0], 0, 12U * sizeof(real_T));
  for (b_i = 0; b_i < 3; b_i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    wayPoints_1 = wayPoints[b_i];
    s0 = wayPoints_1;
    wayPoints_0 = wayPoints[b_i + 3];
    sF = wayPoints_0;
    deltaSign = 1;

    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    if (wayPoints_0 < wayPoints_1) {
      s0 = wayPoints_0;
      sF = wayPoints_1;
      deltaSign = -1;
    }

    indivPolyDim = varargin_2;
    segVel = (sF - s0) * 1.5 / varargin_2;
    numComputedPolynomials = ((s0 - sF) + segVel * varargin_2) / segVel;
    segAcc = segVel / numComputedPolynomials;
    if (s0 == sF) {
      segAcc = 0.0;
      segVel = 0.0;

      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      if (rtIsNaN(varargin_2) || (varargin_2 == 0.0)) {
        indivPolyDim = 1.0;
      }

      numComputedPolynomials = indivPolyDim / 3.0;
    }

    segVel *= (real_T)deltaSign;
    segAcc *= (real_T)deltaSign;

    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    parameterMat[b_i] = wayPoints_1;
    parameterMat[b_i + 3] = wayPoints_0;
    parameterMat[b_i + 6] = segVel;
    parameterMat[b_i + 9] = segAcc;
    parameterMat[b_i + 12] = numComputedPolynomials;
    parameterMat[b_i + 15] = indivPolyDim;
    memset(&coefs[0], 0, 9U * sizeof(real_T));

    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    if (segVel == 0.0) {
      coefs[6] = wayPoints_1;
      coefs[7] = wayPoints_1;
      coefs[8] = wayPoints_1;
    } else {
      coefs[0] = segAcc / 2.0;
      coefs[3] = 0.0;
      coefs[6] = wayPoints_1;
      coefs[1] = 0.0;
      coefs[4] = segVel;
      s0 = segAcc / 2.0 * (numComputedPolynomials * numComputedPolynomials);
      coefs[7] = s0 + wayPoints_1;
      coefs[2] = -segAcc / 2.0;
      coefs[5] = segVel;
      coefs[8] = (s0 + wayPoints_0) - segVel * numComputedPolynomials;
    }

    for (deltaSign = 0; deltaSign < 9; deltaSign++) {
      coefIndex[deltaSign] = false;
    }

    for (deltaSign = 0; deltaSign < 3; deltaSign++) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      lspbSegIndices_data[deltaSign] = (int8_T)((3 * deltaSign + b_i) + 1);
    }

    for (deltaSign = 0; deltaSign < 3; deltaSign++) {
      f_data[deltaSign] = lspbSegIndices_data[deltaSign];
      coefIndex[f_data[deltaSign] - 1] = true;
    }

    b_idx = 0;
    for (deltaSign = 0; deltaSign < 9; deltaSign++) {
      if (coefIndex[deltaSign]) {
        b_idx++;
      }
    }

    tmp_size_idx_0 = b_idx;
    b_idx = 0;
    for (deltaSign = 0; deltaSign < 9; deltaSign++) {
      if (coefIndex[deltaSign]) {
        tmp_data[b_idx] = (int8_T)deltaSign;
        b_idx++;
      }
    }

    tmp[0] = (int8_T)tmp_size_idx_0;
    b_idx = (int8_T)tmp_size_idx_0;
    for (deltaSign = 0; deltaSign < 3; deltaSign++) {
      for (tmp_size_idx_0 = 0; tmp_size_idx_0 < b_idx; tmp_size_idx_0++) {
        coeffMat[tmp_data[tmp_size_idx_0] + 9 * deltaSign] = coefs[tmp[0] *
          deltaSign + tmp_size_idx_0];
      }
    }

    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    wayPoints_1 = breakMat[b_i];
    breakMat[b_i + 3] = numComputedPolynomials + wayPoints_1;
    breakMat[b_i + 6] = (indivPolyDim - numComputedPolynomials) + wayPoints_1;
    breakMat[b_i + 9] = indivPolyDim + wayPoints_1;
  }

  hasMultipleBreaks = TSTr_checkPolyForMultipleBreaks(breakMat);
  TSTraj_processPolynomialResults(breakMat, coeffMat, hasMultipleBreaks,
    breaksCell, coeffsCell);

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  varargin_1[0] = parameterMat[15];
  varargin_1[1] = parameterMat[16];
  varargin_1[2] = parameterMat[17];
  if (!rtIsNaN(parameterMat[15])) {
    b_idx = 1;
  } else {
    b_idx = 0;
    b_i = 2;
    exitg1 = false;
    while ((!exitg1) && (b_i < 4)) {
      if (!rtIsNaN(varargin_1[b_i - 1])) {
        b_idx = b_i;
        exitg1 = true;
      } else {
        b_i++;
      }
    }
  }

  if (b_idx == 0) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    t[1] = parameterMat[15];
  } else {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    indivPolyDim = varargin_1[b_idx - 1];
    for (b_i = b_idx + 1; b_i < 4; b_i++) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      numComputedPolynomials = varargin_1[b_i - 1];
      if (indivPolyDim < numComputedPolynomials) {
        indivPolyDim = numComputedPolynomials;
      }
    }

    t[1] = indivPolyDim;
  }

  t[0] = 0.0;
  if (hasMultipleBreaks) {
    numComputedPolynomials = 3.0;
    indivPolyDim = 1.0;
  } else {
    numComputedPolynomials = 1.0;
    indivPolyDim = 3.0;
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  b_i = (int32_T)numComputedPolynomials;
  *ppCell_size = b_i;
  for (b_idx = 0; b_idx < b_i; b_idx++) {
    if (hasMultipleBreaks) {
      rowSelection_size_idx_1 = 1;

      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      rowSelection_data[0] = (int8_T)(b_idx + 1);
      numComputedPolynomials = (real_T)b_idx + 1.0;
    } else {
      rowSelection_size_idx_1 = 3;
      rowSelection_data[0] = 1;
      rowSelection_data[1] = 2;
      rowSelection_data[2] = 3;
      numComputedPolynomials = 1.0;
    }

    deltaSign = (int32_T)numComputedPolynomials - 1;

    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    T_generateTrajectoriesFromCoefs(breaksCell[deltaSign].f1,
      coeffsCell[deltaSign].f1.data, coeffsCell[deltaSign].f1.size, indivPolyDim,
      t, b_data, b_size, c_data, c_size, d_data, d_size, ppCell_data[b_idx].
      breaks, ppCell_data[b_idx].coefs.data, ppCell_data[b_idx].coefs.size);
    tmp_size_idx_0 = (rowSelection_size_idx_1 / 16) << 4;
    vectorUB = tmp_size_idx_0 - 16;
    for (deltaSign = 0; deltaSign <= vectorUB; deltaSign += 16) {
      tmp_0 = _mm_loadu_si128((const __m128i *)&rowSelection_data[deltaSign]);
      _mm_storeu_si128((__m128i *)&k_tmp_data[deltaSign], _mm_sub_epi8(tmp_0,
        _mm_set1_epi8(1)));
    }

    for (deltaSign = tmp_size_idx_0; deltaSign < rowSelection_size_idx_1;
         deltaSign++) {
      k_tmp_data[deltaSign] = (int8_T)(rowSelection_data[deltaSign] - 1);
    }

    for (deltaSign = 0; deltaSign < 2; deltaSign++) {
      for (tmp_size_idx_0 = 0; tmp_size_idx_0 < rowSelection_size_idx_1;
           tmp_size_idx_0++) {
        /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
        vectorUB = 3 * deltaSign + k_tmp_data[tmp_size_idx_0];
        q[vectorUB] = b_data[b_size[0] * deltaSign + tmp_size_idx_0];
        qd[vectorUB] = c_data[c_size[0] * deltaSign + tmp_size_idx_0];
        qdd[vectorUB] = d_data[d_size[0] * deltaSign + tmp_size_idx_0];
      }
    }
  }
}

static real_T TSTraj_ppval_o(const real_T pp_breaks[6], const real_T pp_coefs[15],
  real_T x)
{
  real_T v;

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  if (rtIsNaN(x)) {
    v = (rtNaN);
  } else {
    int32_T high_i;
    int32_T low_i;
    int32_T low_ip1;
    low_i = 0;
    low_ip1 = 1;
    high_i = 6;
    while (high_i > low_ip1 + 1) {
      int32_T mid_i;
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (x >= pp_breaks[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i;
      } else {
        high_i = mid_i;
      }
    }

    real_T xloc;
    xloc = x - pp_breaks[low_i];
    v = (xloc * pp_coefs[low_i] + pp_coefs[low_i + 5]) * xloc + pp_coefs[low_i +
      10];
  }

  /* End of Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  return v;
}

static void TrapVelTrajSys_generate1DPVAP_d(const real_T breaks[6], const real_T
  coefs_data[], const int32_T coefs_size[2], real_T pp_breaks[6], real_T
  pp_coefs[15], real_T ppd_breaks[6], real_T ppd_coefs[15], real_T ppdd_coefs[15])
{
  real_T dCoefs_data[45];
  real_T ddCoefs_data[45];
  int32_T b_idx_0_tmp;
  int32_T dCoefs_size_idx_0;
  int32_T i;
  int32_T loop_ub;
  int32_T vectorUB;
  dCoefs_size_idx_0 = coefs_size[0];
  loop_ub = coefs_size[0] * 3;
  if (loop_ub - 1 >= 0) {
    memset(&dCoefs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  for (b_idx_0_tmp = 0; b_idx_0_tmp < 2; b_idx_0_tmp++) {
    loop_ub = (dCoefs_size_idx_0 / 2) << 1;
    vectorUB = loop_ub - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      _mm_storeu_pd(&dCoefs_data[i + dCoefs_size_idx_0 * (b_idx_0_tmp + 1)],
                    _mm_mul_pd(_mm_loadu_pd(&coefs_data[i + coefs_size[0] *
        b_idx_0_tmp]), _mm_set1_pd(2.0 - (real_T)b_idx_0_tmp)));
    }

    for (i = loop_ub; i < dCoefs_size_idx_0; i++) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      dCoefs_data[i + dCoefs_size_idx_0 * (b_idx_0_tmp + 1)] =
        coefs_data[coefs_size[0] * b_idx_0_tmp + i] * (2.0 - (real_T)b_idx_0_tmp);
    }
  }

  loop_ub = coefs_size[0] * 3;
  if (loop_ub - 1 >= 0) {
    memset(&ddCoefs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  for (b_idx_0_tmp = 0; b_idx_0_tmp < 2; b_idx_0_tmp++) {
    loop_ub = (dCoefs_size_idx_0 / 2) << 1;
    vectorUB = loop_ub - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      __m128d tmp;

      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      tmp = _mm_loadu_pd(&dCoefs_data[dCoefs_size_idx_0 * b_idx_0_tmp + i]);
      _mm_storeu_pd(&ddCoefs_data[i + dCoefs_size_idx_0 * (b_idx_0_tmp + 1)],
                    _mm_mul_pd(tmp, _mm_set1_pd(2.0 - (real_T)b_idx_0_tmp)));
    }

    for (i = loop_ub; i < dCoefs_size_idx_0; i++) {
      /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
      ddCoefs_data[i + dCoefs_size_idx_0 * (b_idx_0_tmp + 1)] =
        dCoefs_data[dCoefs_size_idx_0 * b_idx_0_tmp + i] * (2.0 - (real_T)
        b_idx_0_tmp);
    }
  }

  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    pp_breaks[i] = breaks[i];
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  memcpy(&pp_coefs[0], &coefs_data[0], 15U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
    ppd_breaks[i] = breaks[i];
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  memcpy(&ppd_coefs[0], &dCoefs_data[0], 15U * sizeof(real_T));
  memcpy(&ppdd_coefs[0], &ddCoefs_data[0], 15U * sizeof(real_T));
}

/* System initialize for atomic system: */
void TrapezoidalVelocityProfile_Init(DW_TrapezoidalVelocityProfile_T *localDW)
{
  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  localDW->obj.EndTime[0] = 1.0;
  localDW->obj.EndTime[1] = 1.0;
  localDW->obj.PPFormUpdatedNeeded = false;
  localDW->objisempty = true;
  localDW->obj.isInitialized = 1;
  TSTraj_TrapVelTrajSys_setupImpl(&localDW->obj);
  localDW->obj.TunablePropsChanged = false;
}

/* Output and update for atomic system: */
void TrapezoidalVelocityProfileTraje(real_T rtu_0, const real_T rtu_1[6], real_T
  rtu_2, B_TrapezoidalVelocityProfileT_T *localB,
  DW_TrapezoidalVelocityProfile_T *localDW)
{
  emxArray_s_vjEZ2dxatR8VOmLd9o_T trajPP;
  siswYcTR8LLamuD4YWmtXHC_TSTra_T ppCell[3];
  siswYcTR8LLamuD4YWmtXHC_TSTra_T ppdCell[3];
  siswYcTR8LLamuD4YWmtXHC_TSTra_T ppddCell[3];
  real_T evalCoeffs_data[45];
  real_T oneDimCoeffs_data[45];
  real_T a__22[6];
  real_T a__23[6];
  real_T breaks[6];
  real_T a__24[2];
  int32_T evalCoeffs_size[2];
  int32_T b_k;
  int32_T oneDimCoeffs;
  int32_T oneDimCoeffs_size_idx_0;
  int8_T trajPP_0[2];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T out;
  boolean_T p;

  /* MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  if (localDW->obj.TunablePropsChanged) {
    localDW->obj.TunablePropsChanged = false;
    localDW->obj.PPFormUpdatedNeeded = false;
  }

  guard1 = false;
  if (localDW->obj.PPFormUpdatedNeeded) {
    guard1 = true;
  } else {
    p = false;
    out = true;
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k < 6)) {
      if (!(rtu_1[b_k] == localDW->obj.PrevOptInputs.f1[b_k])) {
        out = false;
        exitg1 = true;
      } else {
        b_k++;
      }
    }

    if (out && (rtu_2 == localDW->obj.PrevOptInputs.f2)) {
      p = true;
    }

    for (b_k = 0; b_k < 6; b_k++) {
      localDW->obj.PrevOptInputs.f1[b_k] = rtu_1[b_k];
    }

    localDW->obj.PrevOptInputs.f2 = rtu_2;
    if (!p) {
      guard1 = true;
    }
  }

  if (guard1) {
    TSTraj_trapveltraj(rtu_1, rtu_2, breaks, a__22, a__23, a__24, trajPP.data,
                       &trajPP.size);
    if (trajPP.size > 1) {
      for (b_k = 0; b_k < 6; b_k++) {
        breaks[b_k] = trajPP.data[0].breaks[b_k];
      }

      oneDimCoeffs = (int32_T)((uint32_T)(trajPP.data[0].coefs.size[0] * 5 * 3) /
        3U);
      oneDimCoeffs_size_idx_0 = oneDimCoeffs;
      oneDimCoeffs *= 3;
      for (b_k = 0; b_k < oneDimCoeffs; b_k++) {
        oneDimCoeffs_data[b_k] = trajPP.data[0].coefs.data[b_k];
      }
    } else {
      for (b_k = 0; b_k < 6; b_k++) {
        breaks[b_k] = trajPP.data[0].breaks[b_k];
      }

      trajPP_0[0] = (int8_T)((uint32_T)(trajPP.data[0].coefs.size[0] * 5 * 3) /
        3U);
      oneDimCoeffs_size_idx_0 = 5;
      for (b_k = 0; b_k < 3; b_k++) {
        for (oneDimCoeffs = 0; oneDimCoeffs < 5; oneDimCoeffs++) {
          oneDimCoeffs_data[oneDimCoeffs + 5 * b_k] = trajPP.data[0].coefs.data
            [3 * oneDimCoeffs + trajPP_0[0] * b_k];
        }
      }
    }

    evalCoeffs_size[0] = oneDimCoeffs_size_idx_0;
    evalCoeffs_size[1] = 3;
    oneDimCoeffs = oneDimCoeffs_size_idx_0 * 3;
    if (oneDimCoeffs - 1 >= 0) {
      memset(&evalCoeffs_data[0], 0, (uint32_T)oneDimCoeffs * sizeof(real_T));
    }

    for (b_k = 0; b_k < 3; b_k++) {
      for (oneDimCoeffs = 0; oneDimCoeffs < oneDimCoeffs_size_idx_0;
           oneDimCoeffs++) {
        evalCoeffs_data[oneDimCoeffs + oneDimCoeffs_size_idx_0 * b_k] =
          oneDimCoeffs_data[oneDimCoeffs_size_idx_0 * b_k + oneDimCoeffs];
      }
    }

    for (b_k = 0; b_k < 6; b_k++) {
      ppddCell[0].breaks[b_k] = breaks[b_k];
    }

    TrapVelTrajSys_generate1DPVAP_d(ppddCell[0].breaks, evalCoeffs_data,
      evalCoeffs_size, ppCell[0].breaks, ppCell[0].coefs, ppdCell[0].breaks,
      ppdCell[0].coefs, ppddCell[0].coefs);
    if (trajPP.size > 1) {
      for (b_k = 0; b_k < 6; b_k++) {
        breaks[b_k] = trajPP.data[1].breaks[b_k];
      }

      oneDimCoeffs = (int32_T)((uint32_T)(trajPP.data[1].coefs.size[0] * 5 * 3) /
        3U);
      oneDimCoeffs_size_idx_0 = oneDimCoeffs;
      oneDimCoeffs *= 3;
      for (b_k = 0; b_k < oneDimCoeffs; b_k++) {
        oneDimCoeffs_data[b_k] = trajPP.data[1].coefs.data[b_k];
      }
    } else {
      trajPP_0[0] = (int8_T)((uint32_T)(trajPP.data[0].coefs.size[0] * 5 * 3) /
        3U);
      oneDimCoeffs_size_idx_0 = 5;
      for (b_k = 0; b_k < 3; b_k++) {
        for (oneDimCoeffs = 0; oneDimCoeffs < 5; oneDimCoeffs++) {
          oneDimCoeffs_data[oneDimCoeffs + 5 * b_k] = trajPP.data[0].coefs.data
            [(3 * oneDimCoeffs + trajPP_0[0] * b_k) + 1];
        }
      }
    }

    evalCoeffs_size[0] = oneDimCoeffs_size_idx_0;
    evalCoeffs_size[1] = 3;
    oneDimCoeffs = oneDimCoeffs_size_idx_0 * 3;
    if (oneDimCoeffs - 1 >= 0) {
      memset(&evalCoeffs_data[0], 0, (uint32_T)oneDimCoeffs * sizeof(real_T));
    }

    for (b_k = 0; b_k < 3; b_k++) {
      for (oneDimCoeffs = 0; oneDimCoeffs < oneDimCoeffs_size_idx_0;
           oneDimCoeffs++) {
        evalCoeffs_data[oneDimCoeffs + oneDimCoeffs_size_idx_0 * b_k] =
          oneDimCoeffs_data[oneDimCoeffs_size_idx_0 * b_k + oneDimCoeffs];
      }
    }

    for (b_k = 0; b_k < 6; b_k++) {
      ppddCell[1].breaks[b_k] = breaks[b_k];
    }

    TrapVelTrajSys_generate1DPVAP_d(ppddCell[1].breaks, evalCoeffs_data,
      evalCoeffs_size, ppCell[1].breaks, ppCell[1].coefs, ppdCell[1].breaks,
      ppdCell[1].coefs, ppddCell[1].coefs);
    if (trajPP.size > 1) {
      for (b_k = 0; b_k < 6; b_k++) {
        breaks[b_k] = trajPP.data[2].breaks[b_k];
      }

      oneDimCoeffs = (int32_T)((uint32_T)(trajPP.data[2].coefs.size[0] * 5 * 3) /
        3U);
      oneDimCoeffs_size_idx_0 = oneDimCoeffs;
      oneDimCoeffs *= 3;
      for (b_k = 0; b_k < oneDimCoeffs; b_k++) {
        oneDimCoeffs_data[b_k] = trajPP.data[2].coefs.data[b_k];
      }
    } else {
      for (b_k = 0; b_k < 6; b_k++) {
        breaks[b_k] = trajPP.data[0].breaks[b_k];
      }

      trajPP_0[0] = (int8_T)((uint32_T)(trajPP.data[0].coefs.size[0] * 5 * 3) /
        3U);
      oneDimCoeffs_size_idx_0 = 5;
      for (b_k = 0; b_k < 3; b_k++) {
        for (oneDimCoeffs = 0; oneDimCoeffs < 5; oneDimCoeffs++) {
          oneDimCoeffs_data[oneDimCoeffs + 5 * b_k] = trajPP.data[0].coefs.data
            [(3 * oneDimCoeffs + trajPP_0[0] * b_k) + 2];
        }
      }
    }

    evalCoeffs_size[0] = oneDimCoeffs_size_idx_0;
    evalCoeffs_size[1] = 3;
    oneDimCoeffs = oneDimCoeffs_size_idx_0 * 3;
    if (oneDimCoeffs - 1 >= 0) {
      memset(&evalCoeffs_data[0], 0, (uint32_T)oneDimCoeffs * sizeof(real_T));
    }

    for (b_k = 0; b_k < 3; b_k++) {
      for (oneDimCoeffs = 0; oneDimCoeffs < oneDimCoeffs_size_idx_0;
           oneDimCoeffs++) {
        evalCoeffs_data[oneDimCoeffs + oneDimCoeffs_size_idx_0 * b_k] =
          oneDimCoeffs_data[oneDimCoeffs_size_idx_0 * b_k + oneDimCoeffs];
      }
    }

    for (b_k = 0; b_k < 6; b_k++) {
      ppddCell[2].breaks[b_k] = breaks[b_k];
    }

    TrapVelTrajSys_generate1DPVAP_d(ppddCell[2].breaks, evalCoeffs_data,
      evalCoeffs_size, ppCell[2].breaks, ppCell[2].coefs, ppdCell[2].breaks,
      ppdCell[2].coefs, ppddCell[2].coefs);
    memcpy(&localDW->obj.PPCell[0], &ppCell[0], 3U * sizeof
           (siswYcTR8LLamuD4YWmtXHC_TSTra_T));
    memcpy(&localDW->obj.PPDCell[0], &ppdCell[0], 3U * sizeof
           (siswYcTR8LLamuD4YWmtXHC_TSTra_T));
    memcpy(&localDW->obj.PPDDCell[0], &ppddCell[0], 3U * sizeof
           (siswYcTR8LLamuD4YWmtXHC_TSTra_T));
    localDW->obj.PPFormUpdatedNeeded = false;
  }

  /* Start for MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory' */
  localB->TrapezoidalVelocityProfileTra_j[0] = TSTraj_ppval_o
    (localDW->obj.PPCell[0].breaks, localDW->obj.PPCell[0].coefs, rtu_0);
  localB->TrapezoidalVelocityProfileTra_j[1] = TSTraj_ppval_o
    (localDW->obj.PPCell[1].breaks, localDW->obj.PPCell[1].coefs, rtu_0);
  localB->TrapezoidalVelocityProfileTra_j[2] = TSTraj_ppval_o
    (localDW->obj.PPCell[2].breaks, localDW->obj.PPCell[2].coefs, rtu_0);
  localB->TrapezoidalVelocityProfileTra_l[0] = TSTraj_ppval_o
    (localDW->obj.PPDCell[0].breaks, localDW->obj.PPDCell[0].coefs, rtu_0);
  localB->TrapezoidalVelocityProfileTra_l[1] = TSTraj_ppval_o
    (localDW->obj.PPDCell[1].breaks, localDW->obj.PPDCell[1].coefs, rtu_0);
  localB->TrapezoidalVelocityProfileTra_l[2] = TSTraj_ppval_o
    (localDW->obj.PPDCell[2].breaks, localDW->obj.PPDCell[2].coefs, rtu_0);
  localB->TrapezoidalVelocityProfileTra_f[0] = TSTraj_ppval_o
    (localDW->obj.PPDDCell[0].breaks, localDW->obj.PPDDCell[0].coefs, rtu_0);
  localB->TrapezoidalVelocityProfileTra_f[1] = TSTraj_ppval_o
    (localDW->obj.PPDDCell[1].breaks, localDW->obj.PPDDCell[1].coefs, rtu_0);
  localB->TrapezoidalVelocityProfileTra_f[2] = TSTraj_ppval_o
    (localDW->obj.PPDDCell[2].breaks, localDW->obj.PPDDCell[2].coefs, rtu_0);
}

/* Function for MATLAB Function: '<S1>/MATLAB Function1' */
static real_T TSTraj_norm(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Model step function */
void TSTraj_step(void)
{
  __m128d tmp_7;
  __m128d tmp_8;
  __m128d tmp_a;
  real_T a[9];
  real_T ceil_xi[9];
  real_T omgmat[9];
  real_T rtb_R_t[9];
  real_T rtb_Sum1_0[9];
  real_T tmp[9];
  real_T tmp_0[9];
  real_T tmp_1[9];
  real_T y[9];
  real_T rtb_MatrixConcatenate[6];
  real_T rtb_MatrixConcatenate1[6];
  real_T omgtheta[3];
  real_T rtb_omega_t[3];
  real_T tmp_9[2];
  real_T R;
  real_T acosinput;
  real_T rtb_Sum1;
  real_T rtb_Switch;
  real_T theta;
  real_T tmp_2;
  real_T tmp_3;
  real_T tmp_4;
  real_T tmp_5;
  real_T tmp_6;
  int32_T ceil_xi_tmp;
  int32_T i;
  int32_T i_0;
  int8_T b_I[9];

  /* Sum: '<S1>/Sum' incorporates:
   *  Inport: '<Root>/T0'
   *  Inport: '<Root>/gt'
   */
  rtb_Sum1 = TSTraj_U.gt - TSTraj_U.T0;

  /* Switch: '<S1>/Switch' incorporates:
   *  Constant: '<S1>/Constant'
   */
  if (rtb_Sum1 > TSTraj_P.Switch_Threshold) {
    rtb_Switch = rtb_Sum1;
  } else {
    rtb_Switch = TSTraj_P.Constant_Value;
  }

  /* End of Switch: '<S1>/Switch' */

  /* MATLAB Function: '<S1>/TransToRp' incorporates:
   *  Inport: '<Root>/T_start'
   */
  TSTraj_TransToRp(TSTraj_U.T_start, &rtb_MatrixConcatenate[0],
                   &TSTraj_B.sf_TransToRp);

  /* MATLAB Function: '<S1>/TransToRp1' incorporates:
   *  Inport: '<Root>/T_end'
   */
  TSTraj_TransToRp(TSTraj_U.T_end, &rtb_MatrixConcatenate[3],
                   &TSTraj_B.sf_TransToRp1);

  /* Sum: '<S1>/Sum1' incorporates:
   *  Inport: '<Root>/T0'
   *  Inport: '<Root>/Tf'
   */
  rtb_Sum1 = TSTraj_U.Tf - TSTraj_U.T0;

  /* Switch: '<S1>/Switch2' incorporates:
   *  Constant: '<S1>/Constant2'
   */
  if (!(rtb_Sum1 > TSTraj_P.Switch2_Threshold)) {
    rtb_Sum1 = TSTraj_P.Constant2_Value;
  }

  /* End of Switch: '<S1>/Switch2' */
  TrapezoidalVelocityProfileTraje(rtb_Switch, rtb_MatrixConcatenate, rtb_Sum1,
    &TSTraj_B.TrapezoidalVelocityProfileTra_p,
    &TSTraj_DW.TrapezoidalVelocityProfileTra_p);

  /* MATLAB Function: '<S1>/MATLAB Function' */
  for (i = 0; i < 3; i++) {
    rtb_MatrixConcatenate1[i] = 0.0;
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    acosinput = TSTraj_B.sf_TransToRp1.R[3 * i_0 + 1];
    theta = TSTraj_B.sf_TransToRp1.R[3 * i_0];
    R = TSTraj_B.sf_TransToRp1.R[3 * i_0 + 2];
    for (i = 0; i < 3; i++) {
      omgmat[i + 3 * i_0] = (TSTraj_B.sf_TransToRp.R[3 * i + 1] * acosinput +
        TSTraj_B.sf_TransToRp.R[3 * i] * theta) + TSTraj_B.sf_TransToRp.R[3 * i
        + 2] * R;
    }
  }

  acosinput = (((omgmat[0] + omgmat[4]) + omgmat[8]) - 1.0) / 2.0;
  if (acosinput >= 1.0) {
    memset(&rtb_R_t[0], 0, 9U * sizeof(real_T));
  } else if (acosinput <= -1.0) {
    if (!(fabs(omgmat[8] + 1.0) < 2.2204460492503131E-16)) {
      acosinput = 1.0 / sqrt((omgmat[8] + 1.0) * 2.0);
      tmp_a = _mm_mul_pd(_mm_set1_pd(acosinput), _mm_loadu_pd(&omgmat[6]));
      _mm_storeu_pd(&omgtheta[0], tmp_a);
      omgtheta[2] = (omgmat[8] + 1.0) * acosinput;
    } else if (!(fabs(omgmat[4] + 1.0) < 2.2204460492503131E-16)) {
      theta = 1.0 / sqrt((omgmat[4] + 1.0) * 2.0);
      omgtheta[0] = theta * omgmat[3];
      omgtheta[1] = (omgmat[4] + 1.0) * theta;
      omgtheta[2] = theta * omgmat[5];
    } else {
      acosinput = 1.0 / sqrt((omgmat[0] + 1.0) * 2.0);
      omgtheta[0] = (omgmat[0] + 1.0) * acosinput;
      tmp_a = _mm_mul_pd(_mm_set1_pd(acosinput), _mm_loadu_pd(&omgmat[1]));
      _mm_storeu_pd(&omgtheta[1], tmp_a);
    }

    tmp_a = _mm_mul_pd(_mm_set1_pd(3.1415926535897931), _mm_loadu_pd(&omgtheta[0]));
    _mm_storeu_pd(&omgtheta[0], tmp_a);
    omgtheta[2] *= 3.1415926535897931;
    rtb_R_t[6] = omgtheta[1];
    rtb_R_t[1] = omgtheta[2];
    rtb_R_t[5] = omgtheta[0];
  } else {
    theta = acos(acosinput);
    acosinput = 1.0 / (2.0 * sin(theta)) * theta;
    for (i_0 = 0; i_0 < 3; i_0++) {
      tmp_a = _mm_mul_pd(_mm_sub_pd(_mm_loadu_pd(&omgmat[3 * i_0]), _mm_set_pd
        (omgmat[i_0 + 3], omgmat[i_0])), _mm_set1_pd(acosinput));
      _mm_storeu_pd(&rtb_R_t[3 * i_0], tmp_a);
      i = 3 * i_0 + 2;
      rtb_R_t[i] = (omgmat[i] - omgmat[i_0 + 6]) * acosinput;
    }
  }

  rtb_MatrixConcatenate1[3] = rtb_R_t[5];
  rtb_MatrixConcatenate1[4] = rtb_R_t[6];
  rtb_MatrixConcatenate1[5] = rtb_R_t[1];

  /* End of MATLAB Function: '<S1>/MATLAB Function' */
  TrapezoidalVelocityProfileTraje(rtb_Switch, rtb_MatrixConcatenate1, rtb_Sum1,
    &TSTraj_B.TrapezoidalVelocityProfileTr_pn,
    &TSTraj_DW.TrapezoidalVelocityProfileTr_pn);

  /* MATLAB Function: '<S1>/MATLAB Function1' incorporates:
   *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory1'
   */
  omgmat[0] = 0.0;
  omgmat[3] =
    -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[2];
  omgmat[6] =
    TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[1];
  omgmat[1] =
    TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[2];
  omgmat[4] = 0.0;
  omgmat[7] =
    -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[0];
  omgmat[2] =
    -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[1];
  omgmat[5] =
    TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[0];
  omgmat[8] = 0.0;
  omgtheta[0] =
    TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[0];
  omgtheta[1] =
    TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[1];
  omgtheta[2] =
    TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[2];
  theta = TSTraj_norm(omgtheta);
  if (fabs(theta) < 2.2204460492503131E-16) {
    memset(&ceil_xi[0], 0, 9U * sizeof(real_T));
    ceil_xi[0] = 1.0;
    ceil_xi[4] = 1.0;
    ceil_xi[8] = 1.0;
  } else {
    acosinput = sin(theta);
    rtb_Switch = cos(theta);
    for (i_0 = 0; i_0 < 9; i_0++) {
      omgmat[i_0] /= theta;
      b_I[i_0] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (i_0 = 0; i_0 < 3; i_0++) {
      for (i = 0; i < 3; i++) {
        ceil_xi_tmp = 3 * i + i_0;
        ceil_xi[ceil_xi_tmp] = (((1.0 - rtb_Switch) * omgmat[i_0 + 3] * omgmat[3
          * i + 1] + (1.0 - rtb_Switch) * omgmat[i_0] * omgmat[3 * i]) +
          omgmat[i_0 + 6] * (1.0 - rtb_Switch) * omgmat[3 * i + 2]) +
          (omgmat[ceil_xi_tmp] * acosinput + (real_T)b_I[ceil_xi_tmp]);
      }
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Switch = ceil_xi[3 * i_0 + 1];
    rtb_Sum1 = ceil_xi[3 * i_0];
    acosinput = ceil_xi[3 * i_0 + 2];
    for (i = 0; i <= 0; i += 2) {
      tmp_a = _mm_loadu_pd(&TSTraj_B.sf_TransToRp.R[i + 3]);
      tmp_7 = _mm_loadu_pd(&TSTraj_B.sf_TransToRp.R[i]);
      tmp_8 = _mm_loadu_pd(&TSTraj_B.sf_TransToRp.R[i + 6]);
      _mm_storeu_pd(&rtb_R_t[i + 3 * i_0], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(rtb_Switch), tmp_a), _mm_mul_pd(_mm_set1_pd(rtb_Sum1),
        tmp_7)), _mm_mul_pd(_mm_set1_pd(acosinput), tmp_8)));
    }

    for (i = 2; i < 3; i++) {
      rtb_R_t[i + 3 * i_0] = (TSTraj_B.sf_TransToRp.R[i + 3] * rtb_Switch +
        rtb_Sum1 * TSTraj_B.sf_TransToRp.R[i]) + TSTraj_B.sf_TransToRp.R[i + 6] *
        acosinput;
    }
  }

  for (i = 0; i <= 0; i += 2) {
    tmp_a = _mm_loadu_pd
      (&TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[
       i]);
    _mm_storeu_pd(&omgtheta[i], _mm_mul_pd(tmp_a, _mm_set1_pd(-1.0)));
  }

  for (i = 2; i < 3; i++) {
    omgtheta[i] =
      -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[i];
  }

  rtb_Switch = TSTraj_norm(omgtheta);
  if (rtb_Switch < 2.2204460492503131E-16) {
    memset(&a[0], 0, 9U * sizeof(real_T));
    a[0] = 1.0;
    a[4] = 1.0;
    a[8] = 1.0;
  } else {
    ceil_xi[0] = 0.0;
    ceil_xi[3] = -omgtheta[2];
    ceil_xi[6] = omgtheta[1];
    ceil_xi[1] = omgtheta[2];
    ceil_xi[4] = 0.0;
    ceil_xi[7] = -omgtheta[0];
    ceil_xi[2] = -omgtheta[1];
    ceil_xi[5] = omgtheta[0];
    ceil_xi[8] = 0.0;
    rtb_Sum1 = sin(rtb_Switch / 2.0) / (rtb_Switch / 2.0);
    acosinput = rtb_Sum1 * rtb_Sum1 / 2.0;
    theta = (1.0 - cos(rtb_Switch / 2.0) * rtb_Sum1) / (rtb_Switch * rtb_Switch);
    for (i_0 = 0; i_0 < 9; i_0++) {
      b_I[i_0] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (i_0 = 0; i_0 < 3; i_0++) {
      for (i = 0; i < 3; i++) {
        ceil_xi_tmp = 3 * i + i_0;
        a[ceil_xi_tmp] = ((ceil_xi[i_0 + 3] * theta * ceil_xi[3 * i + 1] + theta
                           * ceil_xi[i_0] * ceil_xi[3 * i]) + ceil_xi[i_0 + 6] *
                          theta * ceil_xi[3 * i + 2]) + (ceil_xi[ceil_xi_tmp] *
          acosinput + (real_T)b_I[ceil_xi_tmp]);
      }
    }
  }

  rtb_Sum1 =
    TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[1];
  acosinput =
    TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[0];
  theta =
    TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[2];
  for (i_0 = 0; i_0 <= 0; i_0 += 2) {
    tmp_a = _mm_loadu_pd(&a[i_0 + 3]);
    tmp_7 = _mm_loadu_pd(&a[i_0]);
    tmp_8 = _mm_loadu_pd(&a[i_0 + 6]);
    _mm_storeu_pd(&rtb_omega_t[i_0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_a,
      _mm_set1_pd(rtb_Sum1)), _mm_mul_pd(tmp_7, _mm_set1_pd(acosinput))),
      _mm_mul_pd(tmp_8, _mm_set1_pd(theta))));
  }

  for (i_0 = 2; i_0 < 3; i_0++) {
    rtb_omega_t[i_0] = (a[i_0 + 3] * rtb_Sum1 + a[i_0] * acosinput) + a[i_0 + 6]
      * theta;
  }

  if (rtb_Switch < 2.2204460492503131E-16) {
    memset(&a[0], 0, 9U * sizeof(real_T));
    a[0] = 1.0;
    a[4] = 1.0;
    a[8] = 1.0;
    omgmat[0] = 0.0;
    tmp_a = _mm_set1_pd(0.5);
    _mm_storeu_pd(&tmp_9[0], _mm_mul_pd(tmp_a, _mm_set_pd
      (-TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[
       1],
       TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
       [2])));
    omgmat[3] = tmp_9[0];
    omgmat[6] = tmp_9[1];
    omgmat[1] = 0.5 *
      -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
      [2];
    omgmat[4] = 0.0;
    tmp_a = _mm_mul_pd(tmp_a, _mm_loadu_pd
                       (&TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
                        [0]));
    _mm_storeu_pd(&tmp_9[0], tmp_a);
    omgmat[7] = tmp_9[0];
    omgmat[2] = tmp_9[1];
    omgmat[5] = 0.5 *
      -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
      [0];
    omgmat[8] = 0.0;
  } else {
    ceil_xi[0] = 0.0;
    ceil_xi[3] = -omgtheta[2];
    ceil_xi[6] = omgtheta[1];
    ceil_xi[1] = omgtheta[2];
    ceil_xi[4] = 0.0;
    ceil_xi[7] = -omgtheta[0];
    ceil_xi[2] = -omgtheta[1];
    ceil_xi[5] = omgtheta[0];
    ceil_xi[8] = 0.0;
    theta = sin(rtb_Switch / 2.0) / (rtb_Switch / 2.0);
    R = theta * theta;
    rtb_Sum1 = R / 2.0;
    theta *= cos(rtb_Switch / 2.0);
    rtb_Switch *= rtb_Switch;
    acosinput = (1.0 - theta) / rtb_Switch;
    for (i_0 = 0; i_0 < 9; i_0++) {
      b_I[i_0] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (i_0 = 0; i_0 < 3; i_0++) {
      for (i = 0; i < 3; i++) {
        ceil_xi_tmp = 3 * i + i_0;
        a[ceil_xi_tmp] = ((ceil_xi[i_0 + 3] * acosinput * ceil_xi[3 * i + 1] +
                           acosinput * ceil_xi[i_0] * ceil_xi[3 * i]) +
                          ceil_xi[i_0 + 6] * acosinput * ceil_xi[3 * i + 2]) +
          (ceil_xi[ceil_xi_tmp] * rtb_Sum1 + (real_T)b_I[ceil_xi_tmp]);
      }
    }

    ceil_xi[0] = 0.0;
    ceil_xi[3] = -omgtheta[2];
    ceil_xi[6] = omgtheta[1];
    ceil_xi[1] = omgtheta[2];
    ceil_xi[4] = 0.0;
    ceil_xi[7] = -omgtheta[0];
    ceil_xi[2] = -omgtheta[1];
    ceil_xi[5] = omgtheta[0];
    ceil_xi[8] = 0.0;
    tmp_a = _mm_set_pd(((1.0 - theta) * 3.0 / rtb_Switch - R / 2.0) * (1.0 /
      rtb_Switch), (theta - R) / rtb_Switch);
    _mm_storeu_pd(&tmp_9[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_mul_pd(tmp_a,
      _mm_set1_pd(omgtheta[0])), _mm_set1_pd
      (-TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[
       0])), _mm_mul_pd(_mm_mul_pd(tmp_a, _mm_set1_pd(omgtheta[1])), _mm_set1_pd
                        (-TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
                         [1]))), _mm_mul_pd(_mm_mul_pd(tmp_a, _mm_set1_pd
      (omgtheta[2])), _mm_set1_pd
      (-TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[
       2]))));
    theta = tmp_9[0];
    rtb_Switch = tmp_9[1];
    rtb_Sum1_0[0] = 0.0;
    rtb_Sum1_0[3] =
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[2];
    rtb_Sum1_0[6] =
      -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
      [1];
    rtb_Sum1_0[1] =
      -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
      [2];
    rtb_Sum1_0[4] = 0.0;
    rtb_Sum1_0[7] =
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[0];
    rtb_Sum1_0[2] =
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[1];
    rtb_Sum1_0[5] =
      -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
      [0];
    rtb_Sum1_0[8] = 0.0;
    omgmat[0] = 0.0;
    omgmat[3] = -omgtheta[2];
    omgmat[6] = omgtheta[1];
    omgmat[1] = omgtheta[2];
    omgmat[4] = 0.0;
    omgmat[7] = -omgtheta[0];
    omgmat[2] = -omgtheta[1];
    omgmat[5] = omgtheta[0];
    omgmat[8] = 0.0;
    y[0] = 0.0;
    y[3] = -omgtheta[2];
    y[6] = omgtheta[1];
    y[1] = omgtheta[2];
    y[4] = 0.0;
    y[7] = -omgtheta[0];
    y[2] = -omgtheta[1];
    y[5] = omgtheta[0];
    y[8] = 0.0;
    tmp[0] = 0.0;
    tmp[3] =
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[2];
    tmp[6] =
      -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
      [1];
    tmp[1] =
      -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
      [2];
    tmp[4] = 0.0;
    tmp[7] =
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[0];
    tmp[2] =
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[1];
    tmp[5] =
      -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
      [0];
    tmp[8] = 0.0;
    for (i_0 = 0; i_0 < 3; i_0++) {
      i = 3 * i_0 + 1;
      R = tmp[i];
      tmp_2 = tmp[3 * i_0];
      ceil_xi_tmp = 3 * i_0 + 2;
      tmp_3 = tmp[ceil_xi_tmp];
      tmp_4 = omgmat[i];
      tmp_5 = omgmat[3 * i_0];
      tmp_6 = omgmat[ceil_xi_tmp];
      for (i = 0; i <= 0; i += 2) {
        tmp_a = _mm_loadu_pd(&y[i + 3]);
        tmp_7 = _mm_loadu_pd(&y[i]);
        tmp_8 = _mm_loadu_pd(&y[i + 6]);
        ceil_xi_tmp = 3 * i_0 + i;
        _mm_storeu_pd(&tmp_1[ceil_xi_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(R), tmp_a), _mm_mul_pd(_mm_set1_pd(tmp_2), tmp_7)),
          _mm_mul_pd(_mm_set1_pd(tmp_3), tmp_8)));
        tmp_a = _mm_loadu_pd(&rtb_Sum1_0[i + 3]);
        tmp_7 = _mm_loadu_pd(&rtb_Sum1_0[i]);
        tmp_8 = _mm_loadu_pd(&rtb_Sum1_0[i + 6]);
        _mm_storeu_pd(&tmp_0[ceil_xi_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(tmp_4), tmp_a), _mm_mul_pd(_mm_set1_pd(tmp_5), tmp_7)),
          _mm_mul_pd(_mm_set1_pd(tmp_6), tmp_8)));
      }

      for (i = 2; i < 3; i++) {
        _mm_storeu_pd(&tmp_9[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
          (tmp_4, R), _mm_set_pd(rtb_Sum1_0[i + 3], y[i + 3])), _mm_mul_pd
          (_mm_set_pd(tmp_5, tmp_2), _mm_set_pd(rtb_Sum1_0[i], y[i]))),
          _mm_mul_pd(_mm_set_pd(tmp_6, tmp_3), _mm_set_pd(rtb_Sum1_0[i + 6], y[i
          + 6]))));
        ceil_xi_tmp = 3 * i_0 + i;
        tmp_1[ceil_xi_tmp] = tmp_9[0];
        tmp_0[ceil_xi_tmp] = tmp_9[1];
      }
    }

    rtb_Sum1_0[0] = rtb_Sum1 * 0.0;
    tmp_a = _mm_set1_pd(rtb_Sum1);
    _mm_storeu_pd(&tmp_9[0], _mm_mul_pd(tmp_a, _mm_set_pd
      (-TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[
       1],
       TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
       [2])));
    rtb_Sum1_0[3] = tmp_9[0];
    rtb_Sum1_0[6] = tmp_9[1];
    rtb_Sum1_0[1] = rtb_Sum1 *
      -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
      [2];
    rtb_Sum1_0[4] = rtb_Sum1 * 0.0;
    tmp_a = _mm_mul_pd(tmp_a, _mm_loadu_pd
                       (&TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
                        [0]));
    _mm_storeu_pd(&tmp_9[0], tmp_a);
    rtb_Sum1_0[7] = tmp_9[0];
    rtb_Sum1_0[2] = tmp_9[1];
    rtb_Sum1_0[5] = rtb_Sum1 *
      -TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
      [0];
    rtb_Sum1_0[8] = rtb_Sum1 * 0.0;
    y[0] = theta * 0.0;
    tmp_a = _mm_set1_pd(theta);
    _mm_storeu_pd(&tmp_9[0], _mm_mul_pd(tmp_a, _mm_set_pd(omgtheta[1],
      -omgtheta[2])));
    y[3] = tmp_9[0];
    y[6] = tmp_9[1];
    y[1] = theta * omgtheta[2];
    y[4] = theta * 0.0;
    _mm_storeu_pd(&tmp_9[0], _mm_mul_pd(tmp_a, _mm_set_pd(-omgtheta[1],
      -omgtheta[0])));
    y[7] = tmp_9[0];
    y[2] = tmp_9[1];
    y[5] = theta * omgtheta[0];
    y[8] = theta * 0.0;
    for (i_0 = 0; i_0 < 3; i_0++) {
      for (i = 0; i < 3; i++) {
        ceil_xi_tmp = 3 * i + i_0;
        omgmat[ceil_xi_tmp] = (((tmp_0[ceil_xi_tmp] + tmp_1[ceil_xi_tmp]) *
          acosinput + rtb_Sum1_0[ceil_xi_tmp]) + y[ceil_xi_tmp]) - ((ceil_xi[i_0
          + 3] * rtb_Switch * ceil_xi[3 * i + 1] + rtb_Switch * ceil_xi[i_0] *
          ceil_xi[3 * i]) + ceil_xi[i_0 + 6] * rtb_Switch * ceil_xi[3 * i + 2]);
      }
    }
  }

  /* Outport: '<Root>/T_t' incorporates:
   *  MATLAB Function: '<S1>/GetT,V,Vdot'
   *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory'
   */
  for (i_0 = 0; i_0 < 3; i_0++) {
    i = i_0 << 2;
    TSTraj_Y.T_t[i] = rtb_R_t[3 * i_0];
    TSTraj_Y.T_t[i + 1] = rtb_R_t[3 * i_0 + 1];
    TSTraj_Y.T_t[i + 2] = rtb_R_t[3 * i_0 + 2];
    TSTraj_Y.T_t[i_0 + 12] =
      TSTraj_B.TrapezoidalVelocityProfileTra_p.TrapezoidalVelocityProfileTra_j[i_0];
  }

  TSTraj_Y.T_t[3] = 0.0;
  TSTraj_Y.T_t[7] = 0.0;
  TSTraj_Y.T_t[11] = 0.0;
  TSTraj_Y.T_t[15] = 1.0;

  /* End of Outport: '<Root>/T_t' */

  /* MATLAB Function: '<S1>/GetT,V,Vdot' incorporates:
   *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory'
   */
  for (i_0 = 0; i_0 < 3; i_0++) {
    ceil_xi[3 * i_0] = rtb_R_t[i_0];
    ceil_xi[3 * i_0 + 1] = rtb_R_t[i_0 + 3];
    ceil_xi[3 * i_0 + 2] = rtb_R_t[i_0 + 6];
  }

  rtb_Sum1_0[0] = 0.0;
  rtb_Sum1_0[3] = -rtb_omega_t[2];
  rtb_Sum1_0[6] = rtb_omega_t[1];
  rtb_Sum1_0[1] = rtb_omega_t[2];
  rtb_Sum1_0[4] = 0.0;
  rtb_Sum1_0[7] = -rtb_omega_t[0];
  rtb_Sum1_0[2] = -rtb_omega_t[1];
  rtb_Sum1_0[5] = rtb_omega_t[0];
  rtb_Sum1_0[8] = 0.0;
  for (i = 0; i < 3; i++) {
    /* Outport: '<Root>/V_t' */
    TSTraj_Y.V_t[i + 3] = rtb_omega_t[i];
    rtb_Switch = 0.0;
    rtb_Sum1 = 0.0;
    acosinput = rtb_R_t[i + 3];
    theta = rtb_R_t[i];
    R = rtb_R_t[i + 6];
    for (i_0 = 0; i_0 < 3; i_0++) {
      rtb_Switch += ceil_xi[3 * i_0 + i] *
        TSTraj_B.TrapezoidalVelocityProfileTra_p.TrapezoidalVelocityProfileTra_l[
        i_0];
      rtb_Sum1 += ((rtb_Sum1_0[3 * i_0 + 1] * acosinput + rtb_Sum1_0[3 * i_0] *
                    theta) + rtb_Sum1_0[3 * i_0 + 2] * R) *
        TSTraj_B.TrapezoidalVelocityProfileTra_p.TrapezoidalVelocityProfileTra_l[
        i_0];
    }

    /* Outport: '<Root>/V_t' incorporates:
     *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory'
     */
    TSTraj_Y.V_t[i] = rtb_Switch;

    /* Outport: '<Root>/Vdot_t' incorporates:
     *  MATLAB Function: '<S1>/MATLAB Function1'
     *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory'
     *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory1'
     */
    TSTraj_Y.Vdot_t[i] = ((ceil_xi[i + 3] *
      TSTraj_B.TrapezoidalVelocityProfileTra_p.TrapezoidalVelocityProfileTra_f[1]
      + ceil_xi[i] *
      TSTraj_B.TrapezoidalVelocityProfileTra_p.TrapezoidalVelocityProfileTra_f[0])
                          + ceil_xi[i + 6] *
                          TSTraj_B.TrapezoidalVelocityProfileTra_p.TrapezoidalVelocityProfileTra_f
                          [2]) + rtb_Sum1;
    TSTraj_Y.Vdot_t[i + 3] = ((a[i + 3] *
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_f[1]
      + a[i] *
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_f[0])
      + a[i + 6] *
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_f[2])
      + ((omgmat[i + 3] *
          TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
          [1] + omgmat[i] *
          TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
          [0]) + omgmat[i + 6] *
         TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l
         [2]);

    /* Outport: '<Root>/p_t' incorporates:
     *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory'
     */
    TSTraj_Y.p_t[i] =
      TSTraj_B.TrapezoidalVelocityProfileTra_p.TrapezoidalVelocityProfileTra_j[i];

    /* Outport: '<Root>/pdot_t' incorporates:
     *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory'
     */
    TSTraj_Y.pdot_t[i] =
      TSTraj_B.TrapezoidalVelocityProfileTra_p.TrapezoidalVelocityProfileTra_l[i];

    /* Outport: '<Root>/pddot_t' incorporates:
     *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory'
     */
    TSTraj_Y.pddot_t[i] =
      TSTraj_B.TrapezoidalVelocityProfileTra_p.TrapezoidalVelocityProfileTra_f[i];

    /* Outport: '<Root>/xi_t' incorporates:
     *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory1'
     */
    TSTraj_Y.xi_t[i] =
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_j[i];

    /* Outport: '<Root>/xidot_t' incorporates:
     *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory1'
     */
    TSTraj_Y.xidot_t[i] =
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_l[i];

    /* Outport: '<Root>/xiddot_t' incorporates:
     *  MATLABSystem: '<S1>/Trapezoidal Velocity Profile Trajectory1'
     */
    TSTraj_Y.xiddot_t[i] =
      TSTraj_B.TrapezoidalVelocityProfileTr_pn.TrapezoidalVelocityProfileTra_f[i];
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(TSTraj_M->rtwLogInfo, (&TSTraj_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.001s, 0.0s] */
    if ((rtmGetTFinal(TSTraj_M)!=-1) &&
        !((rtmGetTFinal(TSTraj_M)-TSTraj_M->Timing.taskTime0) >
          TSTraj_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(TSTraj_M, "Simulation finished");
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
  if (!(++TSTraj_M->Timing.clockTick0)) {
    ++TSTraj_M->Timing.clockTickH0;
  }

  TSTraj_M->Timing.taskTime0 = TSTraj_M->Timing.clockTick0 *
    TSTraj_M->Timing.stepSize0 + TSTraj_M->Timing.clockTickH0 *
    TSTraj_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void TSTraj_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)TSTraj_M, 0,
                sizeof(RT_MODEL_TSTraj_T));
  rtmSetTFinal(TSTraj_M, 10.0);
  TSTraj_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    TSTraj_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(TSTraj_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(TSTraj_M->rtwLogInfo, (NULL));
    rtliSetLogT(TSTraj_M->rtwLogInfo, "tout");
    rtliSetLogX(TSTraj_M->rtwLogInfo, "");
    rtliSetLogXFinal(TSTraj_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(TSTraj_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(TSTraj_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(TSTraj_M->rtwLogInfo, 0);
    rtliSetLogDecimation(TSTraj_M->rtwLogInfo, 1);
    rtliSetLogY(TSTraj_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(TSTraj_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(TSTraj_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &TSTraj_B), 0,
                sizeof(B_TSTraj_T));

  /* states (dwork) */
  (void) memset((void *)&TSTraj_DW, 0,
                sizeof(DW_TSTraj_T));

  /* external inputs */
  (void)memset(&TSTraj_U, 0, sizeof(ExtU_TSTraj_T));

  /* external outputs */
  (void)memset(&TSTraj_Y, 0, sizeof(ExtY_TSTraj_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(TSTraj_M->rtwLogInfo, 0.0, rtmGetTFinal
    (TSTraj_M), TSTraj_M->Timing.stepSize0, (&rtmGetErrorStatus(TSTraj_M)));
  TrapezoidalVelocityProfile_Init(&TSTraj_DW.TrapezoidalVelocityProfileTra_p);
  TrapezoidalVelocityProfile_Init(&TSTraj_DW.TrapezoidalVelocityProfileTr_pn);
}

/* Model terminate function */
void TSTraj_terminate(void)
{
  /* (no terminate code required) */
}
