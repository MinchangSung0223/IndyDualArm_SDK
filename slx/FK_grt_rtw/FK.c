/*
 * FK.c
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

#include "FK.h"
#include "rtwtypes.h"
#include <string.h>
#include <emmintrin.h>
#include "FK_private.h"
#include <math.h>

/* External inputs (root inport signals with default storage) */
ExtU_FK_T FK_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_FK_T FK_Y;

/* Real-time model */
static RT_MODEL_FK_T FK_M_;
RT_MODEL_FK_T *const FK_M = &FK_M_;

/* Forward declaration for local functions */
static real_T FK_norm(const real_T x[3]);

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real_T FK_norm(const real_T x[3])
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

/* Function for MATLAB Function: '<S1>/MATLAB Function7' */
void FK_exp6(const real_T lambda[6], real_T T[16])
{
  __m128d tmp;
  real_T se3mat[16];
  real_T R[9];
  real_T omgmat_tmp[9];
  real_T se3mat_0[3];
  real_T a_tmp;
  real_T b_a;
  real_T b_a_tmp;
  real_T theta;
  int32_T R_tmp;
  int32_T T_tmp;
  int32_T i;
  int8_T b_I[9];
  static const int8_T d_a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  se3mat[0] = 0.0;
  se3mat[4] = -lambda[5];
  se3mat[8] = lambda[4];
  se3mat[1] = lambda[5];
  se3mat[5] = 0.0;
  se3mat[9] = -lambda[3];
  se3mat[2] = -lambda[4];
  se3mat[6] = lambda[3];
  se3mat[10] = 0.0;
  se3mat[12] = lambda[0];
  se3mat[13] = lambda[1];
  se3mat[14] = lambda[2];
  se3mat[3] = 0.0;
  se3mat[7] = 0.0;
  se3mat[11] = 0.0;
  se3mat[15] = 0.0;
  se3mat_0[0] = lambda[3];
  se3mat_0[1] = lambda[4];
  se3mat_0[2] = lambda[5];
  theta = FK_norm(se3mat_0);
  if (fabs(theta) < 2.2204460492503131E-16) {
    for (i = 0; i < 9; i++) {
      b_I[i] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (i = 0; i < 3; i++) {
      T_tmp = i << 2;
      T[T_tmp] = b_I[3 * i];
      T[T_tmp + 1] = b_I[3 * i + 1];
      T[T_tmp + 2] = b_I[3 * i + 2];
      T[i + 12] = se3mat[i + 12];
    }

    T[3] = 0.0;
    T[7] = 0.0;
    T[11] = 0.0;
    T[15] = 1.0;
  } else {
    for (i = 0; i < 3; i++) {
      T_tmp = i << 2;
      tmp = _mm_div_pd(_mm_loadu_pd(&se3mat[T_tmp]), _mm_set1_pd(theta));
      _mm_storeu_pd(&omgmat_tmp[3 * i], tmp);
      omgmat_tmp[3 * i + 2] = se3mat[T_tmp + 2] / theta;
    }

    a_tmp = cos(theta);
    b_a_tmp = sin(theta);
    b_a = theta - b_a_tmp;
    for (i = 0; i < 9; i++) {
      b_I[i] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (i = 0; i < 3; i++) {
      for (T_tmp = 0; T_tmp < 3; T_tmp++) {
        R_tmp = 3 * T_tmp + i;
        R[R_tmp] = (((1.0 - a_tmp) * omgmat_tmp[i] * omgmat_tmp[3 * T_tmp] +
                     (1.0 - a_tmp) * omgmat_tmp[i + 3] * omgmat_tmp[3 * T_tmp +
                     1]) + (1.0 - a_tmp) * omgmat_tmp[i + 6] * omgmat_tmp[3 *
                    T_tmp + 2]) + (omgmat_tmp[R_tmp] * b_a_tmp + (real_T)
          b_I[R_tmp]);
      }
    }

    for (i = 0; i < 3; i++) {
      b_a_tmp = 0.0;
      for (T_tmp = 0; T_tmp < 3; T_tmp++) {
        R_tmp = 3 * T_tmp + i;
        b_a_tmp += (((omgmat_tmp[i + 3] * b_a * omgmat_tmp[3 * T_tmp + 1] + b_a *
                      omgmat_tmp[i] * omgmat_tmp[3 * T_tmp]) + omgmat_tmp[i + 6]
                     * b_a * omgmat_tmp[3 * T_tmp + 2]) + ((1.0 - a_tmp) *
          omgmat_tmp[R_tmp] + (real_T)d_a[R_tmp] * theta)) * se3mat[T_tmp + 12];
        T[T_tmp + (i << 2)] = R[3 * i + T_tmp];
      }

      T[i + 12] = b_a_tmp / theta;
    }

    T[3] = 0.0;
    T[7] = 0.0;
    T[11] = 0.0;
    T[15] = 1.0;
  }
}

/* Model step function */
void FK_step(void)
{
  __m128d tmp_b;
  __m128d tmp_c;
  real_T rtb_J_m[72];
  real_T rtb_Jdot_f[72];
  real_T omgmat_0[36];
  real_T rtb_J[36];
  real_T rtb_J_h[36];
  real_T rtb_Jdot[36];
  real_T rtb_Jdot_l[36];
  real_T rtb_T_m[16];
  real_T tmp_2[16];
  real_T tmp_3[16];
  real_T tmp_4[16];
  real_T tmp_5[16];
  real_T rtb_TmpSignalConversionAtSFunct[12];
  real_T rtb_q_lr_h[12];
  real_T omgmat[9];
  real_T dJidt[6];
  real_T rtb_MatrixMultiply2[6];
  real_T x[6];
  real_T rtb_T_e[3];
  real_T tmp_d[2];
  real_T rtb_Sum2;
  real_T rtb_T_e_0;
  real_T tmp;
  real_T tmp_6;
  real_T tmp_7;
  int32_T i;
  int32_T i_0;
  int32_T k;
  int32_T rtb_T_e_tmp;
  int32_T tmp_e;
  int8_T tmp_0[9];
  int8_T tmp_1[3];
  int8_T tmp_8;
  int8_T tmp_9;
  int8_T tmp_a;
  static const int8_T y[9] = { 0, 1, 0, -1, 0, 0, 0, 0, 0 };

  static const int8_T b_y[3] = { 0, 0, 1 };

  static const int8_T c[6] = { 0, 0, 0, 0, 0, 1 };

  /* Sum: '<S2>/Sum2' incorporates:
   *  Constant: '<S2>/Constant20'
   *  Inport: '<Root>/q'
   */
  rtb_Sum2 = FK_U.q[0] + FK_P.Constant20_Value;

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/q'
   *  MATLAB Function: '<S1>/MATLAB Function1'
   */
  rtb_TmpSignalConversionAtSFunct[0] = rtb_Sum2;
  memcpy(&rtb_TmpSignalConversionAtSFunct[1], &FK_U.q[1], 11U * sizeof(real_T));

  /* MATLAB Function: '<S1>/MATLAB Function1' */
  for (i = 0; i < 6; i++) {
    x[i] = rtb_TmpSignalConversionAtSFunct[i];
  }

  tmp = x[0];
  x[0] = x[5];
  x[5] = tmp;
  tmp = x[1];
  x[1] = x[4];
  x[4] = tmp;
  tmp = x[2];
  x[2] = x[3];
  x[3] = tmp;
  for (i = 0; i <= 4; i += 2) {
    tmp_c = _mm_loadu_pd(&x[i]);
    _mm_storeu_pd(&rtb_q_lr_h[i], _mm_mul_pd(tmp_c, _mm_set1_pd(-1.0)));
    tmp_c = _mm_loadu_pd(&rtb_TmpSignalConversionAtSFunct[i + 6]);
    _mm_storeu_pd(&rtb_q_lr_h[i + 6], tmp_c);

    /* MATLAB Function: '<S1>/MATLAB Function2' incorporates:
     *  Inport: '<Root>/qdot'
     */
    _mm_storeu_pd(&x[i], _mm_loadu_pd(&FK_U.qdot[i]));
  }

  /* MATLAB Function: '<S1>/MATLAB Function2' incorporates:
   *  Inport: '<Root>/qdot'
   */
  tmp = x[0];
  x[0] = x[5];
  x[5] = tmp;
  tmp = x[1];
  x[1] = x[4];
  x[4] = tmp;
  tmp = x[2];
  x[2] = x[3];
  x[3] = tmp;
  for (i = 0; i <= 4; i += 2) {
    tmp_c = _mm_loadu_pd(&x[i]);
    _mm_storeu_pd(&rtb_TmpSignalConversionAtSFunct[i], _mm_mul_pd(tmp_c,
      _mm_set1_pd(-1.0)));
    _mm_storeu_pd(&rtb_TmpSignalConversionAtSFunct[i + 6], _mm_loadu_pd
                  (&FK_U.qdot[i + 6]));
  }

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Inport: '<Root>/lambda_lr'
   */
  memset(&rtb_J_m[0], 0, 72U * sizeof(real_T));
  FK_exp6(&FK_U.lambda_lr[72], rtb_T_m);
  for (i_0 = 0; i_0 < 9; i_0++) {
    /* MATLAB Function: '<S1>/MATLAB Function8' incorporates:
     *  MATLAB Function: '<S1>/MATLAB Function7'
     */
    tmp_0[i_0] = y[i_0];
  }

  for (i = 0; i < 3; i++) {
    /* MATLAB Function: '<S1>/MATLAB Function8' incorporates:
     *  MATLAB Function: '<S1>/MATLAB Function7'
     */
    tmp_1[i] = b_y[i];
    tmp = 0.0;
    for (i_0 = 0; i_0 < 3; i_0++) {
      rtb_T_e_tmp = i << 2;
      tmp += (((real_T)tmp_0[3 * i_0 + 1] * rtb_T_m[rtb_T_e_tmp + 1] + (real_T)
               tmp_0[3 * i_0] * rtb_T_m[rtb_T_e_tmp]) + (real_T)tmp_0[3 * i_0 +
              2] * rtb_T_m[rtb_T_e_tmp + 2]) * rtb_T_m[i_0 + 12];
    }

    rtb_T_e[i] = tmp;
  }

  tmp_8 = tmp_1[1];
  tmp_9 = tmp_1[0];
  tmp_a = tmp_1[2];
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_J_m[i_0 + 66] = rtb_T_e[i_0];
    i = i_0 << 2;
    rtb_J_m[i_0 + 69] = (rtb_T_m[i + 1] * (real_T)tmp_8 + rtb_T_m[i] * (real_T)
                         tmp_9) + rtb_T_m[i + 2] * (real_T)tmp_a;
  }

  for (k = 0; k < 12; k++) {
    FK_exp6(&FK_U.lambda_lr[(11 - k) * 6], tmp_2);
    tmp = rtb_q_lr_h[11 - k];
    for (i_0 = 0; i_0 <= 4; i_0 += 2) {
      _mm_storeu_pd(&x[i_0], _mm_mul_pd(_mm_set_pd(c[i_0 + 1], c[i_0]),
        _mm_set1_pd(tmp)));
    }

    FK_exp6(x, tmp_3);
    for (i_0 = 0; i_0 < 4; i_0++) {
      tmp = tmp_2[i_0 + 4];
      rtb_T_e_0 = tmp_2[i_0];
      tmp_6 = tmp_2[i_0 + 8];
      tmp_7 = tmp_2[i_0 + 12];
      for (i = 0; i <= 2; i += 2) {
        rtb_T_e_tmp = (i + 1) << 2;
        tmp_e = i << 2;
        _mm_storeu_pd(&tmp_d[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(tmp_3[rtb_T_e_tmp + 1], tmp_3[tmp_e + 1]), _mm_set1_pd(tmp)),
          _mm_mul_pd(_mm_set_pd(tmp_3[rtb_T_e_tmp], tmp_3[tmp_e]), _mm_set1_pd
                     (rtb_T_e_0))), _mm_mul_pd(_mm_set_pd(tmp_3[rtb_T_e_tmp + 2],
          tmp_3[tmp_e + 2]), _mm_set1_pd(tmp_6))), _mm_mul_pd(_mm_set_pd
          (tmp_3[rtb_T_e_tmp + 3], tmp_3[tmp_e + 3]), _mm_set1_pd(tmp_7))));
        tmp_4[i_0 + tmp_e] = tmp_d[0];
        tmp_4[i_0 + rtb_T_e_tmp] = tmp_d[1];
      }

      tmp = tmp_4[i_0 + 4];
      rtb_T_e_0 = tmp_4[i_0];
      tmp_6 = tmp_4[i_0 + 8];
      tmp_7 = tmp_4[i_0 + 12];
      for (i = 0; i <= 2; i += 2) {
        rtb_T_e_tmp = (i + 1) << 2;
        tmp_e = i << 2;
        _mm_storeu_pd(&tmp_d[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(rtb_T_m[rtb_T_e_tmp + 1], rtb_T_m[tmp_e + 1]), _mm_set1_pd
           (tmp)), _mm_mul_pd(_mm_set_pd(rtb_T_m[rtb_T_e_tmp], rtb_T_m[tmp_e]),
                              _mm_set1_pd(rtb_T_e_0))), _mm_mul_pd(_mm_set_pd
          (rtb_T_m[rtb_T_e_tmp + 2], rtb_T_m[tmp_e + 2]), _mm_set1_pd(tmp_6))),
          _mm_mul_pd(_mm_set_pd(rtb_T_m[rtb_T_e_tmp + 3], rtb_T_m[tmp_e + 3]),
                     _mm_set1_pd(tmp_7))));
        tmp_5[i_0 + tmp_e] = tmp_d[0];
        tmp_5[i_0 + rtb_T_e_tmp] = tmp_d[1];
      }
    }

    memcpy(&rtb_T_m[0], &tmp_5[0], sizeof(real_T) << 4U);
    if (12 - k > 1) {
      for (i_0 = 0; i_0 < 3; i_0++) {
        tmp = 0.0;
        rtb_T_e_0 = 0.0;
        for (i = 0; i < 3; i++) {
          rtb_T_e_tmp = i_0 << 2;
          tmp += (((real_T)tmp_0[3 * i + 1] * rtb_T_m[rtb_T_e_tmp + 1] + (real_T)
                   tmp_0[3 * i] * rtb_T_m[rtb_T_e_tmp]) + (real_T)tmp_0[3 * i +
                  2] * rtb_T_m[rtb_T_e_tmp + 2]) * rtb_T_m[i + 12];
          rtb_T_e_0 += rtb_T_m[rtb_T_e_tmp + i] * (real_T)tmp_1[i];
        }

        i = (10 - k) * 6 + i_0;
        rtb_J_m[i] = tmp;
        rtb_J_m[i + 3] = rtb_T_e_0;
      }
    }
  }

  for (k = 0; k < 12; k++) {
    for (i = 0; i < 6; i++) {
      dJidt[i] = 0.0;
    }

    for (rtb_T_e_tmp = 0; rtb_T_e_tmp < 12; rtb_T_e_tmp++) {
      for (i = 0; i < 6; i++) {
        x[i] = 0.0;
      }

      if (k < rtb_T_e_tmp) {
        omgmat[0] = 0.0;
        tmp = rtb_J_m[6 * k + 5];
        omgmat[3] = -tmp;
        rtb_T_e_0 = rtb_J_m[6 * k + 4];
        omgmat[6] = rtb_T_e_0;
        omgmat[1] = tmp;
        omgmat[4] = 0.0;
        tmp = rtb_J_m[6 * k + 3];
        omgmat[7] = -tmp;
        omgmat[2] = -rtb_T_e_0;
        omgmat[5] = tmp;
        omgmat[8] = 0.0;
        omgmat_0[18] = 0.0;
        tmp = rtb_J_m[6 * k + 2];
        omgmat_0[24] = -tmp;
        rtb_T_e_0 = rtb_J_m[6 * k + 1];
        omgmat_0[30] = rtb_T_e_0;
        omgmat_0[19] = tmp;
        omgmat_0[25] = 0.0;
        tmp = rtb_J_m[6 * k];
        omgmat_0[31] = -tmp;
        omgmat_0[20] = -rtb_T_e_0;
        omgmat_0[26] = tmp;
        omgmat_0[32] = 0.0;
        for (i_0 = 0; i_0 < 3; i_0++) {
          tmp = omgmat[3 * i_0];
          omgmat_0[6 * i_0] = tmp;
          omgmat_0[6 * i_0 + 3] = 0.0;
          i = (i_0 + 3) * 6;
          omgmat_0[i + 3] = tmp;
          tmp = omgmat[3 * i_0 + 1];
          omgmat_0[6 * i_0 + 1] = tmp;
          omgmat_0[6 * i_0 + 4] = 0.0;
          omgmat_0[i + 4] = tmp;
          tmp = omgmat[3 * i_0 + 2];
          omgmat_0[6 * i_0 + 2] = tmp;
          omgmat_0[6 * i_0 + 5] = 0.0;
          omgmat_0[i + 5] = tmp;
        }

        for (i_0 = 0; i_0 < 6; i_0++) {
          tmp = 0.0;
          for (i = 0; i < 6; i++) {
            tmp += omgmat_0[6 * i + i_0] * rtb_J_m[6 * rtb_T_e_tmp + i];
          }

          x[i_0] = tmp;
        }
      }

      tmp = rtb_TmpSignalConversionAtSFunct[rtb_T_e_tmp];
      for (i_0 = 0; i_0 <= 4; i_0 += 2) {
        tmp_c = _mm_loadu_pd(&x[i_0]);
        tmp_b = _mm_loadu_pd(&dJidt[i_0]);
        _mm_storeu_pd(&dJidt[i_0], _mm_add_pd(_mm_mul_pd(tmp_c, _mm_set1_pd(tmp)),
          tmp_b));
      }
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      rtb_Jdot_f[i_0 + 6 * k] = dJidt[i_0];
    }
  }

  /* End of MATLAB Function: '<S1>/MATLAB Function' */

  /* Outport: '<Root>/T_lr' */
  memcpy(&FK_Y.T_lr[0], &rtb_T_m[0], sizeof(real_T) << 4U);

  /* MATLAB Function: '<S1>/MATLAB Function8' incorporates:
   *  Inport: '<Root>/lambda_r'
   *  Inport: '<Root>/q'
   */
  memset(&rtb_J[0], 0, 36U * sizeof(real_T));
  FK_exp6(&FK_U.lambda_r[36], rtb_T_m);
  for (i_0 = 0; i_0 < 3; i_0++) {
    tmp = 0.0;
    rtb_T_e_0 = 0.0;
    for (i = 0; i < 3; i++) {
      rtb_T_e_tmp = i_0 << 2;
      tmp += (((real_T)tmp_0[3 * i + 1] * rtb_T_m[rtb_T_e_tmp + 1] + (real_T)
               tmp_0[3 * i] * rtb_T_m[rtb_T_e_tmp]) + (real_T)tmp_0[3 * i + 2] *
              rtb_T_m[rtb_T_e_tmp + 2]) * rtb_T_m[i + 12];
      rtb_T_e_0 += rtb_T_m[rtb_T_e_tmp + i] * (real_T)tmp_1[i];
    }

    rtb_J[i_0 + 30] = tmp;
    rtb_J[i_0 + 33] = rtb_T_e_0;
  }

  for (k = 0; k < 6; k++) {
    FK_exp6(&FK_U.lambda_r[(5 - k) * 6], tmp_2);
    tmp = FK_U.q[11 - k];
    for (i_0 = 0; i_0 <= 4; i_0 += 2) {
      _mm_storeu_pd(&x[i_0], _mm_mul_pd(_mm_set_pd(c[i_0 + 1], c[i_0]),
        _mm_set1_pd(tmp)));
    }

    FK_exp6(x, tmp_3);
    for (i_0 = 0; i_0 < 4; i_0++) {
      tmp = tmp_2[i_0 + 4];
      rtb_T_e_0 = tmp_2[i_0];
      tmp_6 = tmp_2[i_0 + 8];
      tmp_7 = tmp_2[i_0 + 12];
      for (i = 0; i <= 2; i += 2) {
        rtb_T_e_tmp = (i + 1) << 2;
        tmp_e = i << 2;
        _mm_storeu_pd(&tmp_d[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(tmp_3[rtb_T_e_tmp + 1], tmp_3[tmp_e + 1]), _mm_set1_pd(tmp)),
          _mm_mul_pd(_mm_set_pd(tmp_3[rtb_T_e_tmp], tmp_3[tmp_e]), _mm_set1_pd
                     (rtb_T_e_0))), _mm_mul_pd(_mm_set_pd(tmp_3[rtb_T_e_tmp + 2],
          tmp_3[tmp_e + 2]), _mm_set1_pd(tmp_6))), _mm_mul_pd(_mm_set_pd
          (tmp_3[rtb_T_e_tmp + 3], tmp_3[tmp_e + 3]), _mm_set1_pd(tmp_7))));
        tmp_4[i_0 + tmp_e] = tmp_d[0];
        tmp_4[i_0 + rtb_T_e_tmp] = tmp_d[1];
      }

      tmp = tmp_4[i_0 + 4];
      rtb_T_e_0 = tmp_4[i_0];
      tmp_6 = tmp_4[i_0 + 8];
      tmp_7 = tmp_4[i_0 + 12];
      for (i = 0; i <= 2; i += 2) {
        rtb_T_e_tmp = (i + 1) << 2;
        tmp_e = i << 2;
        _mm_storeu_pd(&tmp_d[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(rtb_T_m[rtb_T_e_tmp + 1], rtb_T_m[tmp_e + 1]), _mm_set1_pd
           (tmp)), _mm_mul_pd(_mm_set_pd(rtb_T_m[rtb_T_e_tmp], rtb_T_m[tmp_e]),
                              _mm_set1_pd(rtb_T_e_0))), _mm_mul_pd(_mm_set_pd
          (rtb_T_m[rtb_T_e_tmp + 2], rtb_T_m[tmp_e + 2]), _mm_set1_pd(tmp_6))),
          _mm_mul_pd(_mm_set_pd(rtb_T_m[rtb_T_e_tmp + 3], rtb_T_m[tmp_e + 3]),
                     _mm_set1_pd(tmp_7))));
        tmp_5[i_0 + tmp_e] = tmp_d[0];
        tmp_5[i_0 + rtb_T_e_tmp] = tmp_d[1];
      }
    }

    memcpy(&rtb_T_m[0], &tmp_5[0], sizeof(real_T) << 4U);
    if (6 - k > 1) {
      for (i_0 = 0; i_0 < 3; i_0++) {
        tmp = 0.0;
        rtb_T_e_0 = 0.0;
        for (i = 0; i < 3; i++) {
          rtb_T_e_tmp = i_0 << 2;
          tmp += (((real_T)tmp_0[3 * i + 1] * rtb_T_m[rtb_T_e_tmp + 1] + (real_T)
                   tmp_0[3 * i] * rtb_T_m[rtb_T_e_tmp]) + (real_T)tmp_0[3 * i +
                  2] * rtb_T_m[rtb_T_e_tmp + 2]) * rtb_T_m[i + 12];
          rtb_T_e_0 += rtb_T_m[rtb_T_e_tmp + i] * (real_T)tmp_1[i];
        }

        i = (4 - k) * 6 + i_0;
        rtb_J[i] = tmp;
        rtb_J[i + 3] = rtb_T_e_0;
      }
    }
  }

  for (k = 0; k < 6; k++) {
    for (i = 0; i < 6; i++) {
      dJidt[i] = 0.0;
    }

    for (rtb_T_e_tmp = 0; rtb_T_e_tmp < 6; rtb_T_e_tmp++) {
      for (i = 0; i < 6; i++) {
        x[i] = 0.0;
      }

      if (k < rtb_T_e_tmp) {
        omgmat[0] = 0.0;
        tmp = rtb_J[6 * k + 5];
        omgmat[3] = -tmp;
        rtb_T_e_0 = rtb_J[6 * k + 4];
        omgmat[6] = rtb_T_e_0;
        omgmat[1] = tmp;
        omgmat[4] = 0.0;
        tmp = rtb_J[6 * k + 3];
        omgmat[7] = -tmp;
        omgmat[2] = -rtb_T_e_0;
        omgmat[5] = tmp;
        omgmat[8] = 0.0;
        omgmat_0[18] = 0.0;
        tmp = rtb_J[6 * k + 2];
        omgmat_0[24] = -tmp;
        rtb_T_e_0 = rtb_J[6 * k + 1];
        omgmat_0[30] = rtb_T_e_0;
        omgmat_0[19] = tmp;
        omgmat_0[25] = 0.0;
        tmp = rtb_J[6 * k];
        omgmat_0[31] = -tmp;
        omgmat_0[20] = -rtb_T_e_0;
        omgmat_0[26] = tmp;
        omgmat_0[32] = 0.0;
        for (i_0 = 0; i_0 < 3; i_0++) {
          tmp = omgmat[3 * i_0];
          omgmat_0[6 * i_0] = tmp;
          omgmat_0[6 * i_0 + 3] = 0.0;
          i = (i_0 + 3) * 6;
          omgmat_0[i + 3] = tmp;
          tmp = omgmat[3 * i_0 + 1];
          omgmat_0[6 * i_0 + 1] = tmp;
          omgmat_0[6 * i_0 + 4] = 0.0;
          omgmat_0[i + 4] = tmp;
          tmp = omgmat[3 * i_0 + 2];
          omgmat_0[6 * i_0 + 2] = tmp;
          omgmat_0[6 * i_0 + 5] = 0.0;
          omgmat_0[i + 5] = tmp;
        }

        for (i_0 = 0; i_0 < 6; i_0++) {
          tmp = 0.0;
          for (i = 0; i < 6; i++) {
            tmp += omgmat_0[6 * i + i_0] * rtb_J[6 * rtb_T_e_tmp + i];
          }

          x[i_0] = tmp;
        }
      }

      tmp = FK_U.q[rtb_T_e_tmp + 6];
      for (i_0 = 0; i_0 <= 4; i_0 += 2) {
        tmp_c = _mm_loadu_pd(&x[i_0]);
        tmp_b = _mm_loadu_pd(&dJidt[i_0]);
        _mm_storeu_pd(&dJidt[i_0], _mm_add_pd(_mm_mul_pd(tmp_c, _mm_set1_pd(tmp)),
          tmp_b));
      }
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      rtb_Jdot[i_0 + 6 * k] = dJidt[i_0];
    }
  }

  /* Outport: '<Root>/T_r' */
  memcpy(&FK_Y.T_r[0], &rtb_T_m[0], sizeof(real_T) << 4U);

  /* SignalConversion generated from: '<S6>/ SFunction ' incorporates:
   *  Inport: '<Root>/q'
   *  MATLAB Function: '<S1>/MATLAB Function7'
   */
  rtb_MatrixMultiply2[0] = rtb_Sum2;
  for (i = 0; i < 5; i++) {
    rtb_MatrixMultiply2[i + 1] = FK_U.q[i + 1];
  }

  /* End of SignalConversion generated from: '<S6>/ SFunction ' */

  /* MATLAB Function: '<S1>/MATLAB Function7' incorporates:
   *  Inport: '<Root>/lambda_l'
   *  MATLAB Function: '<S1>/MATLAB Function8'
   */
  memset(&rtb_J_h[0], 0, 36U * sizeof(real_T));
  FK_exp6(&FK_U.lambda_l[36], rtb_T_m);
  for (i_0 = 0; i_0 < 3; i_0++) {
    tmp = 0.0;
    rtb_T_e_0 = 0.0;
    for (i = 0; i < 3; i++) {
      rtb_T_e_tmp = i_0 << 2;
      tmp += (((real_T)tmp_0[3 * i + 1] * rtb_T_m[rtb_T_e_tmp + 1] + (real_T)
               tmp_0[3 * i] * rtb_T_m[rtb_T_e_tmp]) + (real_T)tmp_0[3 * i + 2] *
              rtb_T_m[rtb_T_e_tmp + 2]) * rtb_T_m[i + 12];
      rtb_T_e_0 += rtb_T_m[rtb_T_e_tmp + i] * (real_T)tmp_1[i];
    }

    rtb_J_h[i_0 + 30] = tmp;
    rtb_J_h[i_0 + 33] = rtb_T_e_0;
  }

  for (k = 0; k < 6; k++) {
    FK_exp6(&FK_U.lambda_l[(5 - k) * 6], tmp_2);
    rtb_Sum2 = rtb_MatrixMultiply2[5 - k];
    for (i_0 = 0; i_0 <= 4; i_0 += 2) {
      _mm_storeu_pd(&x[i_0], _mm_mul_pd(_mm_set_pd(c[i_0 + 1], c[i_0]),
        _mm_set1_pd(rtb_Sum2)));
    }

    FK_exp6(x, tmp_3);
    for (i_0 = 0; i_0 < 4; i_0++) {
      tmp = tmp_2[i_0 + 4];
      rtb_T_e_0 = tmp_2[i_0];
      tmp_6 = tmp_2[i_0 + 8];
      tmp_7 = tmp_2[i_0 + 12];
      for (i = 0; i <= 2; i += 2) {
        rtb_T_e_tmp = (i + 1) << 2;
        tmp_e = i << 2;
        _mm_storeu_pd(&tmp_d[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(tmp_3[rtb_T_e_tmp + 1], tmp_3[tmp_e + 1]), _mm_set1_pd(tmp)),
          _mm_mul_pd(_mm_set_pd(tmp_3[rtb_T_e_tmp], tmp_3[tmp_e]), _mm_set1_pd
                     (rtb_T_e_0))), _mm_mul_pd(_mm_set_pd(tmp_3[rtb_T_e_tmp + 2],
          tmp_3[tmp_e + 2]), _mm_set1_pd(tmp_6))), _mm_mul_pd(_mm_set_pd
          (tmp_3[rtb_T_e_tmp + 3], tmp_3[tmp_e + 3]), _mm_set1_pd(tmp_7))));
        tmp_4[i_0 + tmp_e] = tmp_d[0];
        tmp_4[i_0 + rtb_T_e_tmp] = tmp_d[1];
      }

      tmp = tmp_4[i_0 + 4];
      rtb_T_e_0 = tmp_4[i_0];
      tmp_6 = tmp_4[i_0 + 8];
      tmp_7 = tmp_4[i_0 + 12];
      for (i = 0; i <= 2; i += 2) {
        rtb_T_e_tmp = (i + 1) << 2;
        tmp_e = i << 2;
        _mm_storeu_pd(&tmp_d[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(rtb_T_m[rtb_T_e_tmp + 1], rtb_T_m[tmp_e + 1]), _mm_set1_pd
           (tmp)), _mm_mul_pd(_mm_set_pd(rtb_T_m[rtb_T_e_tmp], rtb_T_m[tmp_e]),
                              _mm_set1_pd(rtb_T_e_0))), _mm_mul_pd(_mm_set_pd
          (rtb_T_m[rtb_T_e_tmp + 2], rtb_T_m[tmp_e + 2]), _mm_set1_pd(tmp_6))),
          _mm_mul_pd(_mm_set_pd(rtb_T_m[rtb_T_e_tmp + 3], rtb_T_m[tmp_e + 3]),
                     _mm_set1_pd(tmp_7))));
        tmp_5[i_0 + tmp_e] = tmp_d[0];
        tmp_5[i_0 + rtb_T_e_tmp] = tmp_d[1];
      }
    }

    memcpy(&rtb_T_m[0], &tmp_5[0], sizeof(real_T) << 4U);
    if (6 - k > 1) {
      for (i_0 = 0; i_0 < 3; i_0++) {
        tmp = 0.0;
        rtb_T_e_0 = 0.0;
        for (i = 0; i < 3; i++) {
          rtb_T_e_tmp = i_0 << 2;
          tmp += (((real_T)tmp_0[3 * i + 1] * rtb_T_m[rtb_T_e_tmp + 1] + (real_T)
                   tmp_0[3 * i] * rtb_T_m[rtb_T_e_tmp]) + (real_T)tmp_0[3 * i +
                  2] * rtb_T_m[rtb_T_e_tmp + 2]) * rtb_T_m[i + 12];
          rtb_T_e_0 += rtb_T_m[rtb_T_e_tmp + i] * (real_T)tmp_1[i];
        }

        i = (4 - k) * 6 + i_0;
        rtb_J_h[i] = tmp;
        rtb_J_h[i + 3] = rtb_T_e_0;
      }
    }
  }

  for (k = 0; k < 6; k++) {
    for (i = 0; i < 6; i++) {
      dJidt[i] = 0.0;
    }

    for (rtb_T_e_tmp = 0; rtb_T_e_tmp < 6; rtb_T_e_tmp++) {
      for (i = 0; i < 6; i++) {
        x[i] = 0.0;
      }

      if (k < rtb_T_e_tmp) {
        omgmat[0] = 0.0;
        rtb_Sum2 = rtb_J_h[6 * k + 5];
        omgmat[3] = -rtb_Sum2;
        tmp = rtb_J_h[6 * k + 4];
        omgmat[6] = tmp;
        omgmat[1] = rtb_Sum2;
        omgmat[4] = 0.0;
        rtb_Sum2 = rtb_J_h[6 * k + 3];
        omgmat[7] = -rtb_Sum2;
        omgmat[2] = -tmp;
        omgmat[5] = rtb_Sum2;
        omgmat[8] = 0.0;
        omgmat_0[18] = 0.0;
        rtb_Sum2 = rtb_J_h[6 * k + 2];
        omgmat_0[24] = -rtb_Sum2;
        tmp = rtb_J_h[6 * k + 1];
        omgmat_0[30] = tmp;
        omgmat_0[19] = rtb_Sum2;
        omgmat_0[25] = 0.0;
        rtb_Sum2 = rtb_J_h[6 * k];
        omgmat_0[31] = -rtb_Sum2;
        omgmat_0[20] = -tmp;
        omgmat_0[26] = rtb_Sum2;
        omgmat_0[32] = 0.0;
        for (i_0 = 0; i_0 < 3; i_0++) {
          tmp = omgmat[3 * i_0];
          omgmat_0[6 * i_0] = tmp;
          omgmat_0[6 * i_0 + 3] = 0.0;
          i = (i_0 + 3) * 6;
          omgmat_0[i + 3] = tmp;
          tmp = omgmat[3 * i_0 + 1];
          omgmat_0[6 * i_0 + 1] = tmp;
          omgmat_0[6 * i_0 + 4] = 0.0;
          omgmat_0[i + 4] = tmp;
          tmp = omgmat[3 * i_0 + 2];
          omgmat_0[6 * i_0 + 2] = tmp;
          omgmat_0[6 * i_0 + 5] = 0.0;
          omgmat_0[i + 5] = tmp;
        }

        for (i_0 = 0; i_0 < 6; i_0++) {
          tmp = 0.0;
          for (i = 0; i < 6; i++) {
            tmp += omgmat_0[6 * i + i_0] * rtb_J_h[6 * rtb_T_e_tmp + i];
          }

          x[i_0] = tmp;
        }
      }

      rtb_Sum2 = rtb_MatrixMultiply2[rtb_T_e_tmp];
      for (i_0 = 0; i_0 <= 4; i_0 += 2) {
        tmp_c = _mm_loadu_pd(&x[i_0]);
        tmp_b = _mm_loadu_pd(&dJidt[i_0]);
        _mm_storeu_pd(&dJidt[i_0], _mm_add_pd(_mm_mul_pd(tmp_c, _mm_set1_pd
          (rtb_Sum2)), tmp_b));
      }
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      rtb_Jdot_l[i_0 + 6 * k] = dJidt[i_0];
    }
  }

  /* Outport: '<Root>/T_l' */
  memcpy(&FK_Y.T_l[0], &rtb_T_m[0], sizeof(real_T) << 4U);

  /* Outport: '<Root>/Jb_l' */
  memcpy(&FK_Y.Jb_l[0], &rtb_J_h[0], 36U * sizeof(real_T));

  /* Outport: '<Root>/Jb_r' */
  memcpy(&FK_Y.Jb_r[0], &rtb_J[0], 36U * sizeof(real_T));

  /* Outport: '<Root>/Jbdot_l' */
  memcpy(&FK_Y.Jbdot_l[0], &rtb_Jdot_l[0], 36U * sizeof(real_T));

  /* Outport: '<Root>/Jbdot_r' */
  memcpy(&FK_Y.Jbdot_r[0], &rtb_Jdot[0], 36U * sizeof(real_T));

  /* Outport: '<Root>/Jb_lr' */
  memcpy(&FK_Y.Jb_lr[0], &rtb_J_m[0], 72U * sizeof(real_T));

  /* Outport: '<Root>/Jbdot_lr' */
  memcpy(&FK_Y.Jbdot_lr[0], &rtb_Jdot_f[0], 72U * sizeof(real_T));
  for (i_0 = 0; i_0 < 6; i_0++) {
    /* Outport: '<Root>/V_r' incorporates:
     *  Product: '<S1>/Matrix Multiply1'
     */
    rtb_Sum2 = 0.0;

    /* Outport: '<Root>/V_l' incorporates:
     *  Product: '<S1>/Matrix Multiply'
     */
    tmp = 0.0;
    for (i = 0; i < 6; i++) {
      /* Product: '<S1>/Matrix Multiply' incorporates:
       *  Inport: '<Root>/qdot'
       *  Outport: '<Root>/V_l'
       *  Outport: '<Root>/V_r'
       *  Product: '<S1>/Matrix Multiply1'
       */
      rtb_T_e_tmp = 6 * i + i_0;
      _mm_storeu_pd(&tmp_d[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
        (rtb_J_h[rtb_T_e_tmp], rtb_J[rtb_T_e_tmp]), _mm_set_pd(FK_U.qdot[i],
        FK_U.qdot[i + 6])), _mm_set_pd(tmp, rtb_Sum2)));

      /* Outport: '<Root>/V_r' incorporates:
       *  Product: '<S1>/Matrix Multiply1'
       */
      rtb_Sum2 = tmp_d[0];

      /* Outport: '<Root>/V_l' incorporates:
       *  Product: '<S1>/Matrix Multiply'
       */
      tmp = tmp_d[1];
    }

    /* Outport: '<Root>/V_l' incorporates:
     *  Product: '<S1>/Matrix Multiply'
     */
    FK_Y.V_l[i_0] = tmp;

    /* Outport: '<Root>/V_r' incorporates:
     *  Product: '<S1>/Matrix Multiply1'
     */
    FK_Y.V_r[i_0] = rtb_Sum2;

    /* Product: '<S1>/Matrix Multiply2' incorporates:
     *  Outport: '<Root>/V_lr'
     */
    rtb_Sum2 = 0.0;

    /* Outport: '<Root>/V_lr' incorporates:
     *  Product: '<S1>/Matrix Multiply2'
     */
    for (i = 0; i < 12; i++) {
      rtb_Sum2 += rtb_J_m[6 * i + i_0] * rtb_TmpSignalConversionAtSFunct[i];
    }

    FK_Y.V_lr[i_0] = rtb_Sum2;
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(FK_M->rtwLogInfo, (&FK_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.001s, 0.0s] */
    if ((rtmGetTFinal(FK_M)!=-1) &&
        !((rtmGetTFinal(FK_M)-FK_M->Timing.taskTime0) > FK_M->Timing.taskTime0 *
          (DBL_EPSILON))) {
      rtmSetErrorStatus(FK_M, "Simulation finished");
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
  if (!(++FK_M->Timing.clockTick0)) {
    ++FK_M->Timing.clockTickH0;
  }

  FK_M->Timing.taskTime0 = FK_M->Timing.clockTick0 * FK_M->Timing.stepSize0 +
    FK_M->Timing.clockTickH0 * FK_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void FK_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)FK_M, 0,
                sizeof(RT_MODEL_FK_T));
  rtmSetTFinal(FK_M, -1);
  FK_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    FK_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(FK_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(FK_M->rtwLogInfo, (NULL));
    rtliSetLogT(FK_M->rtwLogInfo, "tout");
    rtliSetLogX(FK_M->rtwLogInfo, "");
    rtliSetLogXFinal(FK_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(FK_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(FK_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(FK_M->rtwLogInfo, 0);
    rtliSetLogDecimation(FK_M->rtwLogInfo, 1);
    rtliSetLogY(FK_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(FK_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(FK_M->rtwLogInfo, (NULL));
  }

  /* external inputs */
  (void)memset(&FK_U, 0, sizeof(ExtU_FK_T));

  /* external outputs */
  (void)memset(&FK_Y, 0, sizeof(ExtY_FK_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(FK_M->rtwLogInfo, 0.0, rtmGetTFinal(FK_M),
    FK_M->Timing.stepSize0, (&rtmGetErrorStatus(FK_M)));
}

/* Model terminate function */
void FK_terminate(void)
{
  /* (no terminate code required) */
}
