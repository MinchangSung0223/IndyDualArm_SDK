/*
 * LR.c
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

#include "LR.h"
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include <string.h>
#include <math.h>
#include <emmintrin.h>
#include "LR_private.h"
#include "rt_defines.h"

/* External inputs (root inport signals with default storage) */
ExtU_LR_T LR_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_LR_T LR_Y;

/* Real-time model */
static RT_MODEL_LR_T LR_M_;
RT_MODEL_LR_T *const LR_M = &LR_M_;

/* Forward declaration for local functions */
static real_T LR_norm(const real_T x[16]);
static void LR_mpower(const real_T a[16], real_T b, real_T c[16]);
static real_T LR_log2(real_T x);
static void LR_padeApproximation(const real_T A[16], const real_T A2[16], const
  real_T A4[16], const real_T A6[16], int32_T m, real_T F[16]);
static void LR_recomputeBlockDiag(const real_T A[16], real_T F[16], const
  int32_T blockFormat[3]);
static real_T LR_xnrm2_o(int32_T n, const real_T x[16], int32_T ix0);
static void LR_xzsyhetrd(real_T A[16], real_T D[4], real_T E[3], real_T tau[3]);
static void LR_xzlascl_m(real_T cfrom, real_T cto, int32_T m, real_T A[4],
  int32_T iA0);
static void LR_xzlascl_mk(real_T cfrom, real_T cto, int32_T m, real_T A[3],
  int32_T iA0);
static void LR_xzlartg(real_T f, real_T g, real_T *cs, real_T *sn, real_T *r);
static void LR_rotateRight_h(int32_T n, real_T z[16], int32_T iz0, const real_T
  cs[6], int32_T ic0, int32_T is0);
static void LR_xdlaev2_a(real_T a, real_T b, real_T c, real_T *rt1, real_T *rt2,
  real_T *cs1, real_T *sn1);
static void LR_rotateRight(int32_T n, real_T z[16], int32_T iz0, const real_T
  cs[6], int32_T ic0, int32_T is0);
static int32_T LR_xzsteqr(real_T d[4], real_T e[3], real_T z[16]);
static void LR_xsyheev(real_T A[16], int32_T *info, real_T W[4]);
static real_T LR_norm_e(const real_T x[3]);
static real_T LR_norm_jl(const real_T x[6]);
static void LR_ddexpInvSo3(const real_T xi[3], const real_T xidot[3], real_T
  ddexpInv[9]);
static real_T LR_xzlangeM(const real_T x[16]);
static void LR_xzlascl(real_T cfrom, real_T cto, real_T A[16]);
static real_T LR_xnrm2(int32_T n, const real_T x[16], int32_T ix0);
static real_T LR_xdotc(int32_T n, const real_T x[16], int32_T ix0, const real_T
  y[16], int32_T iy0);
static void LR_xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[16], int32_T iy0);
static real_T LR_xnrm2_f(int32_T n, const real_T x[4], int32_T ix0);
static void LR_xaxpy_o(int32_T n, real_T a, const real_T x[16], int32_T ix0,
  real_T y[4], int32_T iy0);
static void LR_xaxpy_o2(int32_T n, real_T a, const real_T x[4], int32_T ix0,
  real_T y[16], int32_T iy0);
static void LR_xzlascl_f(real_T cfrom, real_T cto, real_T A[4]);
static void LR_xrotg(real_T *a, real_T *b, real_T *c, real_T *s);
static void LR_svd(const real_T A[16], real_T U[4]);
static void LR_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T tau, real_T C[16],
                      int32_T ic0, real_T work[4]);
static real_T LR_xnrm2_fq(int32_T n, const real_T x[3]);
static void LR_xdlanv2(real_T *a, real_T *b, real_T *c, real_T *d, real_T *rt1r,
  real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T *sn);
static void LR_xrot(int32_T n, real_T x[16], int32_T ix0, int32_T iy0, real_T c,
                    real_T s);
static int32_T LR_xhseqr(real_T h[16], real_T z[16]);
static void LR_schur_o(real_T A[16], real_T V[16]);
static real_T LR_xnrm2_fqh(int32_T n, const real_T x[4], int32_T ix0);
static void LR_xzgehrd(real_T a[4], int32_T ilo, int32_T ihi);
static void LR_xdlahqr(int32_T ilo, int32_T ihi, real_T h[4], real_T *z, int32_T
  *info, real_T wr[2], real_T wi[2]);
static void LR_xzlascl_f2p(real_T cfrom, real_T cto, int32_T m, real_T A[2],
  int32_T iA0);
static void LR_eigStandard(const real_T A[4], creal_T V[2]);
static void LR_xzlascl_f2pa(real_T cfrom, real_T cto, int32_T m, real_T *A);
static void LR_xdlaev2(real_T a, real_T b, real_T c, real_T *rt1, real_T *rt2);
static void LR_insertionsort(real_T x[2], int32_T xstart, int32_T xend);
static int32_T LR_xdsterf(real_T d[2], real_T e);
static void LR_ordeig(const real_T A[16], creal_T E[4]);
static void LR_sylvesterTriKernel_b(int32_T ia0, int32_T ja0, int32_T ib0,
  int32_T jb0, creal_T C[16], int32_T ic0, int32_T jc0, int32_T m, int32_T n);
static void LR_xgemv(int32_T m, int32_T n, int32_T ia0, int32_T ix0, creal_T y
                     [16], int32_T iy0);
static void LR_sylvesterRecursive_g(int32_T ia0, int32_T ja0, int32_T ib0,
  int32_T jb0, creal_T C[16], int32_T ic0, int32_T jc0, int32_T m, int32_T n);
static void LR_sqrt(creal_T *x);
static void LR_sqrtmTriRecursive_n(creal_T T[16], int32_T i, int32_T j, int32_T
  m);
static real_T LR_norm_j(const creal_T x[16]);
static void LR_atanh_h(creal_T *x);
static creal_T LR_power(const creal_T a, real_T b);
static void recomputeDiagBlocksSqrtTriangul(creal_T Troot[16], const creal_T T
  [16], int32_T s);
static void LR_computeLogOfSchurForm_m(const creal_T T[16], const creal_T d[4],
  creal_T L[16], int32_T *exitflag);
static void LR_xgetrf(real_T A[16], int32_T ipiv[4], int32_T *info);
static void LR_linsolve(const real_T A[4], const real_T B[2], real_T C[2]);
static void LR_sylvesterTriKernel(int32_T ia0, int32_T ja0, int32_T ib0, int32_T
  jb0, real_T C[16], int32_T ic0, int32_T jc0, int32_T m, int32_T n);
static void LR_sylvesterRecursive(int32_T ia0, int32_T ja0, int32_T ib0, int32_T
  jb0, real_T C[16], int32_T ic0, int32_T jc0, int32_T m, int32_T n);
static void LR_sqrtmTriRecursive(real_T T[16], int32_T i, int32_T j, int32_T m);
static void LR_logmParams(real_T T[16], creal_T d[4], int32_T *s, int32_T *m,
  int32_T *exitflag);
static void LR_sqrtm2by2(real_T T[4]);
static void LR_atanh(real_T *x);
static real_T LR_sqrtObo(real_T a, int32_T s);
static void LR_computeLogOfSchurForm(const real_T T[16], const creal_T d[4],
  real_T L[16], int32_T *exitflag);

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T LR_norm(const real_T x[16])
{
  real_T y;
  int32_T j;
  boolean_T exitg1;
  y = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 4)) {
    real_T s;
    int32_T s_tmp;
    s_tmp = j << 2;
    s = ((fabs(x[s_tmp + 1]) + fabs(x[s_tmp])) + fabs(x[s_tmp + 2])) + fabs
      (x[s_tmp + 3]);
    if (rtIsNaN(s)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (s > y) {
        y = s;
      }

      j++;
    }
  }

  return y;
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

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void LR_mpower(const real_T a[16], real_T b, real_T c[16])
{
  real_T aBuffer[16];
  real_T b_a[16];
  real_T cBuffer[16];
  real_T cBuffer_0[16];
  real_T cBuffer_1[16];
  real_T tmp_3[2];
  real_T e;
  int32_T b_n;
  int32_T i;
  int32_T n;
  int32_T nbitson;
  e = fabs(b);
  if (e <= 2.147483647E+9) {
    int32_T nb;
    memcpy(&b_a[0], &a[0], sizeof(real_T) << 4U);
    n = (int32_T)e;
    b_n = (int32_T)e;
    nbitson = 0;
    nb = -2;
    while (b_n > 0) {
      nb++;
      if (((uint32_T)b_n & 1U) != 0U) {
        nbitson++;
      }

      b_n >>= 1;
    }

    if ((int32_T)e <= 2) {
      if (b == 2.0) {
        for (nbitson = 0; nbitson < 4; nbitson++) {
          for (i = 0; i <= 2; i += 2) {
            int32_T tmp_4;
            tmp_4 = nbitson << 2;
            _mm_storeu_pd(&c[i + tmp_4], _mm_add_pd(_mm_add_pd(_mm_add_pd
              (_mm_mul_pd(_mm_set1_pd(a[tmp_4 + 1]), _mm_loadu_pd(&a[i + 4])),
               _mm_mul_pd(_mm_set1_pd(a[tmp_4]), _mm_loadu_pd(&a[i]))),
              _mm_mul_pd(_mm_set1_pd(a[tmp_4 + 2]), _mm_loadu_pd(&a[i + 8]))),
              _mm_mul_pd(_mm_set1_pd(a[tmp_4 + 3]), _mm_loadu_pd(&a[i + 12]))));
          }
        }
      } else {
        boolean_T firstmult;
        firstmult = false;
        for (n = 0; n < 16; n++) {
          if (firstmult || rtIsNaN(a[n])) {
            firstmult = true;
          }
        }

        if (firstmult) {
          for (nbitson = 0; nbitson < 16; nbitson++) {
            c[nbitson] = (rtNaN);
          }
        } else {
          memset(&c[0], 0, sizeof(real_T) << 4U);
          c[0] = 1.0;
          c[5] = 1.0;
          c[10] = 1.0;
          c[15] = 1.0;
        }
      }
    } else {
      real_T c_0;
      real_T c_1;
      real_T c_2;
      real_T ed2;
      int32_T tmp_4;
      boolean_T aBufferInUse;
      boolean_T cBufferInUse;
      boolean_T firstmult;
      firstmult = true;
      aBufferInUse = false;
      cBufferInUse = (((uint32_T)nbitson & 1U) == 0U);
      for (b_n = 0; b_n <= nb; b_n++) {
        int32_T tmp_5;
        if (((uint32_T)n & 1U) != 0U) {
          if (firstmult) {
            firstmult = false;
            if (cBufferInUse) {
              if (aBufferInUse) {
                memcpy(&cBuffer[0], &aBuffer[0], sizeof(real_T) << 4U);
              } else {
                memcpy(&cBuffer[0], &b_a[0], sizeof(real_T) << 4U);
              }
            } else if (aBufferInUse) {
              memcpy(&c[0], &aBuffer[0], sizeof(real_T) << 4U);
            } else {
              memcpy(&c[0], &b_a[0], sizeof(real_T) << 4U);
            }
          } else {
            if (aBufferInUse) {
              if (cBufferInUse) {
                for (nbitson = 0; nbitson < 4; nbitson++) {
                  ed2 = cBuffer[nbitson + 4];
                  e = cBuffer[nbitson];
                  c_0 = cBuffer[nbitson + 8];
                  c_1 = cBuffer[nbitson + 12];
                  for (i = 0; i <= 2; i += 2) {
                    tmp_4 = (i + 1) << 2;
                    tmp_5 = i << 2;
                    _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                      (_mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 1], aBuffer[tmp_5 +
                      1]), _mm_set1_pd(ed2)), _mm_mul_pd(_mm_set_pd
                      (aBuffer[tmp_4], aBuffer[tmp_5]), _mm_set1_pd(e))),
                      _mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 2], aBuffer[tmp_5 +
                      2]), _mm_set1_pd(c_0))), _mm_mul_pd(_mm_set_pd
                      (aBuffer[tmp_4 + 3], aBuffer[tmp_5 + 3]), _mm_set1_pd(c_1))));
                    c[nbitson + tmp_5] = tmp_3[0];
                    c[nbitson + tmp_4] = tmp_3[1];
                  }
                }
              } else {
                for (nbitson = 0; nbitson < 4; nbitson++) {
                  e = c[nbitson + 4];
                  c_0 = c[nbitson];
                  c_1 = c[nbitson + 8];
                  c_2 = c[nbitson + 12];
                  for (i = 0; i <= 2; i += 2) {
                    tmp_4 = (i + 1) << 2;
                    tmp_5 = i << 2;
                    _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                      (_mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 1], aBuffer[tmp_5 +
                      1]), _mm_set1_pd(e)), _mm_mul_pd(_mm_set_pd(aBuffer[tmp_4],
                      aBuffer[tmp_5]), _mm_set1_pd(c_0))), _mm_mul_pd(_mm_set_pd
                      (aBuffer[tmp_4 + 2], aBuffer[tmp_5 + 2]), _mm_set1_pd(c_1))),
                      _mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 3], aBuffer[tmp_5 +
                      3]), _mm_set1_pd(c_2))));
                    cBuffer[nbitson + tmp_5] = tmp_3[0];
                    cBuffer[nbitson + tmp_4] = tmp_3[1];
                  }
                }
              }
            } else if (cBufferInUse) {
              for (nbitson = 0; nbitson < 4; nbitson++) {
                ed2 = cBuffer[nbitson + 4];
                e = cBuffer[nbitson];
                c_0 = cBuffer[nbitson + 8];
                c_1 = cBuffer[nbitson + 12];
                for (i = 0; i <= 2; i += 2) {
                  tmp_4 = (i + 1) << 2;
                  tmp_5 = i << 2;
                  _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                    (_mm_mul_pd(_mm_set_pd(b_a[tmp_4 + 1], b_a[tmp_5 + 1]),
                                _mm_set1_pd(ed2)), _mm_mul_pd(_mm_set_pd
                    (b_a[tmp_4], b_a[tmp_5]), _mm_set1_pd(e))), _mm_mul_pd
                    (_mm_set_pd(b_a[tmp_4 + 2], b_a[tmp_5 + 2]), _mm_set1_pd(c_0))),
                    _mm_mul_pd(_mm_set_pd(b_a[tmp_4 + 3], b_a[tmp_5 + 3]),
                               _mm_set1_pd(c_1))));
                  c[nbitson + tmp_5] = tmp_3[0];
                  c[nbitson + tmp_4] = tmp_3[1];
                }
              }
            } else {
              for (nbitson = 0; nbitson < 4; nbitson++) {
                e = c[nbitson + 4];
                c_0 = c[nbitson];
                c_1 = c[nbitson + 8];
                c_2 = c[nbitson + 12];
                for (i = 0; i <= 2; i += 2) {
                  tmp_4 = (i + 1) << 2;
                  tmp_5 = i << 2;
                  _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                    (_mm_mul_pd(_mm_set_pd(b_a[tmp_4 + 1], b_a[tmp_5 + 1]),
                                _mm_set1_pd(e)), _mm_mul_pd(_mm_set_pd(b_a[tmp_4],
                    b_a[tmp_5]), _mm_set1_pd(c_0))), _mm_mul_pd(_mm_set_pd
                    (b_a[tmp_4 + 2], b_a[tmp_5 + 2]), _mm_set1_pd(c_1))),
                    _mm_mul_pd(_mm_set_pd(b_a[tmp_4 + 3], b_a[tmp_5 + 3]),
                               _mm_set1_pd(c_2))));
                  cBuffer[nbitson + tmp_5] = tmp_3[0];
                  cBuffer[nbitson + tmp_4] = tmp_3[1];
                }
              }
            }

            cBufferInUse = !cBufferInUse;
          }
        }

        n >>= 1;
        if (aBufferInUse) {
          for (nbitson = 0; nbitson < 4; nbitson++) {
            for (i = 0; i <= 2; i += 2) {
              tmp_4 = (i + 1) << 2;
              tmp_5 = i << 2;
              _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                (_mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 1], aBuffer[tmp_5 + 1]),
                            _mm_set1_pd(aBuffer[nbitson + 4])), _mm_mul_pd
                 (_mm_set_pd(aBuffer[tmp_4], aBuffer[tmp_5]), _mm_set1_pd
                  (aBuffer[nbitson]))), _mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 2],
                aBuffer[tmp_5 + 2]), _mm_set1_pd(aBuffer[nbitson + 8]))),
                _mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 3], aBuffer[tmp_5 + 3]),
                           _mm_set1_pd(aBuffer[nbitson + 12]))));
              b_a[nbitson + tmp_5] = tmp_3[0];
              b_a[nbitson + tmp_4] = tmp_3[1];
            }
          }
        } else {
          for (nbitson = 0; nbitson < 4; nbitson++) {
            for (i = 0; i <= 2; i += 2) {
              tmp_4 = (i + 1) << 2;
              tmp_5 = i << 2;
              _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                (_mm_mul_pd(_mm_set_pd(b_a[tmp_4 + 1], b_a[tmp_5 + 1]),
                            _mm_set1_pd(b_a[nbitson + 4])), _mm_mul_pd
                 (_mm_set_pd(b_a[tmp_4], b_a[tmp_5]), _mm_set1_pd(b_a[nbitson]))),
                _mm_mul_pd(_mm_set_pd(b_a[tmp_4 + 2], b_a[tmp_5 + 2]),
                           _mm_set1_pd(b_a[nbitson + 8]))), _mm_mul_pd
                (_mm_set_pd(b_a[tmp_4 + 3], b_a[tmp_5 + 3]), _mm_set1_pd
                 (b_a[nbitson + 12]))));
              aBuffer[nbitson + tmp_5] = tmp_3[0];
              aBuffer[nbitson + tmp_4] = tmp_3[1];
            }
          }
        }

        aBufferInUse = !aBufferInUse;
      }

      for (nbitson = 0; nbitson < 4; nbitson++) {
        real_T b_a_0;
        real_T b_a_1;
        real_T b_a_2;
        n = nbitson << 2;
        ed2 = aBuffer[n];
        e = aBuffer[n + 1];
        c_0 = aBuffer[n + 2];
        c_1 = aBuffer[n + 3];
        c_2 = b_a[n];
        b_a_0 = b_a[n + 1];
        b_a_1 = b_a[n + 2];
        b_a_2 = b_a[n + 3];
        for (i = 0; i <= 2; i += 2) {
          __m128d tmp;
          __m128d tmp_0;
          __m128d tmp_1;
          __m128d tmp_2;
          tmp = _mm_loadu_pd(&cBuffer[i]);
          tmp_0 = _mm_loadu_pd(&cBuffer[i + 4]);
          tmp_1 = _mm_loadu_pd(&cBuffer[i + 8]);
          tmp_2 = _mm_loadu_pd(&cBuffer[i + 12]);
          tmp_4 = i + n;
          _mm_storeu_pd(&cBuffer_1[tmp_4], _mm_add_pd(_mm_mul_pd(_mm_set1_pd
            (b_a_2), tmp_2), _mm_add_pd(_mm_mul_pd(_mm_set1_pd(b_a_1), tmp_1),
            _mm_add_pd(_mm_mul_pd(_mm_set1_pd(b_a_0), tmp_0), _mm_mul_pd
                       (_mm_set1_pd(c_2), tmp)))));
          _mm_storeu_pd(&cBuffer_0[tmp_4], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(c_1),
            tmp_2), _mm_add_pd(_mm_mul_pd(_mm_set1_pd(c_0), tmp_1), _mm_add_pd
                               (_mm_mul_pd(_mm_set1_pd(e), tmp_0), _mm_mul_pd
                                (_mm_set1_pd(ed2), tmp)))));
        }
      }

      for (nbitson = 0; nbitson < 16; nbitson++) {
        if (firstmult) {
          if (aBufferInUse) {
            c[nbitson] = aBuffer[nbitson];
          } else {
            c[nbitson] = b_a[nbitson];
          }
        } else if (aBufferInUse) {
          c[nbitson] = cBuffer_0[nbitson];
        } else {
          c[nbitson] = cBuffer_1[nbitson];
        }
      }
    }
  } else {
    memcpy(&b_a[0], &a[0], sizeof(real_T) << 4U);
    if ((!rtIsInf(b)) && (!rtIsNaN(b))) {
      boolean_T firstmult;
      firstmult = true;
      real_T ed2;
      int32_T exitg1;
      do {
        int32_T tmp_4;
        int32_T tmp_5;
        exitg1 = 0;
        ed2 = floor(e / 2.0);
        if (2.0 * ed2 != e) {
          if (firstmult) {
            memcpy(&c[0], &b_a[0], sizeof(real_T) << 4U);
            firstmult = false;
          } else {
            for (nbitson = 0; nbitson < 4; nbitson++) {
              real_T c_0;
              real_T c_1;
              real_T c_2;
              e = c[nbitson + 4];
              c_0 = c[nbitson];
              c_1 = c[nbitson + 8];
              c_2 = c[nbitson + 12];
              for (i = 0; i <= 2; i += 2) {
                tmp_4 = (i + 1) << 2;
                tmp_5 = i << 2;
                _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                  (_mm_mul_pd(_mm_set_pd(b_a[tmp_4 + 1], b_a[tmp_5 + 1]),
                              _mm_set1_pd(e)), _mm_mul_pd(_mm_set_pd(b_a[tmp_4],
                  b_a[tmp_5]), _mm_set1_pd(c_0))), _mm_mul_pd(_mm_set_pd
                  (b_a[tmp_4 + 2], b_a[tmp_5 + 2]), _mm_set1_pd(c_1))),
                  _mm_mul_pd(_mm_set_pd(b_a[tmp_4 + 3], b_a[tmp_5 + 3]),
                             _mm_set1_pd(c_2))));
                cBuffer[nbitson + tmp_5] = tmp_3[0];
                cBuffer[nbitson + tmp_4] = tmp_3[1];
              }
            }

            memcpy(&c[0], &cBuffer[0], sizeof(real_T) << 4U);
          }
        }

        if (ed2 == 0.0) {
          exitg1 = 1;
        } else {
          e = ed2;
          for (nbitson = 0; nbitson < 4; nbitson++) {
            for (i = 0; i <= 2; i += 2) {
              tmp_4 = (i + 1) << 2;
              tmp_5 = i << 2;
              _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                (_mm_mul_pd(_mm_set_pd(b_a[tmp_4 + 1], b_a[tmp_5 + 1]),
                            _mm_set1_pd(b_a[nbitson + 4])), _mm_mul_pd
                 (_mm_set_pd(b_a[tmp_4], b_a[tmp_5]), _mm_set1_pd(b_a[nbitson]))),
                _mm_mul_pd(_mm_set_pd(b_a[tmp_4 + 2], b_a[tmp_5 + 2]),
                           _mm_set1_pd(b_a[nbitson + 8]))), _mm_mul_pd
                (_mm_set_pd(b_a[tmp_4 + 3], b_a[tmp_5 + 3]), _mm_set1_pd
                 (b_a[nbitson + 12]))));
              cBuffer[nbitson + tmp_5] = tmp_3[0];
              cBuffer[nbitson + tmp_4] = tmp_3[1];
            }
          }

          memcpy(&b_a[0], &cBuffer[0], sizeof(real_T) << 4U);
        }
      } while (exitg1 == 0);
    } else {
      for (nbitson = 0; nbitson < 16; nbitson++) {
        c[nbitson] = (rtNaN);
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static real_T LR_log2(real_T x)
{
  real_T f;
  int32_T inte;
  if (x == 0.0) {
    f = (rtMinusInf);
  } else if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
    real_T t;
    t = frexp(x, &inte);
    if (t == 0.5) {
      f = (real_T)inte - 1.0;
    } else if ((inte == 1) && (t < 0.75)) {
      f = log(2.0 * t) / 0.69314718055994529;
    } else {
      f = log(t) / 0.69314718055994529 + (real_T)inte;
    }
  } else {
    f = x;
  }

  return f;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void LR_padeApproximation(const real_T A[16], const real_T A2[16], const
  real_T A4[16], const real_T A6[16], int32_T m, real_T F[16])
{
  __m128d tmp;
  __m128d tmp_0;
  real_T A6_0[16];
  real_T V[16];
  real_T tmp_2[2];
  real_T d;
  real_T s;
  int32_T F_tmp;
  int32_T b_ix;
  int32_T e_k;
  int32_T ijA;
  int32_T ix;
  int32_T iy;
  int32_T jj;
  int32_T k_k;
  int8_T ipiv[4];
  if (m == 3) {
    memcpy(&F[0], &A2[0], sizeof(real_T) << 4U);
    tmp = _mm_set1_pd(60.0);
    _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[5], F[0]), tmp));
    F[0] = tmp_2[0];
    F[5] = tmp_2[1];
    _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[15], F[10]), tmp));
    F[10] = tmp_2[0];
    F[15] = tmp_2[1];
    for (k_k = 0; k_k < 4; k_k++) {
      real_T A6_1;
      real_T A6_2;
      F_tmp = k_k << 2;
      d = F[F_tmp + 1];
      s = F[F_tmp];
      A6_1 = F[F_tmp + 2];
      A6_2 = F[F_tmp + 3];
      for (e_k = 0; e_k <= 2; e_k += 2) {
        _mm_storeu_pd(&A6_0[e_k + F_tmp], _mm_add_pd(_mm_add_pd(_mm_add_pd
          (_mm_mul_pd(_mm_set1_pd(d), _mm_loadu_pd(&A[e_k + 4])), _mm_mul_pd
           (_mm_set1_pd(s), _mm_loadu_pd(&A[e_k]))), _mm_mul_pd(_mm_set1_pd(A6_1),
          _mm_loadu_pd(&A[e_k + 8]))), _mm_mul_pd(_mm_set1_pd(A6_2),
          _mm_loadu_pd(&A[e_k + 12]))));
      }
    }

    for (k_k = 0; k_k <= 14; k_k += 2) {
      tmp = _mm_loadu_pd(&A6_0[k_k]);
      _mm_storeu_pd(&F[k_k], tmp);
      _mm_storeu_pd(&V[k_k], _mm_mul_pd(_mm_set1_pd(12.0), _mm_loadu_pd(&A2[k_k])));
    }

    d = 120.0;
  } else if (m == 5) {
    for (k_k = 0; k_k <= 14; k_k += 2) {
      _mm_storeu_pd(&F[k_k], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(420.0),
        _mm_loadu_pd(&A2[k_k])), _mm_loadu_pd(&A4[k_k])));
    }

    tmp = _mm_set1_pd(15120.0);
    _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[5], F[0]), tmp));
    F[0] = tmp_2[0];
    F[5] = tmp_2[1];
    _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[15], F[10]), tmp));
    F[10] = tmp_2[0];
    F[15] = tmp_2[1];
    for (k_k = 0; k_k < 4; k_k++) {
      real_T A6_1;
      real_T A6_2;
      F_tmp = k_k << 2;
      d = F[F_tmp + 1];
      s = F[F_tmp];
      A6_1 = F[F_tmp + 2];
      A6_2 = F[F_tmp + 3];
      for (e_k = 0; e_k <= 2; e_k += 2) {
        _mm_storeu_pd(&A6_0[e_k + F_tmp], _mm_add_pd(_mm_add_pd(_mm_add_pd
          (_mm_mul_pd(_mm_set1_pd(d), _mm_loadu_pd(&A[e_k + 4])), _mm_mul_pd
           (_mm_set1_pd(s), _mm_loadu_pd(&A[e_k]))), _mm_mul_pd(_mm_set1_pd(A6_1),
          _mm_loadu_pd(&A[e_k + 8]))), _mm_mul_pd(_mm_set1_pd(A6_2),
          _mm_loadu_pd(&A[e_k + 12]))));
      }
    }

    for (k_k = 0; k_k <= 14; k_k += 2) {
      tmp = _mm_loadu_pd(&A6_0[k_k]);
      _mm_storeu_pd(&F[k_k], tmp);
      _mm_storeu_pd(&V[k_k], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(30.0),
        _mm_loadu_pd(&A4[k_k])), _mm_mul_pd(_mm_set1_pd(3360.0), _mm_loadu_pd
        (&A2[k_k]))));
    }

    d = 30240.0;
  } else if (m == 7) {
    for (k_k = 0; k_k <= 14; k_k += 2) {
      _mm_storeu_pd(&F[k_k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(1512.0),
        _mm_loadu_pd(&A4[k_k])), _mm_loadu_pd(&A6[k_k])), _mm_mul_pd(_mm_set1_pd
        (277200.0), _mm_loadu_pd(&A2[k_k]))));
    }

    tmp = _mm_set1_pd(8.64864E+6);
    _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[5], F[0]), tmp));
    F[0] = tmp_2[0];
    F[5] = tmp_2[1];
    _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[15], F[10]), tmp));
    F[10] = tmp_2[0];
    F[15] = tmp_2[1];
    for (k_k = 0; k_k < 4; k_k++) {
      real_T A6_1;
      real_T A6_2;
      F_tmp = k_k << 2;
      d = F[F_tmp + 1];
      s = F[F_tmp];
      A6_1 = F[F_tmp + 2];
      A6_2 = F[F_tmp + 3];
      for (e_k = 0; e_k <= 2; e_k += 2) {
        _mm_storeu_pd(&A6_0[e_k + F_tmp], _mm_add_pd(_mm_add_pd(_mm_add_pd
          (_mm_mul_pd(_mm_set1_pd(d), _mm_loadu_pd(&A[e_k + 4])), _mm_mul_pd
           (_mm_set1_pd(s), _mm_loadu_pd(&A[e_k]))), _mm_mul_pd(_mm_set1_pd(A6_1),
          _mm_loadu_pd(&A[e_k + 8]))), _mm_mul_pd(_mm_set1_pd(A6_2),
          _mm_loadu_pd(&A[e_k + 12]))));
      }
    }

    for (k_k = 0; k_k <= 14; k_k += 2) {
      tmp = _mm_loadu_pd(&A6_0[k_k]);
      _mm_storeu_pd(&F[k_k], tmp);
      _mm_storeu_pd(&V[k_k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(56.0),
        _mm_loadu_pd(&A6[k_k])), _mm_mul_pd(_mm_set1_pd(25200.0), _mm_loadu_pd
        (&A4[k_k]))), _mm_mul_pd(_mm_set1_pd(1.99584E+6), _mm_loadu_pd(&A2[k_k]))));
    }

    d = 1.729728E+7;
  } else if (m == 9) {
    real_T A6_1;
    real_T A6_2;
    for (k_k = 0; k_k < 4; k_k++) {
      jj = k_k << 2;
      d = A2[jj + 1];
      s = A2[jj];
      A6_1 = A2[jj + 2];
      A6_2 = A2[jj + 3];
      for (e_k = 0; e_k <= 2; e_k += 2) {
        _mm_storeu_pd(&V[e_k + jj], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(d), _mm_loadu_pd(&A6[e_k + 4])), _mm_mul_pd(_mm_set1_pd(s),
          _mm_loadu_pd(&A6[e_k]))), _mm_mul_pd(_mm_set1_pd(A6_1), _mm_loadu_pd
          (&A6[e_k + 8]))), _mm_mul_pd(_mm_set1_pd(A6_2), _mm_loadu_pd(&A6[e_k +
          12]))));
      }
    }

    for (k_k = 0; k_k <= 14; k_k += 2) {
      tmp = _mm_loadu_pd(&V[k_k]);
      _mm_storeu_pd(&F[k_k], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(3960.0), _mm_loadu_pd(&A6[k_k])), tmp), _mm_mul_pd
        (_mm_set1_pd(2.16216E+6), _mm_loadu_pd(&A4[k_k]))), _mm_mul_pd
        (_mm_set1_pd(3.027024E+8), _mm_loadu_pd(&A2[k_k]))));
    }

    tmp = _mm_set1_pd(8.8216128E+9);
    _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[5], F[0]), tmp));
    F[0] = tmp_2[0];
    F[5] = tmp_2[1];
    _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[15], F[10]), tmp));
    F[10] = tmp_2[0];
    F[15] = tmp_2[1];
    for (k_k = 0; k_k < 4; k_k++) {
      F_tmp = k_k << 2;
      d = F[F_tmp + 1];
      s = F[F_tmp];
      A6_1 = F[F_tmp + 2];
      A6_2 = F[F_tmp + 3];
      for (e_k = 0; e_k <= 2; e_k += 2) {
        _mm_storeu_pd(&A6_0[e_k + F_tmp], _mm_add_pd(_mm_add_pd(_mm_add_pd
          (_mm_mul_pd(_mm_set1_pd(d), _mm_loadu_pd(&A[e_k + 4])), _mm_mul_pd
           (_mm_set1_pd(s), _mm_loadu_pd(&A[e_k]))), _mm_mul_pd(_mm_set1_pd(A6_1),
          _mm_loadu_pd(&A[e_k + 8]))), _mm_mul_pd(_mm_set1_pd(A6_2),
          _mm_loadu_pd(&A[e_k + 12]))));
      }
    }

    for (k_k = 0; k_k <= 14; k_k += 2) {
      tmp = _mm_loadu_pd(&A6_0[k_k]);
      _mm_storeu_pd(&F[k_k], tmp);
      tmp = _mm_loadu_pd(&V[k_k]);
      _mm_storeu_pd(&V[k_k], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(90.0), tmp), _mm_mul_pd(_mm_set1_pd(110880.0), _mm_loadu_pd
        (&A6[k_k]))), _mm_mul_pd(_mm_set1_pd(3.027024E+7), _mm_loadu_pd(&A4[k_k]))),
        _mm_mul_pd(_mm_set1_pd(2.0756736E+9), _mm_loadu_pd(&A2[k_k]))));
    }

    d = 1.76432256E+10;
  } else {
    real_T A6_1;
    real_T A6_2;
    for (k_k = 0; k_k <= 14; k_k += 2) {
      __m128d tmp_1;
      tmp = _mm_loadu_pd(&A6[k_k]);
      tmp_0 = _mm_loadu_pd(&A4[k_k]);
      tmp_1 = _mm_loadu_pd(&A2[k_k]);
      _mm_storeu_pd(&F[k_k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (3.352212864E+10), tmp), _mm_mul_pd(_mm_set1_pd(1.05594705216E+13),
        tmp_0)), _mm_mul_pd(_mm_set1_pd(1.1873537964288E+15), tmp_1)));
      _mm_storeu_pd(&V[k_k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (16380.0), tmp_0), tmp), _mm_mul_pd(_mm_set1_pd(4.08408E+7), tmp_1)));
    }

    tmp = _mm_set1_pd(3.238237626624E+16);
    _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[5], F[0]), tmp));
    F[0] = tmp_2[0];
    F[5] = tmp_2[1];
    _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[15], F[10]), tmp));
    F[10] = tmp_2[0];
    F[15] = tmp_2[1];
    for (k_k = 0; k_k < 4; k_k++) {
      d = A6[k_k + 4];
      s = A6[k_k];
      A6_1 = A6[k_k + 8];
      A6_2 = A6[k_k + 12];
      for (e_k = 0; e_k <= 2; e_k += 2) {
        jj = (e_k + 1) << 2;
        iy = e_k << 2;
        b_ix = iy + k_k;
        ix = jj + k_k;
        _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd
          (_mm_mul_pd(_mm_set_pd(V[jj + 1], V[iy + 1]), _mm_set1_pd(d)),
           _mm_mul_pd(_mm_set_pd(V[jj], V[iy]), _mm_set1_pd(s))), _mm_mul_pd
          (_mm_set_pd(V[jj + 2], V[iy + 2]), _mm_set1_pd(A6_1))), _mm_mul_pd
          (_mm_set_pd(V[jj + 3], V[iy + 3]), _mm_set1_pd(A6_2))), _mm_set_pd
          (F[ix], F[b_ix])));
        A6_0[b_ix] = tmp_2[0];
        A6_0[ix] = tmp_2[1];
      }
    }

    for (k_k = 0; k_k < 4; k_k++) {
      jj = k_k << 2;
      d = A6_0[jj + 1];
      s = A6_0[jj];
      A6_1 = A6_0[jj + 2];
      A6_2 = A6_0[jj + 3];
      for (e_k = 0; e_k <= 2; e_k += 2) {
        _mm_storeu_pd(&F[e_k + jj], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(d), _mm_loadu_pd(&A[e_k + 4])), _mm_mul_pd(_mm_set1_pd(s),
          _mm_loadu_pd(&A[e_k]))), _mm_mul_pd(_mm_set1_pd(A6_1), _mm_loadu_pd
          (&A[e_k + 8]))), _mm_mul_pd(_mm_set1_pd(A6_2), _mm_loadu_pd(&A[e_k +
          12]))));
      }
    }

    for (k_k = 0; k_k <= 14; k_k += 2) {
      _mm_storeu_pd(&A6_0[k_k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (182.0), _mm_loadu_pd(&A6[k_k])), _mm_mul_pd(_mm_set1_pd(960960.0),
        _mm_loadu_pd(&A4[k_k]))), _mm_mul_pd(_mm_set1_pd(1.32324192E+9),
        _mm_loadu_pd(&A2[k_k]))));
    }

    for (k_k = 0; k_k < 4; k_k++) {
      for (e_k = 0; e_k <= 2; e_k += 2) {
        jj = (e_k + 1) << 2;
        iy = e_k << 2;
        b_ix = jj + k_k;
        ix = iy + k_k;
        _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd
          (_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd(A6_0[jj + 1], A6_0[iy + 1]),
          _mm_set1_pd(A6[k_k + 4])), _mm_mul_pd(_mm_set_pd(A6_0[jj], A6_0[iy]),
          _mm_set1_pd(A6[k_k]))), _mm_mul_pd(_mm_set_pd(A6_0[jj + 2], A6_0[iy +
          2]), _mm_set1_pd(A6[k_k + 8]))), _mm_mul_pd(_mm_set_pd(A6_0[jj + 3],
          A6_0[iy + 3]), _mm_set1_pd(A6[k_k + 12]))), _mm_mul_pd(_mm_set_pd
          (A6[b_ix], A6[ix]), _mm_set1_pd(6.704425728E+11))), _mm_mul_pd
          (_mm_set_pd(A4[b_ix], A4[ix]), _mm_set1_pd(1.29060195264E+14))),
          _mm_mul_pd(_mm_set_pd(A2[b_ix], A2[ix]), _mm_set1_pd
                     (7.7717703038976E+15))));
        V[ix] = tmp_2[0];
        V[b_ix] = tmp_2[1];
      }
    }

    d = 6.476475253248E+16;
  }

  tmp = _mm_set1_pd(d);
  _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(V[5], V[0]), tmp));
  V[0] = tmp_2[0];
  V[5] = tmp_2[1];
  _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(V[15], V[10]), tmp));
  V[10] = tmp_2[0];
  V[15] = tmp_2[1];
  for (e_k = 0; e_k <= 14; e_k += 2) {
    tmp = _mm_loadu_pd(&V[e_k]);
    tmp_0 = _mm_loadu_pd(&F[e_k]);
    _mm_storeu_pd(&V[e_k], _mm_sub_pd(tmp, tmp_0));
    _mm_storeu_pd(&F[e_k], _mm_mul_pd(_mm_set1_pd(2.0), tmp_0));
  }

  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (e_k = 0; e_k < 3; e_k++) {
    jj = e_k * 5;
    iy = 5 - e_k;
    b_ix = 0;
    ix = jj;
    d = fabs(V[jj]);
    for (k_k = 2; k_k < iy; k_k++) {
      ix++;
      s = fabs(V[ix]);
      if (s > d) {
        b_ix = k_k - 1;
        d = s;
      }
    }

    if (V[jj + b_ix] != 0.0) {
      if (b_ix != 0) {
        k_k = e_k + b_ix;
        ipiv[e_k] = (int8_T)(k_k + 1);
        d = V[e_k];
        V[e_k] = V[k_k];
        V[k_k] = d;
        d = V[e_k + 4];
        V[e_k + 4] = V[k_k + 4];
        V[k_k + 4] = d;
        d = V[e_k + 8];
        V[e_k + 8] = V[k_k + 8];
        V[k_k + 8] = d;
        d = V[e_k + 12];
        V[e_k + 12] = V[k_k + 12];
        V[k_k + 12] = d;
      }

      iy = (jj - e_k) + 4;
      k_k = (((((iy - jj) - 1) / 2) << 1) + jj) + 2;
      ix = k_k - 2;
      for (b_ix = jj + 2; b_ix <= ix; b_ix += 2) {
        tmp = _mm_loadu_pd(&V[b_ix - 1]);
        _mm_storeu_pd(&V[b_ix - 1], _mm_div_pd(tmp, _mm_set1_pd(V[jj])));
      }

      for (b_ix = k_k; b_ix <= iy; b_ix++) {
        V[b_ix - 1] /= V[jj];
      }
    }

    iy = jj + 4;
    b_ix = jj + 6;
    ix = 2 - e_k;
    for (k_k = 0; k_k <= ix; k_k++) {
      d = V[iy];
      if (V[iy] != 0.0) {
        F_tmp = (b_ix - e_k) + 2;
        for (ijA = b_ix; ijA <= F_tmp; ijA++) {
          V[ijA - 1] += V[((jj + ijA) - b_ix) + 1] * -d;
        }
      }

      iy += 4;
      b_ix += 4;
    }
  }

  for (e_k = 0; e_k < 3; e_k++) {
    int8_T ipiv_0;
    ipiv_0 = ipiv[e_k];
    if (e_k + 1 != ipiv_0) {
      d = F[e_k];
      F[e_k] = F[ipiv_0 - 1];
      F[ipiv_0 - 1] = d;
      d = F[e_k + 4];
      F[e_k + 4] = F[ipiv_0 + 3];
      F[ipiv_0 + 3] = d;
      d = F[e_k + 8];
      F[e_k + 8] = F[ipiv_0 + 7];
      F[ipiv_0 + 7] = d;
      d = F[e_k + 12];
      F[e_k + 12] = F[ipiv_0 + 11];
      F[ipiv_0 + 11] = d;
    }
  }

  for (e_k = 0; e_k < 4; e_k++) {
    jj = e_k << 2;
    for (iy = 0; iy < 4; iy++) {
      b_ix = iy << 2;
      k_k = iy + jj;
      if (F[k_k] != 0.0) {
        for (ix = iy + 2; ix < 5; ix++) {
          F_tmp = (ix + jj) - 1;
          F[F_tmp] -= V[(ix + b_ix) - 1] * F[k_k];
        }
      }
    }
  }

  for (e_k = 0; e_k < 4; e_k++) {
    jj = e_k << 2;
    for (iy = 3; iy >= 0; iy--) {
      b_ix = iy << 2;
      k_k = iy + jj;
      d = F[k_k];
      if (d != 0.0) {
        F[k_k] = d / V[iy + b_ix];
        for (ix = 0; ix < iy; ix++) {
          F_tmp = ix + jj;
          F[F_tmp] -= V[ix + b_ix] * F[k_k];
        }
      }
    }
  }

  tmp = _mm_set1_pd(1.0);
  _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[5], F[0]), tmp));
  F[0] = tmp_2[0];
  F[5] = tmp_2[1];
  _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_set_pd(F[15], F[10]), tmp));
  F[10] = tmp_2[0];
  F[15] = tmp_2[1];
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void LR_recomputeBlockDiag(const real_T A[16], real_T F[16], const
  int32_T blockFormat[3])
{
  real_T tmp[2];
  real_T delta;
  real_T expa;
  real_T sinchdelta;
  real_T x;
  if (blockFormat[0] != 0) {
    if (blockFormat[0] == 1) {
      delta = exp(A[0]);
      expa = exp(A[5]);
      sinchdelta = (A[0] + A[5]) / 2.0;
      if (fmax(sinchdelta, fabs(A[0] - A[5]) / 2.0) < 709.782712893384) {
        x = (A[5] - A[0]) / 2.0;
        if (x == 0.0) {
          x = 1.0;
        } else {
          x = sinh(x) / x;
        }

        sinchdelta = A[4] * exp(sinchdelta) * x;
      } else {
        sinchdelta = (expa - delta) * A[4] / (A[5] - A[0]);
      }

      F[0] = delta;
      F[4] = sinchdelta;
      F[5] = expa;
    } else {
      delta = sqrt(fabs(A[1] * A[4]));
      expa = exp(A[0]);
      if (delta == 0.0) {
        sinchdelta = 1.0;
      } else {
        sinchdelta = sin(delta) / delta;
      }

      F[0] = expa * cos(delta);
      _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(expa), _mm_set_pd
        (A[4], A[1])), _mm_set1_pd(sinchdelta)));
      F[1] = tmp[0];
      F[4] = tmp[1];
      F[5] = F[0];
    }
  }

  if (blockFormat[1] != 0) {
    if (blockFormat[1] == 1) {
      delta = exp(A[5]);
      expa = exp(A[10]);
      sinchdelta = (A[5] + A[10]) / 2.0;
      if (fmax(sinchdelta, fabs(A[5] - A[10]) / 2.0) < 709.782712893384) {
        x = (A[10] - A[5]) / 2.0;
        if (x == 0.0) {
          x = 1.0;
        } else {
          x = sinh(x) / x;
        }

        sinchdelta = A[9] * exp(sinchdelta) * x;
      } else {
        sinchdelta = (expa - delta) * A[9] / (A[10] - A[5]);
      }

      F[5] = delta;
      F[9] = sinchdelta;
      F[10] = expa;
    } else {
      delta = sqrt(fabs(A[6] * A[9]));
      expa = exp(A[5]);
      if (delta == 0.0) {
        sinchdelta = 1.0;
      } else {
        sinchdelta = sin(delta) / delta;
      }

      F[5] = expa * cos(delta);
      _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(expa), _mm_set_pd
        (A[9], A[6])), _mm_set1_pd(sinchdelta)));
      F[6] = tmp[0];
      F[9] = tmp[1];
      F[10] = F[5];
    }
  }

  if (blockFormat[2] != 0) {
    if (blockFormat[2] == 1) {
      delta = exp(A[10]);
      expa = exp(A[15]);
      sinchdelta = (A[10] + A[15]) / 2.0;
      if (fmax(sinchdelta, fabs(A[10] - A[15]) / 2.0) < 709.782712893384) {
        x = (A[15] - A[10]) / 2.0;
        if (x == 0.0) {
          x = 1.0;
        } else {
          x = sinh(x) / x;
        }

        sinchdelta = A[14] * exp(sinchdelta) * x;
      } else {
        sinchdelta = (expa - delta) * A[14] / (A[15] - A[10]);
      }

      F[10] = delta;
      F[14] = sinchdelta;
      F[15] = expa;
    } else {
      delta = sqrt(fabs(A[11] * A[14]));
      expa = exp(A[10]);
      if (delta == 0.0) {
        sinchdelta = 1.0;
      } else {
        sinchdelta = sin(delta) / delta;
      }

      F[10] = expa * cos(delta);
      _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(expa), _mm_set_pd
        (A[14], A[11])), _mm_set1_pd(sinchdelta)));
      F[11] = tmp[0];
      F[14] = tmp[1];
      F[15] = F[10];
    }
  }

  if (blockFormat[2] == 0) {
    F[15] = exp(A[15]);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static real_T LR_xnrm2_o(int32_T n, const real_T x[16], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      scale = 3.3121686421112381E-170;
      for (k = ix0; k <= ix0 + 1; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = sqrt(b * b + 1.0) * a;
  } else if (rtIsNaN(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void LR_xzsyhetrd(real_T A[16], real_T D[4], real_T E[3], real_T tau[3])
{
  __m128d tmp;
  real_T tmp_0[2];
  real_T taui;
  real_T temp2;
  real_T tmp_1;
  real_T xnorm;
  int32_T b_ix;
  int32_T b_iy;
  int32_T i;
  int32_T iv;
  int32_T knt;
  int32_T scalarLB;
  int32_T tau_tmp;
  int32_T temp2_tmp;
  int32_T temp2_tmp_tmp_tmp;
  int32_T vectorUB;
  for (i = 0; i < 3; i++) {
    temp2_tmp = i << 2;
    temp2_tmp_tmp_tmp = temp2_tmp + i;
    temp2 = A[temp2_tmp_tmp_tmp + 1];
    if (i + 3 <= 4) {
      b_ix = i + 3;
    } else {
      b_ix = 4;
    }

    iv = temp2_tmp + b_ix;
    taui = 0.0;
    xnorm = LR_xnrm2_o(2 - i, A, iv);
    if (xnorm != 0.0) {
      xnorm = rt_hypotd_snf(temp2, xnorm);
      if (temp2 >= 0.0) {
        xnorm = -xnorm;
      }

      if (fabs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        do {
          knt++;
          scalarLB = (iv - i) + 1;
          vectorUB = ((((scalarLB - iv) + 1) / 2) << 1) + iv;
          b_iy = vectorUB - 2;
          for (b_ix = iv; b_ix <= b_iy; b_ix += 2) {
            tmp = _mm_loadu_pd(&A[b_ix - 1]);
            _mm_storeu_pd(&A[b_ix - 1], _mm_mul_pd(tmp, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (b_ix = vectorUB; b_ix <= scalarLB; b_ix++) {
            A[b_ix - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          temp2 *= 9.9792015476736E+291;
        } while ((fabs(xnorm) < 1.0020841800044864E-292) && (knt < 20));

        xnorm = rt_hypotd_snf(temp2, LR_xnrm2_o(2 - i, A, iv));
        if (temp2 >= 0.0) {
          xnorm = -xnorm;
        }

        taui = (xnorm - temp2) / xnorm;
        temp2 = 1.0 / (temp2 - xnorm);
        for (b_ix = iv; b_ix <= b_iy; b_ix += 2) {
          tmp = _mm_loadu_pd(&A[b_ix - 1]);
          _mm_storeu_pd(&A[b_ix - 1], _mm_mul_pd(tmp, _mm_set1_pd(temp2)));
        }

        for (b_ix = vectorUB; b_ix <= scalarLB; b_ix++) {
          A[b_ix - 1] *= temp2;
        }

        for (iv = 0; iv < knt; iv++) {
          xnorm *= 1.0020841800044864E-292;
        }

        temp2 = xnorm;
      } else {
        taui = (xnorm - temp2) / xnorm;
        temp2 = 1.0 / (temp2 - xnorm);
        b_ix = (iv - i) + 1;
        scalarLB = ((((b_ix - iv) + 1) / 2) << 1) + iv;
        vectorUB = scalarLB - 2;
        for (knt = iv; knt <= vectorUB; knt += 2) {
          tmp = _mm_loadu_pd(&A[knt - 1]);
          _mm_storeu_pd(&A[knt - 1], _mm_mul_pd(tmp, _mm_set1_pd(temp2)));
        }

        for (knt = scalarLB; knt <= b_ix; knt++) {
          A[knt - 1] *= temp2;
        }

        temp2 = xnorm;
      }
    }

    E[i] = temp2;
    if (taui != 0.0) {
      A[temp2_tmp_tmp_tmp + 1] = 1.0;
      for (iv = i + 1; iv < 4; iv++) {
        tau[iv - 1] = 0.0;
      }

      vectorUB = 2 - i;
      scalarLB = 4 - i;
      for (iv = 0; iv <= vectorUB; iv++) {
        b_iy = i + iv;
        xnorm = A[(b_iy + temp2_tmp) + 1] * taui;
        temp2 = 0.0;
        tau_tmp = ((b_iy + 1) << 2) + i;
        tau[b_iy] += A[(tau_tmp + iv) + 1] * xnorm;
        for (knt = iv + 2; knt < scalarLB; knt++) {
          b_ix = i + knt;
          _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(A[tau_tmp +
            knt]), _mm_set_pd(A[b_ix + temp2_tmp], xnorm)), _mm_set_pd(temp2,
            tau[b_ix - 1])));
          tau[b_ix - 1] = tmp_0[0];
          temp2 = tmp_0[1];
        }

        tau[b_iy] += taui * temp2;
      }

      iv = temp2_tmp_tmp_tmp + 1;
      b_ix = i;
      b_iy = temp2_tmp_tmp_tmp + 1;
      xnorm = 0.0;
      for (knt = 0; knt <= vectorUB; knt++) {
        xnorm += tau[b_ix] * A[b_iy];
        b_ix++;
        b_iy++;
      }

      xnorm *= -0.5 * taui;
      if (!(xnorm == 0.0)) {
        b_ix = i;
        b_iy = 3 - i;
        for (knt = 0; knt < b_iy; knt++) {
          tau[b_ix] += xnorm * A[iv];
          iv++;
          b_ix++;
        }
      }

      for (iv = 0; iv <= vectorUB; iv++) {
        b_iy = i + iv;
        xnorm = A[(b_iy + temp2_tmp) + 1];
        temp2 = tau[b_iy];
        tmp_1 = temp2 * xnorm;
        b_ix = (b_iy + 1) << 2;
        tau_tmp = b_ix + i;
        A[(b_iy + b_ix) + 1] = (A[(tau_tmp + iv) + 1] - tmp_1) - tmp_1;
        for (knt = iv + 2; knt < scalarLB; knt++) {
          b_iy = i + knt;
          A[b_iy + b_ix] = (A[tau_tmp + knt] - tau[b_iy - 1] * xnorm) - A[b_iy +
            temp2_tmp] * temp2;
        }
      }
    }

    A[temp2_tmp_tmp_tmp + 1] = E[i];
    D[i] = A[temp2_tmp_tmp_tmp];
    tau[i] = taui;
  }

  D[3] = A[15];
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void LR_xzlascl_m(real_T cfrom, real_T cto, int32_T m, real_T A[4],
  int32_T iA0)
{
  real_T cfromc;
  real_T ctoc;
  int32_T b_i;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real_T cfrom1;
    real_T cto1;
    real_T mul;
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    scalarLB = (m / 2) << 1;
    vectorUB = scalarLB - 2;
    for (b_i = 0; b_i <= vectorUB; b_i += 2) {
      __m128d tmp;
      tmp_0 = (b_i + iA0) - 1;
      tmp = _mm_loadu_pd(&A[tmp_0]);
      _mm_storeu_pd(&A[tmp_0], _mm_mul_pd(tmp, _mm_set1_pd(mul)));
    }

    for (b_i = scalarLB; b_i < m; b_i++) {
      tmp_0 = (b_i + iA0) - 1;
      A[tmp_0] *= mul;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void LR_xzlascl_mk(real_T cfrom, real_T cto, int32_T m, real_T A[3],
  int32_T iA0)
{
  real_T cfromc;
  real_T ctoc;
  int32_T b_i;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real_T cfrom1;
    real_T cto1;
    real_T mul;
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    scalarLB = (m / 2) << 1;
    vectorUB = scalarLB - 2;
    for (b_i = 0; b_i <= vectorUB; b_i += 2) {
      __m128d tmp;
      tmp_0 = (b_i + iA0) - 1;
      tmp = _mm_loadu_pd(&A[tmp_0]);
      _mm_storeu_pd(&A[tmp_0], _mm_mul_pd(tmp, _mm_set1_pd(mul)));
    }

    for (b_i = scalarLB; b_i < m; b_i++) {
      tmp_0 = (b_i + iA0) - 1;
      A[tmp_0] *= mul;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void LR_xzlartg(real_T f, real_T g, real_T *cs, real_T *sn, real_T *r)
{
  real_T f1;
  f1 = fabs(f);
  *r = fabs(g);
  if (g == 0.0) {
    *cs = 1.0;
    *sn = 0.0;
    *r = f;
  } else if (f == 0.0) {
    *cs = 0.0;
    if (g >= 0.0) {
      *sn = 1.0;
    } else {
      *sn = -1.0;
    }
  } else if ((f1 > 1.4916681462400413E-154) && (f1 < 4.7403759540545887E+153) &&
             (*r > 1.4916681462400413E-154) && (*r < 4.7403759540545887E+153)) {
    *r = sqrt(f * f + g * g);
    *cs = f1 / *r;
    if (!(f >= 0.0)) {
      *r = -*r;
    }

    *sn = g / *r;
  } else {
    real_T fs;
    real_T gs;
    f1 = fmin(4.49423283715579E+307, fmax(2.2250738585072014E-308, fmax(f1, *r)));
    fs = f / f1;
    gs = g / f1;
    *r = sqrt(fs * fs + gs * gs);
    *cs = fabs(fs) / *r;
    if (!(f >= 0.0)) {
      *r = -*r;
    }

    *sn = gs / *r;
    *r *= f1;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void LR_rotateRight_h(int32_T n, real_T z[16], int32_T iz0, const real_T
  cs[6], int32_T ic0, int32_T is0)
{
  int32_T b_j;
  for (b_j = 0; b_j <= n - 2; b_j++) {
    real_T ctemp;
    real_T stemp;
    int32_T offsetj;
    int32_T offsetjp1;
    ctemp = cs[(ic0 + b_j) - 1];
    stemp = cs[(is0 + b_j) - 1];
    offsetj = ((b_j << 2) + iz0) - 1;
    offsetjp1 = (((b_j + 1) << 2) + iz0) - 1;
    if ((ctemp != 1.0) || (stemp != 0.0)) {
      real_T temp;
      real_T z_0;
      temp = z[offsetjp1];
      z_0 = z[offsetj];
      z[offsetjp1] = ctemp * temp - stemp * z_0;
      z[offsetj] = stemp * temp + ctemp * z_0;
      temp = z[offsetjp1 + 1];
      z_0 = z[offsetj + 1];
      z[offsetjp1 + 1] = ctemp * temp - z_0 * stemp;
      z[offsetj + 1] = z_0 * ctemp + stemp * temp;
      temp = z[offsetjp1 + 2];
      z_0 = z[offsetj + 2];
      z[offsetjp1 + 2] = ctemp * temp - z_0 * stemp;
      z[offsetj + 2] = z_0 * ctemp + stemp * temp;
      temp = z[offsetjp1 + 3];
      z_0 = z[offsetj + 3];
      z[offsetjp1 + 3] = ctemp * temp - z_0 * stemp;
      z[offsetj + 3] = z_0 * ctemp + stemp * temp;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void LR_xdlaev2_a(real_T a, real_T b, real_T c, real_T *rt1, real_T *rt2,
  real_T *cs1, real_T *sn1)
{
  real_T ab;
  real_T acmn;
  real_T acmx;
  real_T adf;
  real_T df;
  real_T sm;
  real_T tb;
  int32_T sgn1;
  int32_T sgn2;
  sm = a + c;
  df = a - c;
  adf = fabs(df);
  tb = b + b;
  ab = fabs(tb);
  if (fabs(a) > fabs(c)) {
    acmx = a;
    acmn = c;
  } else {
    acmx = c;
    acmn = a;
  }

  if (adf > ab) {
    real_T b_a;
    b_a = ab / adf;
    adf *= sqrt(b_a * b_a + 1.0);
  } else if (adf < ab) {
    adf /= ab;
    adf = sqrt(adf * adf + 1.0) * ab;
  } else {
    adf = ab * 1.4142135623730951;
  }

  if (sm < 0.0) {
    *rt1 = (sm - adf) * 0.5;
    sgn1 = -1;
    *rt2 = acmx / *rt1 * acmn - b / *rt1 * b;
  } else if (sm > 0.0) {
    *rt1 = (sm + adf) * 0.5;
    sgn1 = 1;
    *rt2 = acmx / *rt1 * acmn - b / *rt1 * b;
  } else {
    *rt1 = 0.5 * adf;
    *rt2 = -0.5 * adf;
    sgn1 = 1;
  }

  if (df >= 0.0) {
    df += adf;
    sgn2 = 1;
  } else {
    df -= adf;
    sgn2 = -1;
  }

  if (fabs(df) > ab) {
    tb = -tb / df;
    *sn1 = 1.0 / sqrt(tb * tb + 1.0);
    *cs1 = tb * *sn1;
  } else if (ab == 0.0) {
    *cs1 = 1.0;
    *sn1 = 0.0;
  } else {
    tb = -df / tb;
    *cs1 = 1.0 / sqrt(tb * tb + 1.0);
    *sn1 = tb * *cs1;
  }

  if (sgn1 == sgn2) {
    tb = *cs1;
    *cs1 = -*sn1;
    *sn1 = tb;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void LR_rotateRight(int32_T n, real_T z[16], int32_T iz0, const real_T
  cs[6], int32_T ic0, int32_T is0)
{
  int32_T j;
  for (j = n - 1; j >= 1; j--) {
    real_T ctemp;
    real_T stemp;
    int32_T offsetj;
    int32_T offsetjp1;
    ctemp = cs[(ic0 + j) - 2];
    stemp = cs[(is0 + j) - 2];
    offsetj = (((j - 1) << 2) + iz0) - 1;
    offsetjp1 = ((j << 2) + iz0) - 1;
    if ((ctemp != 1.0) || (stemp != 0.0)) {
      real_T temp;
      real_T z_0;
      temp = z[offsetjp1];
      z_0 = z[offsetj];
      z[offsetjp1] = ctemp * temp - stemp * z_0;
      z[offsetj] = stemp * temp + ctemp * z_0;
      temp = z[offsetjp1 + 1];
      z_0 = z[offsetj + 1];
      z[offsetjp1 + 1] = ctemp * temp - z_0 * stemp;
      z[offsetj + 1] = z_0 * ctemp + stemp * temp;
      temp = z[offsetjp1 + 2];
      z_0 = z[offsetj + 2];
      z[offsetjp1 + 2] = ctemp * temp - z_0 * stemp;
      z[offsetj + 2] = z_0 * ctemp + stemp * temp;
      temp = z[offsetjp1 + 3];
      z_0 = z[offsetj + 3];
      z[offsetjp1 + 3] = ctemp * temp - z_0 * stemp;
      z[offsetj + 3] = z_0 * ctemp + stemp * temp;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static int32_T LR_xzsteqr(real_T d[4], real_T e[3], real_T z[16])
{
  real_T work[6];
  real_T b;
  real_T b_anorm;
  real_T b_s;
  real_T c;
  real_T e_0;
  real_T p;
  real_T r;
  real_T s;
  real_T tst;
  int32_T b_anorm_tmp;
  int32_T exitg1;
  int32_T exitg3;
  int32_T exitg4;
  int32_T i;
  int32_T info;
  int32_T iscale;
  int32_T l;
  int32_T l1;
  int32_T lend;
  int32_T lendsv;
  int32_T lsv;
  int32_T m;
  int32_T n_tmp;
  boolean_T exitg2;
  info = 0;
  for (i = 0; i < 6; i++) {
    work[i] = 0.0;
  }

  i = 0;
  l1 = 1;
  do {
    exitg1 = 0;
    if (l1 > 4) {
      for (i = 0; i < 3; i++) {
        l = i;
        p = d[i];
        for (l1 = i + 2; l1 < 5; l1++) {
          tst = d[l1 - 1];
          if (tst < p) {
            l = l1 - 1;
            p = tst;
          }
        }

        if (l != i) {
          d[l] = d[i];
          d[i] = p;
          m = i << 2;
          l <<= 2;
          tst = z[m];
          z[m] = z[l];
          z[l] = tst;
          tst = z[m + 1];
          z[m + 1] = z[l + 1];
          z[l + 1] = tst;
          tst = z[m + 2];
          z[m + 2] = z[l + 2];
          z[l + 2] = tst;
          tst = z[m + 3];
          z[m + 3] = z[l + 3];
          z[l + 3] = tst;
        }
      }

      exitg1 = 1;
    } else {
      if (l1 > 1) {
        e[l1 - 2] = 0.0;
      }

      m = l1;
      exitg2 = false;
      while ((!exitg2) && (m < 4)) {
        tst = fabs(e[m - 1]);
        if (tst == 0.0) {
          exitg2 = true;
        } else if (tst <= sqrt(fabs(d[m - 1])) * sqrt(fabs(d[m])) *
                   2.2204460492503131E-16) {
          e[m - 1] = 0.0;
          exitg2 = true;
        } else {
          m++;
        }
      }

      l = l1 - 1;
      lsv = l1;
      lend = m;
      lendsv = m;
      l1 = m + 1;
      if (l + 1 == m) {
      } else {
        n_tmp = m - l;
        if (n_tmp <= 0) {
          tst = 0.0;
        } else {
          tst = fabs(d[(l + n_tmp) - 1]);
          iscale = 0;
          exitg2 = false;
          while ((!exitg2) && (iscale <= n_tmp - 2)) {
            b_anorm_tmp = l + iscale;
            b_anorm = fabs(d[b_anorm_tmp]);
            if (rtIsNaN(b_anorm)) {
              tst = (rtNaN);
              exitg2 = true;
            } else {
              if (b_anorm > tst) {
                tst = b_anorm;
              }

              b_anorm = fabs(e[b_anorm_tmp]);
              if (rtIsNaN(b_anorm)) {
                tst = (rtNaN);
                exitg2 = true;
              } else {
                if (b_anorm > tst) {
                  tst = b_anorm;
                }

                iscale++;
              }
            }
          }
        }

        iscale = 0;
        if (tst == 0.0) {
        } else if (rtIsInf(tst) || rtIsNaN(tst)) {
          d[0] = (rtNaN);
          d[1] = (rtNaN);
          d[2] = (rtNaN);
          d[3] = (rtNaN);
          for (m = 0; m < 16; m++) {
            z[m] = (rtNaN);
          }

          exitg1 = 1;
        } else {
          if (tst > 2.2346346549904327E+153) {
            iscale = 1;
            LR_xzlascl_m(tst, 2.2346346549904327E+153, n_tmp, d, l + 1);
            LR_xzlascl_mk(tst, 2.2346346549904327E+153, n_tmp - 1, e, l + 1);
          } else if (tst < 3.02546243347603E-123) {
            iscale = 2;
            LR_xzlascl_m(tst, 3.02546243347603E-123, n_tmp, d, l + 1);
            LR_xzlascl_mk(tst, 3.02546243347603E-123, n_tmp - 1, e, l + 1);
          }

          if (fabs(d[m - 1]) < fabs(d[l])) {
            lend = lsv;
            l = m - 1;
          }

          if (lend > l + 1) {
            do {
              exitg4 = 0;
              if (l + 1 != lend) {
                m = l;
                exitg2 = false;
                while ((!exitg2) && (m + 1 < lend)) {
                  b_anorm = fabs(e[m]);
                  if (b_anorm * b_anorm <= 4.9303806576313238E-32 * fabs(d[m]) *
                      fabs(d[m + 1]) + 2.2250738585072014E-308) {
                    exitg2 = true;
                  } else {
                    m++;
                  }
                }
              } else {
                m = lend - 1;
              }

              if (m + 1 < lend) {
                e[m] = 0.0;
              }

              if (m + 1 == l + 1) {
                l++;
                if (l + 1 > lend) {
                  exitg4 = 1;
                }
              } else if (m + 1 == l + 2) {
                LR_xdlaev2_a(d[l], e[l], d[l + 1], &d[l], &b_anorm, &work[l], &s);
                d[l + 1] = b_anorm;
                work[l + 3] = s;
                LR_rotateRight(2, z, (l << 2) + 1, work, l + 1, l + 4);
                e[l] = 0.0;
                l += 2;
                if (l + 1 > lend) {
                  exitg4 = 1;
                }
              } else if (i == 120) {
                exitg4 = 1;
              } else {
                i++;
                b_anorm = (d[l + 1] - d[l]) / (2.0 * e[l]);
                s = rt_hypotd_snf(b_anorm, 1.0);
                if (!(b_anorm >= 0.0)) {
                  s = -s;
                }

                b_anorm = e[l] / (b_anorm + s) + (d[m] - d[l]);
                s = 1.0;
                c = 1.0;
                p = 0.0;
                for (n_tmp = m; n_tmp >= l + 1; n_tmp--) {
                  e_0 = e[n_tmp - 1];
                  b = c * e_0;
                  LR_xzlartg(b_anorm, s * e_0, &c, &b_s, &r);
                  s = b_s;
                  if (n_tmp != m) {
                    e[n_tmp] = r;
                  }

                  b_anorm = d[n_tmp] - p;
                  r = (d[n_tmp - 1] - b_anorm) * b_s + 2.0 * c * b;
                  p = b_s * r;
                  d[n_tmp] = b_anorm + p;
                  b_anorm = c * r - b;
                  work[n_tmp - 1] = c;
                  work[n_tmp + 2] = -b_s;
                }

                LR_rotateRight((m - l) + 1, z, (l << 2) + 1, work, l + 1, l + 4);
                d[l] -= p;
                e[l] = b_anorm;
              }
            } while (exitg4 == 0);
          } else {
            do {
              exitg3 = 0;
              if (l + 1 != lend) {
                m = l + 1;
                exitg2 = false;
                while ((!exitg2) && (m > lend)) {
                  b_anorm = fabs(e[m - 2]);
                  if (b_anorm * b_anorm <= fabs(d[m - 1]) *
                      4.9303806576313238E-32 * fabs(d[m - 2]) +
                      2.2250738585072014E-308) {
                    exitg2 = true;
                  } else {
                    m--;
                  }
                }
              } else {
                m = lend;
              }

              if (m > lend) {
                e[m - 2] = 0.0;
              }

              if (l + 1 == m) {
                l--;
                if (l + 1 < lend) {
                  exitg3 = 1;
                }
              } else if (m == l) {
                LR_xdlaev2_a(d[l - 1], e[l - 1], d[l], &d[l - 1], &b_anorm,
                             &work[m - 1], &s);
                d[l] = b_anorm;
                work[m + 2] = s;
                LR_rotateRight_h(2, z, ((l - 1) << 2) + 1, work, m, m + 3);
                e[l - 1] = 0.0;
                l -= 2;
                if (l + 1 < lend) {
                  exitg3 = 1;
                }
              } else if (i == 120) {
                exitg3 = 1;
              } else {
                i++;
                p = e[l - 1];
                b_anorm = (d[l - 1] - d[l]) / (p * 2.0);
                s = rt_hypotd_snf(b_anorm, 1.0);
                if (!(b_anorm >= 0.0)) {
                  s = -s;
                }

                b_anorm = (d[m - 1] - d[l]) + p / (b_anorm + s);
                s = 1.0;
                c = 1.0;
                p = 0.0;
                for (n_tmp = m; n_tmp <= l; n_tmp++) {
                  e_0 = e[n_tmp - 1];
                  b = c * e_0;
                  LR_xzlartg(b_anorm, s * e_0, &c, &b_s, &r);
                  s = b_s;
                  if (n_tmp != m) {
                    e[n_tmp - 2] = r;
                  }

                  b_anorm = d[n_tmp - 1] - p;
                  r = (d[n_tmp] - b_anorm) * b_s + 2.0 * c * b;
                  p = b_s * r;
                  d[n_tmp - 1] = b_anorm + p;
                  b_anorm = c * r - b;
                  work[n_tmp - 1] = c;
                  work[n_tmp + 2] = b_s;
                }

                LR_rotateRight_h((l - m) + 2, z, ((m - 1) << 2) + 1, work, m, m
                                 + 3);
                d[l] -= p;
                e[l - 1] = b_anorm;
              }
            } while (exitg3 == 0);
          }

          if (iscale == 1) {
            m = lendsv - lsv;
            LR_xzlascl_m(2.2346346549904327E+153, tst, m + 1, d, lsv);
            LR_xzlascl_mk(2.2346346549904327E+153, tst, m, e, lsv);
          } else if (iscale == 2) {
            m = lendsv - lsv;
            LR_xzlascl_m(3.02546243347603E-123, tst, m + 1, d, lsv);
            LR_xzlascl_mk(3.02546243347603E-123, tst, m, e, lsv);
          }

          if (i >= 120) {
            if (e[0] != 0.0) {
              info = 1;
            }

            if (e[1] != 0.0) {
              info++;
            }

            if (e[2] != 0.0) {
              info++;
            }

            exitg1 = 1;
          }
        }
      }
    }
  } while (exitg1 == 0);

  return info;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void LR_xsyheev(real_T A[16], int32_T *info, real_T W[4])
{
  __m128d tmp;
  __m128d tmp_0;
  real_T work[4];
  real_T e[3];
  real_T tau[3];
  real_T absx;
  real_T anrm;
  real_T cfrom1;
  real_T cfromc;
  real_T cto1;
  real_T mul;
  int32_T b_ia;
  int32_T coltop;
  int32_T e_i;
  int32_T exitg1;
  int32_T f;
  int32_T iaii;
  int32_T itau;
  int32_T jy;
  int32_T lastc;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T iscale;
  boolean_T notdone;
  *info = 0;
  anrm = 0.0;
  e_i = 0;
  exitg2 = false;
  while ((!exitg2) && (e_i < 4)) {
    itau = 0;
    do {
      exitg1 = 0;
      if (itau <= e_i) {
        absx = fabs(A[(e_i << 2) + itau]);
        if (rtIsNaN(absx)) {
          anrm = (rtNaN);
          exitg1 = 1;
        } else {
          if (absx > anrm) {
            anrm = absx;
          }

          itau++;
        }
      } else {
        e_i++;
        exitg1 = 2;
      }
    } while (exitg1 == 0);

    if (exitg1 == 1) {
      exitg2 = true;
    }
  }

  if (rtIsInf(anrm) || rtIsNaN(anrm)) {
    W[0] = (rtNaN);
    W[1] = (rtNaN);
    W[2] = (rtNaN);
    W[3] = (rtNaN);
    for (iaii = 0; iaii < 16; iaii++) {
      A[iaii] = (rtNaN);
    }
  } else {
    iscale = false;
    guard1 = false;
    if ((anrm > 0.0) && (anrm < 1.0010415475915505E-146)) {
      iscale = true;
      anrm = 1.0010415475915505E-146 / anrm;
      guard1 = true;
    } else if (anrm > 9.9895953610111751E+145) {
      iscale = true;
      anrm = 9.9895953610111751E+145 / anrm;
      guard1 = true;
    }

    if (guard1) {
      absx = anrm;
      cfromc = 1.0;
      notdone = true;
      while (notdone) {
        cfrom1 = cfromc * 2.0041683600089728E-292;
        cto1 = absx / 4.9896007738368E+291;
        if ((fabs(cfrom1) > absx) && (absx != 0.0)) {
          mul = 2.0041683600089728E-292;
          cfromc = cfrom1;
        } else if (cto1 > fabs(cfromc)) {
          mul = 4.9896007738368E+291;
          absx = cto1;
        } else {
          mul = absx / cfromc;
          notdone = false;
        }

        for (e_i = 0; e_i < 4; e_i++) {
          iaii = e_i << 2;
          A[iaii] *= mul;
          A[iaii + 1] *= mul;
          A[iaii + 2] *= mul;
          A[iaii + 3] *= mul;
        }
      }
    }

    LR_xzsyhetrd(A, W, e, tau);
    for (e_i = 2; e_i >= 0; e_i--) {
      iaii = (e_i + 1) << 2;
      A[iaii] = 0.0;
      for (itau = e_i + 3; itau < 5; itau++) {
        A[(itau + iaii) - 1] = A[((e_i << 2) + itau) - 1];
      }
    }

    A[0] = 1.0;
    A[1] = 0.0;
    A[2] = 0.0;
    A[3] = 0.0;
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (e_i = 2; e_i >= 0; e_i--) {
      iaii = ((e_i << 2) + e_i) + 10;
      if (e_i + 1 < 3) {
        A[iaii - 5] = 1.0;
        if (tau[e_i] != 0.0) {
          itau = 3 - e_i;
          lastc = iaii - e_i;
          while ((itau > 0) && (A[lastc - 3] == 0.0)) {
            itau--;
            lastc--;
          }

          lastc = 1 - e_i;
          exitg2 = false;
          while ((!exitg2) && (lastc + 1 > 0)) {
            coltop = (lastc << 2) + iaii;
            b_ia = coltop;
            do {
              exitg1 = 0;
              if (b_ia <= (coltop + itau) - 1) {
                if (A[b_ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  b_ia++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          itau = 0;
          lastc = -1;
        }

        if (itau > 0) {
          if (lastc + 1 != 0) {
            memset(&work[0], 0, (uint32_T)(lastc + 1) * sizeof(real_T));
            jy = (lastc << 2) + iaii;
            for (coltop = iaii; coltop <= jy; coltop += 4) {
              absx = 0.0;
              f = (coltop + itau) - 1;
              for (b_ia = coltop; b_ia <= f; b_ia++) {
                absx += A[((iaii + b_ia) - coltop) - 5] * A[b_ia - 1];
              }

              b_ia = (coltop - iaii) >> 2;
              work[b_ia] += absx;
            }
          }

          if (!(-tau[e_i] == 0.0)) {
            jy = iaii;
            for (coltop = 0; coltop <= lastc; coltop++) {
              absx = work[coltop];
              if (absx != 0.0) {
                absx *= -tau[e_i];
                f = (itau + jy) - 1;
                for (b_ia = jy; b_ia <= f; b_ia++) {
                  A[b_ia - 1] += A[((iaii + b_ia) - jy) - 5] * absx;
                }
              }

              jy += 4;
            }
          }
        }

        lastc = (iaii - e_i) - 2;
        coltop = (((((lastc - iaii) + 4) / 2) << 1) + iaii) - 3;
        b_ia = coltop - 2;
        for (itau = iaii - 3; itau <= b_ia; itau += 2) {
          tmp = _mm_loadu_pd(&A[itau - 1]);
          _mm_storeu_pd(&A[itau - 1], _mm_mul_pd(tmp, _mm_set1_pd(-tau[e_i])));
        }

        for (itau = coltop; itau <= lastc; itau++) {
          A[itau - 1] *= -tau[e_i];
        }
      }

      A[iaii - 5] = 1.0 - tau[e_i];
      for (itau = 0; itau < e_i; itau++) {
        A[(iaii - itau) - 6] = 0.0;
      }
    }

    *info = LR_xzsteqr(W, e, A);
    if (*info != 0) {
      W[0] = (rtNaN);
      W[1] = (rtNaN);
      W[2] = (rtNaN);
      W[3] = (rtNaN);
      for (iaii = 0; iaii < 16; iaii++) {
        A[iaii] = (rtNaN);
      }
    } else if (iscale) {
      tmp = _mm_set1_pd(1.0 / anrm);
      tmp_0 = _mm_mul_pd(tmp, _mm_loadu_pd(&W[0]));
      _mm_storeu_pd(&W[0], tmp_0);
      tmp = _mm_mul_pd(tmp, _mm_loadu_pd(&W[2]));
      _mm_storeu_pd(&W[2], tmp);
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function10' */
static real_T LR_norm_e(const real_T x[3])
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

/* Function for MATLAB Function: '<Root>/MATLAB Function8' */
static real_T LR_norm_jl(const real_T x[6])
{
  real_T scale;
  real_T y;
  int32_T k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 6; k++) {
    real_T absxk;
    absxk = fabs(x[k]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function13' */
static void LR_ddexpInvSo3(const real_T xi[3], const real_T xidot[3], real_T
  ddexpInv[9])
{
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_3;
  __m128d tmp_4;
  real_T omg[9];
  real_T omgdot[9];
  real_T tmp[9];
  real_T tmp_2[2];
  real_T ddexpInv_tmp_1;
  real_T ddexpInv_tmp_2;
  real_T ddexpInv_tmp_3;
  real_T s;
  real_T theta;
  real_T theta_2;
  real_T theta_sqr;
  int32_T ddexpInv_tmp;
  int32_T ddexpInv_tmp_0;
  int32_T ddexpInv_tmp_4;
  int32_T i;
  int32_T i_0;
  theta = LR_norm_e(xi);
  if (fabs(sin(theta)) < 1.0E-7) {
    if (theta < 1.0E-7) {
      ddexpInv[0] = -0.0;
      tmp_3 = _mm_set1_pd(-0.5);
      _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(tmp_3, _mm_set_pd(xidot[1], -xidot[2])));
      ddexpInv[3] = tmp_2[0];
      ddexpInv[6] = tmp_2[1];
      ddexpInv[1] = -0.5 * xidot[2];
      ddexpInv[4] = -0.0;
      _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(tmp_3, _mm_set_pd(-xidot[1], -xidot[0])));
      ddexpInv[7] = tmp_2[0];
      ddexpInv[2] = tmp_2[1];
      ddexpInv[5] = -0.5 * xidot[0];
      ddexpInv[8] = -0.0;
    } else {
      omg[0] = 0.0;
      omg[3] = -xi[2];
      omg[6] = xi[1];
      omg[1] = xi[2];
      omg[4] = 0.0;
      omg[7] = -xi[0];
      omg[2] = -xi[1];
      omg[5] = xi[0];
      omg[8] = 0.0;
      theta = (0.0027777777777777779 * xi[0] * xidot[0] + 0.0027777777777777779 *
               xi[1] * xidot[1]) + 0.0027777777777777779 * xi[2] * xidot[2];
      omgdot[0] = -0.0;
      tmp_3 = _mm_set1_pd(-0.5);
      _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(tmp_3, _mm_set_pd(xidot[1], -xidot[2])));
      omgdot[3] = tmp_2[0];
      omgdot[6] = tmp_2[1];
      omgdot[1] = -0.5 * xidot[2];
      omgdot[4] = -0.0;
      _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(tmp_3, _mm_set_pd(-xidot[1], -xidot[0])));
      omgdot[7] = tmp_2[0];
      omgdot[2] = tmp_2[1];
      omgdot[5] = -0.5 * xidot[0];
      omgdot[8] = -0.0;
      for (i = 0; i < 3; i++) {
        for (i_0 = 0; i_0 <= 0; i_0 += 2) {
          tmp_3 = _mm_loadu_pd(&omg[i_0 + 3]);
          tmp_4 = _mm_set1_pd(0.083333333333333329);
          tmp_0 = _mm_loadu_pd(&omg[i_0]);
          tmp_1 = _mm_loadu_pd(&omg[i_0 + 6]);
          _mm_storeu_pd(&tmp[i_0 + 3 * i], _mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_mul_pd(tmp_3, tmp_4), _mm_set1_pd(omg[3 * i + 1])), _mm_mul_pd
            (_mm_mul_pd(tmp_4, tmp_0), _mm_set1_pd(omg[3 * i]))), _mm_mul_pd
            (_mm_mul_pd(tmp_1, tmp_4), _mm_set1_pd(omg[3 * i + 2]))));
        }

        for (i_0 = 2; i_0 < 3; i_0++) {
          tmp[i_0 + 3 * i] = (omg[i_0 + 3] * 0.083333333333333329 * omg[3 * i +
                              1] + 0.083333333333333329 * omg[i_0] * omg[3 * i])
            + omg[i_0 + 6] * 0.083333333333333329 * omg[3 * i + 2];
        }
      }

      for (i = 0; i <= 6; i += 2) {
        tmp_3 = _mm_loadu_pd(&omgdot[i]);
        tmp_4 = _mm_loadu_pd(&tmp[i]);
        _mm_storeu_pd(&ddexpInv[i], _mm_sub_pd(_mm_add_pd(tmp_3, tmp_4),
          _mm_set1_pd(theta)));
      }

      for (i = 8; i < 9; i++) {
        ddexpInv[i] = (omgdot[i] + tmp[i]) - theta;
      }
    }
  } else {
    theta_2 = theta / 2.0;
    s = sin(theta_2) / theta_2;
    theta_2 = cos(theta_2) / s;
    theta_sqr = theta * theta;
    omg[0] = 0.0;
    omg[3] = -xi[2];
    omg[6] = xi[1];
    omg[1] = xi[2];
    omg[4] = 0.0;
    omg[7] = -xi[0];
    omg[2] = -xi[1];
    omg[5] = xi[0];
    omg[8] = 0.0;
    omgdot[0] = 0.0;
    omgdot[3] = -xidot[2];
    omgdot[6] = xidot[1];
    omgdot[1] = xidot[2];
    omgdot[4] = 0.0;
    omgdot[7] = -xidot[0];
    omgdot[2] = -xidot[1];
    omgdot[5] = xidot[0];
    omgdot[8] = 0.0;
    theta = (1.0 - theta_2) / theta_sqr;
    s = ((1.0 / (s * s) + theta_2) - 2.0) * (1.0 / theta_sqr) / theta_sqr *
      ((xi[0] * xidot[0] + xi[1] * xidot[1]) + xi[2] * xidot[2]);
    for (i = 0; i < 3; i++) {
      for (i_0 = 0; i_0 < 3; i_0++) {
        ddexpInv_tmp = 3 * i_0 + 1;
        ddexpInv_tmp_0 = 3 * i_0 + 2;
        theta_2 = omg[i + 3];
        theta_sqr = omg[ddexpInv_tmp];
        ddexpInv_tmp_1 = omg[3 * i_0];
        ddexpInv_tmp_2 = omg[i + 6];
        ddexpInv_tmp_3 = omg[ddexpInv_tmp_0];
        ddexpInv_tmp_4 = 3 * i_0 + i;
        ddexpInv[ddexpInv_tmp_4] = ((((omgdot[3 * i_0] * omg[i] +
          omgdot[ddexpInv_tmp] * theta_2) + omgdot[ddexpInv_tmp_0] *
          ddexpInv_tmp_2) + ((omgdot[i + 3] * theta_sqr + ddexpInv_tmp_1 *
                              omgdot[i]) + omgdot[i + 6] * ddexpInv_tmp_3)) *
          theta + omgdot[ddexpInv_tmp_4] * -0.5) + ((theta_2 * s * theta_sqr + s
          * omg[i] * ddexpInv_tmp_1) + ddexpInv_tmp_2 * s * ddexpInv_tmp_3);
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T LR_xzlangeM(const real_T x[16])
{
  real_T y;
  int32_T k;
  boolean_T exitg1;
  y = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 16)) {
    real_T absxk;
    absxk = fabs(x[k]);
    if (rtIsNaN(absxk)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (absxk > y) {
        y = absxk;
      }

      k++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xzlascl(real_T cfrom, real_T cto, real_T A[16])
{
  real_T cfromc;
  real_T ctoc;
  int32_T j;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real_T cfrom1;
    real_T cto1;
    real_T mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    for (j = 0; j < 4; j++) {
      int32_T tmp;
      tmp = j << 2;
      A[tmp] *= mul;
      A[tmp + 1] *= mul;
      A[tmp + 2] *= mul;
      A[tmp + 3] *= mul;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T LR_xnrm2(int32_T n, const real_T x[16], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (k = ix0; k < kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T LR_xdotc(int32_T n, const real_T x[16], int32_T ix0, const real_T
  y[16], int32_T iy0)
{
  real_T d;
  int32_T b;
  int32_T k;
  d = 0.0;
  b = (uint8_T)n;
  for (k = 0; k < b; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }

  return d;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[16], int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    for (k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T LR_xnrm2_f(int32_T n, const real_T x[4], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T k;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = ix0 + n;
  for (k = ix0; k < kend; k++) {
    real_T absxk;
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xaxpy_o(int32_T n, real_T a, const real_T x[16], int32_T ix0,
  real_T y[4], int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (k = scalarLB; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xaxpy_o2(int32_T n, real_T a, const real_T x[4], int32_T ix0,
  real_T y[16], int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (k = scalarLB; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xzlascl_f(real_T cfrom, real_T cto, real_T A[4])
{
  real_T cfromc;
  real_T ctoc;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    __m128d tmp;
    __m128d tmp_0;
    real_T cfrom1;
    real_T cto1;
    real_T mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    tmp_0 = _mm_set1_pd(mul);
    tmp = _mm_mul_pd(_mm_loadu_pd(&A[0]), tmp_0);
    _mm_storeu_pd(&A[0], tmp);
    tmp_0 = _mm_mul_pd(_mm_loadu_pd(&A[2]), tmp_0);
    _mm_storeu_pd(&A[2], tmp_0);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xrotg(real_T *a, real_T *b, real_T *c, real_T *s)
{
  real_T absa;
  real_T absb;
  real_T roe;
  real_T scale;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *a = 0.0;
    *b = 0.0;
  } else {
    real_T ads;
    real_T bds;
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }

    *a = scale;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_svd(const real_T A[16], real_T U[4])
{
  __m128d tmp;
  real_T b_A[16];
  real_T e[4];
  real_T s[4];
  real_T work[4];
  real_T tmp_0[2];
  real_T anrm;
  real_T cscale;
  real_T nrm;
  real_T rt;
  real_T shift;
  real_T smm1;
  real_T sqds;
  real_T ztest;
  int32_T exitg1;
  int32_T m;
  int32_T qjj;
  int32_T qp1;
  int32_T qq;
  int32_T qq_tmp;
  int32_T qs;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T apply_transform;
  boolean_T doscale;
  boolean_T exitg2;
  memcpy(&b_A[0], &A[0], sizeof(real_T) << 4U);
  s[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  s[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  s[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  s[3] = 0.0;
  e[3] = 0.0;
  work[3] = 0.0;
  doscale = false;
  anrm = LR_xzlangeM(A);
  cscale = anrm;
  if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
    doscale = true;
    cscale = 6.7178761075670888E-139;
    LR_xzlascl(anrm, cscale, b_A);
  } else if (anrm > 1.4885657073574029E+138) {
    doscale = true;
    cscale = 1.4885657073574029E+138;
    LR_xzlascl(anrm, cscale, b_A);
  }

  for (m = 0; m < 3; m++) {
    qp1 = m + 2;
    qq_tmp = (m << 2) + m;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = LR_xnrm2(4 - m, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        nrm = -nrm;
      }

      s[m] = nrm;
      if (fabs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qs = (qq_tmp - m) + 4;
        scalarLB = ((((qs - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qjj = qq; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&b_A[qjj - 1]);
          _mm_storeu_pd(&b_A[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qjj = scalarLB; qjj <= qs; qjj++) {
          b_A[qjj - 1] *= nrm;
        }
      } else {
        qs = (qq_tmp - m) + 4;
        scalarLB = ((((qs - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qjj = qq; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&b_A[qjj - 1]);
          _mm_storeu_pd(&b_A[qjj - 1], _mm_div_pd(tmp, _mm_set1_pd(s[m])));
        }

        for (qjj = scalarLB; qjj <= qs; qjj++) {
          b_A[qjj - 1] /= s[m];
        }
      }

      b_A[qq_tmp]++;
      s[m] = -s[m];
    } else {
      s[m] = 0.0;
    }

    for (qs = qp1; qs < 5; qs++) {
      qjj = ((qs - 1) << 2) + m;
      if (apply_transform) {
        LR_xaxpy(4 - m, -(LR_xdotc(4 - m, b_A, qq_tmp + 1, b_A, qjj + 1) /
                          b_A[qq_tmp]), qq_tmp + 1, b_A, qjj + 1);
      }

      e[qs - 1] = b_A[qjj];
    }

    if (m + 1 <= 2) {
      nrm = LR_xnrm2_f(3 - m, e, m + 2);
      if (nrm == 0.0) {
        e[m] = 0.0;
      } else {
        if (e[m + 1] < 0.0) {
          e[m] = -nrm;
        } else {
          e[m] = nrm;
        }

        nrm = e[m];
        if (fabs(e[m]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[m];
          scalarLB = ((((3 - m) / 2) << 1) + m) + 2;
          vectorUB = scalarLB - 2;
          for (qs = qp1; qs <= vectorUB; qs += 2) {
            tmp = _mm_loadu_pd(&e[qs - 1]);
            _mm_storeu_pd(&e[qs - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qs = scalarLB; qs < 5; qs++) {
            e[qs - 1] *= nrm;
          }
        } else {
          scalarLB = ((((3 - m) / 2) << 1) + m) + 2;
          vectorUB = scalarLB - 2;
          for (qs = qp1; qs <= vectorUB; qs += 2) {
            tmp = _mm_loadu_pd(&e[qs - 1]);
            _mm_storeu_pd(&e[qs - 1], _mm_div_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qs = scalarLB; qs < 5; qs++) {
            e[qs - 1] /= nrm;
          }
        }

        e[m + 1]++;
        e[m] = -e[m];
        for (qq = qp1; qq < 5; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 5; qq++) {
          LR_xaxpy_o(3 - m, e[qq - 1], b_A, (m + ((qq - 1) << 2)) + 2, work, m +
                     2);
        }

        for (qq = qp1; qq < 5; qq++) {
          LR_xaxpy_o2(3 - m, -e[qq - 1] / e[m + 1], work, m + 2, b_A, (m + ((qq
            - 1) << 2)) + 2);
        }
      }
    }
  }

  m = 2;
  s[3] = b_A[15];
  e[2] = b_A[14];
  e[3] = 0.0;
  if (s[0] != 0.0) {
    rt = fabs(s[0]);
    nrm = s[0] / rt;
    s[0] = rt;
    e[0] /= nrm;
  }

  if (e[0] != 0.0) {
    rt = fabs(e[0]);
    nrm = rt / e[0];
    e[0] = rt;
    s[1] *= nrm;
  }

  if (s[1] != 0.0) {
    rt = fabs(s[1]);
    nrm = s[1] / rt;
    s[1] = rt;
    e[1] /= nrm;
  }

  if (e[1] != 0.0) {
    rt = fabs(e[1]);
    nrm = rt / e[1];
    e[1] = rt;
    s[2] *= nrm;
  }

  if (s[2] != 0.0) {
    rt = fabs(s[2]);
    nrm = s[2] / rt;
    s[2] = rt;
    e[2] = b_A[14] / nrm;
  }

  if (e[2] != 0.0) {
    rt = fabs(e[2]);
    nrm = rt / e[2];
    e[2] = rt;
    s[3] = b_A[15] * nrm;
  }

  if (s[3] != 0.0) {
    s[3] = fabs(s[3]);
  }

  qp1 = 0;
  nrm = fmax(fmax(fmax(fmax(s[0], e[0]), fmax(s[1], e[1])), fmax(s[2], e[2])),
             fmax(s[3], 0.0));
  while ((m + 2 > 0) && (qp1 < 75)) {
    qjj = m + 1;
    do {
      exitg1 = 0;
      qq = qjj;
      if (qjj == 0) {
        exitg1 = 1;
      } else {
        rt = fabs(e[qjj - 1]);
        if ((rt <= (fabs(s[qjj - 1]) + fabs(s[qjj])) * 2.2204460492503131E-16) ||
            ((rt <= 1.0020841800044864E-292) || ((qp1 > 20) && (rt <=
               2.2204460492503131E-16 * nrm)))) {
          e[qjj - 1] = 0.0;
          exitg1 = 1;
        } else {
          qjj--;
        }
      }
    } while (exitg1 == 0);

    if (m + 1 == qjj) {
      qjj = 4;
    } else {
      qs = m + 2;
      qq_tmp = m + 2;
      exitg2 = false;
      while ((!exitg2) && (qq_tmp >= qjj)) {
        qs = qq_tmp;
        if (qq_tmp == qjj) {
          exitg2 = true;
        } else {
          rt = 0.0;
          if (qq_tmp < m + 2) {
            rt = fabs(e[qq_tmp - 1]);
          }

          if (qq_tmp > qjj + 1) {
            rt += fabs(e[qq_tmp - 2]);
          }

          ztest = fabs(s[qq_tmp - 1]);
          if ((ztest <= 2.2204460492503131E-16 * rt) || (ztest <=
               1.0020841800044864E-292)) {
            s[qq_tmp - 1] = 0.0;
            exitg2 = true;
          } else {
            qq_tmp--;
          }
        }
      }

      if (qs == qjj) {
        qjj = 3;
      } else if (m + 2 == qs) {
        qjj = 1;
      } else {
        qjj = 2;
        qq = qs;
      }
    }

    switch (qjj) {
     case 1:
      rt = e[m];
      e[m] = 0.0;
      for (qs = m + 1; qs >= qq + 1; qs--) {
        LR_xrotg(&s[qs - 1], &rt, &ztest, &sqds);
        if (qs > qq + 1) {
          smm1 = e[qs - 2];
          rt = -sqds * smm1;
          e[qs - 2] = smm1 * ztest;
        }
      }
      break;

     case 2:
      rt = e[qq - 1];
      e[qq - 1] = 0.0;
      for (qs = qq + 1; qs <= m + 2; qs++) {
        LR_xrotg(&s[qs - 1], &rt, &ztest, &sqds);
        smm1 = e[qs - 1];
        rt = -sqds * smm1;
        e[qs - 1] = smm1 * ztest;
      }
      break;

     case 3:
      rt = s[m + 1];
      ztest = fmax(fmax(fmax(fmax(fabs(rt), fabs(s[m])), fabs(e[m])), fabs(s[qq])),
                   fabs(e[qq]));
      tmp = _mm_set1_pd(ztest);
      _mm_storeu_pd(&tmp_0[0], _mm_div_pd(_mm_set_pd(s[m], rt), tmp));
      rt = tmp_0[0];
      smm1 = tmp_0[1];
      _mm_storeu_pd(&tmp_0[0], _mm_div_pd(_mm_set_pd(s[qq], e[m]), tmp));
      smm1 = ((smm1 + rt) * (smm1 - rt) + tmp_0[0] * tmp_0[0]) / 2.0;
      sqds = rt * tmp_0[0];
      sqds *= sqds;
      if ((smm1 != 0.0) || (sqds != 0.0)) {
        shift = sqrt(smm1 * smm1 + sqds);
        if (smm1 < 0.0) {
          shift = -shift;
        }

        shift = sqds / (smm1 + shift);
      } else {
        shift = 0.0;
      }

      rt = (tmp_0[1] + rt) * (tmp_0[1] - rt) + shift;
      ztest = e[qq] / ztest * tmp_0[1];
      for (qs = qq + 1; qs <= m + 1; qs++) {
        LR_xrotg(&rt, &ztest, &sqds, &smm1);
        if (qs > qq + 1) {
          e[qs - 2] = rt;
        }

        shift = e[qs - 1];
        rt = s[qs - 1];
        e[qs - 1] = shift * sqds - rt * smm1;
        ztest = smm1 * s[qs];
        s[qs] *= sqds;
        s[qs - 1] = rt * sqds + shift * smm1;
        LR_xrotg(&s[qs - 1], &ztest, &sqds, &smm1);
        ztest = e[qs - 1];
        rt = ztest * sqds + smm1 * s[qs];
        s[qs] = ztest * -smm1 + sqds * s[qs];
        ztest = smm1 * e[qs];
        e[qs] *= sqds;
      }

      e[m] = rt;
      qp1++;
      break;

     default:
      if (s[qq] < 0.0) {
        s[qq] = -s[qq];
      }

      qp1 = qq + 1;
      while ((qq + 1 < 4) && (s[qq] < s[qp1])) {
        rt = s[qq];
        s[qq] = s[qp1];
        s[qp1] = rt;
        qq = qp1;
        qp1++;
      }

      qp1 = 0;
      m--;
      break;
    }
  }

  U[0] = s[0];
  U[1] = s[1];
  U[2] = s[2];
  U[3] = s[3];
  if (doscale) {
    LR_xzlascl_f(cscale, anrm, U);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T tau, real_T C[16],
                      int32_T ic0, real_T work[4])
{
  int32_T coltop;
  int32_T jA;
  int32_T lastc;
  int32_T lastv;
  if (tau != 0.0) {
    boolean_T exitg2;
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C[lastc - 2] == 0.0)) {
      lastv--;
      lastc--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      int32_T exitg1;
      coltop = ((lastc - 1) << 2) + ic0;
      jA = coltop;
      do {
        exitg1 = 0;
        if (jA <= (coltop + lastv) - 1) {
          if (C[jA - 1] != 0.0) {
            exitg1 = 1;
          } else {
            jA++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    real_T c;
    int32_T d;
    int32_T e;
    if (lastc != 0) {
      memset(&work[0], 0, (uint8_T)lastc * sizeof(real_T));
      d = ((lastc - 1) << 2) + ic0;
      for (coltop = ic0; coltop <= d; coltop += 4) {
        c = 0.0;
        e = (coltop + lastv) - 1;
        for (jA = coltop; jA <= e; jA++) {
          c += C[((iv0 + jA) - coltop) - 1] * C[jA - 1];
        }

        jA = (coltop - ic0) >> 2;
        work[jA] += c;
      }
    }

    if (!(-tau == 0.0)) {
      jA = ic0;
      d = (uint8_T)lastc - 1;
      for (lastc = 0; lastc <= d; lastc++) {
        c = work[lastc];
        if (c != 0.0) {
          c *= -tau;
          e = (lastv + jA) - 1;
          for (coltop = jA; coltop <= e; coltop++) {
            C[coltop - 1] += C[((iv0 + coltop) - jA) - 1] * c;
          }
        }

        jA += 4;
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T LR_xnrm2_fq(int32_T n, const real_T x[3])
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[1]);
    } else {
      real_T absxk;
      real_T scale;
      real_T t;
      scale = 3.3121686421112381E-170;
      absxk = fabs(x[1]);
      if (absxk > 3.3121686421112381E-170) {
        y = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        y = t * t;
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

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xdlanv2(real_T *a, real_T *b, real_T *c, real_T *d, real_T *rt1r,
  real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T *sn)
{
  real_T bcmax;
  real_T bcmis;
  real_T p;
  real_T scale;
  real_T temp;
  real_T z;
  int32_T count;
  int32_T tmp;
  if (*c == 0.0) {
    *cs = 1.0;
    *sn = 0.0;
  } else if (*b == 0.0) {
    *cs = 0.0;
    *sn = 1.0;
    temp = *d;
    *d = *a;
    *a = temp;
    *b = -*c;
    *c = 0.0;
  } else {
    temp = *a - *d;
    if ((temp == 0.0) && ((*b < 0.0) != (*c < 0.0))) {
      *cs = 1.0;
      *sn = 0.0;
    } else {
      p = 0.5 * temp;
      bcmis = fabs(*b);
      scale = fabs(*c);
      bcmax = fmax(bcmis, scale);
      if (!(*b < 0.0)) {
        count = 1;
      } else {
        count = -1;
      }

      if (!(*c < 0.0)) {
        tmp = 1;
      } else {
        tmp = -1;
      }

      bcmis = fmin(bcmis, scale) * (real_T)count * (real_T)tmp;
      scale = fmax(fabs(p), bcmax);
      z = p / scale * p + bcmax / scale * bcmis;
      if (z >= 8.8817841970012523E-16) {
        if (!(p < 0.0)) {
          temp = sqrt(scale) * sqrt(z);
        } else {
          temp = -(sqrt(scale) * sqrt(z));
        }

        z = p + temp;
        *a = *d + z;
        *d -= bcmax / z * bcmis;
        bcmax = rt_hypotd_snf(*c, z);
        *cs = z / bcmax;
        *sn = *c / bcmax;
        *b -= *c;
        *c = 0.0;
      } else {
        p = *b + *c;
        scale = fmax(fabs(temp), fabs(p));
        count = 0;
        while ((scale >= 7.4428285367870146E+137) && (count <= 20)) {
          p *= 1.3435752215134178E-138;
          temp *= 1.3435752215134178E-138;
          scale = fmax(fabs(temp), fabs(p));
          count++;
        }

        while ((scale <= 1.3435752215134178E-138) && (count <= 20)) {
          p *= 7.4428285367870146E+137;
          temp *= 7.4428285367870146E+137;
          scale = fmax(fabs(temp), fabs(p));
          count++;
        }

        bcmax = rt_hypotd_snf(p, temp);
        *cs = sqrt((fabs(p) / bcmax + 1.0) * 0.5);
        if (!(p < 0.0)) {
          count = 1;
        } else {
          count = -1;
        }

        *sn = -(0.5 * temp / (bcmax * *cs)) * (real_T)count;
        temp = *a * *cs + *b * *sn;
        p = -*a * *sn + *b * *cs;
        bcmax = *c * *cs + *d * *sn;
        bcmis = -*c * *sn + *d * *cs;
        *b = p * *cs + bcmis * *sn;
        *c = -temp * *sn + bcmax * *cs;
        temp = ((temp * *cs + bcmax * *sn) + (-p * *sn + bcmis * *cs)) * 0.5;
        *a = temp;
        *d = temp;
        if (*c != 0.0) {
          if (*b != 0.0) {
            if ((*b < 0.0) == (*c < 0.0)) {
              scale = sqrt(fabs(*b));
              bcmis = sqrt(fabs(*c));
              p = scale * bcmis;
              if (*c < 0.0) {
                p = -p;
              }

              bcmax = 1.0 / sqrt(fabs(*b + *c));
              *a = temp + p;
              *d = temp - p;
              *b -= *c;
              *c = 0.0;
              p = scale * bcmax;
              bcmax *= bcmis;
              temp = *cs * p - *sn * bcmax;
              *sn = *cs * bcmax + *sn * p;
              *cs = temp;
            }
          } else {
            *b = -*c;
            *c = 0.0;
            temp = *cs;
            *cs = -*sn;
            *sn = temp;
          }
        }
      }
    }
  }

  *rt1r = *a;
  *rt2r = *d;
  if (*c == 0.0) {
    *rt1i = 0.0;
    *rt2i = 0.0;
  } else {
    *rt1i = sqrt(fabs(*b)) * sqrt(fabs(*c));
    *rt2i = -*rt1i;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xrot(int32_T n, real_T x[16], int32_T ix0, int32_T iy0, real_T c,
                    real_T s)
{
  int32_T k;
  if (n >= 1) {
    int32_T b;
    b = (uint8_T)n;
    for (k = 0; k < b; k++) {
      real_T temp_tmp;
      real_T temp_tmp_0;
      int32_T temp_tmp_tmp;
      int32_T temp_tmp_tmp_0;
      temp_tmp_tmp = (iy0 + k) - 1;
      temp_tmp = x[temp_tmp_tmp];
      temp_tmp_tmp_0 = (ix0 + k) - 1;
      temp_tmp_0 = x[temp_tmp_tmp_0];
      x[temp_tmp_tmp] = temp_tmp * c - temp_tmp_0 * s;
      x[temp_tmp_tmp_0] = temp_tmp_0 * c + temp_tmp * s;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static int32_T LR_xhseqr(real_T h[16], real_T z[16])
{
  __m128d tmp;
  real_T v[3];
  real_T aa;
  real_T bb;
  real_T h11;
  real_T h12;
  real_T h21;
  real_T h_0;
  real_T tst;
  real_T tst_tmp_0;
  real_T tst_tmp_tmp;
  int32_T b_j;
  int32_T info;
  int32_T ix0;
  int32_T iy;
  int32_T k;
  int32_T kdefl;
  int32_T knt;
  int32_T l;
  int32_T m;
  int32_T nr;
  int32_T tst_tmp;
  int32_T vectorUB;
  int32_T vectorUB_tmp;
  boolean_T converged;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  info = 0;
  h[3] = 0.0;
  h[2] = 0.0;
  h[7] = 0.0;
  kdefl = 0;
  b_j = 3;
  exitg1 = false;
  while ((!exitg1) && (b_j + 1 >= 1)) {
    l = 1;
    converged = false;
    iy = 0;
    exitg2 = false;
    while ((!exitg2) && (iy < 301)) {
      k = b_j;
      exitg3 = false;
      while ((!exitg3) && (k + 1 > l)) {
        knt = ((k - 1) << 2) + k;
        bb = fabs(h[knt]);
        if (bb <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          tst_tmp = (k << 2) + k;
          tst_tmp_0 = h[knt - 1];
          tst = fabs(tst_tmp_0) + fabs(h[tst_tmp]);
          if (tst == 0.0) {
            if (k - 1 >= 1) {
              tst = fabs(h[(((k - 2) << 2) + k) - 1]);
            }

            if (k + 2 <= 4) {
              tst += fabs(h[tst_tmp + 1]);
            }
          }

          if (bb <= 2.2204460492503131E-16 * tst) {
            tst = fabs(tst_tmp_0 - h[tst_tmp]);
            bb = fabs(h[tst_tmp]);
            aa = fmax(bb, tst);
            bb = fmin(bb, tst);
            tst = aa + bb;
            h11 = fabs(h[knt]);
            h21 = fabs(h[tst_tmp - 1]);
            if (fmax(h11, h21) / tst * fmin(h11, h21) <= fmax
                (4.0083367200179456E-292, aa / tst * bb * 2.2204460492503131E-16))
            {
              exitg3 = true;
            } else {
              k--;
            }
          } else {
            k--;
          }
        }
      }

      l = k + 1;
      if (k + 1 > 1) {
        h[k + ((k - 1) << 2)] = 0.0;
      }

      if (k + 1 >= b_j) {
        converged = true;
        exitg2 = true;
      } else {
        kdefl++;
        if (kdefl - kdefl / 20 * 20 == 0) {
          tst = fabs(h[(((b_j - 2) << 2) + b_j) - 1]) + fabs(h[((b_j - 1) << 2)
            + b_j]);
          h11 = h[(b_j << 2) + b_j] + 0.75 * tst;
          h12 = -0.4375 * tst;
          h21 = tst;
          aa = h11;
        } else if (kdefl - kdefl / 10 * 10 == 0) {
          tst_tmp = (k << 2) + k;
          tst = fabs(h[(((k + 1) << 2) + k) + 2]) + fabs(h[tst_tmp + 1]);
          h11 = 0.75 * tst + h[tst_tmp];
          h12 = -0.4375 * tst;
          h21 = tst;
          aa = h11;
        } else {
          tst_tmp = ((b_j - 1) << 2) + b_j;
          h11 = h[tst_tmp - 1];
          h21 = h[tst_tmp];
          tst_tmp = (b_j << 2) + b_j;
          h12 = h[tst_tmp - 1];
          aa = h[tst_tmp];
        }

        tst = ((fabs(h11) + fabs(h12)) + fabs(h21)) + fabs(aa);
        if (tst == 0.0) {
          h11 = 0.0;
          aa = 0.0;
          bb = 0.0;
          h21 = 0.0;
        } else {
          h11 /= tst;
          aa /= tst;
          bb = (h11 + aa) / 2.0;
          h11 = (h11 - bb) * (aa - bb) - h12 / tst * (h21 / tst);
          h21 = sqrt(fabs(h11));
          if (h11 >= 0.0) {
            h11 = bb * tst;
            bb = h11;
            aa = h21 * tst;
            h21 = -aa;
          } else {
            h11 = bb + h21;
            bb -= h21;
            if (fabs(h11 - aa) <= fabs(bb - aa)) {
              h11 *= tst;
              bb = h11;
            } else {
              bb *= tst;
              h11 = bb;
            }

            aa = 0.0;
            h21 = 0.0;
          }
        }

        m = b_j - 1;
        exitg3 = false;
        while ((!exitg3) && (m >= k + 1)) {
          tst_tmp = ((m - 1) << 2) + m;
          tst_tmp_tmp = h[tst_tmp - 1];
          tst_tmp_0 = tst_tmp_tmp - bb;
          tst = (fabs(tst_tmp_0) + fabs(h21)) + fabs(h[tst_tmp]);
          h12 = h[tst_tmp] / tst;
          nr = (m << 2) + m;
          v[0] = (tst_tmp_0 / tst * tst_tmp_0 + h[nr - 1] * h12) - h21 / tst *
            aa;
          v[1] = (((tst_tmp_tmp + h[nr]) - h11) - bb) * h12;
          v[2] = h[nr + 1] * h12;
          tst = (fabs(v[0]) + fabs(v[1])) + fabs(v[2]);
          tmp = _mm_div_pd(_mm_loadu_pd(&v[0]), _mm_set1_pd(tst));
          _mm_storeu_pd(&v[0], tmp);
          v[2] /= tst;
          if ((k + 1 == m) || (fabs(h[m - 1]) * (fabs(v[1]) + fabs(v[2])) <=
                               ((fabs(h[tst_tmp - 1]) + fabs(h[0])) + fabs(h[nr]))
                               * (2.2204460492503131E-16 * fabs(v[0])))) {
            exitg3 = true;
          } else {
            m--;
          }
        }

        for (tst_tmp = m; tst_tmp <= b_j; tst_tmp++) {
          nr = (b_j - tst_tmp) + 2;
          if (nr >= 3) {
            nr = 3;
          }

          if (tst_tmp > m) {
            ix0 = (((tst_tmp - 2) << 2) + tst_tmp) - 1;
            for (knt = 0; knt < nr; knt++) {
              v[knt] = h[ix0 + knt];
            }
          }

          bb = v[0];
          tst = 0.0;
          if (nr > 0) {
            tst_tmp_0 = LR_xnrm2_fq(nr - 1, v);
            if (tst_tmp_0 != 0.0) {
              aa = rt_hypotd_snf(v[0], tst_tmp_0);
              if (v[0] >= 0.0) {
                aa = -aa;
              }

              if (fabs(aa) < 1.0020841800044864E-292) {
                knt = 0;
                do {
                  knt++;
                  vectorUB = (((nr - 1) / 2) << 1) + 2;
                  vectorUB_tmp = vectorUB - 2;
                  for (ix0 = 2; ix0 <= vectorUB_tmp; ix0 += 2) {
                    tmp = _mm_loadu_pd(&v[ix0 - 1]);
                    _mm_storeu_pd(&v[ix0 - 1], _mm_mul_pd(tmp, _mm_set1_pd
                      (9.9792015476736E+291)));
                  }

                  for (ix0 = vectorUB; ix0 <= nr; ix0++) {
                    v[ix0 - 1] *= 9.9792015476736E+291;
                  }

                  aa *= 9.9792015476736E+291;
                  bb *= 9.9792015476736E+291;
                } while ((fabs(aa) < 1.0020841800044864E-292) && (knt < 20));

                aa = rt_hypotd_snf(bb, LR_xnrm2_fq(nr - 1, v));
                if (bb >= 0.0) {
                  aa = -aa;
                }

                tst = (aa - bb) / aa;
                bb = 1.0 / (bb - aa);
                for (ix0 = 2; ix0 <= vectorUB_tmp; ix0 += 2) {
                  tmp = _mm_loadu_pd(&v[ix0 - 1]);
                  _mm_storeu_pd(&v[ix0 - 1], _mm_mul_pd(tmp, _mm_set1_pd(bb)));
                }

                for (ix0 = vectorUB; ix0 <= nr; ix0++) {
                  v[ix0 - 1] *= bb;
                }

                for (ix0 = 0; ix0 < knt; ix0++) {
                  aa *= 1.0020841800044864E-292;
                }

                bb = aa;
              } else {
                tst = (aa - v[0]) / aa;
                bb = 1.0 / (v[0] - aa);
                ix0 = (((nr - 1) / 2) << 1) + 2;
                vectorUB = ix0 - 2;
                for (knt = 2; knt <= vectorUB; knt += 2) {
                  tmp = _mm_loadu_pd(&v[knt - 1]);
                  _mm_storeu_pd(&v[knt - 1], _mm_mul_pd(tmp, _mm_set1_pd(bb)));
                }

                for (knt = ix0; knt <= nr; knt++) {
                  v[knt - 1] *= bb;
                }

                bb = aa;
              }
            }
          }

          if (tst_tmp > m) {
            knt = ((tst_tmp - 2) << 2) + tst_tmp;
            h[knt - 1] = bb;
            h[knt] = 0.0;
            if (tst_tmp < b_j) {
              h[tst_tmp + 1] = 0.0;
            }
          } else if (m > k + 1) {
            h[tst_tmp - 1] *= 1.0 - tst;
          }

          aa = v[1];
          bb = tst * v[1];
          if (nr == 3) {
            tst_tmp_0 = v[2];
            h12 = tst * v[2];
            for (nr = tst_tmp; nr < 5; nr++) {
              ix0 = ((nr - 1) << 2) + tst_tmp;
              h21 = h[ix0 - 1];
              tst_tmp_tmp = h[ix0];
              h_0 = h[ix0 + 1];
              h11 = (aa * tst_tmp_tmp + h21) + tst_tmp_0 * h_0;
              h[ix0 - 1] = h21 - h11 * tst;
              h[ix0] = tst_tmp_tmp - h11 * bb;
              h[ix0 + 1] = h_0 - h11 * h12;
            }

            if (tst_tmp + 3 <= b_j + 1) {
              knt = tst_tmp + 3;
            } else {
              knt = b_j + 1;
            }

            for (nr = 0; nr < knt; nr++) {
              ix0 = ((tst_tmp - 1) << 2) + nr;
              h21 = h[ix0];
              vectorUB = (tst_tmp << 2) + nr;
              tst_tmp_tmp = h[vectorUB];
              vectorUB_tmp = ((tst_tmp + 1) << 2) + nr;
              h_0 = h[vectorUB_tmp];
              h11 = (aa * tst_tmp_tmp + h21) + tst_tmp_0 * h_0;
              h[ix0] = h21 - h11 * tst;
              h[vectorUB] = tst_tmp_tmp - h11 * bb;
              h[vectorUB_tmp] = h_0 - h11 * h12;
            }

            nr = (tst_tmp - 1) << 2;
            aa = z[nr];
            knt = tst_tmp << 2;
            h21 = z[knt];
            ix0 = (tst_tmp + 1) << 2;
            tst_tmp_tmp = z[ix0];
            h11 = (v[1] * h21 + aa) + v[2] * tst_tmp_tmp;
            z[nr] = aa - h11 * tst;
            z[knt] = h21 - h11 * bb;
            z[ix0] = tst_tmp_tmp - h11 * h12;
            aa = z[nr + 1];
            h21 = z[knt + 1];
            tst_tmp_tmp = z[ix0 + 1];
            h11 = (v[1] * h21 + aa) + v[2] * tst_tmp_tmp;
            z[nr + 1] = aa - h11 * tst;
            z[knt + 1] = h21 - h11 * bb;
            z[ix0 + 1] = tst_tmp_tmp - h11 * h12;
            aa = z[nr + 2];
            h21 = z[knt + 2];
            tst_tmp_tmp = z[ix0 + 2];
            h11 = (v[1] * h21 + aa) + v[2] * tst_tmp_tmp;
            z[nr + 2] = aa - h11 * tst;
            z[knt + 2] = h21 - h11 * bb;
            z[ix0 + 2] = tst_tmp_tmp - h11 * h12;
            aa = z[nr + 3];
            h21 = z[knt + 3];
            tst_tmp_tmp = z[ix0 + 3];
            h11 = (v[1] * h21 + aa) + v[2] * tst_tmp_tmp;
            z[nr + 3] = aa - h11 * tst;
            z[knt + 3] = h21 - h11 * bb;
            z[ix0 + 3] = tst_tmp_tmp - h11 * h12;
          } else if (nr == 2) {
            for (nr = tst_tmp; nr < 5; nr++) {
              ix0 = ((nr - 1) << 2) + tst_tmp;
              h21 = h[ix0 - 1];
              tst_tmp_tmp = h[ix0];
              h11 = aa * tst_tmp_tmp + h21;
              h[ix0 - 1] = h21 - h11 * tst;
              h[ix0] = tst_tmp_tmp - h11 * bb;
            }

            for (nr = 0; nr <= b_j; nr++) {
              ix0 = ((tst_tmp - 1) << 2) + nr;
              h21 = h[ix0];
              vectorUB = (tst_tmp << 2) + nr;
              tst_tmp_tmp = h[vectorUB];
              h11 = aa * tst_tmp_tmp + h21;
              h[ix0] = h21 - h11 * tst;
              h[vectorUB] = tst_tmp_tmp - h11 * bb;
            }

            nr = (tst_tmp - 1) << 2;
            aa = z[nr];
            knt = tst_tmp << 2;
            h21 = z[knt];
            h11 = v[1] * h21 + aa;
            z[nr] = aa - h11 * tst;
            z[knt] = h21 - h11 * bb;
            aa = z[nr + 1];
            h21 = z[knt + 1];
            h11 = v[1] * h21 + aa;
            z[nr + 1] = aa - h11 * tst;
            z[knt + 1] = h21 - h11 * bb;
            aa = z[nr + 2];
            h21 = z[knt + 2];
            h11 = v[1] * h21 + aa;
            z[nr + 2] = aa - h11 * tst;
            z[knt + 2] = h21 - h11 * bb;
            aa = z[nr + 3];
            h21 = z[knt + 3];
            h11 = v[1] * h21 + aa;
            z[nr + 3] = aa - h11 * tst;
            z[knt + 3] = h21 - h11 * bb;
          }
        }

        iy++;
      }
    }

    if (!converged) {
      info = b_j + 1;
      exitg1 = true;
    } else {
      if ((b_j + 1 != l) && (l == b_j)) {
        kdefl = b_j << 2;
        m = kdefl + b_j;
        bb = h[m - 1];
        iy = (b_j - 1) << 2;
        tst_tmp = iy + b_j;
        h11 = h[tst_tmp];
        h21 = h[m];
        LR_xdlanv2(&h[tst_tmp - 1], &bb, &h11, &h21, &h12, &tst_tmp_tmp,
                   &tst_tmp_0, &h_0, &tst, &aa);
        h[m - 1] = bb;
        h[tst_tmp] = h11;
        h[m] = h21;
        if (b_j + 1 < 4) {
          nr = ((b_j + 1) << 2) + b_j;
          k = 3 - b_j;
          for (tst_tmp = 0; tst_tmp < k; tst_tmp++) {
            knt = (tst_tmp << 2) + nr;
            bb = h[knt];
            h11 = h[knt - 1];
            h[knt] = bb * tst - h11 * aa;
            h[knt - 1] = h11 * tst + bb * aa;
          }
        }

        LR_xrot(b_j - 1, h, iy + 1, kdefl + 1, tst, aa);
        LR_xrot(4, z, iy + 1, kdefl + 1, tst, aa);
      }

      kdefl = 0;
      b_j = l - 2;
    }
  }

  for (b_j = 0; b_j < 2; b_j++) {
    for (l = b_j + 3; l < 5; l++) {
      h[(l + (b_j << 2)) - 1] = 0.0;
    }
  }

  return info;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_schur_o(real_T A[16], real_T V[16])
{
  __m128d tmp;
  __m128d tmp_0;
  real_T work[4];
  real_T tau[3];
  real_T alpha1_tmp;
  real_T beta1;
  int32_T b_ix;
  int32_T exitg1;
  int32_T im1n_tmp;
  int32_T im1n_tmp_0;
  int32_T in;
  int32_T itau;
  int32_T jy;
  int32_T knt;
  int32_T lastv;
  int32_T rowleft;
  int32_T vectorUB;
  int32_T vectorUB_tmp;
  int32_T work_tmp;
  boolean_T exitg2;
  boolean_T p;
  p = true;
  for (itau = 0; itau < 16; itau++) {
    if (p && ((!rtIsInf(A[itau])) && (!rtIsNaN(A[itau])))) {
    } else {
      p = false;
    }
  }

  if (!p) {
    for (lastv = 0; lastv < 16; lastv++) {
      V[lastv] = (rtNaN);
    }

    itau = 2;
    for (jy = 0; jy < 3; jy++) {
      if (itau <= 4) {
        memset(&V[((jy << 2) + itau) + -1], 0, (uint32_T)(-itau + 5) * sizeof
               (real_T));
      }

      itau++;
    }

    for (lastv = 0; lastv < 16; lastv++) {
      A[lastv] = (rtNaN);
    }
  } else {
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (itau = 0; itau < 3; itau++) {
      im1n_tmp = itau << 2;
      in = (itau + 1) << 2;
      im1n_tmp_0 = im1n_tmp + itau;
      alpha1_tmp = A[im1n_tmp_0 + 1];
      if (itau + 3 <= 4) {
        lastv = itau + 3;
      } else {
        lastv = 4;
      }

      lastv += im1n_tmp;
      tau[itau] = 0.0;
      beta1 = LR_xnrm2(2 - itau, A, lastv);
      if (beta1 != 0.0) {
        beta1 = rt_hypotd_snf(alpha1_tmp, beta1);
        if (alpha1_tmp >= 0.0) {
          beta1 = -beta1;
        }

        if (fabs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          do {
            knt++;
            b_ix = (lastv - itau) + 1;
            jy = ((((b_ix - lastv) + 1) / 2) << 1) + lastv;
            vectorUB_tmp = jy - 2;
            for (rowleft = lastv; rowleft <= vectorUB_tmp; rowleft += 2) {
              tmp_0 = _mm_loadu_pd(&A[rowleft - 1]);
              _mm_storeu_pd(&A[rowleft - 1], _mm_mul_pd(tmp_0, _mm_set1_pd
                (9.9792015476736E+291)));
            }

            for (rowleft = jy; rowleft <= b_ix; rowleft++) {
              A[rowleft - 1] *= 9.9792015476736E+291;
            }

            beta1 *= 9.9792015476736E+291;
            alpha1_tmp *= 9.9792015476736E+291;
          } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

          beta1 = rt_hypotd_snf(alpha1_tmp, LR_xnrm2(2 - itau, A, lastv));
          if (alpha1_tmp >= 0.0) {
            beta1 = -beta1;
          }

          tau[itau] = (beta1 - alpha1_tmp) / beta1;
          alpha1_tmp = 1.0 / (alpha1_tmp - beta1);
          for (rowleft = lastv; rowleft <= vectorUB_tmp; rowleft += 2) {
            tmp_0 = _mm_loadu_pd(&A[rowleft - 1]);
            _mm_storeu_pd(&A[rowleft - 1], _mm_mul_pd(tmp_0, _mm_set1_pd
              (alpha1_tmp)));
          }

          for (rowleft = jy; rowleft <= b_ix; rowleft++) {
            A[rowleft - 1] *= alpha1_tmp;
          }

          for (lastv = 0; lastv < knt; lastv++) {
            beta1 *= 1.0020841800044864E-292;
          }

          alpha1_tmp = beta1;
        } else {
          tau[itau] = (beta1 - alpha1_tmp) / beta1;
          alpha1_tmp = 1.0 / (alpha1_tmp - beta1);
          knt = (lastv - itau) + 1;
          b_ix = ((((knt - lastv) + 1) / 2) << 1) + lastv;
          vectorUB = b_ix - 2;
          for (jy = lastv; jy <= vectorUB; jy += 2) {
            tmp_0 = _mm_loadu_pd(&A[jy - 1]);
            _mm_storeu_pd(&A[jy - 1], _mm_mul_pd(tmp_0, _mm_set1_pd(alpha1_tmp)));
          }

          for (jy = b_ix; jy <= knt; jy++) {
            A[jy - 1] *= alpha1_tmp;
          }

          alpha1_tmp = beta1;
        }
      }

      A[im1n_tmp_0 + 1] = 1.0;
      jy = im1n_tmp_0 + 1;
      if (tau[itau] != 0.0) {
        lastv = 2 - itau;
        knt = (im1n_tmp_0 - itau) + 3;
        while ((lastv + 1 > 0) && (A[knt] == 0.0)) {
          lastv--;
          knt--;
        }

        knt = 4;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          rowleft = in + knt;
          im1n_tmp = rowleft;
          do {
            exitg1 = 0;
            if (im1n_tmp <= (lastv << 2) + rowleft) {
              if (A[im1n_tmp - 1] != 0.0) {
                exitg1 = 1;
              } else {
                im1n_tmp += 4;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = -1;
        knt = 0;
      }

      if (lastv + 1 > 0) {
        if (knt != 0) {
          memset(&work[0], 0, (uint32_T)knt * sizeof(real_T));
          rowleft = im1n_tmp_0 + 1;
          im1n_tmp = ((lastv << 2) + in) + 1;
          for (b_ix = in + 1; b_ix <= im1n_tmp; b_ix += 4) {
            vectorUB_tmp = (b_ix + knt) - 1;
            for (vectorUB = b_ix; vectorUB <= vectorUB_tmp; vectorUB++) {
              work_tmp = vectorUB - b_ix;
              work[work_tmp] += A[vectorUB - 1] * A[rowleft];
            }

            rowleft++;
          }
        }

        if (!(-tau[itau] == 0.0)) {
          rowleft = in + 1;
          for (im1n_tmp = 0; im1n_tmp <= lastv; im1n_tmp++) {
            if (A[jy] != 0.0) {
              beta1 = A[jy] * -tau[itau];
              work_tmp = (knt + rowleft) - 1;
              b_ix = ((((work_tmp - rowleft) + 1) / 2) << 1) + rowleft;
              vectorUB = b_ix - 2;
              for (vectorUB_tmp = rowleft; vectorUB_tmp <= vectorUB;
                   vectorUB_tmp += 2) {
                tmp_0 = _mm_loadu_pd(&work[vectorUB_tmp - rowleft]);
                tmp = _mm_loadu_pd(&A[vectorUB_tmp - 1]);
                _mm_storeu_pd(&A[vectorUB_tmp - 1], _mm_add_pd(_mm_mul_pd(tmp_0,
                  _mm_set1_pd(beta1)), tmp));
              }

              for (vectorUB_tmp = b_ix; vectorUB_tmp <= work_tmp; vectorUB_tmp++)
              {
                A[vectorUB_tmp - 1] += work[vectorUB_tmp - rowleft] * beta1;
              }
            }

            jy++;
            rowleft += 4;
          }
        }
      }

      LR_xzlarf(3 - itau, 3 - itau, im1n_tmp_0 + 2, tau[itau], A, (itau + in) +
                2, work);
      A[im1n_tmp_0 + 1] = alpha1_tmp;
    }

    memcpy(&V[0], &A[0], sizeof(real_T) << 4U);
    for (itau = 2; itau >= 0; itau--) {
      jy = (itau + 1) << 2;
      for (in = 0; in <= itau; in++) {
        V[jy + in] = 0.0;
      }

      for (in = itau + 3; in < 5; in++) {
        lastv = jy + in;
        V[lastv - 1] = V[lastv - 5];
      }
    }

    V[1] = 0.0;
    V[2] = 0.0;
    V[3] = 0.0;
    V[0] = 1.0;
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (jy = 2; jy >= 0; jy--) {
      in = ((jy << 2) + jy) + 5;
      if (jy + 1 < 3) {
        V[in] = 1.0;
        LR_xzlarf(3 - jy, 2 - jy, in + 1, tau[jy], V, in + 5, work);
        lastv = (in - jy) + 3;
        b_ix = (((((lastv - in) - 1) / 2) << 1) + in) + 2;
        vectorUB = b_ix - 2;
        for (knt = in + 2; knt <= vectorUB; knt += 2) {
          tmp_0 = _mm_loadu_pd(&V[knt - 1]);
          _mm_storeu_pd(&V[knt - 1], _mm_mul_pd(tmp_0, _mm_set1_pd(-tau[jy])));
        }

        for (knt = b_ix; knt <= lastv; knt++) {
          V[knt - 1] *= -tau[jy];
        }
      }

      V[in] = 1.0 - tau[jy];
      for (lastv = 0; lastv < jy; lastv++) {
        V[(in - lastv) - 1] = 0.0;
      }
    }

    LR_xhseqr(A, V);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T LR_xnrm2_fqh(int32_T n, const real_T x[4], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      scale = 3.3121686421112381E-170;
      for (k = ix0; k <= ix0 + 1; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xzgehrd(real_T a[4], int32_T ilo, int32_T ihi)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T work[2];
  real_T alpha1;
  real_T tau;
  real_T xnorm;
  int32_T b_ia;
  int32_T d;
  int32_T exitg1;
  int32_T i;
  int32_T ix;
  int32_T knt;
  int32_T lastc;
  boolean_T exitg2;
  if ((ihi - ilo) + 1 > 1) {
    work[0] = 0.0;
    work[1] = 0.0;
    for (i = ilo; i < ihi; i++) {
      alpha1 = a[1];
      tau = 0.0;
      if (ihi - 1 > 0) {
        xnorm = LR_xnrm2_fqh(0, a, 2);
        if (xnorm != 0.0) {
          xnorm = rt_hypotd_snf(a[1], xnorm);
          if (a[1] >= 0.0) {
            xnorm = -xnorm;
          }

          if (fabs(xnorm) < 1.0020841800044864E-292) {
            knt = 0;
            do {
              knt++;
              xnorm *= 9.9792015476736E+291;
              alpha1 *= 9.9792015476736E+291;
            } while ((fabs(xnorm) < 1.0020841800044864E-292) && (knt < 20));

            xnorm = rt_hypotd_snf(alpha1, LR_xnrm2_fqh(0, a, 2));
            if (alpha1 >= 0.0) {
              xnorm = -xnorm;
            }

            tau = (xnorm - alpha1) / xnorm;
            for (lastc = 0; lastc < knt; lastc++) {
              xnorm *= 1.0020841800044864E-292;
            }

            alpha1 = xnorm;
          } else {
            tau = (xnorm - a[1]) / xnorm;
            alpha1 = xnorm;
          }
        }
      }

      a[1] = 1.0;
      if (tau != 0.0) {
        knt = ihi - 1;
        lastc = ihi;
        while ((knt > 0) && (a[lastc - 1] == 0.0)) {
          knt = 0;
          lastc--;
        }

        lastc = ihi;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          b_ia = lastc + 1;
          do {
            exitg1 = 0;
            if (b_ia + 1 <= (((knt - 1) << 1) + lastc) + 2) {
              if (a[b_ia] != 0.0) {
                exitg1 = 1;
              } else {
                b_ia += 2;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        knt = 0;
        lastc = 0;
      }

      if (knt > 0) {
        if (lastc != 0) {
          memset(&work[0], 0, (uint8_T)lastc * sizeof(real_T));
          knt = ((lastc / 2) << 1) + 3;
          ix = knt - 2;
          for (b_ia = 3; b_ia <= ix; b_ia += 2) {
            tmp = _mm_loadu_pd(&a[b_ia - 1]);
            tmp_0 = _mm_loadu_pd(&work[b_ia - 3]);
            _mm_storeu_pd(&work[b_ia - 3], _mm_add_pd(_mm_mul_pd(tmp,
              _mm_set1_pd(a[1])), tmp_0));
          }

          for (b_ia = knt; b_ia <= lastc + 2; b_ia++) {
            work[b_ia - 3] += a[b_ia - 1] * a[1];
          }
        }

        if ((!(-tau == 0.0)) && (a[1] != 0.0)) {
          xnorm = a[1] * -tau;
          d = lastc + 2;
          knt = ((lastc / 2) << 1) + 3;
          ix = knt - 2;
          for (b_ia = 3; b_ia <= ix; b_ia += 2) {
            tmp = _mm_loadu_pd(&work[b_ia - 3]);
            tmp_0 = _mm_loadu_pd(&a[b_ia - 1]);
            _mm_storeu_pd(&a[b_ia - 1], _mm_add_pd(_mm_mul_pd(tmp, _mm_set1_pd
              (xnorm)), tmp_0));
          }

          for (b_ia = knt; b_ia <= d; b_ia++) {
            a[b_ia - 1] += work[b_ia - 3] * xnorm;
          }
        }
      }

      if (tau != 0.0) {
        knt = ihi - 1;
        lastc = ihi;
        while ((knt > 0) && (a[lastc - 1] == 0.0)) {
          knt = 0;
          lastc--;
        }

        lastc = 1;
        b_ia = 4;
        do {
          exitg1 = 0;
          if (b_ia <= knt + 3) {
            if (a[3] != 0.0) {
              exitg1 = 1;
            } else {
              b_ia++;
            }
          } else {
            lastc = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      } else {
        knt = 0;
        lastc = 0;
      }

      if (knt > 0) {
        if (lastc != 0) {
          work[0] = a[1] * a[3];
        }

        if (!(-tau == 0.0)) {
          ix = 4;
          for (knt = 0; knt < lastc; knt++) {
            if (work[0] != 0.0) {
              xnorm = work[0] * -tau;
              for (b_ia = ix; b_ia <= ix; b_ia++) {
                a[b_ia - 1] += a[(b_ia - ix) + 1] * xnorm;
              }
            }

            ix += 2;
          }
        }
      }

      a[1] = alpha1;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xdlahqr(int32_T ilo, int32_T ihi, real_T h[4], real_T *z, int32_T
  *info, real_T wr[2], real_T wi[2])
{
  real_T aa;
  real_T aa_tmp;
  real_T bb;
  real_T c;
  real_T cs;
  real_T s;
  real_T smlnum;
  real_T sn;
  int32_T aa_tmp_0;
  int32_T b_i;
  int32_T bb_tmp;
  int32_T k;
  boolean_T exitg1;
  *z = 1.0;
  *info = 0;
  k = (uint8_T)(ilo - 1);
  for (b_i = 0; b_i < k; b_i++) {
    wr[b_i] = h[(b_i << 1) + b_i];
    wi[b_i] = 0.0;
  }

  if (ihi + 1 <= 2) {
    wr[1] = h[3];
    wi[1] = 0.0;
  }

  if (ilo == ihi) {
    wr[ilo - 1] = h[(((ilo - 1) << 1) + ilo) - 1];
    wi[ilo - 1] = 0.0;
  } else {
    smlnum = (real_T)((ihi - ilo) + 1) / 2.2204460492503131E-16 *
      2.2250738585072014E-308;
    for (b_i = ihi - 1; b_i + 1 >= ilo; b_i = k - 2) {
      k = b_i + 1;
      exitg1 = false;
      while ((!exitg1) && (k > ilo)) {
        aa_tmp = fabs(h[1]);
        if (aa_tmp <= smlnum) {
          exitg1 = true;
        } else {
          bb = fabs(h[3]);
          if (aa_tmp <= (fabs(h[0]) + bb) * 2.2204460492503131E-16) {
            aa_tmp = fabs(h[0] - h[3]);
            aa = fmax(bb, aa_tmp);
            bb = fmin(bb, aa_tmp);
            s = aa + bb;
            aa_tmp = fabs(h[1]);
            c = fabs(h[2]);
            if (fmax(aa_tmp, c) / s * fmin(aa_tmp, c) <= fmax(smlnum, aa / s *
                 bb * 2.2204460492503131E-16)) {
              exitg1 = true;
            } else {
              k = 1;
            }
          } else {
            k = 1;
          }
        }
      }

      if (k > ilo) {
        h[1] = 0.0;
      }

      if (b_i + 1 == k) {
        wr[b_i] = h[(b_i << 1) + b_i];
        wi[b_i] = 0.0;
      } else if (k == b_i) {
        aa_tmp_0 = b_i << 1;
        aa = h[aa_tmp_0];
        aa_tmp = h[b_i];
        bb_tmp = aa_tmp_0 + b_i;
        bb = h[bb_tmp];
        LR_xdlanv2(&h[0], &aa, &aa_tmp, &bb, &wr[0], &wi[0], &s, &c, &cs, &sn);
        wr[b_i] = s;
        wi[b_i] = c;
        h[aa_tmp_0] = aa;
        h[b_i] = aa_tmp;
        h[bb_tmp] = bb;
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xzlascl_f2p(real_T cfrom, real_T cto, int32_T m, real_T A[2],
  int32_T iA0)
{
  real_T cfromc;
  real_T ctoc;
  int32_T i;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real_T cfrom1;
    real_T cto1;
    real_T mul;
    int32_T b;
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    b = (uint8_T)m;
    scalarLB = ((uint8_T)m / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      __m128d tmp;
      tmp_0 = (iA0 + i) - 1;
      tmp = _mm_loadu_pd(&A[tmp_0]);
      _mm_storeu_pd(&A[tmp_0], _mm_mul_pd(tmp, _mm_set1_pd(mul)));
    }

    for (i = scalarLB; i < b; i++) {
      tmp_0 = (iA0 + i) - 1;
      A[tmp_0] *= mul;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_eigStandard(const real_T A[4], creal_T V[2])
{
  __m128d tmp;
  real_T b_A[4];
  real_T wi[2];
  real_T wr[2];
  real_T absxk;
  real_T anrm;
  real_T b_absxk;
  real_T c;
  real_T g;
  real_T r;
  real_T s;
  real_T scale;
  real_T t;
  int32_T b_A_tmp;
  int32_T b_k;
  int32_T c_tmp;
  int32_T c_tmp_0;
  int32_T exitg2;
  int32_T exitg3;
  int32_T exitg4;
  int32_T exitg5;
  int32_T f_ix;
  int32_T ica;
  int32_T ica_tmp;
  int32_T ix;
  int32_T l;
  boolean_T exitg1;
  boolean_T exitg6;
  boolean_T exitg7;
  boolean_T notdone;
  boolean_T scalea;
  boolean_T skipThisColumn;
  b_A[0] = A[0];
  b_A[1] = A[1];
  b_A[2] = A[2];
  b_A[3] = A[3];
  anrm = 0.0;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 4)) {
    absxk = fabs(A[b_k]);
    if (rtIsNaN(absxk)) {
      anrm = (rtNaN);
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }

      b_k++;
    }
  }

  if (rtIsInf(anrm) || rtIsNaN(anrm)) {
    V[0].re = (rtNaN);
    V[0].im = 0.0;
    V[1].re = (rtNaN);
    V[1].im = 0.0;
  } else {
    absxk = anrm;
    scalea = false;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      scalea = true;
      absxk = 6.7178761075670888E-139;
      LR_xzlascl_f(anrm, absxk, b_A);
    } else if (anrm > 1.4885657073574029E+138) {
      scalea = true;
      absxk = 1.4885657073574029E+138;
      LR_xzlascl_f(anrm, absxk, b_A);
    }

    wr[0] = 1.0;
    wr[1] = 1.0;
    b_k = 0;
    l = 2;
    notdone = true;
    do {
      exitg5 = 0;
      if (notdone) {
        notdone = false;
        f_ix = l;
        do {
          exitg4 = 0;
          if (f_ix > 0) {
            skipThisColumn = false;
            ica = 0;
            exitg6 = false;
            while ((!exitg6) && (ica <= l - 1)) {
              if ((ica + 1 == f_ix) || (!(b_A[((ica << 1) + f_ix) - 1] != 0.0)))
              {
                ica++;
              } else {
                skipThisColumn = true;
                exitg6 = true;
              }
            }

            if (skipThisColumn) {
              f_ix--;
            } else {
              wr[l - 1] = f_ix;
              if (f_ix != l) {
                ica = (f_ix - 1) << 1;
                ix = (l - 1) << 1;
                for (ica_tmp = 0; ica_tmp < l; ica_tmp++) {
                  c_tmp = ica + ica_tmp;
                  c = b_A[c_tmp];
                  b_A_tmp = ix + ica_tmp;
                  b_A[c_tmp] = b_A[b_A_tmp];
                  b_A[b_A_tmp] = c;
                }

                c = b_A[f_ix - 1];
                b_A[f_ix - 1] = b_A[l - 1];
                b_A[l - 1] = c;
                c = b_A[f_ix + 1];
                b_A[f_ix + 1] = b_A[l + 1];
                b_A[l + 1] = c;
              }

              exitg4 = 1;
            }
          } else {
            exitg4 = 2;
          }
        } while (exitg4 == 0);

        if (exitg4 == 1) {
          if (l == 1) {
            exitg5 = 1;
          } else {
            l = 1;
            notdone = true;
          }
        }
      } else {
        notdone = true;
        while (notdone) {
          notdone = false;
          f_ix = b_k + 1;
          exitg6 = false;
          while ((!exitg6) && (f_ix <= l)) {
            skipThisColumn = false;
            ica = b_k + 1;
            exitg7 = false;
            while ((!exitg7) && (ica <= l)) {
              if ((ica == f_ix) || (!(b_A[(((f_ix - 1) << 1) + ica) - 1] != 0.0)))
              {
                ica++;
              } else {
                skipThisColumn = true;
                exitg7 = true;
              }
            }

            if (skipThisColumn) {
              f_ix++;
            } else {
              wr[b_k] = f_ix;
              if (b_k + 1 != f_ix) {
                ica = (f_ix - 1) << 1;
                c_tmp_0 = b_k << 1;
                for (ica_tmp = 0; ica_tmp < l; ica_tmp++) {
                  c_tmp = ica + ica_tmp;
                  c = b_A[c_tmp];
                  b_A_tmp = c_tmp_0 + ica_tmp;
                  b_A[c_tmp] = b_A[b_A_tmp];
                  b_A[b_A_tmp] = c;
                }

                f_ix = (c_tmp_0 + f_ix) - 1;
                ica = c_tmp_0 + b_k;
                ix = (uint8_T)(2 - b_k);
                for (ica_tmp = 0; ica_tmp < ix; ica_tmp++) {
                  c_tmp = ica_tmp << 1;
                  c_tmp_0 = c_tmp + f_ix;
                  c = b_A[c_tmp_0];
                  b_A_tmp = c_tmp + ica;
                  b_A[c_tmp_0] = b_A[b_A_tmp];
                  b_A[b_A_tmp] = c;
                }
              }

              b_k++;
              notdone = true;
              exitg6 = true;
            }
          }
        }

        exitg5 = 2;
      }
    } while (exitg5 == 0);

    if (exitg5 == 1) {
    } else {
      exitg1 = false;
      while ((!exitg1) && (!notdone)) {
        notdone = true;
        f_ix = b_k;
        do {
          exitg3 = 0;
          if (f_ix + 1 <= l) {
            c_tmp = l - b_k;
            c_tmp_0 = f_ix << 1;
            c = LR_xnrm2_fqh(c_tmp, b_A, (c_tmp_0 + b_k) + 1);
            ica_tmp = (b_k << 1) + f_ix;
            r = 0.0;
            if (c_tmp >= 1) {
              if (c_tmp == 1) {
                r = fabs(b_A[ica_tmp]);
              } else {
                scale = 3.3121686421112381E-170;
                for (ix = ica_tmp + 1; ix <= ica_tmp + 3; ix += 2) {
                  b_absxk = fabs(b_A[ix - 1]);
                  if (b_absxk > scale) {
                    t = scale / b_absxk;
                    r = r * t * t + 1.0;
                    scale = b_absxk;
                  } else {
                    t = b_absxk / scale;
                    r += t * t;
                  }
                }

                r = scale * sqrt(r);
              }
            }

            ica = 0;
            if ((l > 1) && (fabs(b_A[c_tmp_0 + 1]) > fabs(b_A[c_tmp_0]))) {
              ica = 1;
            }

            scale = fabs(b_A[c_tmp_0 + ica]);
            if (2 - b_k < 1) {
              ica = -2;
            } else {
              ica = -1;
              if ((2 - b_k > 1) && (fabs(b_A[ica_tmp + 2]) > fabs(b_A[ica_tmp])))
              {
                ica = 0;
              }
            }

            t = fabs(b_A[(((ica + b_k) + 1) << 1) + f_ix]);
            if ((c == 0.0) || (r == 0.0)) {
              f_ix++;
            } else {
              g = r / 2.0;
              b_absxk = 1.0;
              s = c + r;
              do {
                exitg2 = 0;
                if ((c < g) && (fmax(b_absxk, fmax(c, scale)) <
                                4.9896007738368E+291) && (fmin(r, fmin(g, t)) >
                     2.0041683600089728E-292)) {
                  if (rtIsNaN(((((c + b_absxk) + scale) + r) + g) + t)) {
                    exitg2 = 1;
                  } else {
                    b_absxk *= 2.0;
                    c *= 2.0;
                    scale *= 2.0;
                    r /= 2.0;
                    g /= 2.0;
                    t /= 2.0;
                  }
                } else {
                  g = c / 2.0;
                  while ((g >= r) && (fmax(r, t) < 4.9896007738368E+291) &&
                         (fmin(fmin(b_absxk, c), fmin(g, scale)) >
                          2.0041683600089728E-292)) {
                    b_absxk /= 2.0;
                    c /= 2.0;
                    g /= 2.0;
                    scale /= 2.0;
                    r *= 2.0;
                    t *= 2.0;
                  }

                  if ((c + r >= 0.95 * s) || ((b_absxk < 1.0) && (wr[f_ix] < 1.0)
                       && (b_absxk * wr[f_ix] <= 1.0020841800044864E-292)) ||
                      ((b_absxk > 1.0) && (wr[f_ix] > 1.0) && (wr[f_ix] >=
                        9.9792015476736E+291 / b_absxk))) {
                  } else {
                    c = 1.0 / b_absxk;
                    wr[f_ix] *= b_absxk;
                    ica = ica_tmp + 1;
                    ix = (((1 - b_k) << 1) + ica_tmp) + 1;
                    for (ica_tmp = ica; ica_tmp <= ix; ica_tmp += 2) {
                      b_A[ica_tmp - 1] *= c;
                    }

                    ix = c_tmp_0 + l;
                    c_tmp = ((((ix - c_tmp_0) / 2) << 1) + c_tmp_0) + 1;
                    ica = c_tmp - 2;
                    for (ica_tmp = c_tmp_0 + 1; ica_tmp <= ica; ica_tmp += 2) {
                      tmp = _mm_loadu_pd(&b_A[ica_tmp - 1]);
                      _mm_storeu_pd(&b_A[ica_tmp - 1], _mm_mul_pd(tmp,
                        _mm_set1_pd(b_absxk)));
                    }

                    for (ica_tmp = c_tmp; ica_tmp <= ix; ica_tmp++) {
                      b_A[ica_tmp - 1] *= b_absxk;
                    }

                    notdone = false;
                  }

                  exitg2 = 2;
                }
              } while (exitg2 == 0);

              if (exitg2 == 1) {
                exitg3 = 2;
              } else {
                f_ix++;
              }
            }
          } else {
            exitg3 = 1;
          }
        } while (exitg3 == 0);

        if (exitg3 == 1) {
        } else {
          exitg1 = true;
        }
      }
    }

    LR_xzgehrd(b_A, b_k + 1, l);
    LR_xdlahqr(b_k + 1, l, b_A, &c, &f_ix, wr, wi);
    if (scalea) {
      LR_xzlascl_f2p(absxk, anrm, 2 - f_ix, wr, f_ix + 1);
      LR_xzlascl_f2p(absxk, anrm, 2 - f_ix, wi, f_ix + 1);
      if (f_ix != 0) {
        LR_xzlascl_f2p(absxk, anrm, b_k, wr, 1);
        LR_xzlascl_f2p(absxk, anrm, b_k, wi, 1);
      }
    }

    if (f_ix != 0) {
      for (l = b_k + 1; l <= f_ix; l++) {
        wr[l - 1] = (rtNaN);
        wi[l - 1] = 0.0;
      }
    }

    V[0].re = wr[0];
    V[0].im = wi[0];
    V[1].re = wr[1];
    V[1].im = wi[1];
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xzlascl_f2pa(real_T cfrom, real_T cto, int32_T m, real_T *A)
{
  real_T cfromc;
  real_T ctoc;
  int32_T i;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real_T cfrom1;
    real_T cto1;
    real_T mul;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    for (i = 0; i < m; i++) {
      *A *= mul;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xdlaev2(real_T a, real_T b, real_T c, real_T *rt1, real_T *rt2)
{
  real_T ab;
  real_T acmn;
  real_T acmx;
  real_T adf;
  real_T sm;
  sm = a + c;
  adf = fabs(a - c);
  ab = fabs(b + b);
  if (fabs(a) > fabs(c)) {
    acmx = a;
    acmn = c;
  } else {
    acmx = c;
    acmn = a;
  }

  if (adf > ab) {
    ab /= adf;
    adf *= sqrt(ab * ab + 1.0);
  } else if (adf < ab) {
    adf /= ab;
    adf = sqrt(adf * adf + 1.0) * ab;
  } else {
    adf = ab * 1.4142135623730951;
  }

  if (sm < 0.0) {
    *rt1 = (sm - adf) * 0.5;
    *rt2 = acmx / *rt1 * acmn - b / *rt1 * b;
  } else if (sm > 0.0) {
    *rt1 = (sm + adf) * 0.5;
    *rt2 = acmx / *rt1 * acmn - b / *rt1 * b;
  } else {
    *rt1 = 0.5 * adf;
    *rt2 = -0.5 * adf;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_insertionsort(real_T x[2], int32_T xstart, int32_T xend)
{
  int32_T k;
  for (k = xstart + 1; k <= xend; k++) {
    real_T xc;
    int32_T idx;
    xc = x[1];
    idx = 1;
    while ((idx >= xstart) && (xc < x[0])) {
      x[1] = x[0];
      idx = 0;
    }

    x[idx] = xc;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static int32_T LR_xdsterf(real_T d[2], real_T e)
{
  real_T anorm;
  real_T b_anorm;
  real_T b_gamma;
  real_T oldc;
  real_T p;
  real_T rte;
  real_T s;
  int32_T d_i;
  int32_T exitg1;
  int32_T exitg3;
  int32_T exitg4;
  int32_T info;
  int32_T iscale;
  int32_T iscale_tmp;
  int32_T jtot;
  int32_T l;
  int32_T l1;
  int32_T lend;
  int32_T lendsv_tmp_tmp;
  int32_T lsv;
  int32_T m;
  boolean_T exitg2;
  info = 0;
  jtot = 0;
  l1 = 1;
  do {
    exitg1 = 0;
    if (l1 > 2) {
      LR_insertionsort(d, 1, 2);
      exitg1 = 1;
    } else {
      if (l1 > 1) {
        e = 0.0;
      }

      m = l1;
      exitg2 = false;
      while ((!exitg2) && (m < 2)) {
        if (fabs(e) <= sqrt(fabs(d[0])) * sqrt(fabs(d[1])) *
            2.2204460492503131E-16) {
          e = 0.0;
          exitg2 = true;
        } else {
          m = 2;
        }
      }

      l = l1;
      lsv = l1;
      lend = m;
      lendsv_tmp_tmp = m + 1;
      l1 = m + 1;
      if (m == l) {
      } else {
        iscale_tmp = m - l;
        if (iscale_tmp + 1 <= 0) {
          anorm = 0.0;
        } else {
          anorm = fabs(d[(l + iscale_tmp) - 1]);
          d_i = 0;
          exitg2 = false;
          while ((!exitg2) && (d_i <= iscale_tmp - 1)) {
            b_anorm = fabs(d[l - 1]);
            if (rtIsNaN(b_anorm)) {
              anorm = (rtNaN);
              exitg2 = true;
            } else {
              if (b_anorm > anorm) {
                anorm = b_anorm;
              }

              b_anorm = fabs(e);
              if (rtIsNaN(b_anorm)) {
                anorm = (rtNaN);
                exitg2 = true;
              } else {
                if (b_anorm > anorm) {
                  anorm = b_anorm;
                }

                d_i = 1;
              }
            }
          }
        }

        if (anorm == 0.0) {
        } else {
          iscale = 0;
          if (anorm > 2.2346346549904327E+153) {
            iscale = 1;
            LR_xzlascl_f2p(anorm, 2.2346346549904327E+153, iscale_tmp + 1, d, l);
            LR_xzlascl_f2pa(anorm, 2.2346346549904327E+153, iscale_tmp, &e);
          } else if (anorm < 3.02546243347603E-123) {
            iscale = 2;
            LR_xzlascl_f2p(anorm, 3.02546243347603E-123, iscale_tmp + 1, d, l);
            LR_xzlascl_f2pa(anorm, 3.02546243347603E-123, iscale_tmp, &e);
          }

          for (d_i = l; d_i < m; d_i++) {
            e *= e;
          }

          if (fabs(d[m - 1]) < fabs(d[l - 1])) {
            lend = lsv;
            l = m;
          }

          if (lend >= l) {
            do {
              exitg4 = 0;
              if (l != lend) {
                m = l;
                while ((m < lend) && (!(fabs(e) <= 4.9303806576313238E-32 * fabs
                                        (d[0]) * fabs(d[1])))) {
                  m = 2;
                }
              } else {
                m = lend;
              }

              if (m < lend) {
                e = 0.0;
              }

              if (m == l) {
                l++;
                if (l > lend) {
                  exitg4 = 1;
                }
              } else if (l + 1 == m) {
                LR_xdlaev2(d[0], sqrt(e), d[1], &d[0], &d[1]);
                e = 0.0;
                exitg4 = 1;
              } else if (jtot == 60) {
                exitg4 = 1;
              } else {
                jtot++;
                rte = sqrt(e);
                s = d[l - 1];
                b_anorm = (d[1] - s) / (2.0 * rte);
                b_gamma = rt_hypotd_snf(b_anorm, 1.0);
                if (!(b_anorm >= 0.0)) {
                  b_gamma = -b_gamma;
                }

                b_anorm = s - rte / (b_anorm + b_gamma);
                rte = 1.0;
                s = 0.0;
                b_gamma = d[m - 1] - b_anorm;
                p = b_gamma * b_gamma;
                for (d_i = m - 1; d_i >= 1; d_i--) {
                  s = p + e;
                  if (m - 1 != 1) {
                    /* Check node always fails. would cause program termination and was eliminated */
                  }

                  oldc = rte;
                  rte = p / s;
                  s = e / s;
                  p = b_gamma;
                  b_gamma = (d[0] - b_anorm) * rte - s * b_gamma;
                  d[1] = (d[0] - b_gamma) + p;
                  if (rte != 0.0) {
                    p = b_gamma * b_gamma / rte;
                  } else {
                    p = oldc * e;
                  }
                }

                e = s * p;
                d[0] = b_anorm + b_gamma;
              }
            } while (exitg4 == 0);
          } else {
            do {
              exitg3 = 0;
              m = l;
              while ((m > 1) && (!(fabs(e) <= 4.9303806576313238E-32 * fabs(d[1])
                                   * fabs(d[0])))) {
                m = 1;
              }

              if (m > 1) {
                e = 0.0;
              }

              if (m == l) {
                l--;
                if (l < 1) {
                  exitg3 = 1;
                }
              } else if (l - 1 == m) {
                LR_xdlaev2(d[l - 1], sqrt(e), d[0], &d[l - 1], &b_anorm);
                d[0] = b_anorm;
                e = 0.0;
                exitg3 = 1;
              } else if (jtot == 60) {
                exitg3 = 1;
              } else {
                jtot++;
                rte = sqrt(e);
                s = d[l - 1];
                b_anorm = (d[0] - s) / (2.0 * rte);
                b_gamma = rt_hypotd_snf(b_anorm, 1.0);
                if (!(b_anorm >= 0.0)) {
                  b_gamma = -b_gamma;
                }

                b_anorm = s - rte / (b_anorm + b_gamma);
                rte = 1.0;
                s = 0.0;
                b_gamma = d[m - 1] - b_anorm;
                p = b_gamma * b_gamma;
                for (lend = m; lend < l; lend++) {
                  s = p + e;
                  if (m != 1) {
                    /* Check node always fails. would cause program termination and was eliminated */
                  }

                  oldc = rte;
                  rte = p / s;
                  s = e / s;
                  p = b_gamma;
                  b_gamma = (d[1] - b_anorm) * rte - s * b_gamma;
                  d[0] = (d[1] - b_gamma) + p;
                  if (rte != 0.0) {
                    p = b_gamma * b_gamma / rte;
                  } else {
                    p = oldc * e;
                  }
                }

                e = s * p;
                d[l - 1] = b_anorm + b_gamma;
              }
            } while (exitg3 == 0);
          }

          if (iscale == 1) {
            LR_xzlascl_f2p(2.2346346549904327E+153, anorm, lendsv_tmp_tmp - lsv,
                           d, lsv);
          } else if (iscale == 2) {
            LR_xzlascl_f2p(3.02546243347603E-123, anorm, lendsv_tmp_tmp - lsv, d,
                           lsv);
          }

          if (jtot >= 60) {
            if (e != 0.0) {
              info = 1;
            }

            exitg1 = 1;
          }
        }
      }
    }
  } while (exitg1 == 0);

  return info;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_ordeig(const real_T A[16], creal_T E[4])
{
  __m128d tmp;
  creal_T etemp[2];
  real_T At[4];
  real_T a__3[2];
  real_T wi[2];
  real_T absx;
  real_T anrm;
  int32_T At_tmp;
  int32_T c_i;
  int32_T exitg1;
  int32_T exitg2;
  int32_T i;
  int32_T k;
  boolean_T exitg3;
  boolean_T guard1;
  boolean_T iscale;
  iscale = true;
  for (k = 0; k < 16; k++) {
    if (iscale && ((!rtIsInf(A[k])) && (!rtIsNaN(A[k])))) {
    } else {
      iscale = false;
    }
  }

  if (!iscale) {
    E[0].re = (rtNaN);
    E[0].im = 0.0;
    E[1].re = (rtNaN);
    E[1].im = 0.0;
    E[2].re = (rtNaN);
    E[2].im = 0.0;
    E[3].re = (rtNaN);
    E[3].im = 0.0;
  } else {
    k = 0;
    do {
      exitg1 = 0;
      if (k + 1 <= 4) {
        guard1 = false;
        if (k + 1 != 4) {
          At_tmp = (k << 2) + k;
          anrm = A[At_tmp + 1];
          if (anrm != 0.0) {
            At[0] = A[At_tmp];
            At_tmp = ((k + 1) << 2) + k;
            At[2] = A[At_tmp];
            At[1] = anrm;
            At[3] = A[At_tmp + 1];
            iscale = true;
            for (At_tmp = 0; At_tmp < 4; At_tmp++) {
              if (iscale && ((!rtIsInf(At[At_tmp])) && (!rtIsNaN(At[At_tmp]))))
              {
              } else {
                iscale = false;
              }
            }

            if (!iscale) {
              etemp[0].re = (rtNaN);
              etemp[0].im = 0.0;
              etemp[1].re = (rtNaN);
              etemp[1].im = 0.0;
            } else {
              At_tmp = 0;
              exitg3 = false;
              while ((!exitg3) && (At_tmp < 2)) {
                i = 0;
                do {
                  exitg2 = 0;
                  if (i <= At_tmp) {
                    if (!(At[(At_tmp << 1) + i] == At[(i << 1) + At_tmp])) {
                      iscale = false;
                      exitg2 = 1;
                    } else {
                      i++;
                    }
                  } else {
                    At_tmp++;
                    exitg2 = 2;
                  }
                } while (exitg2 == 0);

                if (exitg2 == 1) {
                  exitg3 = true;
                }
              }

              if (iscale) {
                anrm = 0.0;
                At_tmp = 0;
                exitg3 = false;
                while ((!exitg3) && (At_tmp < 2)) {
                  i = 0;
                  do {
                    exitg2 = 0;
                    if (i <= At_tmp) {
                      absx = fabs(At[(At_tmp << 1) + i]);
                      if (rtIsNaN(absx)) {
                        anrm = (rtNaN);
                        exitg2 = 1;
                      } else {
                        if (absx > anrm) {
                          anrm = absx;
                        }

                        i++;
                      }
                    } else {
                      At_tmp++;
                      exitg2 = 2;
                    }
                  } while (exitg2 == 0);

                  if (exitg2 == 1) {
                    exitg3 = true;
                  }
                }

                if (rtIsInf(anrm) || rtIsNaN(anrm)) {
                  a__3[0] = (rtNaN);
                  a__3[1] = (rtNaN);
                } else {
                  iscale = false;
                  if ((anrm > 0.0) && (anrm < 1.0010415475915505E-146)) {
                    iscale = true;
                    anrm = 1.0010415475915505E-146 / anrm;
                    LR_xzlascl_f(1.0, anrm, At);
                  } else if (anrm > 9.9895953610111751E+145) {
                    iscale = true;
                    anrm = 9.9895953610111751E+145 / anrm;
                    LR_xzlascl_f(1.0, anrm, At);
                  }

                  a__3[0] = At[0];
                  a__3[1] = At[3];
                  At_tmp = LR_xdsterf(a__3, At[1]);
                  if (At_tmp != 0) {
                    a__3[0] = (rtNaN);
                    a__3[1] = (rtNaN);
                  } else if (iscale) {
                    anrm = 1.0 / anrm;
                    tmp = _mm_mul_pd(_mm_set1_pd(anrm), _mm_loadu_pd(&a__3[0]));
                    _mm_storeu_pd(&a__3[0], tmp);
                  }
                }

                etemp[0].re = a__3[0];
                etemp[0].im = 0.0;
                etemp[1].re = a__3[1];
                etemp[1].im = 0.0;
              } else {
                iscale = true;
                At_tmp = 0;
                exitg3 = false;
                while ((!exitg3) && (At_tmp < 2)) {
                  i = 0;
                  do {
                    exitg2 = 0;
                    if (i <= At_tmp) {
                      if (!(At[(At_tmp << 1) + i] == -At[(i << 1) + At_tmp])) {
                        iscale = false;
                        exitg2 = 1;
                      } else {
                        i++;
                      }
                    } else {
                      At_tmp++;
                      exitg2 = 2;
                    }
                  } while (exitg2 == 0);

                  if (exitg2 == 1) {
                    exitg3 = true;
                  }
                }

                if (iscale) {
                  LR_xzgehrd(At, 1, 2);
                  LR_xdlahqr(1, 2, At, &anrm, &At_tmp, a__3, wi);
                  i = (uint8_T)At_tmp;
                  for (c_i = 0; c_i < i; c_i++) {
                    etemp[c_i].re = (rtNaN);
                    etemp[c_i].im = 0.0;
                  }

                  for (i = At_tmp + 1; i < 3; i++) {
                    etemp[i - 1].re = 0.0;
                    etemp[i - 1].im = wi[i - 1];
                  }
                } else {
                  LR_eigStandard(At, etemp);
                }
              }
            }

            E[k] = etemp[0];
            E[k + 1] = etemp[1];
            k += 2;
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          E[k].re = A[(k << 2) + k];
          E[k].im = 0.0;
          k++;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_sylvesterTriKernel_b(int32_T ia0, int32_T ja0, int32_T ib0,
  int32_T jb0, creal_T C[16], int32_T ic0, int32_T jc0, int32_T m, int32_T n)
{
  real_T tmp[2];
  int32_T cmod_re_tmp;
  int32_T ii;
  int32_T jj;
  for (ii = m - 2; ii + 2 >= 1; ii--) {
    for (jj = -1; jj + 2 <= n; jj++) {
      __m128d tmp_0;
      real_T cmod_im;
      real_T cmod_re;
      int32_T cmod_re_tmp_tmp;
      int32_T cmod_re_tmp_tmp_tmp;
      int32_T tmp_1;
      int32_T tmp_2;
      cmod_re_tmp_tmp_tmp = (jc0 + jj) << 2;
      cmod_re_tmp_tmp = cmod_re_tmp_tmp_tmp + ic0;
      cmod_re_tmp = cmod_re_tmp_tmp + ii;
      cmod_re = C[cmod_re_tmp].re;
      cmod_im = C[cmod_re_tmp].im;
      for (cmod_re_tmp = ii + 3; cmod_re_tmp <= m; cmod_re_tmp++) {
        tmp_1 = ((((ja0 + cmod_re_tmp) - 2) << 2) + ia0) + ii;
        tmp_2 = (cmod_re_tmp_tmp + cmod_re_tmp) - 2;
        tmp_0 = _mm_sub_pd(_mm_set_pd(cmod_im, cmod_re), _mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(C[tmp_1].re), _mm_loadu_pd((const real_T *)&C[tmp_2])),
          _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(C[tmp_1].im), _mm_shuffle_pd
          (_mm_loadu_pd((const real_T *)&C[tmp_2]), _mm_loadu_pd((const real_T *)
          &C[tmp_2]), 1)), _mm_set_pd(1.0, -1.0))));
        _mm_storeu_pd(&tmp[0], tmp_0);
        cmod_re = tmp[0];
        cmod_im = tmp[1];
      }

      cmod_re_tmp_tmp = (uint8_T)(jj + 1);
      for (cmod_re_tmp = 0; cmod_re_tmp < cmod_re_tmp_tmp; cmod_re_tmp++) {
        tmp_1 = ((((jc0 + cmod_re_tmp) - 1) << 2) + ic0) + ii;
        tmp_2 = ((((jb0 + jj) << 2) + ib0) + cmod_re_tmp) - 1;
        tmp_0 = _mm_sub_pd(_mm_set_pd(cmod_im, cmod_re), _mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(C[tmp_1].re), _mm_loadu_pd((const real_T *)&C[tmp_2])),
          _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(C[tmp_1].im), _mm_shuffle_pd
          (_mm_loadu_pd((const real_T *)&C[tmp_2]), _mm_loadu_pd((const real_T *)
          &C[tmp_2]), 1)), _mm_set_pd(1.0, -1.0))));
        _mm_storeu_pd(&tmp[0], tmp_0);
        cmod_re = tmp[0];
        cmod_im = tmp[1];
      }

      tmp_0 = _mm_add_pd(_mm_loadu_pd((const real_T *)&C[(((ja0 + ii) << 2) +
        ia0) + ii]), _mm_loadu_pd((const real_T *)&C[(((jb0 + jj) << 2) + ib0) +
        jj]));
      _mm_storeu_pd(&tmp[0], tmp_0);
      if (tmp[1] == 0.0) {
        if (cmod_im == 0.0) {
          C[(ic0 + ii) + cmod_re_tmp_tmp_tmp].re = cmod_re / tmp[0];
          cmod_re = 0.0;
        } else if (cmod_re == 0.0) {
          C[(ic0 + ii) + cmod_re_tmp_tmp_tmp].re = 0.0;
          cmod_re = cmod_im / tmp[0];
        } else {
          C[(ic0 + ii) + cmod_re_tmp_tmp_tmp].re = cmod_re / tmp[0];
          cmod_re = cmod_im / tmp[0];
        }
      } else if (tmp[0] == 0.0) {
        if (cmod_re == 0.0) {
          C[(ic0 + ii) + cmod_re_tmp_tmp_tmp].re = cmod_im / tmp[1];
          cmod_re = 0.0;
        } else if (cmod_im == 0.0) {
          C[(ic0 + ii) + cmod_re_tmp_tmp_tmp].re = 0.0;
          cmod_re = -(cmod_re / tmp[1]);
        } else {
          C[(ic0 + ii) + cmod_re_tmp_tmp_tmp].re = cmod_im / tmp[1];
          cmod_re = -(cmod_re / tmp[1]);
        }
      } else {
        real_T bim;
        real_T brm;
        brm = fabs(tmp[0]);
        bim = fabs(tmp[1]);
        if (brm > bim) {
          brm = tmp[1] / tmp[0];
          bim = brm * tmp[1] + tmp[0];
          C[(ic0 + ii) + cmod_re_tmp_tmp_tmp].re = (brm * cmod_im + cmod_re) /
            bim;
          cmod_re = (cmod_im - brm * cmod_re) / bim;
        } else if (bim == brm) {
          real_T sgnbi;
          if (tmp[0] > 0.0) {
            bim = 0.5;
          } else {
            bim = -0.5;
          }

          if (tmp[1] > 0.0) {
            sgnbi = 0.5;
          } else {
            sgnbi = -0.5;
          }

          C[(ic0 + ii) + cmod_re_tmp_tmp_tmp].re = (cmod_re * bim + cmod_im *
            sgnbi) / brm;
          cmod_re = (cmod_im * bim - cmod_re * sgnbi) / brm;
        } else {
          brm = tmp[0] / tmp[1];
          bim = brm * tmp[0] + tmp[1];
          C[(ic0 + ii) + cmod_re_tmp_tmp_tmp].re = (brm * cmod_re + cmod_im) /
            bim;
          cmod_re = (brm * cmod_im - cmod_re) / bim;
        }
      }

      C[(ic0 + ii) + cmod_re_tmp_tmp_tmp].im = cmod_re;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xgemv(int32_T m, int32_T n, int32_T ia0, int32_T ix0, creal_T y
                     [16], int32_T iy0)
{
  int32_T c_re_tmp;
  int32_T iac;
  if ((m != 0) && (n != 0)) {
    int32_T b;
    b = ((n - 1) << 2) + ia0;
    for (iac = ia0; iac <= b; iac += 4) {
      real_T c_im;
      real_T c_re;
      real_T c_re_tmp_0;
      int32_T d;
      c_re_tmp = (((iac - ia0) >> 2) + ix0) - 1;
      c_im = y[c_re_tmp].re;
      c_re_tmp_0 = y[c_re_tmp].im;
      c_re = -c_im - c_re_tmp_0 * 0.0;
      c_im = c_im * 0.0 - c_re_tmp_0;
      d = (iac + m) - 1;
      for (c_re_tmp = iac; c_re_tmp <= d; c_re_tmp++) {
        real_T y_im_tmp;
        int32_T tmp;
        c_re_tmp_0 = y[c_re_tmp - 1].re;
        y_im_tmp = y[c_re_tmp - 1].im;
        tmp = ((iy0 + c_re_tmp) - iac) - 1;
        y[tmp].re += c_re_tmp_0 * c_re - y_im_tmp * c_im;
        y[tmp].im += c_re_tmp_0 * c_im + y_im_tmp * c_re;
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_sylvesterRecursive_g(int32_T ia0, int32_T ja0, int32_T ib0,
  int32_T jb0, creal_T C[16], int32_T ic0, int32_T jc0, int32_T m, int32_T n)
{
  int32_T b_kk;
  int32_T n1;
  int32_T tmp;
  int32_T tmp_0;
  int32_T xStart;
  if ((m >= n) && (m > 8)) {
    n1 = m / 2;
    tmp = ja0 + n1;
    tmp_0 = m - n1;
    LR_sylvesterRecursive_g(ia0 + n1, tmp, ib0, jb0, C, ic0 + n1, jc0, tmp_0, n);
    xStart = ((tmp - 1) << 2) + ia0;
    for (b_kk = 0; b_kk < n; b_kk++) {
      tmp = (((jc0 + b_kk) - 1) << 2) + ic0;
      LR_xgemv(n1, tmp_0, xStart, tmp + n1, C, tmp);
    }

    LR_sylvesterRecursive_g(ia0, ja0, ib0, jb0, C, ic0, jc0, n1, n);
  } else if (n > 8) {
    n1 = n / 2;
    LR_sylvesterRecursive_g(ia0, ja0, ib0, jb0, C, ic0, jc0, m, n1);
    xStart = ((jc0 - 1) << 2) + ic0;
    for (b_kk = n1; b_kk < n; b_kk++) {
      LR_xgemv(m, n1, xStart, (((jb0 + b_kk) - 1) << 2) + ib0, C, (((jc0 + b_kk)
                 - 1) << 2) + ic0);
    }

    LR_sylvesterRecursive_g(ia0, ja0, ib0 + n1, jb0 + n1, C, ic0, jc0 + n1, m, n
      - n1);
  } else {
    LR_sylvesterTriKernel_b(ia0, ja0, ib0, jb0, C, ic0, jc0, m, n);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_sqrt(creal_T *x)
{
  real_T absxr;
  real_T xr;
  xr = x->re;
  if (x->im == 0.0) {
    if (x->re < 0.0) {
      absxr = 0.0;
      xr = sqrt(-x->re);
    } else {
      absxr = sqrt(x->re);
      xr = 0.0;
    }
  } else if (x->re == 0.0) {
    if (x->im < 0.0) {
      absxr = sqrt(-x->im / 2.0);
      xr = -absxr;
    } else {
      absxr = sqrt(x->im / 2.0);
      xr = absxr;
    }
  } else if (rtIsNaN(x->re)) {
    absxr = (rtNaN);
  } else if (rtIsNaN(x->im)) {
    absxr = (rtNaN);
    xr = (rtNaN);
  } else if (rtIsInf(x->im)) {
    absxr = fabs(x->im);
    xr = x->im;
  } else if (rtIsInf(x->re)) {
    if (x->re < 0.0) {
      absxr = 0.0;
      xr = x->im * -x->re;
    } else {
      absxr = x->re;
      xr = 0.0;
    }
  } else {
    absxr = fabs(x->re);
    xr = fabs(x->im);
    if ((absxr > 4.4942328371557893E+307) || (xr > 4.4942328371557893E+307)) {
      absxr *= 0.5;
      xr = rt_hypotd_snf(absxr, xr * 0.5);
      if (xr > absxr) {
        absxr = sqrt(absxr / xr + 1.0) * sqrt(xr);
      } else {
        absxr = sqrt(xr) * 1.4142135623730951;
      }
    } else {
      absxr = sqrt((rt_hypotd_snf(absxr, xr) + absxr) * 0.5);
    }

    if (x->re > 0.0) {
      xr = x->im / absxr * 0.5;
    } else {
      if (x->im < 0.0) {
        xr = -absxr;
      } else {
        xr = absxr;
      }

      absxr = x->im / xr * 0.5;
    }
  }

  x->re = absxr;
  x->im = xr;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_sqrtmTriRecursive_n(creal_T T[16], int32_T i, int32_T j, int32_T
  m)
{
  __m128d tmp;
  creal_T r11;
  creal_T r22;
  real_T tmp_0[2];
  real_T ai;
  real_T ar;
  real_T bim;
  real_T brm;
  real_T sgnbi;
  int32_T m1;
  int32_T m2;
  int32_T tmp_1;
  int32_T tmp_2;
  if (m == 1) {
    LR_sqrt(&T[(i + ((j - 1) << 2)) - 1]);
  } else if (m == 2) {
    m1 = (((j - 1) << 2) + i) - 1;
    r11 = T[m1];
    LR_sqrt(&r11);
    m2 = (j << 2) + i;
    r22 = T[m2];
    LR_sqrt(&r22);
    T[m2] = r22;
    T[m1] = r11;
    ar = T[m2 - 1].re;
    ai = T[m2 - 1].im;
    tmp = _mm_add_pd(_mm_loadu_pd((const real_T *)&r11), _mm_loadu_pd((const
      real_T *)&r22));
    _mm_storeu_pd(&tmp_0[0], tmp);
    if (tmp_0[1] == 0.0) {
      if (ai == 0.0) {
        T[m2 - 1].re = ar / tmp_0[0];
        ar = 0.0;
      } else if (ar == 0.0) {
        T[m2 - 1].re = 0.0;
        ar = ai / tmp_0[0];
      } else {
        T[m2 - 1].re = ar / tmp_0[0];
        ar = ai / tmp_0[0];
      }
    } else if (tmp_0[0] == 0.0) {
      if (ar == 0.0) {
        T[m2 - 1].re = ai / tmp_0[1];
        ar = 0.0;
      } else if (ai == 0.0) {
        T[m2 - 1].re = 0.0;
        ar = -(ar / tmp_0[1]);
      } else {
        T[m2 - 1].re = ai / tmp_0[1];
        ar = -(ar / tmp_0[1]);
      }
    } else {
      brm = fabs(tmp_0[0]);
      bim = fabs(tmp_0[1]);
      if (brm > bim) {
        brm = tmp_0[1] / tmp_0[0];
        bim = brm * tmp_0[1] + tmp_0[0];
        T[m2 - 1].re = (brm * ai + ar) / bim;
        ar = (ai - brm * ar) / bim;
      } else if (bim == brm) {
        if (tmp_0[0] > 0.0) {
          bim = 0.5;
        } else {
          bim = -0.5;
        }

        if (tmp_0[1] > 0.0) {
          sgnbi = 0.5;
        } else {
          sgnbi = -0.5;
        }

        T[m2 - 1].re = (ar * bim + ai * sgnbi) / brm;
        ar = (ai * bim - ar * sgnbi) / brm;
      } else {
        brm = tmp_0[0] / tmp_0[1];
        bim = brm * tmp_0[0] + tmp_0[1];
        T[m2 - 1].re = (brm * ar + ai) / bim;
        ar = (brm * ai - ar) / bim;
      }
    }

    T[m2 - 1].im = ar;
  } else {
    m1 = m / 2;
    m2 = m - m1;
    LR_sqrtmTriRecursive_n(T, i, j, m1);
    tmp_1 = i + m1;
    tmp_2 = j + m1;
    LR_sqrtmTriRecursive_n(T, tmp_1, tmp_2, m2);
    LR_sylvesterRecursive_g(i, j, tmp_1, tmp_2, T, i, tmp_2, m1, m2);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T LR_norm_j(const creal_T x[16])
{
  real_T s;
  real_T y;
  int32_T j;
  int32_T s_tmp_tmp;
  boolean_T exitg1;
  y = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 4)) {
    s_tmp_tmp = j << 2;
    s = ((rt_hypotd_snf(x[s_tmp_tmp + 1].re, x[s_tmp_tmp + 1].im) +
          rt_hypotd_snf(x[s_tmp_tmp].re, x[s_tmp_tmp].im)) + rt_hypotd_snf
         (x[s_tmp_tmp + 2].re, x[s_tmp_tmp + 2].im)) + rt_hypotd_snf(x[s_tmp_tmp
      + 3].re, x[s_tmp_tmp + 3].im);
    if (rtIsNaN(s)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (s > y) {
        y = s;
      }

      j++;
    }
  }

  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_atanh_h(creal_T *x)
{
  real_T t;
  real_T xi;
  real_T xr;
  xr = fabs(x->re);
  xi = fabs(x->im);
  if ((xr > 3.3519519824856489E+153) || (xi > 3.3519519824856489E+153)) {
    if (xi == 0.0) {
      xr = 1.0 / xr;
    } else if (xr == 0.0) {
      xr = 0.0;
    } else if (xr > xi) {
      t = xi / xr;
      xr = (t * 0.0 + 1.0) / (t * xi + xr);
    } else if (xi == xr) {
      xr = 0.5 / xr;
    } else {
      t = xr / xi;
      xr = t / (t * xr + xi);
    }

    xi = 1.5707963267948966;
  } else if ((xr == 1.0) && (xi == 0.0)) {
    xr = (rtInf);
  } else if (xr == 1.0) {
    xr = log(sqrt(sqrt(xi * xi + 4.0)) / sqrt(xi + 2.9833362924800834E-154));
    xi = (atan((xi + 2.9833362924800834E-154) / 2.0) + 1.5707963267948966) / 2.0;
  } else if ((xi == 0.0) && (!(xr > 1.0))) {
    if (xr < 0.5) {
      t = xr + xr;
      xr = xr / (1.0 - xr) * t + t;
      if (!(xr < 2.2204460492503131E-16)) {
        xr = xr / ((xr + 1.0) - 1.0) * log(xr + 1.0);
      }

      xr /= 2.0;
    } else if (xr == 1.0) {
      xr = (rtInf);
    } else {
      xr = (xr + xr) / (1.0 - xr);
      if ((xr > 4.503599627370496E+15) || rtIsNaN(xr)) {
        xr = log(xr + 1.0);
      } else {
        xr = xr / ((xr + 1.0) - 1.0) * log(xr + 1.0);
      }

      xr /= 2.0;
    }
  } else {
    t = (xi + 2.9833362924800834E-154) * (xi + 2.9833362924800834E-154);
    xi = rt_atan2d_snf(2.0 * xi, (1.0 - xr) * (xr + 1.0) - t) / 2.0;
    xr = xr / ((1.0 - xr) * (1.0 - xr) + t) * 4.0;
    t = fabs(xr);
    if ((t > 4.503599627370496E+15) || (rtIsInf(xr) || rtIsNaN(xr))) {
      xr = log(xr + 1.0);
    } else if (!(t < 2.2204460492503131E-16)) {
      xr = xr / ((xr + 1.0) - 1.0) * log(xr + 1.0);
    }

    xr /= 4.0;
  }

  if (x->re < 0.0) {
    xr = -xr;
  }

  if ((x->im < 0.0) || ((x->im == 0.0) && (x->re < -1.0))) {
    xi = -xi;
  }

  x->re = xr;
  x->im = xi;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static creal_T LR_power(const creal_T a, real_T b)
{
  creal_T y;
  real_T ar;
  real_T b_b_im;
  if ((a.im == 0.0) && (a.re >= 0.0)) {
    y.re = rt_powd_snf(a.re, b);
    y.im = 0.0;
  } else if ((a.re == 0.0) && (floor(b) == b)) {
    y.re = 0.0;
    y.im = -rt_powd_snf(a.im, -1.0);
  } else if ((a.im == 0.0) && rtIsInf(b) && (fabs(a.re) == 1.0)) {
    y.re = 1.0;
    y.im = 0.0;
  } else {
    if (a.im == 0.0) {
      if (a.re < 0.0) {
        ar = log(fabs(a.re));
        b_b_im = 3.1415926535897931;
      } else {
        ar = log(a.re);
        b_b_im = 0.0;
      }
    } else if ((fabs(a.re) > 8.9884656743115785E+307) || (fabs(a.im) >
                8.9884656743115785E+307)) {
      ar = log(rt_hypotd_snf(a.re / 2.0, a.im / 2.0)) + 0.69314718055994529;
      b_b_im = rt_atan2d_snf(a.im, a.re);
    } else {
      ar = log(rt_hypotd_snf(a.re, a.im));
      b_b_im = rt_atan2d_snf(a.im, a.re);
    }

    _mm_storeu_pd((real_T *)&y, _mm_mul_pd(_mm_set1_pd(b), _mm_set_pd(b_b_im, ar)));
    if (y.re == 0.0) {
      y.re = cos(y.im);
      y.im = sin(y.im);
    } else if (y.im == 0.0) {
      y.re = exp(y.re);
      y.im = 0.0;
    } else if (rtIsInf(y.im) && rtIsInf(y.re) && (y.re < 0.0)) {
      y.re = 0.0;
      y.im = 0.0;
    } else {
      ar = exp(y.re / 2.0);
      y.re = ar * cos(y.im) * ar;
      y.im = ar * sin(y.im) * ar;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void recomputeDiagBlocksSqrtTriangul(creal_T Troot[16], const creal_T T
  [16], int32_T s)
{
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  __m128d tmp_4;
  creal_T A[4];
  creal_T P[4];
  creal_T P_0[4];
  creal_T r11;
  creal_T r22;
  real_T tmp[2];
  real_T Z0_idx_0_im;
  real_T Z0_idx_0_re_tmp_tmp;
  real_T Z0_idx_1_re_tmp;
  real_T Z0_idx_3_im;
  real_T Z0_tmp_idx_0_re;
  real_T Z0_tmp_idx_3_im;
  real_T ai;
  real_T bim;
  real_T im;
  real_T loga2_im;
  real_T loga2_re;
  real_T loga2_re_tmp;
  real_T re;
  real_T re_0;
  real_T sgnbi;
  int32_T A_tmp;
  int32_T A_tmp_tmp;
  int32_T i;
  int32_T j;
  int32_T r1;
  int32_T r2;
  int32_T tmp_5;
  for (j = 0; j < 3; j++) {
    if (s == 0) {
      i = (j << 2) + j;
      Troot[i].re = T[i].re - 1.0;
      Troot[i].im = T[i].im;
      Troot[i + 1] = T[i + 1];
      i = (j + 1) << 2;
      tmp_5 = i + j;
      Troot[j + i] = T[tmp_5];
      Troot[tmp_5 + 1].re = T[tmp_5 + 1].re - 1.0;
      Troot[tmp_5 + 1].im = T[tmp_5 + 1].im;
    } else {
      A_tmp_tmp = (j << 2) + j;
      A[1] = T[A_tmp_tmp + 1];
      A_tmp = ((j + 1) << 2) + j;
      A[2] = T[A_tmp];
      r11 = T[A_tmp_tmp];
      LR_sqrt(&r11);
      r22 = T[A_tmp + 1];
      LR_sqrt(&r22);
      tmp_0 = _mm_add_pd(_mm_loadu_pd((const real_T *)&r11), _mm_loadu_pd((const
        real_T *)&r22));
      _mm_storeu_pd(&tmp[0], tmp_0);
      if (tmp[1] == 0.0) {
        if (A[2].im == 0.0) {
          re = A[2].re / tmp[0];
          im = 0.0;
        } else if (A[2].re == 0.0) {
          re = 0.0;
          im = A[2].im / tmp[0];
        } else {
          tmp_0 = _mm_div_pd(_mm_loadu_pd((const real_T *)&A[2]), _mm_set1_pd
                             (tmp[0]));
          _mm_storeu_pd(&tmp[0], tmp_0);
          re = tmp[0];
          im = tmp[1];
        }
      } else if (tmp[0] == 0.0) {
        if (A[2].re == 0.0) {
          re = A[2].im / tmp[1];
          im = 0.0;
        } else if (A[2].im == 0.0) {
          re = 0.0;
          im = -(A[2].re / tmp[1]);
        } else {
          re = A[2].im / tmp[1];
          im = -(A[2].re / tmp[1]);
        }
      } else {
        loga2_re = fabs(tmp[0]);
        bim = fabs(tmp[1]);
        if (loga2_re > bim) {
          loga2_re = tmp[1] / tmp[0];
          bim = loga2_re * tmp[1] + tmp[0];
          re = (loga2_re * A[2].im + A[2].re) / bim;
          im = (A[2].im - loga2_re * A[2].re) / bim;
        } else if (bim == loga2_re) {
          if (tmp[0] > 0.0) {
            bim = 0.5;
          } else {
            bim = -0.5;
          }

          if (tmp[1] > 0.0) {
            sgnbi = 0.5;
          } else {
            sgnbi = -0.5;
          }

          tmp_0 = _mm_div_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)
            &A[2]), _mm_set1_pd(bim)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
            (_mm_loadu_pd((const real_T *)&A[2]), _mm_loadu_pd((const real_T *)
            &A[2]), 1), _mm_set1_pd(sgnbi)), _mm_set_pd(-1.0, 1.0))),
                             _mm_set1_pd(loga2_re));
          _mm_storeu_pd(&tmp[0], tmp_0);
          re = tmp[0];
          im = tmp[1];
        } else {
          loga2_re = tmp[0] / tmp[1];
          tmp_0 = _mm_div_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(loga2_re),
            _mm_loadu_pd((const real_T *)&A[2])), _mm_mul_pd(_mm_shuffle_pd
            (_mm_loadu_pd((const real_T *)&A[2]), _mm_loadu_pd((const real_T *)
            &A[2]), 1), _mm_set_pd(-1.0, 1.0))), _mm_set1_pd(loga2_re * tmp[0] +
            tmp[1]));
          _mm_storeu_pd(&tmp[0], tmp_0);
          re = tmp[0];
          im = tmp[1];
        }
      }

      Z0_idx_0_re_tmp_tmp = r11.re - 1.0;
      Z0_idx_0_im = r11.im;
      Z0_idx_1_re_tmp = A[1].re;
      loga2_im = A[1].im;
      loga2_re_tmp = r22.re - 1.0;
      Z0_idx_3_im = r22.im;
      if (s == 1) {
        Troot[A_tmp_tmp].re = r11.re - 1.0;
        Troot[A_tmp_tmp].im = r11.im;
        Troot[A_tmp_tmp + 1] = A[1];
        Troot[A_tmp].re = re;
        Troot[A_tmp].im = im;
        Troot[A_tmp + 1].re = r22.re - 1.0;
        Troot[A_tmp + 1].im = r22.im;
      } else {
        LR_sqrt(&r11);
        LR_sqrt(&r22);
        tmp_0 = _mm_add_pd(_mm_loadu_pd((const real_T *)&r11), _mm_loadu_pd((
          const real_T *)&r22));
        _mm_storeu_pd(&tmp[0], tmp_0);
        if (tmp[1] == 0.0) {
          if (im == 0.0) {
            re_0 = re / tmp[0];
            loga2_re = 0.0;
          } else if (re == 0.0) {
            re_0 = 0.0;
            loga2_re = im / tmp[0];
          } else {
            re_0 = re / tmp[0];
            loga2_re = im / tmp[0];
          }
        } else if (tmp[0] == 0.0) {
          if (re == 0.0) {
            re_0 = im / tmp[1];
            loga2_re = 0.0;
          } else if (im == 0.0) {
            re_0 = 0.0;
            loga2_re = -(re / tmp[1]);
          } else {
            re_0 = im / tmp[1];
            loga2_re = -(re / tmp[1]);
          }
        } else {
          loga2_re = fabs(tmp[0]);
          bim = fabs(tmp[1]);
          if (loga2_re > bim) {
            loga2_re = tmp[1] / tmp[0];
            bim = loga2_re * tmp[1] + tmp[0];
            re_0 = (loga2_re * im + re) / bim;
            loga2_re = (im - loga2_re * re) / bim;
          } else if (bim == loga2_re) {
            if (tmp[0] > 0.0) {
              bim = 0.5;
            } else {
              bim = -0.5;
            }

            if (tmp[1] > 0.0) {
              sgnbi = 0.5;
            } else {
              sgnbi = -0.5;
            }

            re_0 = (re * bim + im * sgnbi) / loga2_re;
            loga2_re = (im * bim - re * sgnbi) / loga2_re;
          } else {
            loga2_re = tmp[0] / tmp[1];
            bim = loga2_re * tmp[0] + tmp[1];
            re_0 = (loga2_re * re + im) / bim;
            loga2_re = (loga2_re * im - re) / bim;
          }
        }

        A[2].re = re_0;
        A[2].im = loga2_re;
        P[0].re = r11.re + 1.0;
        P[0].im = r11.im;
        P[1] = A[1];
        P[2].re = re_0;
        P[2].im = loga2_re;
        P[3].re = r22.re + 1.0;
        P[3].im = r22.im;
        r1 = (uint8_T)(s - 2);
        for (r2 = 0; r2 < r1; r2++) {
          LR_sqrt(&r11);
          LR_sqrt(&r22);
          tmp_0 = _mm_add_pd(_mm_loadu_pd((const real_T *)&r11), _mm_loadu_pd((
            const real_T *)&r22));
          _mm_storeu_pd(&tmp[0], tmp_0);
          if (tmp[1] == 0.0) {
            if (A[2].im == 0.0) {
              re_0 = A[2].re / tmp[0];
              loga2_re = 0.0;
            } else if (A[2].re == 0.0) {
              re_0 = 0.0;
              loga2_re = A[2].im / tmp[0];
            } else {
              re_0 = A[2].re / tmp[0];
              loga2_re = A[2].im / tmp[0];
            }
          } else if (tmp[0] == 0.0) {
            if (A[2].re == 0.0) {
              re_0 = A[2].im / tmp[1];
              loga2_re = 0.0;
            } else if (A[2].im == 0.0) {
              re_0 = 0.0;
              loga2_re = -(A[2].re / tmp[1]);
            } else {
              re_0 = A[2].im / tmp[1];
              loga2_re = -(A[2].re / tmp[1]);
            }
          } else {
            loga2_re = fabs(tmp[0]);
            bim = fabs(tmp[1]);
            if (loga2_re > bim) {
              loga2_re = tmp[1] / tmp[0];
              bim = loga2_re * tmp[1] + tmp[0];
              re_0 = (loga2_re * A[2].im + A[2].re) / bim;
              loga2_re = (A[2].im - loga2_re * A[2].re) / bim;
            } else if (bim == loga2_re) {
              if (tmp[0] > 0.0) {
                bim = 0.5;
              } else {
                bim = -0.5;
              }

              if (tmp[1] > 0.0) {
                sgnbi = 0.5;
              } else {
                sgnbi = -0.5;
              }

              re_0 = (A[2].re * bim + A[2].im * sgnbi) / loga2_re;
              loga2_re = (A[2].im * bim - A[2].re * sgnbi) / loga2_re;
            } else {
              loga2_re = tmp[0] / tmp[1];
              bim = loga2_re * tmp[0] + tmp[1];
              re_0 = (loga2_re * A[2].re + A[2].im) / bim;
              loga2_re = (loga2_re * A[2].im - A[2].re) / bim;
            }
          }

          A[2].re = re_0;
          A[2].im = loga2_re;
          Z0_tmp_idx_0_re = r11.re + 1.0;
          bim = r11.im;
          sgnbi = r22.re + 1.0;
          Z0_tmp_idx_3_im = r22.im;
          for (i = 0; i < 2; i++) {
            tmp_0 = _mm_set_pd(1.0, -1.0);
            tmp_1 = _mm_set1_pd(P[i + 2].re);
            tmp_2 = _mm_set1_pd(P[i + 2].im);
            tmp_3 = _mm_set1_pd(P[i].re);
            tmp_4 = _mm_set1_pd(P[i].im);
            _mm_storeu_pd((real_T *)&P_0[i], _mm_add_pd(_mm_add_pd(_mm_mul_pd
              (tmp_1, _mm_set_pd(loga2_im, Z0_idx_1_re_tmp)), _mm_mul_pd
              (_mm_mul_pd(tmp_2, _mm_set_pd(Z0_idx_1_re_tmp, loga2_im)), tmp_0)),
              _mm_add_pd(_mm_mul_pd(tmp_3, _mm_set_pd(bim, Z0_tmp_idx_0_re)),
                         _mm_mul_pd(_mm_mul_pd(tmp_4, _mm_set_pd(Z0_tmp_idx_0_re,
              bim)), tmp_0))));
            _mm_storeu_pd((real_T *)&P_0[i + 2], _mm_add_pd(_mm_add_pd
              (_mm_mul_pd(tmp_1, _mm_set_pd(Z0_tmp_idx_3_im, sgnbi)), _mm_mul_pd
               (_mm_mul_pd(tmp_2, _mm_set_pd(sgnbi, Z0_tmp_idx_3_im)), tmp_0)),
              _mm_add_pd(_mm_mul_pd(tmp_3, _mm_set_pd(loga2_re, re_0)),
                         _mm_mul_pd(_mm_mul_pd(tmp_4, _mm_set_pd(re_0, loga2_re)),
              tmp_0))));
          }

          memcpy(&P[0], &P_0[0], sizeof(creal_T) << 2U);
        }

        if (fabs(P[1].re) + fabs(P[1].im) > fabs(P[0].re) + fabs(P[0].im)) {
          r1 = 1;
          r2 = 0;
        } else {
          r1 = 0;
          r2 = 1;
        }

        Z0_tmp_idx_3_im = P[r2].re;
        ai = P[r2].im;
        re_0 = P[r1].re;
        Z0_tmp_idx_0_re = P[r1].im;
        if (Z0_tmp_idx_0_re == 0.0) {
          if (ai == 0.0) {
            r11.re = Z0_tmp_idx_3_im / re_0;
            r11.im = 0.0;
          } else if (Z0_tmp_idx_3_im == 0.0) {
            r11.re = 0.0;
            r11.im = ai / re_0;
          } else {
            _mm_storeu_pd((real_T *)&r11, _mm_div_pd(_mm_set_pd(ai,
              Z0_tmp_idx_3_im), _mm_set1_pd(re_0)));
          }
        } else if (re_0 == 0.0) {
          if (Z0_tmp_idx_3_im == 0.0) {
            r11.re = ai / Z0_tmp_idx_0_re;
            r11.im = 0.0;
          } else if (ai == 0.0) {
            r11.re = 0.0;
            r11.im = -(Z0_tmp_idx_3_im / Z0_tmp_idx_0_re);
          } else {
            r11.re = ai / Z0_tmp_idx_0_re;
            r11.im = -(Z0_tmp_idx_3_im / Z0_tmp_idx_0_re);
          }
        } else {
          loga2_re = fabs(re_0);
          bim = fabs(Z0_tmp_idx_0_re);
          if (loga2_re > bim) {
            loga2_re = Z0_tmp_idx_0_re / re_0;
            bim = loga2_re * Z0_tmp_idx_0_re + re_0;
            r11.re = (loga2_re * ai + Z0_tmp_idx_3_im) / bim;
            r11.im = (ai - loga2_re * Z0_tmp_idx_3_im) / bim;
          } else if (bim == loga2_re) {
            if (re_0 > 0.0) {
              bim = 0.5;
            } else {
              bim = -0.5;
            }

            if (Z0_tmp_idx_0_re > 0.0) {
              sgnbi = 0.5;
            } else {
              sgnbi = -0.5;
            }

            _mm_storeu_pd((real_T *)&r11, _mm_div_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set_pd(ai, Z0_tmp_idx_3_im), _mm_set1_pd(bim)), _mm_mul_pd
              (_mm_mul_pd(_mm_set_pd(Z0_tmp_idx_3_im, ai), _mm_set1_pd(sgnbi)),
               _mm_set_pd(-1.0, 1.0))), _mm_set1_pd(loga2_re)));
          } else {
            loga2_re = re_0 / Z0_tmp_idx_0_re;
            _mm_storeu_pd((real_T *)&r11, _mm_div_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set1_pd(loga2_re), _mm_set_pd(ai, Z0_tmp_idx_3_im)),
              _mm_mul_pd(_mm_set_pd(Z0_tmp_idx_3_im, ai), _mm_set_pd(-1.0, 1.0))),
              _mm_set1_pd(loga2_re * re_0 + Z0_tmp_idx_0_re)));
          }
        }

        tmp_0 = _mm_set_pd(1.0, -1.0);
        tmp_1 = _mm_sub_pd(_mm_loadu_pd((const real_T *)&P[r2 + 2]), _mm_add_pd
                           (_mm_mul_pd(_mm_loadu_pd((const real_T *)&P[r1 + 2]),
          _mm_set1_pd(r11.re)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
          (_mm_loadu_pd((const real_T *)&P[r1 + 2]), _mm_loadu_pd((const real_T *)
          &P[r1 + 2]), 1), _mm_set1_pd(r11.im)), tmp_0)));
        _mm_storeu_pd((real_T *)&r22, tmp_1);
        if (Z0_tmp_idx_0_re == 0.0) {
          if (Z0_idx_0_im == 0.0) {
            i = r1 << 1;
            A[i].re = Z0_idx_0_re_tmp_tmp / re_0;
            A[i].im = 0.0;
          } else if (Z0_idx_0_re_tmp_tmp == 0.0) {
            i = r1 << 1;
            A[i].re = 0.0;
            A[i].im = Z0_idx_0_im / re_0;
          } else {
            _mm_storeu_pd((real_T *)&A[r1 << 1], _mm_div_pd(_mm_set_pd
              (Z0_idx_0_im, Z0_idx_0_re_tmp_tmp), _mm_set1_pd(re_0)));
          }
        } else if (re_0 == 0.0) {
          if (Z0_idx_0_re_tmp_tmp == 0.0) {
            i = r1 << 1;
            A[i].re = Z0_idx_0_im / Z0_tmp_idx_0_re;
            A[i].im = 0.0;
          } else if (Z0_idx_0_im == 0.0) {
            i = r1 << 1;
            A[i].re = 0.0;
            A[i].im = -(Z0_idx_0_re_tmp_tmp / Z0_tmp_idx_0_re);
          } else {
            i = r1 << 1;
            A[i].re = Z0_idx_0_im / Z0_tmp_idx_0_re;
            A[i].im = -(Z0_idx_0_re_tmp_tmp / Z0_tmp_idx_0_re);
          }
        } else {
          loga2_re = fabs(re_0);
          bim = fabs(Z0_tmp_idx_0_re);
          if (loga2_re > bim) {
            loga2_re = Z0_tmp_idx_0_re / re_0;
            bim = loga2_re * Z0_tmp_idx_0_re + re_0;
            i = r1 << 1;
            A[i].re = (loga2_re * Z0_idx_0_im + Z0_idx_0_re_tmp_tmp) / bim;
            A[i].im = (Z0_idx_0_im - loga2_re * Z0_idx_0_re_tmp_tmp) / bim;
          } else if (bim == loga2_re) {
            if (re_0 > 0.0) {
              bim = 0.5;
            } else {
              bim = -0.5;
            }

            if (Z0_tmp_idx_0_re > 0.0) {
              sgnbi = 0.5;
            } else {
              sgnbi = -0.5;
            }

            _mm_storeu_pd((real_T *)&A[r1 << 1], _mm_div_pd(_mm_add_pd
              (_mm_mul_pd(_mm_set_pd(Z0_idx_0_im, Z0_idx_0_re_tmp_tmp),
                          _mm_set1_pd(bim)), _mm_mul_pd(_mm_mul_pd(_mm_set_pd
              (Z0_idx_0_re_tmp_tmp, Z0_idx_0_im), _mm_set1_pd(sgnbi)),
              _mm_set_pd(-1.0, 1.0))), _mm_set1_pd(loga2_re)));
          } else {
            loga2_re = re_0 / Z0_tmp_idx_0_re;
            _mm_storeu_pd((real_T *)&A[r1 << 1], _mm_div_pd(_mm_add_pd
              (_mm_mul_pd(_mm_set1_pd(loga2_re), _mm_set_pd(Z0_idx_0_im,
              Z0_idx_0_re_tmp_tmp)), _mm_mul_pd(_mm_set_pd(Z0_idx_0_re_tmp_tmp,
              Z0_idx_0_im), _mm_set_pd(-1.0, 1.0))), _mm_set1_pd(loga2_re * re_0
              + Z0_tmp_idx_0_re)));
          }
        }

        i = r1 << 1;
        tmp_1 = _mm_sub_pd(_mm_set_pd(im, re), _mm_add_pd(_mm_mul_pd(_mm_set1_pd
          (A[i].re), _mm_loadu_pd((const real_T *)&P[r1 + 2])), _mm_mul_pd
          (_mm_mul_pd(_mm_set1_pd(A[i].im), _mm_shuffle_pd(_mm_loadu_pd((const
          real_T *)&P[r1 + 2]), _mm_loadu_pd((const real_T *)&P[r1 + 2]), 1)),
           tmp_0)));
        _mm_storeu_pd(&tmp[0], tmp_1);
        if (r22.im == 0.0) {
          if (tmp[1] == 0.0) {
            tmp_5 = r2 << 1;
            A[tmp_5].re = tmp[0] / r22.re;
            A[tmp_5].im = 0.0;
          } else if (tmp[0] == 0.0) {
            tmp_5 = r2 << 1;
            A[tmp_5].re = 0.0;
            A[tmp_5].im = tmp[1] / r22.re;
          } else {
            _mm_storeu_pd((real_T *)&A[r2 << 1], _mm_div_pd(_mm_set_pd(tmp[1],
              tmp[0]), _mm_set1_pd(r22.re)));
          }
        } else if (r22.re == 0.0) {
          if (tmp[0] == 0.0) {
            tmp_5 = r2 << 1;
            A[tmp_5].re = tmp[1] / r22.im;
            A[tmp_5].im = 0.0;
          } else if (tmp[1] == 0.0) {
            tmp_5 = r2 << 1;
            A[tmp_5].re = 0.0;
            A[tmp_5].im = -(tmp[0] / r22.im);
          } else {
            tmp_5 = r2 << 1;
            A[tmp_5].re = tmp[1] / r22.im;
            A[tmp_5].im = -(tmp[0] / r22.im);
          }
        } else {
          loga2_re = fabs(r22.re);
          bim = fabs(r22.im);
          if (loga2_re > bim) {
            loga2_re = r22.im / r22.re;
            bim = loga2_re * r22.im + r22.re;
            tmp_5 = r2 << 1;
            A[tmp_5].re = (loga2_re * tmp[1] + tmp[0]) / bim;
            A[tmp_5].im = (tmp[1] - loga2_re * tmp[0]) / bim;
          } else if (bim == loga2_re) {
            if (r22.re > 0.0) {
              bim = 0.5;
            } else {
              bim = -0.5;
            }

            if (r22.im > 0.0) {
              sgnbi = 0.5;
            } else {
              sgnbi = -0.5;
            }

            _mm_storeu_pd((real_T *)&A[r2 << 1], _mm_div_pd(_mm_add_pd
              (_mm_mul_pd(_mm_set_pd(tmp[1], tmp[0]), _mm_set1_pd(bim)),
               _mm_mul_pd(_mm_mul_pd(_mm_set_pd(tmp[0], tmp[1]), _mm_set1_pd
              (sgnbi)), _mm_set_pd(-1.0, 1.0))), _mm_set1_pd(loga2_re)));
          } else {
            loga2_re = r22.re / r22.im;
            _mm_storeu_pd((real_T *)&A[r2 << 1], _mm_div_pd(_mm_add_pd
              (_mm_mul_pd(_mm_set1_pd(loga2_re), _mm_set_pd(tmp[1], tmp[0])),
               _mm_mul_pd(_mm_set_pd(tmp[0], tmp[1]), _mm_set_pd(-1.0, 1.0))),
              _mm_set1_pd(loga2_re * r22.re + r22.im)));
          }
        }

        r2 <<= 1;
        re = A[r2].re;
        im = A[r2].im;
        A[i].re -= re * r11.re - im * r11.im;
        A[i].im -= re * r11.im + im * r11.re;
        if (Z0_tmp_idx_0_re == 0.0) {
          if (loga2_im == 0.0) {
            A[i + 1].re = Z0_idx_1_re_tmp / re_0;
            A[i + 1].im = 0.0;
          } else if (Z0_idx_1_re_tmp == 0.0) {
            A[i + 1].re = 0.0;
            A[i + 1].im = loga2_im / re_0;
          } else {
            _mm_storeu_pd((real_T *)&A[i + 1], _mm_div_pd(_mm_set_pd(loga2_im,
              Z0_idx_1_re_tmp), _mm_set1_pd(re_0)));
          }
        } else if (re_0 == 0.0) {
          if (Z0_idx_1_re_tmp == 0.0) {
            A[i + 1].re = loga2_im / Z0_tmp_idx_0_re;
            A[i + 1].im = 0.0;
          } else if (loga2_im == 0.0) {
            A[i + 1].re = 0.0;
            A[i + 1].im = -(Z0_idx_1_re_tmp / Z0_tmp_idx_0_re);
          } else {
            A[i + 1].re = loga2_im / Z0_tmp_idx_0_re;
            A[i + 1].im = -(Z0_idx_1_re_tmp / Z0_tmp_idx_0_re);
          }
        } else {
          loga2_re = fabs(re_0);
          bim = fabs(Z0_tmp_idx_0_re);
          if (loga2_re > bim) {
            loga2_re = Z0_tmp_idx_0_re / re_0;
            bim = loga2_re * Z0_tmp_idx_0_re + re_0;
            A[i + 1].re = (loga2_re * loga2_im + Z0_idx_1_re_tmp) / bim;
            A[i + 1].im = (loga2_im - loga2_re * Z0_idx_1_re_tmp) / bim;
          } else if (bim == loga2_re) {
            if (re_0 > 0.0) {
              bim = 0.5;
            } else {
              bim = -0.5;
            }

            if (Z0_tmp_idx_0_re > 0.0) {
              sgnbi = 0.5;
            } else {
              sgnbi = -0.5;
            }

            _mm_storeu_pd((real_T *)&A[i + 1], _mm_div_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set_pd(loga2_im, Z0_idx_1_re_tmp), _mm_set1_pd(bim)),
              _mm_mul_pd(_mm_mul_pd(_mm_set_pd(Z0_idx_1_re_tmp, loga2_im),
              _mm_set1_pd(sgnbi)), _mm_set_pd(-1.0, 1.0))), _mm_set1_pd(loga2_re)));
          } else {
            loga2_re = re_0 / Z0_tmp_idx_0_re;
            _mm_storeu_pd((real_T *)&A[i + 1], _mm_div_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set1_pd(loga2_re), _mm_set_pd(loga2_im, Z0_idx_1_re_tmp)),
              _mm_mul_pd(_mm_set_pd(Z0_idx_1_re_tmp, loga2_im), _mm_set_pd(-1.0,
              1.0))), _mm_set1_pd(loga2_re * re_0 + Z0_tmp_idx_0_re)));
          }
        }

        tmp_1 = _mm_sub_pd(_mm_set_pd(Z0_idx_3_im, loga2_re_tmp), _mm_add_pd
                           (_mm_mul_pd(_mm_set1_pd(A[i + 1].re), _mm_loadu_pd((
          const real_T *)&P[r1 + 2])), _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(A[i + 1]
          .im), _mm_shuffle_pd(_mm_loadu_pd((const real_T *)&P[r1 + 2]),
          _mm_loadu_pd((const real_T *)&P[r1 + 2]), 1)), tmp_0)));
        _mm_storeu_pd(&tmp[0], tmp_1);
        if (r22.im == 0.0) {
          if (tmp[1] == 0.0) {
            A[r2 + 1].re = tmp[0] / r22.re;
            A[r2 + 1].im = 0.0;
          } else if (tmp[0] == 0.0) {
            A[r2 + 1].re = 0.0;
            A[r2 + 1].im = tmp[1] / r22.re;
          } else {
            _mm_storeu_pd((real_T *)&A[r2 + 1], _mm_div_pd(_mm_set_pd(tmp[1],
              tmp[0]), _mm_set1_pd(r22.re)));
          }
        } else if (r22.re == 0.0) {
          if (tmp[0] == 0.0) {
            A[r2 + 1].re = tmp[1] / r22.im;
            A[r2 + 1].im = 0.0;
          } else if (tmp[1] == 0.0) {
            A[r2 + 1].re = 0.0;
            A[r2 + 1].im = -(tmp[0] / r22.im);
          } else {
            A[r2 + 1].re = tmp[1] / r22.im;
            A[r2 + 1].im = -(tmp[0] / r22.im);
          }
        } else {
          loga2_re = fabs(r22.re);
          bim = fabs(r22.im);
          if (loga2_re > bim) {
            loga2_re = r22.im / r22.re;
            bim = loga2_re * r22.im + r22.re;
            A[r2 + 1].re = (loga2_re * tmp[1] + tmp[0]) / bim;
            A[r2 + 1].im = (tmp[1] - loga2_re * tmp[0]) / bim;
          } else if (bim == loga2_re) {
            if (r22.re > 0.0) {
              bim = 0.5;
            } else {
              bim = -0.5;
            }

            if (r22.im > 0.0) {
              sgnbi = 0.5;
            } else {
              sgnbi = -0.5;
            }

            _mm_storeu_pd((real_T *)&A[r2 + 1], _mm_div_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set_pd(tmp[1], tmp[0]), _mm_set1_pd(bim)), _mm_mul_pd
              (_mm_mul_pd(_mm_set_pd(tmp[0], tmp[1]), _mm_set1_pd(sgnbi)),
               _mm_set_pd(-1.0, 1.0))), _mm_set1_pd(loga2_re)));
          } else {
            loga2_re = r22.re / r22.im;
            _mm_storeu_pd((real_T *)&A[r2 + 1], _mm_div_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set1_pd(loga2_re), _mm_set_pd(tmp[1], tmp[0])), _mm_mul_pd
              (_mm_set_pd(tmp[0], tmp[1]), _mm_set_pd(-1.0, 1.0))), _mm_set1_pd
              (loga2_re * r22.re + r22.im)));
          }
        }

        re = A[r2 + 1].re;
        im = A[r2 + 1].im;
        A[i + 1].re -= re * r11.re - im * r11.im;
        A[i + 1].im -= re * r11.im + im * r11.re;
        Troot[A_tmp_tmp] = A[0];
        Troot[A_tmp_tmp + 1] = A[1];
        Troot[A_tmp] = A[2];
        Troot[A_tmp + 1] = A[3];
        if ((T[A_tmp_tmp + 1].re == 0.0) && (T[A_tmp_tmp + 1].im == 0.0) &&
            (T[A_tmp_tmp].re >= 0.0) && (T[A_tmp + 1].re >= 0.0)) {
          A[0] = T[A_tmp_tmp];
          Z0_idx_1_re_tmp = 1.0 / rt_powd_snf(2.0, (real_T)s);
          if ((T[A_tmp + 1].re == T[A_tmp_tmp].re) && (T[A_tmp + 1].im ==
               T[A_tmp_tmp].im)) {
            r11 = LR_power(A[0], Z0_idx_1_re_tmp - 1.0);
            _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_loadu_pd((const real_T *)
              &T[A_tmp]), _mm_set1_pd(Z0_idx_1_re_tmp)));
            tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(tmp[0]), _mm_loadu_pd((
              const real_T *)&r11)), _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(tmp[1]),
              _mm_shuffle_pd(_mm_loadu_pd((const real_T *)&r11), _mm_loadu_pd((
              const real_T *)&r11), 1)), tmp_0));
            _mm_storeu_pd((real_T *)&Troot[A_tmp], tmp_0);
          } else {
            tmp_1 = _mm_loadu_pd((const real_T *)&T[A_tmp + 1]);
            tmp_2 = _mm_loadu_pd((const real_T *)&T[A_tmp_tmp]);
            tmp_3 = _mm_sub_pd(tmp_1, tmp_2);
            _mm_storeu_pd(&tmp[0], tmp_3);
            Z0_tmp_idx_3_im = tmp[0];
            ai = tmp[1];
            _mm_storeu_pd(&tmp[0], _mm_add_pd(tmp_1, tmp_2));
            if (tmp[1] == 0.0) {
              if (ai == 0.0) {
                r11.re = Z0_tmp_idx_3_im / tmp[0];
                r11.im = 0.0;
              } else if (Z0_tmp_idx_3_im == 0.0) {
                r11.re = 0.0;
                r11.im = ai / tmp[0];
              } else {
                _mm_storeu_pd((real_T *)&r11, _mm_div_pd(_mm_set_pd(ai,
                  Z0_tmp_idx_3_im), _mm_set1_pd(tmp[0])));
              }
            } else if (tmp[0] == 0.0) {
              if (Z0_tmp_idx_3_im == 0.0) {
                r11.re = ai / tmp[1];
                r11.im = 0.0;
              } else if (ai == 0.0) {
                r11.re = 0.0;
                r11.im = -(Z0_tmp_idx_3_im / tmp[1]);
              } else {
                r11.re = ai / tmp[1];
                r11.im = -(Z0_tmp_idx_3_im / tmp[1]);
              }
            } else {
              loga2_re = fabs(tmp[0]);
              bim = fabs(tmp[1]);
              if (loga2_re > bim) {
                loga2_re = tmp[1] / tmp[0];
                bim = loga2_re * tmp[1] + tmp[0];
                r11.re = (loga2_re * ai + Z0_tmp_idx_3_im) / bim;
                r11.im = (ai - loga2_re * Z0_tmp_idx_3_im) / bim;
              } else if (bim == loga2_re) {
                if (tmp[0] > 0.0) {
                  bim = 0.5;
                } else {
                  bim = -0.5;
                }

                if (tmp[1] > 0.0) {
                  sgnbi = 0.5;
                } else {
                  sgnbi = -0.5;
                }

                _mm_storeu_pd((real_T *)&r11, _mm_div_pd(_mm_add_pd(_mm_mul_pd
                  (_mm_set_pd(ai, Z0_tmp_idx_3_im), _mm_set1_pd(bim)),
                  _mm_mul_pd(_mm_mul_pd(_mm_set_pd(Z0_tmp_idx_3_im, ai),
                  _mm_set1_pd(sgnbi)), _mm_set_pd(-1.0, 1.0))), _mm_set1_pd
                  (loga2_re)));
              } else {
                loga2_re = tmp[0] / tmp[1];
                _mm_storeu_pd((real_T *)&r11, _mm_div_pd(_mm_add_pd(_mm_mul_pd
                  (_mm_set1_pd(loga2_re), _mm_set_pd(ai, Z0_tmp_idx_3_im)),
                  _mm_mul_pd(_mm_set_pd(Z0_tmp_idx_3_im, ai), _mm_set_pd(-1.0,
                  1.0))), _mm_set1_pd(loga2_re * tmp[0] + tmp[1])));
              }
            }

            if ((rt_hypotd_snf(r11.re, r11.im) > 90.509667991878089) ||
                (rt_hypotd_snf(r11.re - 1.0, r11.im) < 0.011048543456039804) ||
                (rt_hypotd_snf(r11.re + 1.0, r11.im) < 0.011048543456039804)) {
              r11 = LR_power(T[(((j + 1) << 2) + j) + 1], Z0_idx_1_re_tmp);
              r22 = LR_power(A[0], Z0_idx_1_re_tmp);
              tmp_1 = _mm_sub_pd(_mm_loadu_pd((const real_T *)&r11),
                                 _mm_loadu_pd((const real_T *)&r22));
              _mm_storeu_pd(&tmp[0], tmp_1);
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(T[A_tmp].
                re), _mm_set_pd(tmp[1], tmp[0])), _mm_mul_pd(_mm_mul_pd
                (_mm_set1_pd(T[A_tmp].im), _mm_set_pd(tmp[0], tmp[1])), tmp_0)));
              Z0_idx_1_re_tmp = tmp[0];
              loga2_im = tmp[1];
              _mm_storeu_pd(&tmp[0], tmp_3);
              if (tmp[1] == 0.0) {
                if (loga2_im == 0.0) {
                  Troot[A_tmp].re = Z0_idx_1_re_tmp / tmp[0];
                  im = 0.0;
                } else if (Z0_idx_1_re_tmp == 0.0) {
                  Troot[A_tmp].re = 0.0;
                  im = loga2_im / tmp[0];
                } else {
                  Troot[A_tmp].re = Z0_idx_1_re_tmp / tmp[0];
                  im = loga2_im / tmp[0];
                }
              } else if (tmp[0] == 0.0) {
                if (Z0_idx_1_re_tmp == 0.0) {
                  Troot[A_tmp].re = loga2_im / tmp[1];
                  im = 0.0;
                } else if (loga2_im == 0.0) {
                  Troot[A_tmp].re = 0.0;
                  im = -(Z0_idx_1_re_tmp / tmp[1]);
                } else {
                  Troot[A_tmp].re = loga2_im / tmp[1];
                  im = -(Z0_idx_1_re_tmp / tmp[1]);
                }
              } else {
                loga2_re = fabs(tmp[0]);
                bim = fabs(tmp[1]);
                if (loga2_re > bim) {
                  loga2_re = tmp[1] / tmp[0];
                  bim = loga2_re * tmp[1] + tmp[0];
                  Troot[A_tmp].re = (loga2_re * loga2_im + Z0_idx_1_re_tmp) /
                    bim;
                  im = (loga2_im - loga2_re * Z0_idx_1_re_tmp) / bim;
                } else if (bim == loga2_re) {
                  if (tmp[0] > 0.0) {
                    bim = 0.5;
                  } else {
                    bim = -0.5;
                  }

                  if (tmp[1] > 0.0) {
                    sgnbi = 0.5;
                  } else {
                    sgnbi = -0.5;
                  }

                  Troot[A_tmp].re = (Z0_idx_1_re_tmp * bim + loga2_im * sgnbi) /
                    loga2_re;
                  im = (loga2_im * bim - Z0_idx_1_re_tmp * sgnbi) / loga2_re;
                } else {
                  loga2_re = tmp[0] / tmp[1];
                  bim = loga2_re * tmp[0] + tmp[1];
                  Troot[A_tmp].re = (loga2_re * Z0_idx_1_re_tmp + loga2_im) /
                    bim;
                  im = (loga2_re * loga2_im - Z0_idx_1_re_tmp) / bim;
                }
              }

              Troot[A_tmp].im = im;
            } else {
              if (T[A_tmp_tmp].im == 0.0) {
                if (T[A_tmp_tmp].re < 0.0) {
                  r22.re = log(fabs(T[A_tmp_tmp].re));
                  r22.im = 3.1415926535897931;
                } else {
                  r22.re = log(fabs(T[A_tmp_tmp].re));
                  r22.im = 0.0;
                }
              } else if ((fabs(T[A_tmp_tmp].re) > 8.9884656743115785E+307) ||
                         (fabs(T[A_tmp_tmp].im) > 8.9884656743115785E+307)) {
                loga2_im = T[A_tmp_tmp].im;
                loga2_re = T[A_tmp_tmp].re;
                r22.re = log(rt_hypotd_snf(loga2_re / 2.0, loga2_im / 2.0)) +
                  0.69314718055994529;
                r22.im = rt_atan2d_snf(loga2_im, loga2_re);
              } else {
                loga2_im = T[A_tmp_tmp].im;
                loga2_re = T[A_tmp_tmp].re;
                r22.re = log(rt_hypotd_snf(loga2_re, loga2_im));
                r22.im = rt_atan2d_snf(loga2_im, loga2_re);
              }

              if (T[A_tmp + 1].im == 0.0) {
                if (T[A_tmp + 1].re < 0.0) {
                  loga2_re = log(fabs(T[A_tmp + 1].re));
                  loga2_im = 3.1415926535897931;
                } else {
                  loga2_re = log(fabs(T[A_tmp + 1].re));
                  loga2_im = 0.0;
                }
              } else if ((fabs(T[A_tmp + 1].re) > 8.9884656743115785E+307) ||
                         (fabs(T[A_tmp + 1].im) > 8.9884656743115785E+307)) {
                loga2_im = T[A_tmp + 1].im;
                Z0_idx_3_im = T[A_tmp + 1].re;
                loga2_re = log(rt_hypotd_snf(Z0_idx_3_im / 2.0, loga2_im / 2.0))
                  + 0.69314718055994529;
                loga2_im = rt_atan2d_snf(loga2_im, Z0_idx_3_im);
              } else {
                loga2_im = T[A_tmp + 1].im;
                Z0_idx_3_im = T[A_tmp + 1].re;
                loga2_re = log(rt_hypotd_snf(Z0_idx_3_im, loga2_im));
                loga2_im = rt_atan2d_snf(loga2_im, Z0_idx_3_im);
              }

              tmp_1 = _mm_set1_pd(Z0_idx_1_re_tmp);
              tmp_2 = _mm_mul_pd(_mm_add_pd(_mm_loadu_pd((const real_T *)&r22),
                _mm_set_pd(loga2_im, loga2_re)), tmp_1);
              _mm_storeu_pd(&tmp[0], tmp_2);
              if (tmp[1] == 0.0) {
                Z0_idx_1_re_tmp = tmp[0] / 2.0;
                loga2_re = 0.0;
              } else if (tmp[0] == 0.0) {
                Z0_idx_1_re_tmp = 0.0;
                loga2_re = tmp[1] / 2.0;
              } else {
                Z0_idx_1_re_tmp = tmp[0] / 2.0;
                loga2_re = tmp[1] / 2.0;
              }

              if (Z0_idx_1_re_tmp == 0.0) {
                Z0_idx_1_re_tmp = cos(loga2_re);
                loga2_re = sin(loga2_re);
              } else if (loga2_re == 0.0) {
                Z0_idx_1_re_tmp = exp(Z0_idx_1_re_tmp);
                loga2_re = 0.0;
              } else if (rtIsInf(loga2_re) && rtIsInf(Z0_idx_1_re_tmp) &&
                         (Z0_idx_1_re_tmp < 0.0)) {
                Z0_idx_1_re_tmp = 0.0;
                loga2_re = 0.0;
              } else {
                Z0_idx_3_im = exp(Z0_idx_1_re_tmp / 2.0);
                Z0_idx_1_re_tmp = Z0_idx_3_im * cos(loga2_re) * Z0_idx_3_im;
                loga2_re = Z0_idx_3_im * sin(loga2_re) * Z0_idx_3_im;
              }

              LR_atanh_h(&r11);
              tmp_1 = _mm_mul_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(ceil
                (((loga2_im - r22.im) - 3.1415926535897931) / 6.2831853071795862)),
                _mm_set_pd(3.1415926535897931, 0.0)), _mm_loadu_pd((const real_T
                *)&r11)), tmp_1);
              _mm_storeu_pd((real_T *)&r11, tmp_1);
              if (r11.im == 0.0) {
                r11.re = sinh(r11.re);
                r11.im = 0.0;
              } else {
                loga2_im = r11.re;
                r11.re = sinh(r11.re) * cos(r11.im);
                r11.im = cosh(loga2_im) * sin(r11.im);
              }

              tmp_1 = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(2.0 * Z0_idx_1_re_tmp),
                _mm_loadu_pd((const real_T *)&r11)), _mm_mul_pd(_mm_mul_pd
                (_mm_set1_pd(2.0 * loga2_re), _mm_shuffle_pd(_mm_loadu_pd((const
                real_T *)&r11), _mm_loadu_pd((const real_T *)&r11), 1)), tmp_0));
              _mm_storeu_pd(&tmp[0], tmp_1);
              re = tmp[0];
              im = tmp[1];
              _mm_storeu_pd(&tmp[0], tmp_3);
              if (tmp[1] == 0.0) {
                if (im == 0.0) {
                  re_0 = re / tmp[0];
                  im = 0.0;
                } else if (re == 0.0) {
                  re_0 = 0.0;
                  im /= tmp[0];
                } else {
                  re_0 = re / tmp[0];
                  im /= tmp[0];
                }
              } else if (tmp[0] == 0.0) {
                if (re == 0.0) {
                  re_0 = im / tmp[1];
                  im = 0.0;
                } else if (im == 0.0) {
                  re_0 = 0.0;
                  im = -(re / tmp[1]);
                } else {
                  re_0 = im / tmp[1];
                  im = -(re / tmp[1]);
                }
              } else {
                loga2_re = fabs(tmp[0]);
                bim = fabs(tmp[1]);
                if (loga2_re > bim) {
                  loga2_re = tmp[1] / tmp[0];
                  bim = loga2_re * tmp[1] + tmp[0];
                  re_0 = (loga2_re * im + re) / bim;
                  im = (im - loga2_re * re) / bim;
                } else if (bim == loga2_re) {
                  if (tmp[0] > 0.0) {
                    bim = 0.5;
                  } else {
                    bim = -0.5;
                  }

                  if (tmp[1] > 0.0) {
                    sgnbi = 0.5;
                  } else {
                    sgnbi = -0.5;
                  }

                  re_0 = (re * bim + im * sgnbi) / loga2_re;
                  im = (im * bim - re * sgnbi) / loga2_re;
                } else {
                  loga2_re = tmp[0] / tmp[1];
                  bim = loga2_re * tmp[0] + tmp[1];
                  re_0 = (loga2_re * re + im) / bim;
                  im = (loga2_re * im - re) / bim;
                }
              }

              _mm_storeu_pd((real_T *)&Troot[A_tmp], _mm_add_pd(_mm_mul_pd
                (_mm_set1_pd(T[A_tmp].re), _mm_set_pd(im, re_0)), _mm_mul_pd
                (_mm_mul_pd(_mm_set1_pd(T[A_tmp].im), _mm_set_pd(re_0, im)),
                 tmp_0)));
            }
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_computeLogOfSchurForm_m(const creal_T T[16], const creal_T d[4],
  creal_T L[16], int32_T *exitflag)
{
  __m128d tmp_0;
  creal_T Troot[16];
  creal_T TrootmI[16];
  creal_T TrootmI_0[16];
  creal_T TrootmI_tmp[16];
  creal_T d3_tmp[16];
  creal_T d3_tmp_0[16];
  creal_T b_d[4];
  creal_T z;
  real_T tmp[2];
  real_T T_0;
  real_T TrootmI_1;
  real_T TrootmI_2;
  real_T a3;
  real_T absx;
  real_T d3;
  real_T d3_tmp_1;
  real_T d3_tmp_2;
  real_T d3_tmp_3;
  real_T d3_tmp_4;
  real_T loga2_im;
  real_T loga2_re;
  int32_T TrootmI_im_tmp;
  int32_T b_k;
  int32_T c_ix;
  int32_T c_k;
  int32_T h_k;
  int32_T ind;
  int32_T ipiv_tmp;
  int32_T ix;
  int32_T iy;
  int32_T m;
  int32_T r_tmp;
  int32_T s0;
  int8_T b_I[16];
  int8_T ipiv[4];
  int8_T ipiv_0;
  boolean_T foundm;
  boolean_T more;
  static const real_T r[49] = { 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.21132486540518711, 0.78867513459481287, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.11270166537925831, 0.5, 0.8872983346207417, 0.0, 0.0, 0.0, 0.0,
    0.069431844202973714, 0.33000947820757187, 0.66999052179242813,
    0.93056815579702634, 0.0, 0.0, 0.0, 0.046910077030668004,
    0.23076534494715845, 0.5, 0.7692346550528415, 0.953089922969332, 0.0, 0.0,
    0.033765242898423989, 0.16939530676686773, 0.38069040695840156,
    0.61930959304159849, 0.83060469323313224, 0.966234757101576, 0.0,
    0.025446043828620736, 0.12923440720030277, 0.29707742431130141, 0.5,
    0.70292257568869854, 0.87076559279969723, 0.9745539561713793 };

  static const real_T q[7] = { 1.5869707387720632E-5, 0.0023138078842429789,
    0.019381793135332531, 0.062091715889947621, 0.12764048108067749,
    0.20609626234528361, 0.28790937142411938 };

  static const real_T p[49] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.27777777777777779, 0.44444444444444442,
    0.27777777777777779, 0.0, 0.0, 0.0, 0.0, 0.17392742256872692,
    0.32607257743127305, 0.32607257743127305, 0.17392742256872692, 0.0, 0.0, 0.0,
    0.11846344252809454, 0.23931433524968324, 0.28444444444444444,
    0.23931433524968324, 0.11846344252809454, 0.0, 0.0, 0.085662246189585178,
    0.1803807865240693, 0.23395696728634552, 0.23395696728634552,
    0.1803807865240693, 0.085662246189585178, 0.0, 0.064742483084434851,
    0.13985269574463832, 0.19091502525255946, 0.2089795918367347,
    0.19091502525255946, 0.13985269574463832, 0.064742483084434851 };

  int32_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T guard1;
  boolean_T guard2;
  memcpy(&b_d[0], &d[0], sizeof(creal_T) << 2U);
  m = 0;
  *exitflag = 0;
  for (ipiv_tmp = 0; ipiv_tmp < 16; ipiv_tmp++) {
    Troot[ipiv_tmp] = T[ipiv_tmp];
    b_I[ipiv_tmp] = 0;
  }

  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  c_k = 0;
  do {
    exitg1 = 0;
    d3 = 0.0;
    for (s0 = 0; s0 < 4; s0++) {
      absx = rt_hypotd_snf(b_d[s0].re - 1.0, b_d[s0].im);
      if (rtIsNaN(absx) || (absx > d3)) {
        d3 = absx;
      }
    }

    if ((d3 > 0.28790937142411938) && (c_k < 100)) {
      LR_sqrt(&b_d[0]);
      LR_sqrt(&b_d[1]);
      LR_sqrt(&b_d[2]);
      LR_sqrt(&b_d[3]);
      c_k++;
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  s0 = c_k;
  if (c_k == 100) {
    *exitflag = 1;
  }

  for (b_k = 0; b_k < c_k; b_k++) {
    LR_sqrtmTriRecursive_n(Troot, 1, 1, 4);
  }

  for (ipiv_tmp = 0; ipiv_tmp < 16; ipiv_tmp++) {
    b_k = b_I[ipiv_tmp];
    TrootmI_tmp[ipiv_tmp].re = b_k;
    TrootmI_tmp[ipiv_tmp].im = 0.0;
    TrootmI[ipiv_tmp].re = Troot[ipiv_tmp].re - (real_T)b_k;
    TrootmI[ipiv_tmp].im = Troot[ipiv_tmp].im;
  }

  for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
    for (h_k = 0; h_k < 4; h_k++) {
      ind = ipiv_tmp << 2;
      tmp_0 = _mm_set_pd(1.0, -1.0);
      tmp_0 = _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_loadu_pd((const real_T *)&TrootmI[ind + 1]), _mm_set1_pd
         (TrootmI[h_k + 4].re)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
        (_mm_loadu_pd((const real_T *)&TrootmI[ind + 1]), _mm_loadu_pd((const
        real_T *)&TrootmI[ind + 1]), 1), _mm_set1_pd(TrootmI[h_k + 4].im)),
        tmp_0)), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)&TrootmI[ind]),
        _mm_set1_pd(TrootmI[h_k].re)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
        (_mm_loadu_pd((const real_T *)&TrootmI[ind]), _mm_loadu_pd((const real_T
        *)&TrootmI[ind]), 1), _mm_set1_pd(TrootmI[h_k].im)), tmp_0))),
        _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)&TrootmI[ind + 2]),
        _mm_set1_pd(TrootmI[h_k + 8].re)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
        (_mm_loadu_pd((const real_T *)&TrootmI[ind + 2]), _mm_loadu_pd((const
        real_T *)&TrootmI[ind + 2]), 1), _mm_set1_pd(TrootmI[h_k + 8].im)),
        tmp_0))), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)
        &TrootmI[ind + 3]), _mm_set1_pd(TrootmI[h_k + 12].re)), _mm_mul_pd
        (_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)&TrootmI[ind + 3]),
        _mm_loadu_pd((const real_T *)&TrootmI[ind + 3]), 1), _mm_set1_pd
                    (TrootmI[h_k + 12].im)), tmp_0)));
      _mm_storeu_pd((real_T *)&d3_tmp[h_k + ind], tmp_0);
    }

    b_k = ipiv_tmp << 2;
    d3 = d3_tmp[b_k + 1].re;
    absx = d3_tmp[b_k + 1].im;
    d3_tmp_1 = d3_tmp[b_k].re;
    d3_tmp_2 = d3_tmp[b_k].im;
    d3_tmp_3 = d3_tmp[b_k + 2].re;
    loga2_re = d3_tmp[b_k + 2].im;
    loga2_im = d3_tmp[b_k + 3].re;
    d3_tmp_4 = d3_tmp[b_k + 3].im;
    for (h_k = 0; h_k < 4; h_k++) {
      tmp_0 = _mm_set_pd(1.0, -1.0);
      _mm_storeu_pd((real_T *)&TrootmI_0[h_k + b_k], _mm_add_pd(_mm_add_pd
        (_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd(absx, d3), _mm_set1_pd
        (TrootmI[h_k + 4].re)), _mm_mul_pd(_mm_mul_pd(_mm_set_pd(d3, absx),
        _mm_set1_pd(TrootmI[h_k + 4].im)), tmp_0)), _mm_add_pd(_mm_mul_pd
        (_mm_set_pd(d3_tmp_2, d3_tmp_1), _mm_set1_pd(TrootmI[h_k].re)),
        _mm_mul_pd(_mm_mul_pd(_mm_set_pd(d3_tmp_1, d3_tmp_2), _mm_set1_pd
        (TrootmI[h_k].im)), tmp_0))), _mm_add_pd(_mm_mul_pd(_mm_set_pd(loga2_re,
        d3_tmp_3), _mm_set1_pd(TrootmI[h_k + 8].re)), _mm_mul_pd(_mm_mul_pd
        (_mm_set_pd(d3_tmp_3, loga2_re), _mm_set1_pd(TrootmI[h_k + 8].im)),
        tmp_0))), _mm_add_pd(_mm_mul_pd(_mm_set_pd(d3_tmp_4, loga2_im),
        _mm_set1_pd(TrootmI[h_k + 12].re)), _mm_mul_pd(_mm_mul_pd(_mm_set_pd
        (loga2_im, d3_tmp_4), _mm_set1_pd(TrootmI[h_k + 12].im)), tmp_0))));
    }
  }

  d3 = rt_powd_snf(LR_norm_j(TrootmI_0), 0.33333333333333331);
  absx = fmax(sqrt(LR_norm_j(d3_tmp)), d3);
  if (absx <= 1.5869707387720632E-5) {
    foundm = true;
  } else if (absx <= 0.0023138078842429789) {
    m = 1;
    foundm = true;
  } else {
    foundm = false;
  }

  b_k = 0;
  exitg2 = false;
  while ((!exitg2) && (!foundm)) {
    more = false;
    if (c_k > s0) {
      for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
        for (h_k = 0; h_k < 4; h_k++) {
          ind = h_k << 2;
          tmp_0 = _mm_set_pd(1.0, -1.0);
          tmp_0 = _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_loadu_pd((const real_T *)&TrootmI[ind + 1]), _mm_set1_pd
             (TrootmI[ipiv_tmp + 4].re)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
            (_mm_loadu_pd((const real_T *)&TrootmI[ind + 1]), _mm_loadu_pd((
            const real_T *)&TrootmI[ind + 1]), 1), _mm_set1_pd(TrootmI[ipiv_tmp
            + 4].im)), tmp_0)), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T
            *)&TrootmI[ind]), _mm_set1_pd(TrootmI[ipiv_tmp].re)), _mm_mul_pd
            (_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)&TrootmI[ind]),
            _mm_loadu_pd((const real_T *)&TrootmI[ind]), 1), _mm_set1_pd
                        (TrootmI[ipiv_tmp].im)), tmp_0))), _mm_add_pd(_mm_mul_pd
            (_mm_loadu_pd((const real_T *)&TrootmI[ind + 2]), _mm_set1_pd
             (TrootmI[ipiv_tmp + 8].re)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
            (_mm_loadu_pd((const real_T *)&TrootmI[ind + 2]), _mm_loadu_pd((
            const real_T *)&TrootmI[ind + 2]), 1), _mm_set1_pd(TrootmI[ipiv_tmp
            + 8].im)), tmp_0))), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const
            real_T *)&TrootmI[ind + 3]), _mm_set1_pd(TrootmI[ipiv_tmp + 12].re)),
            _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)
            &TrootmI[ind + 3]), _mm_loadu_pd((const real_T *)&TrootmI[ind + 3]),
            1), _mm_set1_pd(TrootmI[ipiv_tmp + 12].im)), tmp_0)));
          _mm_storeu_pd((real_T *)&TrootmI_0[ipiv_tmp + ind], tmp_0);
        }
      }

      for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
        d3_tmp_1 = TrootmI[ipiv_tmp + 4].re;
        d3_tmp_2 = TrootmI[ipiv_tmp + 4].im;
        d3_tmp_3 = TrootmI[ipiv_tmp].re;
        loga2_re = TrootmI[ipiv_tmp].im;
        loga2_im = TrootmI[ipiv_tmp + 8].re;
        d3_tmp_4 = TrootmI[ipiv_tmp + 8].im;
        TrootmI_1 = TrootmI[ipiv_tmp + 12].re;
        TrootmI_2 = TrootmI[ipiv_tmp + 12].im;
        for (h_k = 0; h_k < 4; h_k++) {
          ind = h_k << 2;
          tmp_0 = _mm_set_pd(1.0, -1.0);
          tmp_0 = _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_loadu_pd((const real_T *)&TrootmI_0[ind + 1]), _mm_set1_pd
             (d3_tmp_1)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((
            const real_T *)&TrootmI_0[ind + 1]), _mm_loadu_pd((const real_T *)
            &TrootmI_0[ind + 1]), 1), _mm_set1_pd(d3_tmp_2)), tmp_0)),
            _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)&TrootmI_0[ind]),
            _mm_set1_pd(d3_tmp_3)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
            (_mm_loadu_pd((const real_T *)&TrootmI_0[ind]), _mm_loadu_pd((const
            real_T *)&TrootmI_0[ind]), 1), _mm_set1_pd(loga2_re)), tmp_0))),
            _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)&TrootmI_0[ind +
            2]), _mm_set1_pd(loga2_im)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
            (_mm_loadu_pd((const real_T *)&TrootmI_0[ind + 2]), _mm_loadu_pd((
            const real_T *)&TrootmI_0[ind + 2]), 1), _mm_set1_pd(d3_tmp_4)),
            tmp_0))), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)
            &TrootmI_0[ind + 3]), _mm_set1_pd(TrootmI_1)), _mm_mul_pd(_mm_mul_pd
            (_mm_shuffle_pd(_mm_loadu_pd((const real_T *)&TrootmI_0[ind + 3]),
                            _mm_loadu_pd((const real_T *)&TrootmI_0[ind + 3]), 1),
             _mm_set1_pd(TrootmI_2)), tmp_0)));
          _mm_storeu_pd((real_T *)&d3_tmp[ipiv_tmp + ind], tmp_0);
        }
      }

      d3 = rt_powd_snf(LR_norm_j(d3_tmp), 0.33333333333333331);
    }

    for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
      for (h_k = 0; h_k < 4; h_k++) {
        ind = h_k << 2;
        tmp_0 = _mm_set_pd(1.0, -1.0);
        tmp_0 = _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd((const real_T *)&TrootmI[ind + 1]), _mm_set1_pd
           (TrootmI[ipiv_tmp + 4].re)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
          (_mm_loadu_pd((const real_T *)&TrootmI[ind + 1]), _mm_loadu_pd((const
          real_T *)&TrootmI[ind + 1]), 1), _mm_set1_pd(TrootmI[ipiv_tmp + 4].im)),
          tmp_0)), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)
          &TrootmI[ind]), _mm_set1_pd(TrootmI[ipiv_tmp].re)), _mm_mul_pd
                              (_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const
          real_T *)&TrootmI[ind]), _mm_loadu_pd((const real_T *)&TrootmI[ind]),
          1), _mm_set1_pd(TrootmI[ipiv_tmp].im)), tmp_0))), _mm_add_pd
          (_mm_mul_pd(_mm_loadu_pd((const real_T *)&TrootmI[ind + 2]),
                      _mm_set1_pd(TrootmI[ipiv_tmp + 8].re)), _mm_mul_pd
           (_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)&TrootmI[ind
          + 2]), _mm_loadu_pd((const real_T *)&TrootmI[ind + 2]), 1),
                       _mm_set1_pd(TrootmI[ipiv_tmp + 8].im)), tmp_0))),
                           _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)
          &TrootmI[ind + 3]), _mm_set1_pd(TrootmI[ipiv_tmp + 12].re)),
          _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)
          &TrootmI[ind + 3]), _mm_loadu_pd((const real_T *)&TrootmI[ind + 3]), 1),
          _mm_set1_pd(TrootmI[ipiv_tmp + 12].im)), tmp_0)));
        _mm_storeu_pd((real_T *)&d3_tmp[ipiv_tmp + ind], tmp_0);
      }
    }

    for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
      for (h_k = 0; h_k < 4; h_k++) {
        ind = h_k << 2;
        tmp_0 = _mm_set_pd(1.0, -1.0);
        tmp_0 = _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd((const real_T *)&d3_tmp[ind + 1]), _mm_set1_pd
           (d3_tmp[ipiv_tmp + 4].re)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
          (_mm_loadu_pd((const real_T *)&d3_tmp[ind + 1]), _mm_loadu_pd((const
          real_T *)&d3_tmp[ind + 1]), 1), _mm_set1_pd(d3_tmp[ipiv_tmp + 4].im)),
          tmp_0)), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)
          &d3_tmp[ind]), _mm_set1_pd(d3_tmp[ipiv_tmp].re)), _mm_mul_pd
                              (_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const
          real_T *)&d3_tmp[ind]), _mm_loadu_pd((const real_T *)&d3_tmp[ind]), 1),
          _mm_set1_pd(d3_tmp[ipiv_tmp].im)), tmp_0))), _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd((const real_T *)&d3_tmp[ind + 2]), _mm_set1_pd
           (d3_tmp[ipiv_tmp + 8].re)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
          (_mm_loadu_pd((const real_T *)&d3_tmp[ind + 2]), _mm_loadu_pd((const
          real_T *)&d3_tmp[ind + 2]), 1), _mm_set1_pd(d3_tmp[ipiv_tmp + 8].im)),
          tmp_0))), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)
          &d3_tmp[ind + 3]), _mm_set1_pd(d3_tmp[ipiv_tmp + 12].re)), _mm_mul_pd
          (_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)&d3_tmp[ind +
          3]), _mm_loadu_pd((const real_T *)&d3_tmp[ind + 3]), 1), _mm_set1_pd
                      (d3_tmp[ipiv_tmp + 12].im)), tmp_0)));
        _mm_storeu_pd((real_T *)&d3_tmp_0[ipiv_tmp + ind], tmp_0);
      }
    }

    absx = rt_powd_snf(LR_norm_j(d3_tmp_0), 0.25);
    a3 = fmax(d3, absx);
    guard1 = false;
    guard2 = false;
    if (a3 <= 0.28790937142411938) {
      ind = 0;
      iy = 3;
      exitg3 = false;
      while ((!exitg3) && (iy < 8)) {
        if (a3 <= q[iy - 1]) {
          ind = iy;
          exitg3 = true;
        } else {
          iy++;
        }
      }

      if (ind <= 6) {
        m = ind - 1;
        exitg2 = true;
      } else {
        if ((a3 / 2.0 <= 0.12764048108067749) && (b_k < 2)) {
          more = true;
          b_k++;
        }

        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      if (!more) {
        for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
          for (h_k = 0; h_k < 4; h_k++) {
            ind = h_k << 2;
            tmp_0 = _mm_set_pd(1.0, -1.0);
            tmp_0 = _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
              (_mm_loadu_pd((const real_T *)&d3_tmp[ind + 1]), _mm_set1_pd
               (d3_tmp[ipiv_tmp + 4].re)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
              (_mm_loadu_pd((const real_T *)&d3_tmp[ind + 1]), _mm_loadu_pd((
              const real_T *)&d3_tmp[ind + 1]), 1), _mm_set1_pd(d3_tmp[ipiv_tmp
              + 4].im)), tmp_0)), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const
              real_T *)&d3_tmp[ind]), _mm_set1_pd(d3_tmp[ipiv_tmp].re)),
              _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)
              &d3_tmp[ind]), _mm_loadu_pd((const real_T *)&d3_tmp[ind]), 1),
              _mm_set1_pd(d3_tmp[ipiv_tmp].im)), tmp_0))), _mm_add_pd(_mm_mul_pd
              (_mm_loadu_pd((const real_T *)&d3_tmp[ind + 2]), _mm_set1_pd
               (d3_tmp[ipiv_tmp + 8].re)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
              (_mm_loadu_pd((const real_T *)&d3_tmp[ind + 2]), _mm_loadu_pd((
              const real_T *)&d3_tmp[ind + 2]), 1), _mm_set1_pd(d3_tmp[ipiv_tmp
              + 8].im)), tmp_0))), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const
              real_T *)&d3_tmp[ind + 3]), _mm_set1_pd(d3_tmp[ipiv_tmp + 12].re)),
              _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)
              &d3_tmp[ind + 3]), _mm_loadu_pd((const real_T *)&d3_tmp[ind + 3]),
              1), _mm_set1_pd(d3_tmp[ipiv_tmp + 12].im)), tmp_0)));
            _mm_storeu_pd((real_T *)&d3_tmp_0[ipiv_tmp + ind], tmp_0);
          }
        }

        for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
          d3_tmp_1 = TrootmI[ipiv_tmp + 4].re;
          d3_tmp_2 = TrootmI[ipiv_tmp + 4].im;
          d3_tmp_3 = TrootmI[ipiv_tmp].re;
          loga2_re = TrootmI[ipiv_tmp].im;
          loga2_im = TrootmI[ipiv_tmp + 8].re;
          d3_tmp_4 = TrootmI[ipiv_tmp + 8].im;
          TrootmI_1 = TrootmI[ipiv_tmp + 12].re;
          TrootmI_2 = TrootmI[ipiv_tmp + 12].im;
          for (h_k = 0; h_k < 4; h_k++) {
            ind = h_k << 2;
            tmp_0 = _mm_set_pd(1.0, -1.0);
            tmp_0 = _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
              (_mm_loadu_pd((const real_T *)&d3_tmp_0[ind + 1]), _mm_set1_pd
               (d3_tmp_1)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((
              const real_T *)&d3_tmp_0[ind + 1]), _mm_loadu_pd((const real_T *)
              &d3_tmp_0[ind + 1]), 1), _mm_set1_pd(d3_tmp_2)), tmp_0)),
              _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)&d3_tmp_0[ind]),
              _mm_set1_pd(d3_tmp_3)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
              (_mm_loadu_pd((const real_T *)&d3_tmp_0[ind]), _mm_loadu_pd((const
              real_T *)&d3_tmp_0[ind]), 1), _mm_set1_pd(loga2_re)), tmp_0))),
              _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)&d3_tmp_0[ind +
              2]), _mm_set1_pd(loga2_im)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd
              (_mm_loadu_pd((const real_T *)&d3_tmp_0[ind + 2]), _mm_loadu_pd((
              const real_T *)&d3_tmp_0[ind + 2]), 1), _mm_set1_pd(d3_tmp_4)),
              tmp_0))), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd((const real_T *)
              &d3_tmp_0[ind + 3]), _mm_set1_pd(TrootmI_1)), _mm_mul_pd
              (_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)
              &d3_tmp_0[ind + 3]), _mm_loadu_pd((const real_T *)&d3_tmp_0[ind +
              3]), 1), _mm_set1_pd(TrootmI_2)), tmp_0)));
            _mm_storeu_pd((real_T *)&TrootmI_0[ipiv_tmp + ind], tmp_0);
          }
        }

        absx = fmin(a3, fmax(absx, rt_powd_snf(LR_norm_j(TrootmI_0), 0.2)));
        if (absx <= 0.28790937142411938) {
          m = 6;
          if (absx <= 0.20609626234528361) {
            m = 5;
          }

          exitg2 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    }

    if (guard1) {
      if (c_k == 100) {
        *exitflag = 1;
        m = 6;
        exitg2 = true;
      } else {
        LR_sqrtmTriRecursive_n(Troot, 1, 1, 4);
        for (ipiv_tmp = 0; ipiv_tmp < 16; ipiv_tmp++) {
          tmp_0 = _mm_sub_pd(_mm_loadu_pd((const real_T *)&Troot[ipiv_tmp]),
                             _mm_loadu_pd((const real_T *)&TrootmI_tmp[ipiv_tmp]));
          _mm_storeu_pd((real_T *)&TrootmI[ipiv_tmp], tmp_0);
        }

        c_k++;
      }
    }
  }

  recomputeDiagBlocksSqrtTriangul(Troot, T, c_k);
  memset(&L[0], 0, sizeof(creal_T) << 4U);
  for (s0 = 0; s0 <= m; s0++) {
    r_tmp = 7 * m + s0;
    d3 = r[r_tmp];
    for (ipiv_tmp = 0; ipiv_tmp < 16; ipiv_tmp++) {
      tmp_0 = _mm_mul_pd(_mm_set1_pd(d3), _mm_loadu_pd((const real_T *)
        &Troot[ipiv_tmp]));
      _mm_storeu_pd((real_T *)&TrootmI[ipiv_tmp], tmp_0);
      TrootmI_tmp[ipiv_tmp] = Troot[ipiv_tmp];
    }

    tmp_0 = _mm_set1_pd(1.0);
    _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_set_pd(TrootmI[5].re, TrootmI[0].re),
      tmp_0));
    TrootmI[0].re = tmp[0];
    TrootmI[5].re = tmp[1];
    _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_set_pd(TrootmI[15].re, TrootmI[10].re),
      tmp_0));
    TrootmI[10].re = tmp[0];
    TrootmI[15].re = tmp[1];
    ipiv[0] = 1;
    ipiv[1] = 2;
    ipiv[2] = 3;
    ipiv[3] = 4;
    for (b_k = 0; b_k < 3; b_k++) {
      ind = b_k * 5;
      iy = 5 - b_k;
      c_ix = 0;
      ix = ind;
      d3 = fabs(TrootmI[ind].re) + fabs(TrootmI[ind].im);
      for (h_k = 2; h_k < iy; h_k++) {
        ix++;
        absx = fabs(TrootmI[ix].re) + fabs(TrootmI[ix].im);
        if (absx > d3) {
          c_ix = h_k - 1;
          d3 = absx;
        }
      }

      ipiv_tmp = ind + c_ix;
      if ((TrootmI[ipiv_tmp].re != 0.0) || (TrootmI[ipiv_tmp].im != 0.0)) {
        if (c_ix != 0) {
          ipiv_tmp = b_k + c_ix;
          ipiv[b_k] = (int8_T)(ipiv_tmp + 1);
          d3 = TrootmI[b_k].re;
          absx = TrootmI[b_k].im;
          TrootmI[b_k] = TrootmI[ipiv_tmp];
          TrootmI[ipiv_tmp].re = d3;
          TrootmI[ipiv_tmp].im = absx;
          d3 = TrootmI[b_k + 4].re;
          absx = TrootmI[b_k + 4].im;
          TrootmI[b_k + 4] = TrootmI[ipiv_tmp + 4];
          TrootmI[ipiv_tmp + 4].re = d3;
          TrootmI[ipiv_tmp + 4].im = absx;
          d3 = TrootmI[b_k + 8].re;
          absx = TrootmI[b_k + 8].im;
          TrootmI[b_k + 8] = TrootmI[ipiv_tmp + 8];
          TrootmI[ipiv_tmp + 8].re = d3;
          TrootmI[ipiv_tmp + 8].im = absx;
          d3 = TrootmI[b_k + 12].re;
          absx = TrootmI[b_k + 12].im;
          TrootmI[b_k + 12] = TrootmI[ipiv_tmp + 12];
          TrootmI[ipiv_tmp + 12].re = d3;
          TrootmI[ipiv_tmp + 12].im = absx;
        }

        iy = (ind - b_k) + 4;
        for (c_ix = ind + 2; c_ix <= iy; c_ix++) {
          d3 = TrootmI[c_ix - 1].re;
          absx = TrootmI[c_ix - 1].im;
          TrootmI_2 = TrootmI[ind].re;
          TrootmI_1 = TrootmI[ind].im;
          if (TrootmI_1 == 0.0) {
            if (absx == 0.0) {
              TrootmI[c_ix - 1].re = d3 / TrootmI_2;
              TrootmI[c_ix - 1].im = 0.0;
            } else if (d3 == 0.0) {
              TrootmI[c_ix - 1].re = 0.0;
              TrootmI[c_ix - 1].im = absx / TrootmI_2;
            } else {
              _mm_storeu_pd((real_T *)&TrootmI[c_ix - 1], _mm_div_pd(_mm_set_pd
                (absx, d3), _mm_set1_pd(TrootmI_2)));
            }
          } else if (TrootmI_2 == 0.0) {
            if (d3 == 0.0) {
              TrootmI[c_ix - 1].re = absx / TrootmI_1;
              TrootmI[c_ix - 1].im = 0.0;
            } else if (absx == 0.0) {
              TrootmI[c_ix - 1].re = 0.0;
              TrootmI[c_ix - 1].im = -(d3 / TrootmI_1);
            } else {
              TrootmI[c_ix - 1].re = absx / TrootmI_1;
              TrootmI[c_ix - 1].im = -(d3 / TrootmI_1);
            }
          } else {
            d3_tmp_1 = fabs(TrootmI_2);
            d3_tmp_4 = fabs(TrootmI_1);
            if (d3_tmp_1 > d3_tmp_4) {
              d3_tmp_1 = TrootmI_1 / TrootmI_2;
              d3_tmp_4 = d3_tmp_1 * TrootmI_1 + TrootmI_2;
              TrootmI[c_ix - 1].re = (d3_tmp_1 * absx + d3) / d3_tmp_4;
              TrootmI[c_ix - 1].im = (absx - d3_tmp_1 * d3) / d3_tmp_4;
            } else if (d3_tmp_4 == d3_tmp_1) {
              if (TrootmI_2 > 0.0) {
                d3_tmp_4 = 0.5;
              } else {
                d3_tmp_4 = -0.5;
              }

              if (TrootmI_1 > 0.0) {
                a3 = 0.5;
              } else {
                a3 = -0.5;
              }

              _mm_storeu_pd((real_T *)&TrootmI[c_ix - 1], _mm_div_pd(_mm_add_pd
                (_mm_mul_pd(_mm_set_pd(absx, d3), _mm_set1_pd(d3_tmp_4)),
                 _mm_mul_pd(_mm_mul_pd(_mm_set_pd(d3, absx), _mm_set1_pd(a3)),
                            _mm_set_pd(-1.0, 1.0))), _mm_set1_pd(d3_tmp_1)));
            } else {
              d3_tmp_1 = TrootmI_2 / TrootmI_1;
              _mm_storeu_pd((real_T *)&TrootmI[c_ix - 1], _mm_div_pd(_mm_add_pd
                (_mm_mul_pd(_mm_set1_pd(d3_tmp_1), _mm_set_pd(absx, d3)),
                 _mm_mul_pd(_mm_set_pd(d3, absx), _mm_set_pd(-1.0, 1.0))),
                _mm_set1_pd(d3_tmp_1 * TrootmI_2 + TrootmI_1)));
            }
          }
        }
      }

      iy = ind + 6;
      ix = 2 - b_k;
      for (h_k = 0; h_k <= ix; h_k++) {
        ipiv_tmp = ((h_k << 2) + ind) + 4;
        if ((TrootmI[ipiv_tmp].re != 0.0) || (TrootmI[ipiv_tmp].im != 0.0)) {
          absx = TrootmI[ipiv_tmp].re;
          d3_tmp_1 = TrootmI[ipiv_tmp].im;
          d3 = -absx - d3_tmp_1 * 0.0;
          absx = absx * 0.0 - d3_tmp_1;
          ipiv_tmp = (iy - b_k) + 2;
          for (c_ix = iy; c_ix <= ipiv_tmp; c_ix++) {
            TrootmI_im_tmp = ((ind + c_ix) - iy) + 1;
            d3_tmp_1 = TrootmI[TrootmI_im_tmp].re;
            d3_tmp_2 = TrootmI[TrootmI_im_tmp].im;
            TrootmI[c_ix - 1].re += d3_tmp_1 * d3 - d3_tmp_2 * absx;
            TrootmI[c_ix - 1].im += d3_tmp_1 * absx + d3_tmp_2 * d3;
          }
        }

        iy += 4;
      }
    }

    for (b_k = 0; b_k < 3; b_k++) {
      ipiv_0 = ipiv[b_k];
      if (b_k + 1 != ipiv_0) {
        d3 = TrootmI_tmp[b_k].re;
        absx = TrootmI_tmp[b_k].im;
        TrootmI_tmp[b_k] = TrootmI_tmp[ipiv_0 - 1];
        TrootmI_tmp[ipiv_0 - 1].re = d3;
        TrootmI_tmp[ipiv_0 - 1].im = absx;
        d3 = TrootmI_tmp[b_k + 4].re;
        absx = TrootmI_tmp[b_k + 4].im;
        TrootmI_tmp[b_k + 4] = TrootmI_tmp[ipiv_0 + 3];
        TrootmI_tmp[ipiv_0 + 3].re = d3;
        TrootmI_tmp[ipiv_0 + 3].im = absx;
        d3 = TrootmI_tmp[b_k + 8].re;
        absx = TrootmI_tmp[b_k + 8].im;
        TrootmI_tmp[b_k + 8] = TrootmI_tmp[ipiv_0 + 7];
        TrootmI_tmp[ipiv_0 + 7].re = d3;
        TrootmI_tmp[ipiv_0 + 7].im = absx;
        d3 = TrootmI_tmp[b_k + 12].re;
        absx = TrootmI_tmp[b_k + 12].im;
        TrootmI_tmp[b_k + 12] = TrootmI_tmp[ipiv_0 + 11];
        TrootmI_tmp[ipiv_0 + 11].re = d3;
        TrootmI_tmp[ipiv_0 + 11].im = absx;
      }
    }

    for (b_k = 0; b_k < 4; b_k++) {
      ind = b_k << 2;
      for (iy = 0; iy < 4; iy++) {
        c_ix = iy << 2;
        ipiv_tmp = iy + ind;
        if ((TrootmI_tmp[ipiv_tmp].re != 0.0) || (TrootmI_tmp[ipiv_tmp].im !=
             0.0)) {
          for (ix = iy + 2; ix < 5; ix++) {
            h_k = (ix + c_ix) - 1;
            d3 = TrootmI[h_k].re;
            absx = TrootmI_tmp[ipiv_tmp].re;
            d3_tmp_1 = TrootmI[h_k].im;
            d3_tmp_2 = TrootmI_tmp[ipiv_tmp].im;
            h_k = (ix + ind) - 1;
            TrootmI_tmp[h_k].re -= d3 * absx - d3_tmp_1 * d3_tmp_2;
            TrootmI_tmp[h_k].im -= d3_tmp_1 * absx + d3 * d3_tmp_2;
          }
        }
      }
    }

    for (b_k = 0; b_k < 4; b_k++) {
      ind = b_k << 2;
      for (iy = 3; iy >= 0; iy--) {
        c_ix = iy << 2;
        ipiv_tmp = iy + ind;
        if ((TrootmI_tmp[ipiv_tmp].re != 0.0) || (TrootmI_tmp[ipiv_tmp].im !=
             0.0)) {
          d3 = TrootmI_tmp[ipiv_tmp].re;
          absx = TrootmI_tmp[ipiv_tmp].im;
          ix = iy + c_ix;
          TrootmI_2 = TrootmI[ix].re;
          TrootmI_1 = TrootmI[ix].im;
          if (TrootmI_1 == 0.0) {
            if (absx == 0.0) {
              TrootmI_tmp[ipiv_tmp].re = d3 / TrootmI_2;
              TrootmI_tmp[ipiv_tmp].im = 0.0;
            } else if (d3 == 0.0) {
              TrootmI_tmp[ipiv_tmp].re = 0.0;
              TrootmI_tmp[ipiv_tmp].im = absx / TrootmI_2;
            } else {
              _mm_storeu_pd((real_T *)&TrootmI_tmp[ipiv_tmp], _mm_div_pd
                            (_mm_set_pd(absx, d3), _mm_set1_pd(TrootmI_2)));
            }
          } else if (TrootmI_2 == 0.0) {
            if (d3 == 0.0) {
              TrootmI_tmp[ipiv_tmp].re = absx / TrootmI_1;
              TrootmI_tmp[ipiv_tmp].im = 0.0;
            } else if (absx == 0.0) {
              TrootmI_tmp[ipiv_tmp].re = 0.0;
              TrootmI_tmp[ipiv_tmp].im = -(d3 / TrootmI_1);
            } else {
              TrootmI_tmp[ipiv_tmp].re = absx / TrootmI_1;
              TrootmI_tmp[ipiv_tmp].im = -(d3 / TrootmI_1);
            }
          } else {
            d3_tmp_1 = fabs(TrootmI_2);
            d3_tmp_4 = fabs(TrootmI_1);
            if (d3_tmp_1 > d3_tmp_4) {
              d3_tmp_1 = TrootmI_1 / TrootmI_2;
              d3_tmp_4 = d3_tmp_1 * TrootmI_1 + TrootmI_2;
              TrootmI_tmp[ipiv_tmp].re = (d3_tmp_1 * absx + d3) / d3_tmp_4;
              TrootmI_tmp[ipiv_tmp].im = (absx - d3_tmp_1 * d3) / d3_tmp_4;
            } else if (d3_tmp_4 == d3_tmp_1) {
              if (TrootmI_2 > 0.0) {
                d3_tmp_4 = 0.5;
              } else {
                d3_tmp_4 = -0.5;
              }

              if (TrootmI_1 > 0.0) {
                a3 = 0.5;
              } else {
                a3 = -0.5;
              }

              _mm_storeu_pd((real_T *)&TrootmI_tmp[ipiv_tmp], _mm_div_pd
                            (_mm_add_pd(_mm_mul_pd(_mm_set_pd(absx, d3),
                _mm_set1_pd(d3_tmp_4)), _mm_mul_pd(_mm_mul_pd(_mm_set_pd(d3,
                absx), _mm_set1_pd(a3)), _mm_set_pd(-1.0, 1.0))), _mm_set1_pd
                             (d3_tmp_1)));
            } else {
              d3_tmp_1 = TrootmI_2 / TrootmI_1;
              _mm_storeu_pd((real_T *)&TrootmI_tmp[ipiv_tmp], _mm_div_pd
                            (_mm_add_pd(_mm_mul_pd(_mm_set1_pd(d3_tmp_1),
                _mm_set_pd(absx, d3)), _mm_mul_pd(_mm_set_pd(d3, absx),
                _mm_set_pd(-1.0, 1.0))), _mm_set1_pd(d3_tmp_1 * TrootmI_2 +
                TrootmI_1)));
            }
          }

          for (ix = 0; ix < iy; ix++) {
            h_k = ix + c_ix;
            d3 = TrootmI_tmp[ipiv_tmp].re;
            absx = TrootmI[h_k].re;
            d3_tmp_1 = TrootmI_tmp[ipiv_tmp].im;
            d3_tmp_2 = TrootmI[h_k].im;
            h_k = ix + ind;
            TrootmI_tmp[h_k].re -= d3 * absx - d3_tmp_1 * d3_tmp_2;
            TrootmI_tmp[h_k].im -= d3 * d3_tmp_2 + d3_tmp_1 * absx;
          }
        }
      }
    }

    d3 = p[r_tmp];
    for (ipiv_tmp = 0; ipiv_tmp < 16; ipiv_tmp++) {
      tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(d3), _mm_loadu_pd((const real_T *)
        &TrootmI_tmp[ipiv_tmp])), _mm_loadu_pd((const real_T *)&L[ipiv_tmp]));
      _mm_storeu_pd((real_T *)&L[ipiv_tmp], tmp_0);
    }
  }

  d3 = rt_powd_snf(2.0, (real_T)c_k);
  for (ipiv_tmp = 0; ipiv_tmp < 16; ipiv_tmp++) {
    tmp_0 = _mm_mul_pd(_mm_set1_pd(d3), _mm_loadu_pd((const real_T *)&L[ipiv_tmp]));
    _mm_storeu_pd((real_T *)&L[ipiv_tmp], tmp_0);
  }

  for (m = 0; m < 3; m++) {
    c_k = (m << 2) + m;
    a3 = T[c_k].im;
    if (a3 == 0.0) {
      T_0 = T[c_k].re;
      if (T_0 < 0.0) {
        d3 = log(fabs(T_0));
        absx = 3.1415926535897931;
      } else {
        d3 = log(T_0);
        absx = 0.0;
      }
    } else {
      T_0 = T[c_k].re;
      if ((fabs(T_0) > 8.9884656743115785E+307) || (fabs(a3) >
           8.9884656743115785E+307)) {
        d3 = log(rt_hypotd_snf(T_0 / 2.0, a3 / 2.0)) + 0.69314718055994529;
        absx = rt_atan2d_snf(a3, T_0);
      } else {
        d3 = log(rt_hypotd_snf(T_0, a3));
        absx = rt_atan2d_snf(a3, T_0);
      }
    }

    s0 = ((m + 1) << 2) + m;
    TrootmI_1 = T[s0 + 1].im;
    if (TrootmI_1 == 0.0) {
      TrootmI_2 = T[s0 + 1].re;
      if (TrootmI_2 < 0.0) {
        loga2_re = log(fabs(TrootmI_2));
        loga2_im = 3.1415926535897931;
      } else {
        loga2_re = log(TrootmI_2);
        loga2_im = 0.0;
      }
    } else {
      TrootmI_2 = T[s0 + 1].re;
      if ((fabs(TrootmI_2) > 8.9884656743115785E+307) || (fabs(TrootmI_1) >
           8.9884656743115785E+307)) {
        loga2_re = log(rt_hypotd_snf(TrootmI_2 / 2.0, TrootmI_1 / 2.0)) +
          0.69314718055994529;
        loga2_im = rt_atan2d_snf(TrootmI_1, TrootmI_2);
      } else {
        loga2_re = log(rt_hypotd_snf(TrootmI_2, TrootmI_1));
        loga2_im = rt_atan2d_snf(TrootmI_1, TrootmI_2);
      }
    }

    L[c_k].re = d3;
    L[c_k].im = absx;
    L[s0 + 1].re = loga2_re;
    L[s0 + 1].im = loga2_im;
    if (((T_0 < 0.0) && (a3 == 0.0)) || ((TrootmI_2 < 0.0) && (a3 == 0.0))) {
    } else if ((TrootmI_2 == T_0) && (TrootmI_1 == a3)) {
      TrootmI_1 = T[s0].re;
      TrootmI_2 = T[s0].im;
      if (a3 == 0.0) {
        if (TrootmI_2 == 0.0) {
          L[s0].re = TrootmI_1 / T_0;
          L[s0].im = 0.0;
        } else if (TrootmI_1 == 0.0) {
          L[s0].re = 0.0;
          L[s0].im = TrootmI_2 / T_0;
        } else {
          _mm_storeu_pd((real_T *)&L[s0], _mm_div_pd(_mm_set_pd(TrootmI_2,
            TrootmI_1), _mm_set1_pd(T_0)));
        }
      } else if (T_0 == 0.0) {
        if (TrootmI_1 == 0.0) {
          L[s0].re = TrootmI_2 / a3;
          L[s0].im = 0.0;
        } else if (TrootmI_2 == 0.0) {
          L[s0].re = 0.0;
          L[s0].im = -(TrootmI_1 / a3);
        } else {
          L[s0].re = TrootmI_2 / a3;
          L[s0].im = -(TrootmI_1 / a3);
        }
      } else {
        d3_tmp_1 = fabs(T_0);
        d3_tmp_4 = fabs(a3);
        if (d3_tmp_1 > d3_tmp_4) {
          d3_tmp_1 = a3 / T_0;
          d3_tmp_4 = d3_tmp_1 * a3 + T_0;
          L[s0].re = (d3_tmp_1 * TrootmI_2 + TrootmI_1) / d3_tmp_4;
          L[s0].im = (TrootmI_2 - d3_tmp_1 * TrootmI_1) / d3_tmp_4;
        } else if (d3_tmp_4 == d3_tmp_1) {
          if (T_0 > 0.0) {
            d3_tmp_4 = 0.5;
          } else {
            d3_tmp_4 = -0.5;
          }

          if (a3 > 0.0) {
            a3 = 0.5;
          } else {
            a3 = -0.5;
          }

          _mm_storeu_pd((real_T *)&L[s0], _mm_div_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(TrootmI_2, TrootmI_1), _mm_set1_pd(d3_tmp_4)),
            _mm_mul_pd(_mm_mul_pd(_mm_set_pd(TrootmI_1, TrootmI_2), _mm_set1_pd
            (a3)), _mm_set_pd(-1.0, 1.0))), _mm_set1_pd(d3_tmp_1)));
        } else {
          d3_tmp_1 = T_0 / a3;
          _mm_storeu_pd((real_T *)&L[s0], _mm_div_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set1_pd(d3_tmp_1), _mm_set_pd(TrootmI_2, TrootmI_1)),
            _mm_mul_pd(_mm_set_pd(TrootmI_1, TrootmI_2), _mm_set_pd(-1.0, 1.0))),
            _mm_set1_pd(d3_tmp_1 * T_0 + a3)));
        }
      }
    } else {
      d3_tmp_2 = TrootmI_2 - T_0;
      d3_tmp_3 = TrootmI_1 - a3;
      TrootmI_2 += T_0;
      TrootmI_1 += a3;
      if (TrootmI_1 == 0.0) {
        if (d3_tmp_3 == 0.0) {
          z.re = d3_tmp_2 / TrootmI_2;
          z.im = 0.0;
        } else if (d3_tmp_2 == 0.0) {
          z.re = 0.0;
          z.im = d3_tmp_3 / TrootmI_2;
        } else {
          _mm_storeu_pd((real_T *)&z, _mm_div_pd(_mm_set_pd(d3_tmp_3, d3_tmp_2),
            _mm_set1_pd(TrootmI_2)));
        }
      } else if (TrootmI_2 == 0.0) {
        if (d3_tmp_2 == 0.0) {
          z.re = d3_tmp_3 / TrootmI_1;
          z.im = 0.0;
        } else if (d3_tmp_3 == 0.0) {
          z.re = 0.0;
          z.im = -(d3_tmp_2 / TrootmI_1);
        } else {
          z.re = d3_tmp_3 / TrootmI_1;
          z.im = -(d3_tmp_2 / TrootmI_1);
        }
      } else {
        d3_tmp_1 = fabs(TrootmI_2);
        d3_tmp_4 = fabs(TrootmI_1);
        if (d3_tmp_1 > d3_tmp_4) {
          d3_tmp_1 = TrootmI_1 / TrootmI_2;
          d3_tmp_4 = d3_tmp_1 * TrootmI_1 + TrootmI_2;
          z.re = (d3_tmp_1 * d3_tmp_3 + d3_tmp_2) / d3_tmp_4;
          z.im = (d3_tmp_3 - d3_tmp_1 * d3_tmp_2) / d3_tmp_4;
        } else if (d3_tmp_4 == d3_tmp_1) {
          if (TrootmI_2 > 0.0) {
            d3_tmp_4 = 0.5;
          } else {
            d3_tmp_4 = -0.5;
          }

          if (TrootmI_1 > 0.0) {
            a3 = 0.5;
          } else {
            a3 = -0.5;
          }

          _mm_storeu_pd((real_T *)&z, _mm_div_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(d3_tmp_3, d3_tmp_2), _mm_set1_pd(d3_tmp_4)), _mm_mul_pd
            (_mm_mul_pd(_mm_set_pd(d3_tmp_2, d3_tmp_3), _mm_set1_pd(a3)),
             _mm_set_pd(-1.0, 1.0))), _mm_set1_pd(d3_tmp_1)));
        } else {
          d3_tmp_1 = TrootmI_2 / TrootmI_1;
          _mm_storeu_pd((real_T *)&z, _mm_div_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set1_pd(d3_tmp_1), _mm_set_pd(d3_tmp_3, d3_tmp_2)), _mm_mul_pd
            (_mm_set_pd(d3_tmp_2, d3_tmp_3), _mm_set_pd(-1.0, 1.0))),
            _mm_set1_pd(d3_tmp_1 * TrootmI_2 + TrootmI_1)));
        }
      }

      if ((rt_hypotd_snf(z.re, z.im) > 90.509667991878089) || (rt_hypotd_snf
           (z.re - 1.0, z.im) < 0.011048543456039804) || (rt_hypotd_snf(z.re +
            1.0, z.im) < 0.011048543456039804)) {
        loga2_re -= d3;
        loga2_im -= absx;
        TrootmI_1 = T[s0].re;
        TrootmI_2 = T[s0].im;
        d3 = TrootmI_1 * loga2_re - TrootmI_2 * loga2_im;
        absx = TrootmI_1 * loga2_im + TrootmI_2 * loga2_re;
        if (d3_tmp_3 == 0.0) {
          if (absx == 0.0) {
            L[s0].re = d3 / d3_tmp_2;
            L[s0].im = 0.0;
          } else if (d3 == 0.0) {
            L[s0].re = 0.0;
            L[s0].im = absx / d3_tmp_2;
          } else {
            _mm_storeu_pd((real_T *)&L[s0], _mm_div_pd(_mm_set_pd(absx, d3),
              _mm_set1_pd(d3_tmp_2)));
          }
        } else if (d3_tmp_2 == 0.0) {
          if (d3 == 0.0) {
            L[s0].re = absx / d3_tmp_3;
            L[s0].im = 0.0;
          } else if (absx == 0.0) {
            L[s0].re = 0.0;
            L[s0].im = -(d3 / d3_tmp_3);
          } else {
            L[s0].re = absx / d3_tmp_3;
            L[s0].im = -(d3 / d3_tmp_3);
          }
        } else {
          d3_tmp_1 = fabs(d3_tmp_2);
          d3_tmp_4 = fabs(d3_tmp_3);
          if (d3_tmp_1 > d3_tmp_4) {
            d3_tmp_1 = d3_tmp_3 / d3_tmp_2;
            d3_tmp_4 = d3_tmp_1 * d3_tmp_3 + d3_tmp_2;
            L[s0].re = (d3_tmp_1 * absx + d3) / d3_tmp_4;
            L[s0].im = (absx - d3_tmp_1 * d3) / d3_tmp_4;
          } else if (d3_tmp_4 == d3_tmp_1) {
            if (d3_tmp_2 > 0.0) {
              d3_tmp_4 = 0.5;
            } else {
              d3_tmp_4 = -0.5;
            }

            if (d3_tmp_3 > 0.0) {
              a3 = 0.5;
            } else {
              a3 = -0.5;
            }

            _mm_storeu_pd((real_T *)&L[s0], _mm_div_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set_pd(absx, d3), _mm_set1_pd(d3_tmp_4)), _mm_mul_pd
              (_mm_mul_pd(_mm_set_pd(d3, absx), _mm_set1_pd(a3)), _mm_set_pd
               (-1.0, 1.0))), _mm_set1_pd(d3_tmp_1)));
          } else {
            d3_tmp_1 = d3_tmp_2 / d3_tmp_3;
            _mm_storeu_pd((real_T *)&L[s0], _mm_div_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set1_pd(d3_tmp_1), _mm_set_pd(absx, d3)), _mm_mul_pd
              (_mm_set_pd(d3, absx), _mm_set_pd(-1.0, 1.0))), _mm_set1_pd
              (d3_tmp_1 * d3_tmp_2 + d3_tmp_3)));
          }
        }
      } else {
        LR_atanh_h(&z);
        tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(ceil(((loga2_im - absx) -
          3.1415926535897931) / 6.2831853071795862)), _mm_set_pd
          (6.2831853071795862, 0.0)), _mm_mul_pd(_mm_set1_pd(2.0), _mm_loadu_pd
          ((const real_T *)&z)));
        _mm_storeu_pd(&tmp[0], tmp_0);
        TrootmI_1 = T[s0].re;
        TrootmI_2 = T[s0].im;
        d3 = TrootmI_1 * tmp[0] - TrootmI_2 * tmp[1];
        absx = TrootmI_1 * tmp[1] + TrootmI_2 * tmp[0];
        if (d3_tmp_3 == 0.0) {
          if (absx == 0.0) {
            L[s0].re = d3 / d3_tmp_2;
            L[s0].im = 0.0;
          } else if (d3 == 0.0) {
            L[s0].re = 0.0;
            L[s0].im = absx / d3_tmp_2;
          } else {
            _mm_storeu_pd((real_T *)&L[s0], _mm_div_pd(_mm_set_pd(absx, d3),
              _mm_set1_pd(d3_tmp_2)));
          }
        } else if (d3_tmp_2 == 0.0) {
          if (d3 == 0.0) {
            L[s0].re = absx / d3_tmp_3;
            L[s0].im = 0.0;
          } else if (absx == 0.0) {
            L[s0].re = 0.0;
            L[s0].im = -(d3 / d3_tmp_3);
          } else {
            L[s0].re = absx / d3_tmp_3;
            L[s0].im = -(d3 / d3_tmp_3);
          }
        } else {
          d3_tmp_1 = fabs(d3_tmp_2);
          d3_tmp_4 = fabs(d3_tmp_3);
          if (d3_tmp_1 > d3_tmp_4) {
            d3_tmp_1 = d3_tmp_3 / d3_tmp_2;
            d3_tmp_4 = d3_tmp_1 * d3_tmp_3 + d3_tmp_2;
            L[s0].re = (d3_tmp_1 * absx + d3) / d3_tmp_4;
            L[s0].im = (absx - d3_tmp_1 * d3) / d3_tmp_4;
          } else if (d3_tmp_4 == d3_tmp_1) {
            if (d3_tmp_2 > 0.0) {
              d3_tmp_4 = 0.5;
            } else {
              d3_tmp_4 = -0.5;
            }

            if (d3_tmp_3 > 0.0) {
              a3 = 0.5;
            } else {
              a3 = -0.5;
            }

            _mm_storeu_pd((real_T *)&L[s0], _mm_div_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set_pd(absx, d3), _mm_set1_pd(d3_tmp_4)), _mm_mul_pd
              (_mm_mul_pd(_mm_set_pd(d3, absx), _mm_set1_pd(a3)), _mm_set_pd
               (-1.0, 1.0))), _mm_set1_pd(d3_tmp_1)));
          } else {
            d3_tmp_1 = d3_tmp_2 / d3_tmp_3;
            _mm_storeu_pd((real_T *)&L[s0], _mm_div_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set1_pd(d3_tmp_1), _mm_set_pd(absx, d3)), _mm_mul_pd
              (_mm_set_pd(d3, absx), _mm_set_pd(-1.0, 1.0))), _mm_set1_pd
              (d3_tmp_1 * d3_tmp_2 + d3_tmp_3)));
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_xgetrf(real_T A[16], int32_T ipiv[4], int32_T *info)
{
  int32_T iy;
  int32_T j;
  int32_T k;
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  *info = 0;
  for (j = 0; j < 3; j++) {
    real_T smax;
    int32_T b_ix;
    int32_T jj;
    int32_T vectorUB;
    jj = j * 5;
    iy = 3 - j;
    b_ix = 0;
    smax = fabs(A[jj]);
    for (k = 2; k <= iy + 1; k++) {
      real_T s;
      s = fabs(A[(jj + k) - 1]);
      if (s > smax) {
        b_ix = k - 1;
        smax = s;
      }
    }

    if (A[jj + b_ix] != 0.0) {
      if (b_ix != 0) {
        iy = j + b_ix;
        ipiv[j] = iy + 1;
        smax = A[j];
        A[j] = A[iy];
        A[iy] = smax;
        smax = A[j + 4];
        A[j + 4] = A[iy + 4];
        A[iy + 4] = smax;
        smax = A[j + 8];
        A[j + 8] = A[iy + 8];
        A[iy + 8] = smax;
        smax = A[j + 12];
        A[j + 12] = A[iy + 12];
        A[iy + 12] = smax;
      }

      iy = (jj - j) + 4;
      b_ix = (((((iy - jj) - 1) / 2) << 1) + jj) + 2;
      vectorUB = b_ix - 2;
      for (k = jj + 2; k <= vectorUB; k += 2) {
        __m128d tmp;
        tmp = _mm_loadu_pd(&A[k - 1]);
        _mm_storeu_pd(&A[k - 1], _mm_div_pd(tmp, _mm_set1_pd(A[jj])));
      }

      for (k = b_ix; k <= iy; k++) {
        A[k - 1] /= A[jj];
      }
    } else {
      *info = j + 1;
    }

    b_ix = jj + 6;
    vectorUB = 2 - j;
    for (k = 0; k <= vectorUB; k++) {
      smax = A[((k << 2) + jj) + 4];
      if (smax != 0.0) {
        int32_T d;
        d = (b_ix - j) + 2;
        for (iy = b_ix; iy <= d; iy++) {
          A[iy - 1] += A[((jj + iy) - b_ix) + 1] * -smax;
        }
      }

      b_ix += 4;
    }
  }

  if ((*info == 0) && (!(A[15] != 0.0))) {
    *info = 4;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_linsolve(const real_T A[4], const real_T B[2], real_T C[2])
{
  real_T C_tmp;
  real_T a21;
  int32_T r1;
  int32_T r2;
  if (fabs(A[1]) > fabs(A[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = A[r2] / A[r1];
  C_tmp = A[r1 + 2];
  C[1] = (B[r2] - B[r1] * a21) / (A[r2 + 2] - C_tmp * a21);
  C[0] = (B[r1] - C_tmp * C[1]) / A[r1];
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_sylvesterTriKernel(int32_T ia0, int32_T ja0, int32_T ib0, int32_T
  jb0, real_T C[16], int32_T ic0, int32_T jc0, int32_T m, int32_T n)
{
  real_T A[16];
  real_T b_A[16];
  real_T B[4];
  real_T C_0[4];
  real_T cmod1_0[2];
  real_T tmp[2];
  real_T xab[2];
  real_T A_tmp;
  real_T A_tmp_0;
  real_T cmod1;
  real_T cmod2;
  real_T cmod3;
  real_T cmod4;
  int32_T ipiv[4];
  int32_T cmod1_tmp;
  int32_T cmod1_tmp_tmp;
  int32_T cmod1_tmp_tmp_0;
  int32_T cmod1_tmp_tmp_tmp;
  int32_T cmod3_tmp;
  int32_T cmod3_tmp_tmp;
  int32_T ii;
  int32_T jj;
  int32_T kAcol;
  boolean_T blockA;
  boolean_T blockB;
  ii = m - 2;
  while (ii + 2 >= 1) {
    jj = -1;
    if ((ii + 2 != 1) && (C[((((ja0 + ii) - 1) << 2) + ia0) + ii] != 0.0)) {
      blockA = true;
    } else {
      blockA = false;
    }

    while (jj + 2 <= n) {
      if ((jj + 2 != n) && (C[((((jb0 + jj) << 2) + ib0) + jj) + 1] != 0.0)) {
        blockB = true;
      } else {
        blockB = false;
      }

      if (!blockA) {
        if (!blockB) {
          cmod1_tmp_tmp = (jc0 + jj) << 2;
          kAcol = cmod1_tmp_tmp + ic0;
          cmod1 = C[kAcol + ii];
          for (cmod3_tmp = ii + 3; cmod3_tmp <= m; cmod3_tmp++) {
            cmod1 -= C[((((ja0 + cmod3_tmp) - 2) << 2) + ia0) + ii] * C[(kAcol +
              cmod3_tmp) - 2];
          }

          cmod1_tmp_tmp_0 = (uint8_T)(jj + 1);
          for (cmod3_tmp = 0; cmod3_tmp < cmod1_tmp_tmp_0; cmod3_tmp++) {
            cmod1 -= C[((((jc0 + cmod3_tmp) - 1) << 2) + ic0) + ii] * C[((((jb0
              + jj) << 2) + ib0) + cmod3_tmp) - 1];
          }

          C[(ic0 + ii) + cmod1_tmp_tmp] = cmod1 / (C[(((ja0 + ii) << 2) + ia0) +
            ii] + C[(((jb0 + jj) << 2) + ib0) + jj]);
          jj++;
        } else {
          kAcol = jc0 + jj;
          cmod1_tmp_tmp = kAcol << 2;
          cmod1_tmp = cmod1_tmp_tmp + ic0;
          cmod1 = C[cmod1_tmp + ii];
          cmod1_tmp_tmp_tmp = (kAcol + 1) << 2;
          cmod1_tmp_tmp_0 = cmod1_tmp_tmp_tmp + ic0;
          cmod2 = C[cmod1_tmp_tmp_0 + ii];
          for (cmod3_tmp = ii + 3; cmod3_tmp <= m; cmod3_tmp++) {
            kAcol = ((((ja0 + cmod3_tmp) - 2) << 2) + ia0) + ii;
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(cmod2, cmod1),
              _mm_mul_pd(_mm_set_pd(C[(cmod1_tmp_tmp_0 + cmod3_tmp) - 2],
              C[kAcol]), _mm_set_pd(C[kAcol], C[(cmod1_tmp + cmod3_tmp) - 2]))));
            cmod1 = tmp[0];
            cmod2 = tmp[1];
          }

          cmod1_tmp_tmp_0 = (uint8_T)(jj + 1);
          for (cmod3_tmp = 0; cmod3_tmp < cmod1_tmp_tmp_0; cmod3_tmp++) {
            kAcol = ((((jc0 + cmod3_tmp) - 1) << 2) + ic0) + ii;
            cmod1_tmp = jb0 + jj;
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(cmod2, cmod1),
              _mm_mul_pd(_mm_set_pd(C[((((cmod1_tmp + 1) << 2) + ib0) +
              cmod3_tmp) - 1], C[kAcol]), _mm_set_pd(C[kAcol], C[(((cmod1_tmp <<
              2) + ib0) + cmod3_tmp) - 1]))));
            cmod1 = tmp[0];
            cmod2 = tmp[1];
          }

          kAcol = jb0 + jj;
          cmod3_tmp = ((kAcol << 2) + ib0) + jj;
          cmod3 = C[(((ja0 + ii) << 2) + ia0) + ii];
          C_0[0] = cmod3 + C[cmod3_tmp];
          C_0[2] = C[cmod3_tmp + 1];
          cmod3_tmp = (((kAcol + 1) << 2) + ib0) + jj;
          C_0[1] = C[cmod3_tmp];
          C_0[3] = C[cmod3_tmp + 1] + cmod3;
          cmod1_0[0] = cmod1;
          cmod1_0[1] = cmod2;
          LR_linsolve(C_0, cmod1_0, xab);
          kAcol = ic0 + ii;
          C[kAcol + cmod1_tmp_tmp] = xab[0];
          C[kAcol + cmod1_tmp_tmp_tmp] = xab[1];
          jj += 2;
        }
      } else if (!blockB) {
        cmod1_tmp_tmp_tmp = (jc0 + jj) << 2;
        cmod1_tmp_tmp = cmod1_tmp_tmp_tmp + ic0;
        kAcol = cmod1_tmp_tmp + ii;
        cmod1 = C[kAcol - 1];
        cmod2 = C[kAcol];
        for (cmod3_tmp = ii + 3; cmod3_tmp <= m; cmod3_tmp++) {
          kAcol = ((((ja0 + cmod3_tmp) - 2) << 2) + ia0) + ii;
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(cmod2, cmod1), _mm_mul_pd
            (_mm_set_pd(C[kAcol], C[kAcol - 1]), _mm_set1_pd(C[(cmod1_tmp_tmp +
            cmod3_tmp) - 2]))));
          cmod1 = tmp[0];
          cmod2 = tmp[1];
        }

        cmod1_tmp_tmp_0 = (uint8_T)(jj + 1);
        for (cmod3_tmp = 0; cmod3_tmp < cmod1_tmp_tmp_0; cmod3_tmp++) {
          kAcol = ((((jc0 + cmod3_tmp) - 1) << 2) + ic0) + ii;
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(cmod2, cmod1), _mm_mul_pd
            (_mm_set_pd(C[kAcol], C[kAcol - 1]), _mm_set1_pd(C[((((jb0 + jj) <<
            2) + ib0) + cmod3_tmp) - 1]))));
          cmod1 = tmp[0];
          cmod2 = tmp[1];
        }

        cmod3_tmp = ja0 + ii;
        kAcol = (((cmod3_tmp - 1) << 2) + ia0) + ii;
        cmod3 = C[(((jb0 + jj) << 2) + ib0) + jj];
        C_0[0] = C[kAcol - 1] + cmod3;
        cmod3_tmp = ((cmod3_tmp << 2) + ia0) + ii;
        C_0[2] = C[cmod3_tmp - 1];
        C_0[1] = C[kAcol];
        C_0[3] = C[cmod3_tmp] + cmod3;
        cmod1_0[0] = cmod1;
        cmod1_0[1] = cmod2;
        LR_linsolve(C_0, cmod1_0, xab);
        kAcol = (ic0 + ii) + cmod1_tmp_tmp_tmp;
        C[kAcol - 1] = xab[0];
        C[kAcol] = xab[1];
        jj++;
      } else {
        cmod1_tmp_tmp = jc0 + jj;
        cmod1_tmp_tmp_tmp = cmod1_tmp_tmp << 2;
        cmod1_tmp_tmp_0 = cmod1_tmp_tmp_tmp + ic0;
        kAcol = cmod1_tmp_tmp_0 + ii;
        cmod1 = C[kAcol - 1];
        cmod2 = C[kAcol];
        cmod1_tmp_tmp = (cmod1_tmp_tmp + 1) << 2;
        cmod3_tmp_tmp = cmod1_tmp_tmp + ic0;
        cmod3_tmp = cmod3_tmp_tmp + ii;
        cmod3 = C[cmod3_tmp - 1];
        cmod4 = C[cmod3_tmp];
        for (cmod3_tmp = ii + 3; cmod3_tmp <= m; cmod3_tmp++) {
          kAcol = ((((ja0 + cmod3_tmp) - 2) << 2) + ia0) + ii;
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(cmod2, cmod1), _mm_mul_pd
            (_mm_set_pd(C[kAcol], C[kAcol - 1]), _mm_set1_pd(C[(cmod1_tmp_tmp_0
            + cmod3_tmp) - 2]))));
          cmod1 = tmp[0];
          cmod2 = tmp[1];
          cmod1_tmp = (cmod3_tmp_tmp + cmod3_tmp) - 2;
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(cmod4, cmod3), _mm_mul_pd
            (_mm_set_pd(C[cmod1_tmp], C[kAcol - 1]), _mm_set_pd(C[kAcol],
            C[cmod1_tmp]))));
          cmod3 = tmp[0];
          cmod4 = tmp[1];
        }

        cmod1_tmp_tmp_0 = (uint8_T)(jj + 1);
        for (cmod3_tmp = 0; cmod3_tmp < cmod1_tmp_tmp_0; cmod3_tmp++) {
          kAcol = ((((jc0 + cmod3_tmp) - 1) << 2) + ic0) + ii;
          cmod1_tmp = jb0 + jj;
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(cmod2, cmod1), _mm_mul_pd
            (_mm_set_pd(C[kAcol], C[kAcol - 1]), _mm_set1_pd(C[(((cmod1_tmp << 2)
            + ib0) + cmod3_tmp) - 1]))));
          cmod1 = tmp[0];
          cmod2 = tmp[1];
          cmod1_tmp = ((((cmod1_tmp + 1) << 2) + ib0) + cmod3_tmp) - 1;
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(cmod4, cmod3), _mm_mul_pd
            (_mm_set_pd(C[cmod1_tmp], C[kAcol - 1]), _mm_set_pd(C[kAcol],
            C[cmod1_tmp]))));
          cmod3 = tmp[0];
          cmod4 = tmp[1];
        }

        kAcol = ja0 + ii;
        cmod1_tmp = jb0 + jj;
        cmod3_tmp = (((kAcol - 1) << 2) + ia0) + ii;
        kAcol = ((kAcol << 2) + ia0) + ii;
        cmod1_tmp_tmp_0 = ((cmod1_tmp << 2) + ib0) + jj;
        _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_set_pd(C[kAcol], C[cmod3_tmp - 1]),
          _mm_set1_pd(C[cmod1_tmp_tmp_0])));
        A[0] = tmp[0];
        A[5] = tmp[1];
        cmod1_tmp = (((cmod1_tmp + 1) << 2) + ib0) + jj;
        A_tmp = C[cmod1_tmp + 1];
        _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_set_pd(A_tmp, C[cmod3_tmp - 1]),
          _mm_set_pd(C[kAcol], A_tmp)));
        A[10] = tmp[0];
        A[15] = tmp[1];
        A_tmp = C[cmod3_tmp];
        A[1] = A_tmp;
        A_tmp_0 = C[kAcol - 1];
        A[4] = A_tmp_0;
        A[11] = A_tmp;
        A[14] = A_tmp_0;
        A_tmp = C[cmod1_tmp];
        A[2] = A_tmp;
        A_tmp_0 = C[cmod1_tmp_tmp_0 + 1];
        A[8] = A_tmp_0;
        A[7] = A_tmp;
        A[13] = A_tmp_0;
        A[3] = 0.0;
        A[6] = 0.0;
        A[9] = 0.0;
        A[12] = 0.0;
        B[0] = cmod1;
        B[1] = cmod2;
        B[2] = cmod3;
        B[3] = cmod4;
        memcpy(&b_A[0], &A[0], sizeof(real_T) << 4U);
        LR_xgetrf(b_A, ipiv, &cmod3_tmp);
        if (ipiv[0] != 1) {
          B[0] = B[ipiv[0] - 1];
          B[ipiv[0] - 1] = cmod1;
        }

        if (ipiv[1] != 2) {
          cmod1 = B[1];
          B[1] = B[ipiv[1] - 1];
          B[ipiv[1] - 1] = cmod1;
        }

        if (ipiv[2] != 3) {
          cmod1 = B[2];
          B[2] = B[ipiv[2] - 1];
          B[ipiv[2] - 1] = cmod1;
        }

        for (cmod3_tmp = 0; cmod3_tmp < 4; cmod3_tmp++) {
          kAcol = cmod3_tmp << 2;
          if (B[cmod3_tmp] != 0.0) {
            for (cmod1_tmp_tmp_0 = cmod3_tmp + 2; cmod1_tmp_tmp_0 < 5;
                 cmod1_tmp_tmp_0++) {
              B[cmod1_tmp_tmp_0 - 1] -= b_A[(cmod1_tmp_tmp_0 + kAcol) - 1] *
                B[cmod3_tmp];
            }
          }
        }

        for (cmod3_tmp = 3; cmod3_tmp >= 0; cmod3_tmp--) {
          kAcol = cmod3_tmp << 2;
          cmod1 = B[cmod3_tmp];
          if (cmod1 != 0.0) {
            B[cmod3_tmp] = cmod1 / b_A[cmod3_tmp + kAcol];
            for (cmod1_tmp_tmp_0 = 0; cmod1_tmp_tmp_0 < cmod3_tmp;
                 cmod1_tmp_tmp_0++) {
              B[cmod1_tmp_tmp_0] -= b_A[cmod1_tmp_tmp_0 + kAcol] * B[cmod3_tmp];
            }
          }
        }

        kAcol = ic0 + ii;
        cmod1_tmp = kAcol + cmod1_tmp_tmp_tmp;
        C[cmod1_tmp - 1] = B[0];
        C[cmod1_tmp] = B[1];
        kAcol += cmod1_tmp_tmp;
        C[kAcol - 1] = B[2];
        C[kAcol] = B[3];
        jj += 2;
      }
    }

    if (blockA) {
      ii -= 2;
    } else {
      ii--;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_sylvesterRecursive(int32_T ia0, int32_T ja0, int32_T ib0, int32_T
  jb0, real_T C[16], int32_T ic0, int32_T jc0, int32_T m, int32_T n)
{
  real_T c_c;
  int32_T aStart;
  int32_T b_ix;
  int32_T b_iy0;
  int32_T b_kk;
  int32_T e;
  int32_T f;
  int32_T m1;
  int32_T n1;
  int32_T n1_tmp;
  int32_T tmp;
  int32_T xStart;
  if ((m >= n) && (m > 8)) {
    m1 = m / 2;
    if (C[(((((ja0 + m1) - 2) << 2) + ia0) + m1) - 1] != 0.0) {
      m1++;
    }

    tmp = ja0 + m1;
    n1 = m - m1;
    LR_sylvesterRecursive(ia0 + m1, tmp, ib0, jb0, C, ic0 + m1, jc0, n1, n);
    aStart = ((tmp - 1) << 2) + ia0;
    for (b_kk = 0; b_kk < n; b_kk++) {
      n1_tmp = (((jc0 + b_kk) - 1) << 2) + ic0;
      if (n1 != 0) {
        xStart = (n1_tmp + m1) - 1;
        e = ((n1 - 1) << 2) + aStart;
        for (b_iy0 = aStart; b_iy0 <= e; b_iy0 += 4) {
          c_c = -C[((b_iy0 - aStart) >> 2) + xStart];
          f = (b_iy0 + m1) - 1;
          for (b_ix = b_iy0; b_ix <= f; b_ix++) {
            tmp = ((n1_tmp + b_ix) - b_iy0) - 1;
            C[tmp] += C[b_ix - 1] * c_c;
          }
        }
      }
    }

    LR_sylvesterRecursive(ia0, ja0, ib0, jb0, C, ic0, jc0, m1, n);
  } else if (n > 8) {
    n1 = n / 2;
    if (C[(((((jb0 + n1) - 2) << 2) + ib0) + n1) - 1] != 0.0) {
      n1++;
    }

    LR_sylvesterRecursive(ia0, ja0, ib0, jb0, C, ic0, jc0, m, n1);
    xStart = ((jc0 - 1) << 2) + ic0;
    for (b_kk = n1; b_kk < n; b_kk++) {
      b_iy0 = ((((jc0 + b_kk) - 1) << 2) + ic0) - 1;
      if (m != 0) {
        b_ix = ((((jb0 + b_kk) - 1) << 2) + ib0) - 1;
        e = ((n1 - 1) << 2) + xStart;
        for (m1 = xStart; m1 <= e; m1 += 4) {
          c_c = -C[((m1 - xStart) >> 2) + b_ix];
          f = (m1 + m) - 1;
          for (aStart = m1; aStart <= f; aStart++) {
            tmp = (b_iy0 + aStart) - m1;
            C[tmp] += C[aStart - 1] * c_c;
          }
        }
      }
    }

    LR_sylvesterRecursive(ia0, ja0, ib0 + n1, jb0 + n1, C, ic0, jc0 + n1, m, n -
                          n1);
  } else {
    LR_sylvesterTriKernel(ia0, ja0, ib0, jb0, C, ic0, jc0, m, n);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_sqrtmTriRecursive(real_T T[16], int32_T i, int32_T j, int32_T m)
{
  real_T mu;
  real_T r11;
  real_T r22;
  real_T tmp_1;
  int32_T m1;
  int32_T m2;
  int32_T tmp;
  int32_T tmp_0;
  if (m == 1) {
    tmp = (((j - 1) << 2) + i) - 1;
    T[tmp] = sqrt(T[tmp]);
  } else if (m == 2) {
    tmp = ((j - 1) << 2) + i;
    r11 = T[tmp];
    if (r11 != 0.0) {
      m1 = (j << 2) + i;
      r22 = T[m1 - 1];
      mu = sqrt(-r11 * r22);
      tmp_1 = T[tmp - 1];
      if (tmp_1 > 0.0) {
        mu = sqrt((rt_hypotd_snf(T[tmp - 1], mu) + tmp_1) / 2.0);
      } else {
        mu /= sqrt((rt_hypotd_snf(T[tmp - 1], mu) + -tmp_1) * 2.0);
      }

      T[tmp - 1] = mu;
      T[m1] = mu;
      T[m1 - 1] = r22 / (2.0 * mu);
      T[tmp] = r11 / (2.0 * mu);
    } else {
      r11 = sqrt(T[tmp - 1]);
      m1 = (j << 2) + i;
      r22 = sqrt(T[m1]);
      T[m1] = r22;
      T[tmp - 1] = r11;
      T[m1 - 1] /= r11 + r22;
    }
  } else {
    m1 = m / 2;
    if (T[(((((j + m1) - 2) << 2) + i) + m1) - 1] != 0.0) {
      m1++;
    }

    m2 = m - m1;
    LR_sqrtmTriRecursive(T, i, j, m1);
    tmp = i + m1;
    tmp_0 = j + m1;
    LR_sqrtmTriRecursive(T, tmp, tmp_0, m2);
    LR_sylvesterRecursive(i, j, tmp, tmp_0, T, i, tmp_0, m1, m2);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_logmParams(real_T T[16], creal_T d[4], int32_T *s, int32_T *m,
  int32_T *exitflag)
{
  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  real_T TrootmI_tmp_0[16];
  real_T TrootmI_tmp_1[16];
  real_T d3_tmp[16];
  real_T d3_tmp_0[16];
  real_T tmp_3[2];
  real_T TrootmI_tmp_2;
  real_T TrootmI_tmp_3;
  real_T a3;
  real_T absx;
  real_T d3;
  real_T d3_tmp_1;
  real_T d3_tmp_2;
  int32_T b_k;
  int32_T ind;
  int32_T j;
  int32_T s0;
  int32_T tmp_4;
  int32_T tmp_5;
  int8_T TrootmI_tmp[16];
  int8_T b_I[16];
  boolean_T foundm;
  boolean_T more;
  static const real_T c[7] = { 1.5869707387720632E-5, 0.0023138078842429789,
    0.019381793135332531, 0.062091715889947621, 0.12764048108067749,
    0.20609626234528361, 0.28790937142411938 };

  int32_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T guard1;
  boolean_T guard2;
  *m = 1;
  *exitflag = 0;
  for (ind = 0; ind < 16; ind++) {
    b_I[ind] = 0;
  }

  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  *s = 0;
  do {
    exitg1 = 0;
    d3 = 0.0;
    for (s0 = 0; s0 < 4; s0++) {
      absx = rt_hypotd_snf(d[s0].re - 1.0, d[s0].im);
      if (rtIsNaN(absx) || (absx > d3)) {
        d3 = absx;
      }
    }

    if ((d3 > 0.28790937142411938) && (*s < 100)) {
      LR_sqrt(&d[0]);
      LR_sqrt(&d[1]);
      LR_sqrt(&d[2]);
      LR_sqrt(&d[3]);
      (*s)++;
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  s0 = *s;
  if (*s == 100) {
    *exitflag = 1;
  }

  for (b_k = 0; b_k < *s; b_k++) {
    LR_sqrtmTriRecursive(T, 1, 1, 4);
  }

  for (ind = 0; ind < 16; ind++) {
    b_k = b_I[ind];
    TrootmI_tmp[ind] = (int8_T)b_k;
    TrootmI_tmp_0[ind] = T[ind] - (real_T)b_k;
  }

  for (ind = 0; ind < 4; ind++) {
    for (j = 0; j <= 2; j += 2) {
      tmp = _mm_loadu_pd(&TrootmI_tmp_0[j + 4]);
      tmp_4 = ind << 2;
      tmp_0 = _mm_loadu_pd(&TrootmI_tmp_0[j]);
      tmp_1 = _mm_loadu_pd(&TrootmI_tmp_0[j + 8]);
      tmp_2 = _mm_loadu_pd(&TrootmI_tmp_0[j + 12]);
      _mm_storeu_pd(&d3_tmp[j + tmp_4], _mm_add_pd(_mm_add_pd(_mm_add_pd
        (_mm_mul_pd(_mm_set1_pd(TrootmI_tmp_0[tmp_4 + 1]), tmp), _mm_mul_pd
         (_mm_set1_pd(TrootmI_tmp_0[tmp_4]), tmp_0)), _mm_mul_pd(_mm_set1_pd
        (TrootmI_tmp_0[tmp_4 + 2]), tmp_1)), _mm_mul_pd(_mm_set1_pd
        (TrootmI_tmp_0[tmp_4 + 3]), tmp_2)));
    }

    b_k = ind << 2;
    d3 = d3_tmp[b_k + 1];
    absx = d3_tmp[b_k];
    d3_tmp_1 = d3_tmp[b_k + 2];
    d3_tmp_2 = d3_tmp[b_k + 3];
    for (j = 0; j <= 2; j += 2) {
      tmp = _mm_loadu_pd(&TrootmI_tmp_0[j + 4]);
      tmp_0 = _mm_loadu_pd(&TrootmI_tmp_0[j]);
      tmp_1 = _mm_loadu_pd(&TrootmI_tmp_0[j + 8]);
      tmp_2 = _mm_loadu_pd(&TrootmI_tmp_0[j + 12]);
      _mm_storeu_pd(&TrootmI_tmp_1[j + b_k], _mm_add_pd(_mm_add_pd(_mm_add_pd
        (_mm_mul_pd(_mm_set1_pd(d3), tmp), _mm_mul_pd(_mm_set1_pd(absx), tmp_0)),
        _mm_mul_pd(_mm_set1_pd(d3_tmp_1), tmp_1)), _mm_mul_pd(_mm_set1_pd
        (d3_tmp_2), tmp_2)));
    }
  }

  d3 = rt_powd_snf(LR_norm(TrootmI_tmp_1), 0.33333333333333331);
  absx = fmax(sqrt(LR_norm(d3_tmp)), d3);
  if (absx <= 1.5869707387720632E-5) {
    foundm = true;
  } else if (absx <= 0.0023138078842429789) {
    *m = 2;
    foundm = true;
  } else {
    foundm = false;
  }

  b_k = 0;
  exitg2 = false;
  while ((!exitg2) && (!foundm)) {
    more = false;
    if (*s > s0) {
      for (ind = 0; ind < 4; ind++) {
        for (j = 0; j <= 2; j += 2) {
          tmp_4 = (j + 1) << 2;
          tmp_5 = j << 2;
          _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(TrootmI_tmp_0[tmp_4 + 1], TrootmI_tmp_0[tmp_5 + 1]),
             _mm_set1_pd(TrootmI_tmp_0[ind + 4])), _mm_mul_pd(_mm_set_pd
            (TrootmI_tmp_0[tmp_4], TrootmI_tmp_0[tmp_5]), _mm_set1_pd
            (TrootmI_tmp_0[ind]))), _mm_mul_pd(_mm_set_pd(TrootmI_tmp_0[tmp_4 +
            2], TrootmI_tmp_0[tmp_5 + 2]), _mm_set1_pd(TrootmI_tmp_0[ind + 8]))),
            _mm_mul_pd(_mm_set_pd(TrootmI_tmp_0[tmp_4 + 3], TrootmI_tmp_0[tmp_5
            + 3]), _mm_set1_pd(TrootmI_tmp_0[ind + 12]))));
          TrootmI_tmp_1[ind + tmp_5] = tmp_3[0];
          TrootmI_tmp_1[ind + tmp_4] = tmp_3[1];
        }
      }

      for (ind = 0; ind < 4; ind++) {
        d3_tmp_1 = TrootmI_tmp_0[ind + 4];
        d3_tmp_2 = TrootmI_tmp_0[ind];
        TrootmI_tmp_2 = TrootmI_tmp_0[ind + 8];
        TrootmI_tmp_3 = TrootmI_tmp_0[ind + 12];
        for (j = 0; j <= 2; j += 2) {
          tmp_4 = (j + 1) << 2;
          tmp_5 = j << 2;
          _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(TrootmI_tmp_1[tmp_4 + 1], TrootmI_tmp_1[tmp_5 + 1]),
             _mm_set1_pd(d3_tmp_1)), _mm_mul_pd(_mm_set_pd(TrootmI_tmp_1[tmp_4],
            TrootmI_tmp_1[tmp_5]), _mm_set1_pd(d3_tmp_2))), _mm_mul_pd
            (_mm_set_pd(TrootmI_tmp_1[tmp_4 + 2], TrootmI_tmp_1[tmp_5 + 2]),
             _mm_set1_pd(TrootmI_tmp_2))), _mm_mul_pd(_mm_set_pd
            (TrootmI_tmp_1[tmp_4 + 3], TrootmI_tmp_1[tmp_5 + 3]), _mm_set1_pd
            (TrootmI_tmp_3))));
          d3_tmp[ind + tmp_5] = tmp_3[0];
          d3_tmp[ind + tmp_4] = tmp_3[1];
        }
      }

      d3 = rt_powd_snf(LR_norm(d3_tmp), 0.33333333333333331);
    }

    for (ind = 0; ind < 4; ind++) {
      for (j = 0; j <= 2; j += 2) {
        tmp_4 = (j + 1) << 2;
        tmp_5 = j << 2;
        _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(TrootmI_tmp_0[tmp_4 + 1], TrootmI_tmp_0[tmp_5 + 1]),
           _mm_set1_pd(TrootmI_tmp_0[ind + 4])), _mm_mul_pd(_mm_set_pd
          (TrootmI_tmp_0[tmp_4], TrootmI_tmp_0[tmp_5]), _mm_set1_pd
          (TrootmI_tmp_0[ind]))), _mm_mul_pd(_mm_set_pd(TrootmI_tmp_0[tmp_4 + 2],
          TrootmI_tmp_0[tmp_5 + 2]), _mm_set1_pd(TrootmI_tmp_0[ind + 8]))),
          _mm_mul_pd(_mm_set_pd(TrootmI_tmp_0[tmp_4 + 3], TrootmI_tmp_0[tmp_5 +
          3]), _mm_set1_pd(TrootmI_tmp_0[ind + 12]))));
        d3_tmp[ind + tmp_5] = tmp_3[0];
        d3_tmp[ind + tmp_4] = tmp_3[1];
      }
    }

    for (ind = 0; ind < 4; ind++) {
      for (j = 0; j <= 2; j += 2) {
        tmp_4 = (j + 1) << 2;
        tmp_5 = j << 2;
        _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(d3_tmp[tmp_4 + 1], d3_tmp[tmp_5 + 1]), _mm_set1_pd
           (d3_tmp[ind + 4])), _mm_mul_pd(_mm_set_pd(d3_tmp[tmp_4], d3_tmp[tmp_5]),
          _mm_set1_pd(d3_tmp[ind]))), _mm_mul_pd(_mm_set_pd(d3_tmp[tmp_4 + 2],
          d3_tmp[tmp_5 + 2]), _mm_set1_pd(d3_tmp[ind + 8]))), _mm_mul_pd
          (_mm_set_pd(d3_tmp[tmp_4 + 3], d3_tmp[tmp_5 + 3]), _mm_set1_pd
           (d3_tmp[ind + 12]))));
        d3_tmp_0[ind + tmp_5] = tmp_3[0];
        d3_tmp_0[ind + tmp_4] = tmp_3[1];
      }
    }

    absx = rt_powd_snf(LR_norm(d3_tmp_0), 0.25);
    a3 = fmax(d3, absx);
    guard1 = false;
    guard2 = false;
    if (a3 <= 0.28790937142411938) {
      ind = 0;
      j = 3;
      exitg3 = false;
      while ((!exitg3) && (j < 8)) {
        if (a3 <= c[j - 1]) {
          ind = j;
          exitg3 = true;
        } else {
          j++;
        }
      }

      if (ind <= 6) {
        *m = ind;
        exitg2 = true;
      } else {
        if ((a3 / 2.0 <= 0.12764048108067749) && (b_k < 2)) {
          more = true;
          b_k++;
        }

        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      if (!more) {
        for (ind = 0; ind < 4; ind++) {
          for (j = 0; j <= 2; j += 2) {
            tmp_4 = (j + 1) << 2;
            tmp_5 = j << 2;
            _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set_pd(d3_tmp[tmp_4 + 1], d3_tmp[tmp_5 + 1]), _mm_set1_pd
               (d3_tmp[ind + 4])), _mm_mul_pd(_mm_set_pd(d3_tmp[tmp_4],
              d3_tmp[tmp_5]), _mm_set1_pd(d3_tmp[ind]))), _mm_mul_pd(_mm_set_pd
              (d3_tmp[tmp_4 + 2], d3_tmp[tmp_5 + 2]), _mm_set1_pd(d3_tmp[ind + 8]))),
              _mm_mul_pd(_mm_set_pd(d3_tmp[tmp_4 + 3], d3_tmp[tmp_5 + 3]),
                         _mm_set1_pd(d3_tmp[ind + 12]))));
            d3_tmp_0[ind + tmp_5] = tmp_3[0];
            d3_tmp_0[ind + tmp_4] = tmp_3[1];
          }
        }

        for (ind = 0; ind < 4; ind++) {
          d3_tmp_1 = TrootmI_tmp_0[ind + 4];
          d3_tmp_2 = TrootmI_tmp_0[ind];
          TrootmI_tmp_2 = TrootmI_tmp_0[ind + 8];
          TrootmI_tmp_3 = TrootmI_tmp_0[ind + 12];
          for (j = 0; j <= 2; j += 2) {
            tmp_4 = (j + 1) << 2;
            tmp_5 = j << 2;
            _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set_pd(d3_tmp_0[tmp_4 + 1], d3_tmp_0[tmp_5 + 1]), _mm_set1_pd
               (d3_tmp_1)), _mm_mul_pd(_mm_set_pd(d3_tmp_0[tmp_4],
              d3_tmp_0[tmp_5]), _mm_set1_pd(d3_tmp_2))), _mm_mul_pd(_mm_set_pd
              (d3_tmp_0[tmp_4 + 2], d3_tmp_0[tmp_5 + 2]), _mm_set1_pd
              (TrootmI_tmp_2))), _mm_mul_pd(_mm_set_pd(d3_tmp_0[tmp_4 + 3],
              d3_tmp_0[tmp_5 + 3]), _mm_set1_pd(TrootmI_tmp_3))));
            TrootmI_tmp_1[ind + tmp_5] = tmp_3[0];
            TrootmI_tmp_1[ind + tmp_4] = tmp_3[1];
          }
        }

        absx = fmin(a3, fmax(absx, rt_powd_snf(LR_norm(TrootmI_tmp_1), 0.2)));
        if (absx <= 0.28790937142411938) {
          *m = 7;
          if (absx <= 0.20609626234528361) {
            *m = 6;
          }

          exitg2 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    }

    if (guard1) {
      if (*s == 100) {
        *exitflag = 1;
        *m = 7;
        exitg2 = true;
      } else {
        LR_sqrtmTriRecursive(T, 1, 1, 4);
        for (ind = 0; ind < 16; ind++) {
          TrootmI_tmp_0[ind] = T[ind] - (real_T)TrootmI_tmp[ind];
        }

        (*s)++;
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_sqrtm2by2(real_T T[4])
{
  real_T tmp[2];
  real_T r11;
  real_T r22;
  if (T[1] != 0.0) {
    r11 = sqrt(-T[1] * T[2]);
    if (T[0] > 0.0) {
      r11 = sqrt((T[0] + rt_hypotd_snf(T[0], r11)) / 2.0);
    } else {
      r11 /= sqrt((-T[0] + rt_hypotd_snf(T[0], r11)) * 2.0);
    }

    T[0] = r11;
    T[3] = r11;
    _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(T[1], T[2]), _mm_mul_pd
      (_mm_set1_pd(2.0), _mm_set1_pd(r11))));
    T[2] = tmp[0];
    T[1] = tmp[1];
  } else {
    r11 = sqrt(T[0]);
    r22 = sqrt(T[3]);
    T[3] = r22;
    T[0] = r11;
    T[2] /= r11 + r22;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_atanh(real_T *x)
{
  boolean_T negx;
  if (*x < 0.0) {
    negx = true;
    *x = -*x;
  } else {
    negx = false;
  }

  if (*x > 1.0) {
    *x = (rtNaN);
  } else if (*x < 0.5) {
    real_T absz;
    real_T t;
    t = *x + *x;
    t += *x / (1.0 - *x) * t;
    absz = fabs(t);
    if ((absz > 4.503599627370496E+15) || (rtIsInf(t) || rtIsNaN(t))) {
      t = log(t + 1.0);
    } else if (!(absz < 2.2204460492503131E-16)) {
      t = t / ((t + 1.0) - 1.0) * log(t + 1.0);
    }

    *x = t / 2.0;
  } else if (*x == 1.0) {
    *x = (rtInf);
  } else {
    real_T absz;
    real_T t;
    t = (*x + *x) / (1.0 - *x);
    absz = fabs(t);
    if ((absz > 4.503599627370496E+15) || (rtIsInf(t) || rtIsNaN(t))) {
      t = log(t + 1.0);
    } else if (!(absz < 2.2204460492503131E-16)) {
      t = t / ((t + 1.0) - 1.0) * log(t + 1.0);
    }

    *x = t / 2.0;
  }

  if (negx) {
    *x = -*x;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T LR_sqrtObo(real_T a, int32_T s)
{
  real_T r;
  real_T val;
  real_T z0;
  int32_T i;
  int32_T n0;
  if (s == 0) {
    val = a - 1.0;
  } else {
    n0 = s - 2;
    if (rt_atan2d_snf(0.0, a) >= 1.5707963267948966) {
      a = sqrt(a);
      n0 = s - 3;
    }

    z0 = a - 1.0;
    a = sqrt(a);
    r = a + 1.0;
    for (i = 0; i <= n0; i++) {
      a = sqrt(a);
      r *= a + 1.0;
    }

    val = z0 / r;
  }

  return val;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void LR_computeLogOfSchurForm(const real_T T[16], const creal_T d[4],
  real_T L[16], int32_T *exitflag)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T B[16];
  real_T K[16];
  real_T Troot[16];
  real_T A[4];
  real_T P[4];
  real_T P_0[4];
  real_T b_Troot[4];
  real_T tmp_1[2];
  real_T T_tmp;
  real_T T_tmp_0;
  real_T T_tmp_1;
  real_T T_tmp_2;
  real_T Z0_idx_0;
  real_T Z0_idx_1;
  real_T Z0_idx_2;
  real_T Z0_idx_3;
  real_T Z0_tmp_idx_2;
  real_T Z0_tmp_idx_3;
  real_T a21;
  real_T a22;
  int32_T ipiv[4];
  int32_T B_tmp;
  int32_T b_Troot_tmp;
  int32_T b_Troot_tmp_0;
  int32_T b_Troot_tmp_1;
  int32_T b_j;
  int32_T b_lastBlock;
  int32_T d_i;
  int32_T j;
  int32_T kAcol;
  int32_T m;
  int32_T r1;
  int32_T r2;
  int8_T blockStruct[3];
  int8_T blockStruct_0;
  static const real_T e[49] = { 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.21132486540518711, 0.78867513459481287, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.11270166537925831, 0.5, 0.8872983346207417, 0.0, 0.0, 0.0, 0.0,
    0.069431844202973714, 0.33000947820757187, 0.66999052179242813,
    0.93056815579702634, 0.0, 0.0, 0.0, 0.046910077030668004,
    0.23076534494715845, 0.5, 0.7692346550528415, 0.953089922969332, 0.0, 0.0,
    0.033765242898423989, 0.16939530676686773, 0.38069040695840156,
    0.61930959304159849, 0.83060469323313224, 0.966234757101576, 0.0,
    0.025446043828620736, 0.12923440720030277, 0.29707742431130141, 0.5,
    0.70292257568869854, 0.87076559279969723, 0.9745539561713793 };

  static const real_T h[49] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.27777777777777779, 0.44444444444444442,
    0.27777777777777779, 0.0, 0.0, 0.0, 0.0, 0.17392742256872692,
    0.32607257743127305, 0.32607257743127305, 0.17392742256872692, 0.0, 0.0, 0.0,
    0.11846344252809454, 0.23931433524968324, 0.28444444444444444,
    0.23931433524968324, 0.11846344252809454, 0.0, 0.0, 0.085662246189585178,
    0.1803807865240693, 0.23395696728634552, 0.23395696728634552,
    0.1803807865240693, 0.085662246189585178, 0.0, 0.064742483084434851,
    0.13985269574463832, 0.19091502525255946, 0.2089795918367347,
    0.19091502525255946, 0.13985269574463832, 0.064742483084434851 };

  creal_T d_0[4];
  memcpy(&Troot[0], &T[0], sizeof(real_T) << 4U);
  memcpy(&d_0[0], &d[0], sizeof(creal_T) << 2);
  LR_logmParams(Troot, d_0, &b_lastBlock, &m, exitflag);
  blockStruct[0] = 0;
  blockStruct[1] = 0;
  blockStruct[2] = 0;
  j = 0;
  while (j + 1 < 3) {
    T_tmp_0 = T[((j << 2) + j) + 1];
    if (T_tmp_0 != 0.0) {
      blockStruct[j] = 2;
      blockStruct[j + 1] = 0;
      j += 2;
    } else if ((T_tmp_0 == 0.0) && (T[(((j + 1) << 2) + j) + 2] == 0.0)) {
      blockStruct[j] = 1;
      j++;
    } else {
      blockStruct[j] = 0;
      j++;
    }
  }

  if (T[11] != 0.0) {
    blockStruct[2] = 2;
  } else if ((blockStruct[1] == 0) || (blockStruct[1] == 1)) {
    blockStruct[2] = 1;
  }

  j = 0;
  for (b_j = 0; b_j < 3; b_j++) {
    blockStruct_0 = blockStruct[b_j];
    if (blockStruct_0 == 0) {
      if (j != 0) {
        j = 0;
      } else {
        r1 = (b_j << 2) + b_j;
        Troot[r1] = LR_sqrtObo(T[r1], b_lastBlock);
      }
    } else {
      j = blockStruct_0;
      b_Troot_tmp = (b_j << 2) + b_j;
      b_Troot[0] = Troot[b_Troot_tmp];
      T_tmp_0 = T[b_Troot_tmp];
      A[0] = T_tmp_0;
      b_Troot[1] = Troot[b_Troot_tmp + 1];
      T_tmp = T[b_Troot_tmp + 1];
      A[1] = T_tmp;
      b_Troot_tmp_0 = ((b_j + 1) << 2) + b_j;
      b_Troot[2] = Troot[b_Troot_tmp_0];
      T_tmp_2 = T[b_Troot_tmp_0];
      A[2] = T_tmp_2;
      b_Troot[3] = Troot[b_Troot_tmp_0 + 1];
      T_tmp_1 = T[b_Troot_tmp_0 + 1];
      A[3] = T_tmp_1;
      if (b_lastBlock == 0) {
        b_Troot[0] = T_tmp_0 - 1.0;
        b_Troot[1] = T_tmp;
        b_Troot[2] = T_tmp_2;
        b_Troot[3] = T_tmp_1 - 1.0;
      } else {
        LR_sqrtm2by2(A);
        Z0_idx_0 = A[0] - 1.0;
        Z0_idx_1 = A[1];
        Z0_idx_2 = A[2];
        Z0_idx_3 = A[3] - 1.0;
        if (b_lastBlock == 1) {
          b_Troot[0] = A[0] - 1.0;
          b_Troot[1] = A[1];
          b_Troot[2] = A[2];
          b_Troot[3] = A[3] - 1.0;
        } else {
          LR_sqrtm2by2(A);
          P[0] = A[0] + 1.0;
          P[1] = A[1];
          P[2] = A[2];
          P[3] = A[3] + 1.0;
          r1 = (uint8_T)(b_lastBlock - 2);
          for (r2 = 0; r2 < r1; r2++) {
            LR_sqrtm2by2(A);
            a21 = A[0] + 1.0;
            a22 = A[1];
            Z0_tmp_idx_2 = A[2];
            Z0_tmp_idx_3 = A[3] + 1.0;
            for (b_Troot_tmp_1 = 0; b_Troot_tmp_1 <= 0; b_Troot_tmp_1 += 2) {
              tmp = _mm_loadu_pd(&P[b_Troot_tmp_1 + 2]);
              tmp_0 = _mm_loadu_pd(&P[b_Troot_tmp_1]);
              _mm_storeu_pd(&P_0[b_Troot_tmp_1], _mm_add_pd(_mm_mul_pd(tmp,
                _mm_set1_pd(a22)), _mm_mul_pd(tmp_0, _mm_set1_pd(a21))));
              _mm_storeu_pd(&P_0[b_Troot_tmp_1 + 2], _mm_add_pd(_mm_mul_pd(tmp,
                _mm_set1_pd(Z0_tmp_idx_3)), _mm_mul_pd(tmp_0, _mm_set1_pd
                (Z0_tmp_idx_2))));
            }

            P[0] = P_0[0];
            P[1] = P_0[1];
            P[2] = P_0[2];
            P[3] = P_0[3];
          }

          if (fabs(P[1]) > fabs(P[0])) {
            r1 = 1;
            r2 = 0;
          } else {
            r1 = 0;
            r2 = 1;
          }

          a21 = P[r2] / P[r1];
          Z0_tmp_idx_2 = P[r1 + 2];
          a22 = P[r2 + 2] - Z0_tmp_idx_2 * a21;
          b_Troot_tmp_1 = r1 << 1;
          b_Troot[b_Troot_tmp_1] = Z0_idx_0 / P[r1];
          r2 <<= 1;
          b_Troot[r2] = (Z0_idx_2 - b_Troot[b_Troot_tmp_1] * Z0_tmp_idx_2) / a22;
          b_Troot[b_Troot_tmp_1] -= b_Troot[r2] * a21;
          b_Troot[b_Troot_tmp_1 + 1] = Z0_idx_1 / P[r1];
          b_Troot[r2 + 1] = (Z0_idx_3 - b_Troot[b_Troot_tmp_1 + 1] *
                             Z0_tmp_idx_2) / a22;
          b_Troot[b_Troot_tmp_1 + 1] -= b_Troot[r2 + 1] * a21;
          if ((T_tmp == 0.0) && (T_tmp_0 >= 0.0) && (T_tmp_1 >= 0.0)) {
            a21 = 1.0 / rt_powd_snf(2.0, (real_T)b_lastBlock);
            if (T_tmp_1 == T_tmp_0) {
              b_Troot[2] = T_tmp_2 * a21 * rt_powd_snf(T_tmp_0, a21 - 1.0);
            } else {
              T_tmp = T_tmp_1 - T_tmp_0;
              Z0_idx_0 = T_tmp / (T_tmp_1 + T_tmp_0);
              if ((fabs(Z0_idx_0) > 90.509667991878089) || (fabs(Z0_idx_0 - 1.0)
                   < 0.011048543456039804) || (fabs(Z0_idx_0 + 1.0) <
                   0.011048543456039804)) {
                b_Troot[2] = (rt_powd_snf(T_tmp_1, a21) - rt_powd_snf(T_tmp_0,
                  a21)) * T_tmp_2 / T_tmp;
              } else {
                LR_atanh(&Z0_idx_0);
                b_Troot[2] = exp((log(T_tmp_1) + log(T_tmp_0)) * a21 / 2.0) *
                  2.0 * sinh(a21 * Z0_idx_0) / T_tmp * T_tmp_2;
              }
            }
          }
        }
      }

      Troot[b_Troot_tmp] = b_Troot[0];
      Troot[b_Troot_tmp + 1] = b_Troot[1];
      Troot[b_Troot_tmp_0] = b_Troot[2];
      Troot[b_Troot_tmp_0 + 1] = b_Troot[3];
    }
  }

  if (blockStruct[2] == 0) {
    Troot[15] = LR_sqrtObo(T[15], b_lastBlock);
  }

  memset(&L[0], 0, sizeof(real_T) << 4U);
  j = (uint8_T)m;
  for (b_j = 0; b_j < j; b_j++) {
    b_Troot_tmp = (m - 1) * 7 + b_j;
    T_tmp_0 = e[b_Troot_tmp];
    for (b_Troot_tmp_1 = 0; b_Troot_tmp_1 <= 14; b_Troot_tmp_1 += 2) {
      tmp = _mm_loadu_pd(&Troot[b_Troot_tmp_1]);
      _mm_storeu_pd(&K[b_Troot_tmp_1], _mm_mul_pd(_mm_set1_pd(T_tmp_0), tmp));
      _mm_storeu_pd(&B[b_Troot_tmp_1], tmp);
    }

    tmp = _mm_set1_pd(1.0);
    _mm_storeu_pd(&tmp_1[0], _mm_add_pd(_mm_set_pd(K[5], K[0]), tmp));
    K[0] = tmp_1[0];
    K[5] = tmp_1[1];
    _mm_storeu_pd(&tmp_1[0], _mm_add_pd(_mm_set_pd(K[15], K[10]), tmp));
    K[10] = tmp_1[0];
    K[15] = tmp_1[1];
    LR_xgetrf(K, ipiv, &r1);
    for (r1 = 0; r1 < 3; r1++) {
      r2 = ipiv[r1];
      if (r1 + 1 != r2) {
        a21 = B[r1];
        B[r1] = B[r2 - 1];
        B[r2 - 1] = a21;
        a21 = B[r1 + 4];
        B[r1 + 4] = B[r2 + 3];
        B[r2 + 3] = a21;
        a21 = B[r1 + 8];
        B[r1 + 8] = B[r2 + 7];
        B[r2 + 7] = a21;
        a21 = B[r1 + 12];
        B[r1 + 12] = B[r2 + 11];
        B[r2 + 11] = a21;
      }
    }

    for (r1 = 0; r1 < 4; r1++) {
      r2 = r1 << 2;
      for (b_Troot_tmp_0 = 0; b_Troot_tmp_0 < 4; b_Troot_tmp_0++) {
        kAcol = b_Troot_tmp_0 << 2;
        b_Troot_tmp_1 = b_Troot_tmp_0 + r2;
        if (B[b_Troot_tmp_1] != 0.0) {
          for (d_i = b_Troot_tmp_0 + 2; d_i < 5; d_i++) {
            B_tmp = (d_i + r2) - 1;
            B[B_tmp] -= K[(d_i + kAcol) - 1] * B[b_Troot_tmp_1];
          }
        }
      }
    }

    for (r1 = 0; r1 < 4; r1++) {
      r2 = r1 << 2;
      for (b_Troot_tmp_0 = 3; b_Troot_tmp_0 >= 0; b_Troot_tmp_0--) {
        kAcol = b_Troot_tmp_0 << 2;
        b_Troot_tmp_1 = b_Troot_tmp_0 + r2;
        T_tmp_0 = B[b_Troot_tmp_1];
        if (T_tmp_0 != 0.0) {
          B[b_Troot_tmp_1] = T_tmp_0 / K[b_Troot_tmp_0 + kAcol];
          for (d_i = 0; d_i < b_Troot_tmp_0; d_i++) {
            B_tmp = d_i + r2;
            B[B_tmp] -= K[d_i + kAcol] * B[b_Troot_tmp_1];
          }
        }
      }
    }

    T_tmp_0 = h[b_Troot_tmp];
    for (b_Troot_tmp_1 = 0; b_Troot_tmp_1 <= 14; b_Troot_tmp_1 += 2) {
      tmp = _mm_loadu_pd(&B[b_Troot_tmp_1]);
      tmp_0 = _mm_loadu_pd(&L[b_Troot_tmp_1]);
      _mm_storeu_pd(&L[b_Troot_tmp_1], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(T_tmp_0),
        tmp), tmp_0));
    }
  }

  a21 = rt_powd_snf(2.0, (real_T)b_lastBlock);
  for (b_Troot_tmp_1 = 0; b_Troot_tmp_1 <= 14; b_Troot_tmp_1 += 2) {
    tmp = _mm_loadu_pd(&L[b_Troot_tmp_1]);
    _mm_storeu_pd(&L[b_Troot_tmp_1], _mm_mul_pd(_mm_set1_pd(a21), tmp));
  }

  b_lastBlock = 0;
  for (m = 0; m < 3; m++) {
    switch (blockStruct[m]) {
     case 0:
      if (b_lastBlock != 0) {
        b_lastBlock = 0;
      } else {
        j = (m << 2) + m;
        L[j] = log(T[j]);
      }
      break;

     case 1:
      b_lastBlock = 1;
      b_Troot_tmp = (m << 2) + m;
      b_Troot[1] = L[b_Troot_tmp + 1];
      b_Troot_tmp_0 = ((m + 1) << 2) + m;
      b_Troot[2] = L[b_Troot_tmp_0];
      T_tmp_0 = T[b_Troot_tmp];
      a21 = log(T_tmp_0);
      T_tmp_2 = T[b_Troot_tmp_0 + 1];
      a22 = log(T_tmp_2);
      if ((!(T_tmp_0 < 0.0)) && (!(T_tmp_2 < 0.0))) {
        if (T_tmp_2 == T_tmp_0) {
          b_Troot[2] = T[b_Troot_tmp_0] / T_tmp_0;
        } else {
          T_tmp = T_tmp_2 - T_tmp_0;
          Z0_idx_0 = T_tmp / (T_tmp_2 + T_tmp_0);
          if ((fabs(Z0_idx_0) > 90.509667991878089) || (fabs(Z0_idx_0 - 1.0) <
               0.011048543456039804) || (fabs(Z0_idx_0 + 1.0) <
               0.011048543456039804)) {
            b_Troot[2] = (a22 - a21) * T[b_Troot_tmp_0] / T_tmp;
          } else {
            LR_atanh(&Z0_idx_0);
            b_Troot[2] = 2.0 * Z0_idx_0 * T[b_Troot_tmp_0] / T_tmp;
          }
        }
      }

      L[b_Troot_tmp] = a21;
      L[b_Troot_tmp + 1] = b_Troot[1];
      L[b_Troot_tmp_0] = b_Troot[2];
      L[b_Troot_tmp_0 + 1] = a22;
      break;

     default:
      b_lastBlock = 2;
      j = (m << 2) + m;
      T_tmp_0 = T[j];
      b_j = ((m + 1) << 2) + m;
      T_tmp_2 = T[b_j];
      T_tmp_1 = T[j + 1];
      a21 = log(T_tmp_0 * T_tmp_0 - T_tmp_2 * T_tmp_1) * 0.5;
      a22 = sqrt(-T_tmp_2 * T_tmp_1);
      a22 = rt_atan2d_snf(a22, T_tmp_0) / a22;
      L[j] = a21;
      L[j + 1] = T_tmp_1 * a22;
      L[b_j] = T_tmp_2 * a22;
      L[b_j + 1] = a21;
      break;
    }
  }

  if (blockStruct[2] == 0) {
    L[15] = log(T[15]);
  }
}

/* Model step function */
void LR_step(void)
{
  __m128d tmp_3;
  __m128d tmp_4;
  __m128d tmp_b;
  __m128d tmp_c;
  __m128d tmp_d;
  __m128d tmp_e;
  __m128d tmp_g;
  __m128d tmp_h;
  __m128d tmp_i;
  __m128d tmp_m;
  creal_T U[16];
  creal_T U_0[16];
  creal_T b_0[16];
  creal_T se3mat[16];
  creal_T d[4];
  real_T b[36];
  real_T b_b[36];
  real_T c_b[36];
  real_T dlog6mat[36];
  real_T dlog6mat_0[36];
  real_T dlog6mat_1[36];
  real_T tmp_1[36];
  real_T tmp_2[36];
  real_T A[16];
  real_T A2[16];
  real_T A4[16];
  real_T A6[16];
  real_T rtb_T[16];
  real_T tmp_0[16];
  real_T tmp_n[16];
  real_T AA[9];
  real_T AA_0[9];
  real_T AB[9];
  real_T AB_0[9];
  real_T AB_1[9];
  real_T AB_2[9];
  real_T B[9];
  real_T BA[9];
  real_T BA_0[9];
  real_T BA_1[9];
  real_T BA_2[9];
  real_T B_0[9];
  real_T Bdot[9];
  real_T Bdot_0[9];
  real_T Bdot_1[9];
  real_T Bdot_2[9];
  real_T Cxi[9];
  real_T Cxi_0[9];
  real_T Cxi_1[9];
  real_T ceil_xi[9];
  real_T ceil_xi_0[9];
  real_T ceil_xi_1[9];
  real_T dAB[9];
  real_T dAB_0[9];
  real_T dBA[9];
  real_T dBA_0[9];
  real_T ddBA[9];
  real_T dddexp3_[9];
  real_T eta_[9];
  real_T eta__0[9];
  real_T etaddot_[9];
  real_T etadot_[9];
  real_T xn_0[7];
  real_T xn[6];
  real_T w[4];
  real_T tmp_5[2];
  real_T tmp_6[2];
  real_T tmp_7[2];
  real_T tmp_8[2];
  real_T tmp_9[2];
  real_T tmp_a[2];
  real_T tmp_f[2];
  real_T AB_3;
  real_T AB_4;
  real_T BA_3;
  real_T BA_4;
  real_T B_tmp;
  real_T Bdot_3;
  real_T Bdot_4;
  real_T Bdot_5;
  real_T Cxi_2;
  real_T Cxi_3;
  real_T Gamma_1;
  real_T Gamma_2;
  real_T Gamma_3;
  real_T Gamma_4;
  real_T alpha;
  real_T b_dGamma_2;
  real_T b_s;
  real_T b_s_tmp;
  real_T beta;
  real_T beta_2;
  real_T ceil_xi_2;
  real_T ceil_xi_tmp;
  real_T cx;
  real_T d6;
  real_T dAB_1;
  real_T dBA_1;
  real_T dGamma_1;
  real_T dGamma_2;
  real_T ddBA_0;
  real_T ddBA_1;
  real_T ddBA_2;
  real_T ddGamma_1;
  real_T ddOmega_1;
  real_T ddOmega_1_tmp;
  real_T dddexp3__0;
  real_T dddexp3__tmp;
  real_T dddexp3__tmp_0;
  real_T dddexp3__tmp_1;
  real_T eta1;
  real_T n_xi2;
  real_T n_xi5;
  real_T n_xi6;
  real_T normxidot;
  real_T theta_sqr;
  real_T tmp;
  real_T tmp_j;
  real_T tmp_k;
  real_T tmp_l;
  real_T xcx_tmp;
  real_T xiTxidot;
  real_T xixidot;
  real_T xn2sx;
  int32_T blockFormat[3];
  int32_T A6_tmp;
  int32_T dAB_tmp;
  int32_T e_i;
  int32_T e_j;
  int32_T exitg1;
  int32_T exitg3;
  int32_T f_i;
  int32_T i;
  int32_T se3mat_re_tmp;
  int8_T b_I[9];
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T guard3;
  boolean_T guard4;
  boolean_T isPrincipalLog;
  boolean_T p;
  boolean_T recomputeDiags;

  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/exp6_lambda'
   */
  A[0] = 0.0;
  A[4] = -LR_U.exp6_lambda[5];
  A[8] = LR_U.exp6_lambda[4];
  A[1] = LR_U.exp6_lambda[5];
  A[5] = 0.0;
  A[9] = -LR_U.exp6_lambda[3];
  A[2] = -LR_U.exp6_lambda[4];
  A[6] = LR_U.exp6_lambda[3];
  A[10] = 0.0;
  A[12] = LR_U.exp6_lambda[0];
  A[13] = LR_U.exp6_lambda[1];
  A[14] = LR_U.exp6_lambda[2];
  A[3] = 0.0;
  A[7] = 0.0;
  A[11] = 0.0;
  A[15] = 0.0;
  recomputeDiags = true;
  for (e_j = 0; e_j < 16; e_j++) {
    if (recomputeDiags) {
      n_xi2 = A[e_j];
      if ((!rtIsInf(n_xi2)) && (!rtIsNaN(n_xi2))) {
      } else {
        recomputeDiags = false;
      }
    } else {
      recomputeDiags = false;
    }
  }

  if (!recomputeDiags) {
    for (i = 0; i < 16; i++) {
      rtb_T[i] = (rtNaN);
    }
  } else {
    p = true;
    e_j = 0;
    exitg2 = false;
    while ((!exitg2) && (e_j < 4)) {
      e_i = 0;
      do {
        exitg1 = 0;
        if (e_i < 4) {
          if ((e_i != e_j) && (!(A[(e_j << 2) + e_i] == 0.0))) {
            p = false;
            exitg1 = 1;
          } else {
            e_i++;
          }
        } else {
          e_j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (p) {
      memset(&rtb_T[0], 0, sizeof(real_T) << 4U);
      rtb_T[0] = 1.0;
      rtb_T[5] = 1.0;
      rtb_T[10] = 1.0;
      rtb_T[15] = 1.0;
    } else {
      e_j = 0;
      exitg2 = false;
      while ((!exitg2) && (e_j < 4)) {
        e_i = 0;
        do {
          exitg1 = 0;
          if (e_i <= e_j) {
            if (!(A[(e_j << 2) + e_i] == A[(e_i << 2) + e_j])) {
              recomputeDiags = false;
              exitg1 = 1;
            } else {
              e_i++;
            }
          } else {
            e_j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }

      if (recomputeDiags) {
        LR_xsyheev(A, &f_i, w);
        for (f_i = 0; f_i <= 2; f_i += 2) {
          i = f_i << 2;
          e_i = (f_i + 1) << 2;
          tmp_b = _mm_set_pd(exp(w[f_i + 1]), exp(w[f_i]));
          _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(A[e_i], A[i]), tmp_b));
          rtb_T[i] = tmp_f[0];
          rtb_T[e_i] = tmp_f[1];
          _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(A[e_i + 1], A[i + 1]),
            tmp_b));
          rtb_T[i + 1] = tmp_f[0];
          rtb_T[e_i + 1] = tmp_f[1];
          _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(A[e_i + 2], A[i + 2]),
            tmp_b));
          rtb_T[i + 2] = tmp_f[0];
          rtb_T[e_i + 2] = tmp_f[1];
          _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(A[e_i + 3], A[i + 3]),
            tmp_b));
          rtb_T[i + 3] = tmp_f[0];
          rtb_T[e_i + 3] = tmp_f[1];
        }

        for (i = 0; i < 4; i++) {
          n_xi2 = rtb_T[i + 4];
          ddOmega_1 = rtb_T[i];
          eta1 = rtb_T[i + 8];
          n_xi6 = rtb_T[i + 12];
          for (e_i = 0; e_i <= 2; e_i += 2) {
            tmp_b = _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
              (n_xi2), _mm_loadu_pd(&A[e_i + 4])), _mm_mul_pd(_mm_set1_pd
              (ddOmega_1), _mm_loadu_pd(&A[e_i]))), _mm_mul_pd(_mm_set1_pd(eta1),
              _mm_loadu_pd(&A[e_i + 8]))), _mm_mul_pd(_mm_set1_pd(n_xi6),
              _mm_loadu_pd(&A[e_i + 12])));
            _mm_storeu_pd(&tmp_f[0], tmp_b);
            A2[i + (e_i << 2)] = tmp_f[0];
            A2[i + ((e_i + 1) << 2)] = tmp_f[1];
          }
        }

        memcpy(&rtb_T[0], &A2[0], sizeof(real_T) << 4U);
        for (i = 0; i <= 2; i += 2) {
          e_i = i << 2;
          A6_tmp = (i + 1) << 2;
          tmp_b = _mm_set1_pd(2.0);
          tmp_c = _mm_div_pd(_mm_add_pd(_mm_set_pd(rtb_T[A6_tmp], rtb_T[e_i]),
            _mm_loadu_pd(&rtb_T[i])), tmp_b);
          _mm_storeu_pd(&tmp_f[0], tmp_c);
          A2[e_i] = tmp_f[0];
          A2[A6_tmp] = tmp_f[1];
          tmp_c = _mm_div_pd(_mm_add_pd(_mm_set_pd(rtb_T[A6_tmp + 1], rtb_T[e_i
            + 1]), _mm_loadu_pd(&rtb_T[i + 4])), tmp_b);
          _mm_storeu_pd(&tmp_f[0], tmp_c);
          A2[e_i + 1] = tmp_f[0];
          A2[A6_tmp + 1] = tmp_f[1];
          tmp_c = _mm_div_pd(_mm_add_pd(_mm_set_pd(rtb_T[A6_tmp + 2], rtb_T[e_i
            + 2]), _mm_loadu_pd(&rtb_T[i + 8])), tmp_b);
          _mm_storeu_pd(&tmp_f[0], tmp_c);
          A2[e_i + 2] = tmp_f[0];
          A2[A6_tmp + 2] = tmp_f[1];
          tmp_b = _mm_div_pd(_mm_add_pd(_mm_set_pd(rtb_T[A6_tmp + 3], rtb_T[e_i
            + 3]), _mm_loadu_pd(&rtb_T[i + 12])), tmp_b);
          _mm_storeu_pd(&tmp_f[0], tmp_b);
          A2[e_i + 3] = tmp_f[0];
          A2[A6_tmp + 3] = tmp_f[1];
        }

        memcpy(&rtb_T[0], &A2[0], sizeof(real_T) << 4U);
      } else {
        recomputeDiags = true;
        e_j = 3;
        while (recomputeDiags && (e_j <= 4)) {
          e_i = e_j;
          while (recomputeDiags && (e_i <= 4)) {
            recomputeDiags = (A[(((e_j - 3) << 2) + e_i) - 1] == 0.0);
            e_i++;
          }

          e_j++;
        }

        if (recomputeDiags) {
          e_j = 1;
          exitg2 = false;
          while ((!exitg2) && (e_j - 1 < 3)) {
            i = ((e_j - 1) << 2) + e_j;
            ddOmega_1 = A[i];
            if (ddOmega_1 != 0.0) {
              if ((e_j != 3) && (A[((e_j << 2) + e_j) + 1] != 0.0)) {
                recomputeDiags = false;
                exitg2 = true;
              } else {
                e_i = (e_j << 2) + e_j;
                if (A[i - 1] != A[e_i]) {
                  recomputeDiags = false;
                  exitg2 = true;
                } else {
                  n_xi2 = A[e_i - 1];
                  if (rtIsNaN(ddOmega_1)) {
                    dddexp3__tmp = (rtNaN);
                  } else if (ddOmega_1 < 0.0) {
                    dddexp3__tmp = -1.0;
                  } else {
                    dddexp3__tmp = (ddOmega_1 > 0.0);
                  }

                  if (rtIsNaN(n_xi2)) {
                    dddexp3__tmp_0 = (rtNaN);
                  } else if (n_xi2 < 0.0) {
                    dddexp3__tmp_0 = -1.0;
                  } else {
                    dddexp3__tmp_0 = (n_xi2 > 0.0);
                  }

                  if (dddexp3__tmp * dddexp3__tmp_0 != -1.0) {
                    recomputeDiags = false;
                    exitg2 = true;
                  } else {
                    e_j++;
                  }
                }
              }
            } else {
              e_j++;
            }
          }
        }

        ddOmega_1 = 0.0;
        for (i = 0; i < 4; i++) {
          for (e_i = 0; e_i <= 2; e_i += 2) {
            tmp_b = _mm_loadu_pd(&A[e_i + 4]);
            A6_tmp = i << 2;
            tmp_c = _mm_loadu_pd(&A[e_i]);
            tmp_d = _mm_loadu_pd(&A[e_i + 8]);
            tmp_e = _mm_loadu_pd(&A[e_i + 12]);
            _mm_storeu_pd(&A2[e_i + A6_tmp], _mm_add_pd(_mm_add_pd(_mm_add_pd
              (_mm_mul_pd(_mm_set1_pd(A[A6_tmp + 1]), tmp_b), _mm_mul_pd
               (_mm_set1_pd(A[A6_tmp]), tmp_c)), _mm_mul_pd(_mm_set1_pd(A[A6_tmp
              + 2]), tmp_d)), _mm_mul_pd(_mm_set1_pd(A[A6_tmp + 3]), tmp_e)));
          }
        }

        for (i = 0; i < 4; i++) {
          for (e_i = 0; e_i <= 2; e_i += 2) {
            A6_tmp = (e_i + 1) << 2;
            dAB_tmp = e_i << 2;
            _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set_pd(A2[A6_tmp + 1], A2[dAB_tmp + 1]), _mm_set1_pd(A2[i + 4])),
              _mm_mul_pd(_mm_set_pd(A2[A6_tmp], A2[dAB_tmp]), _mm_set1_pd(A2[i]))),
              _mm_mul_pd(_mm_set_pd(A2[A6_tmp + 2], A2[dAB_tmp + 2]),
                         _mm_set1_pd(A2[i + 8]))), _mm_mul_pd(_mm_set_pd
              (A2[A6_tmp + 3], A2[dAB_tmp + 3]), _mm_set1_pd(A2[i + 12]))));
            A4[i + dAB_tmp] = tmp_f[0];
            A4[i + A6_tmp] = tmp_f[1];
          }

          n_xi2 = A4[i + 4];
          n_xi6 = A4[i];
          eta1 = A4[i + 8];
          d6 = A4[i + 12];
          for (e_i = 0; e_i <= 2; e_i += 2) {
            A6_tmp = (e_i + 1) << 2;
            dAB_tmp = e_i << 2;
            _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set_pd(A2[A6_tmp + 1], A2[dAB_tmp + 1]), _mm_set1_pd(n_xi2)),
              _mm_mul_pd(_mm_set_pd(A2[A6_tmp], A2[dAB_tmp]), _mm_set1_pd(n_xi6))),
              _mm_mul_pd(_mm_set_pd(A2[A6_tmp + 2], A2[dAB_tmp + 2]),
                         _mm_set1_pd(eta1))), _mm_mul_pd(_mm_set_pd(A2[A6_tmp +
              3], A2[dAB_tmp + 3]), _mm_set1_pd(d6))));
            A6[i + dAB_tmp] = tmp_f[0];
            A6[i + A6_tmp] = tmp_f[1];
          }
        }

        d6 = rt_powd_snf(LR_norm(A6), 0.16666666666666666);
        eta1 = fmax(rt_powd_snf(LR_norm(A4), 0.25), d6);
        guard1 = false;
        guard2 = false;
        guard3 = false;
        guard4 = false;
        if (eta1 <= 0.01495585217958292) {
          for (e_j = 0; e_j <= 14; e_j += 2) {
            tmp_5[0] = fabs(A[e_j]);
            tmp_5[1] = fabs(A[e_j + 1]);
            tmp_b = _mm_loadu_pd(&tmp_5[0]);
            _mm_storeu_pd(&tmp_0[e_j], _mm_mul_pd(_mm_set1_pd
              (0.19285012468241128), tmp_b));
          }

          LR_mpower(tmp_0, 7.0, tmp_n);
          if (fmax(ceil(LR_log2(LR_norm(tmp_n) / LR_norm(A) * 2.0 /
                                2.2204460492503131E-16) / 6.0), 0.0) == 0.0) {
            f_i = 3;
          } else {
            guard4 = true;
          }
        } else {
          guard4 = true;
        }

        if (guard4) {
          if (eta1 <= 0.253939833006323) {
            for (e_j = 0; e_j <= 14; e_j += 2) {
              tmp_6[0] = fabs(A[e_j]);
              tmp_6[1] = fabs(A[e_j + 1]);
              tmp_b = _mm_loadu_pd(&tmp_6[0]);
              _mm_storeu_pd(&tmp_0[e_j], _mm_mul_pd(_mm_set1_pd
                (0.12321872304378752), tmp_b));
            }

            LR_mpower(tmp_0, 11.0, tmp_n);
            if (fmax(ceil(LR_log2(LR_norm(tmp_n) / LR_norm(A) * 2.0 /
                                  2.2204460492503131E-16) / 10.0), 0.0) == 0.0)
            {
              f_i = 5;
            } else {
              guard3 = true;
            }
          } else {
            guard3 = true;
          }
        }

        if (guard3) {
          LR_mpower(A4, 2.0, tmp_0);
          eta1 = rt_powd_snf(LR_norm(tmp_0), 0.125);
          d6 = fmax(d6, eta1);
          if (d6 <= 0.95041789961629319) {
            for (e_j = 0; e_j <= 14; e_j += 2) {
              tmp_7[0] = fabs(A[e_j]);
              tmp_7[1] = fabs(A[e_j + 1]);
              tmp_b = _mm_loadu_pd(&tmp_7[0]);
              _mm_storeu_pd(&tmp_0[e_j], _mm_mul_pd(_mm_set1_pd
                (0.090475336558796943), tmp_b));
            }

            LR_mpower(tmp_0, 15.0, tmp_n);
            if (fmax(ceil(LR_log2(LR_norm(tmp_n) / LR_norm(A) * 2.0 /
                                  2.2204460492503131E-16) / 14.0), 0.0) == 0.0)
            {
              f_i = 7;
            } else {
              guard2 = true;
            }
          } else {
            guard2 = true;
          }
        }

        if (guard2) {
          if (d6 <= 2.097847961257068) {
            for (e_j = 0; e_j <= 14; e_j += 2) {
              tmp_8[0] = fabs(A[e_j]);
              tmp_8[1] = fabs(A[e_j + 1]);
              tmp_b = _mm_loadu_pd(&tmp_8[0]);
              _mm_storeu_pd(&tmp_0[e_j], _mm_mul_pd(_mm_set1_pd
                (0.071467735648795785), tmp_b));
            }

            LR_mpower(tmp_0, 19.0, tmp_n);
            if (fmax(ceil(LR_log2(LR_norm(tmp_n) / LR_norm(A) * 2.0 /
                                  2.2204460492503131E-16) / 18.0), 0.0) == 0.0)
            {
              f_i = 9;
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
        }

        if (guard1) {
          for (i = 0; i < 4; i++) {
            A6_tmp = i << 2;
            n_xi2 = A6[A6_tmp + 1];
            ddOmega_1 = A6[A6_tmp];
            n_xi6 = A6[A6_tmp + 2];
            b_s = A6[A6_tmp + 3];
            for (e_i = 0; e_i <= 2; e_i += 2) {
              tmp_b = _mm_loadu_pd(&A4[e_i + 4]);
              tmp_c = _mm_loadu_pd(&A4[e_i]);
              tmp_d = _mm_loadu_pd(&A4[e_i + 8]);
              tmp_e = _mm_loadu_pd(&A4[e_i + 12]);
              _mm_storeu_pd(&rtb_T[e_i + A6_tmp], _mm_add_pd(_mm_add_pd
                (_mm_add_pd(_mm_mul_pd(_mm_set1_pd(n_xi2), tmp_b), _mm_mul_pd
                            (_mm_set1_pd(ddOmega_1), tmp_c)), _mm_mul_pd
                 (_mm_set1_pd(n_xi6), tmp_d)), _mm_mul_pd(_mm_set1_pd(b_s),
                tmp_e)));
            }
          }

          ddOmega_1 = fmax(ceil(LR_log2(fmin(d6, fmax(eta1, rt_powd_snf(LR_norm
            (rtb_T), 0.1))) / 5.3719203511481517)), 0.0);
          d6 = rt_powd_snf(2.0, ddOmega_1);
          for (e_j = 0; e_j <= 14; e_j += 2) {
            tmp_b = _mm_loadu_pd(&A[e_j]);
            tmp_b = _mm_div_pd(tmp_b, _mm_set1_pd(d6));
            _mm_storeu_pd(&rtb_T[e_j], tmp_b);
            _mm_storeu_pd(&tmp_a[0], tmp_b);
            tmp_9[0] = fabs(tmp_a[0]);
            tmp_9[1] = fabs(tmp_a[1]);
            tmp_b = _mm_loadu_pd(&tmp_9[0]);
            _mm_storeu_pd(&tmp_0[e_j], _mm_mul_pd(_mm_set1_pd
              (0.05031554467093536), tmp_b));
          }

          LR_mpower(tmp_0, 27.0, tmp_n);
          ddOmega_1 += fmax(ceil(LR_log2(LR_norm(tmp_n) / LR_norm(rtb_T) * 2.0 /
            2.2204460492503131E-16) / 26.0), 0.0);
          if (rtIsInf(ddOmega_1)) {
            d6 = LR_norm(A) / 5.3719203511481517;
            if ((!rtIsInf(d6)) && (!rtIsNaN(d6))) {
              d6 = frexp(d6, &f_i);
            } else {
              f_i = 0;
            }

            ddOmega_1 = f_i;
            if (d6 == 0.5) {
              ddOmega_1 = (real_T)f_i - 1.0;
            }
          }

          f_i = 13;
        }

        if (ddOmega_1 != 0.0) {
          cx = rt_powd_snf(2.0, ddOmega_1);
          for (i = 0; i <= 14; i += 2) {
            tmp_b = _mm_loadu_pd(&A[i]);
            _mm_storeu_pd(&A[i], _mm_div_pd(tmp_b, _mm_set1_pd(cx)));
          }

          cx = rt_powd_snf(2.0, 2.0 * ddOmega_1);
          d6 = rt_powd_snf(2.0, 4.0 * ddOmega_1);
          for (i = 0; i <= 14; i += 2) {
            tmp_b = _mm_loadu_pd(&A2[i]);
            _mm_storeu_pd(&A2[i], _mm_div_pd(tmp_b, _mm_set1_pd(cx)));
            tmp_b = _mm_loadu_pd(&A4[i]);
            _mm_storeu_pd(&A4[i], _mm_div_pd(tmp_b, _mm_set1_pd(d6)));
          }

          d6 = rt_powd_snf(2.0, 6.0 * ddOmega_1);
          for (i = 0; i <= 14; i += 2) {
            tmp_b = _mm_loadu_pd(&A6[i]);
            _mm_storeu_pd(&A6[i], _mm_div_pd(tmp_b, _mm_set1_pd(d6)));
          }
        }

        if (recomputeDiags) {
          blockFormat[0] = 0;
          blockFormat[1] = 0;
          blockFormat[2] = 0;
          e_j = 0;
          while (e_j + 1 < 3) {
            dddexp3__tmp = A[((e_j << 2) + e_j) + 1];
            if (dddexp3__tmp != 0.0) {
              blockFormat[e_j] = 2;
              blockFormat[e_j + 1] = 0;
              e_j += 2;
            } else if ((dddexp3__tmp == 0.0) && (A[(((e_j + 1) << 2) + e_j) + 2]
                        == 0.0)) {
              blockFormat[e_j] = 1;
              e_j++;
            } else {
              blockFormat[e_j] = 0;
              e_j++;
            }
          }

          if (A[11] != 0.0) {
            blockFormat[2] = 2;
          } else if ((blockFormat[1] == 0) || (blockFormat[1] == 1)) {
            blockFormat[2] = 1;
          }
        }

        LR_padeApproximation(A, A2, A4, A6, f_i, rtb_T);
        if (recomputeDiags) {
          LR_recomputeBlockDiag(A, rtb_T, blockFormat);
        }

        f_i = (int32_T)ddOmega_1;
        for (e_j = 0; e_j < f_i; e_j++) {
          for (i = 0; i < 4; i++) {
            for (e_i = 0; e_i <= 2; e_i += 2) {
              A6_tmp = (e_i + 1) << 2;
              dAB_tmp = e_i << 2;
              _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                (_mm_mul_pd(_mm_set_pd(rtb_T[A6_tmp + 1], rtb_T[dAB_tmp + 1]),
                            _mm_set1_pd(rtb_T[i + 4])), _mm_mul_pd(_mm_set_pd
                (rtb_T[A6_tmp], rtb_T[dAB_tmp]), _mm_set1_pd(rtb_T[i]))),
                _mm_mul_pd(_mm_set_pd(rtb_T[A6_tmp + 2], rtb_T[dAB_tmp + 2]),
                           _mm_set1_pd(rtb_T[i + 8]))), _mm_mul_pd(_mm_set_pd
                (rtb_T[A6_tmp + 3], rtb_T[dAB_tmp + 3]), _mm_set1_pd(rtb_T[i +
                12]))));
              A2[i + dAB_tmp] = tmp_f[0];
              A2[i + A6_tmp] = tmp_f[1];
            }
          }

          memcpy(&rtb_T[0], &A2[0], sizeof(real_T) << 4U);
          if (recomputeDiags) {
            for (i = 0; i <= 14; i += 2) {
              tmp_b = _mm_loadu_pd(&A[i]);
              _mm_storeu_pd(&A[i], _mm_mul_pd(_mm_set1_pd(2.0), tmp_b));
            }

            LR_recomputeBlockDiag(A, rtb_T, blockFormat);
          }
        }
      }
    }
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function1' */

  /* Outport: '<Root>/exp6_T' */
  memcpy(&LR_Y.exp6_T[0], &rtb_T[0], sizeof(real_T) << 4U);

  /* MATLAB Function: '<Root>/MATLAB Function2' incorporates:
   *  Inport: '<Root>/dexp6_lambda'
   */
  eta1 = LR_norm_e(&LR_U.dexp6_lambda[3]);
  ceil_xi[0] = 0.0;
  ceil_xi[3] = -LR_U.dexp6_lambda[5];
  ceil_xi[6] = LR_U.dexp6_lambda[4];
  ceil_xi[1] = LR_U.dexp6_lambda[5];
  ceil_xi[4] = 0.0;
  ceil_xi[7] = -LR_U.dexp6_lambda[3];
  ceil_xi[2] = -LR_U.dexp6_lambda[4];
  ceil_xi[5] = LR_U.dexp6_lambda[3];
  ceil_xi[8] = 0.0;
  ddOmega_1_tmp = sin(eta1 / 2.0) / (eta1 / 2.0);
  alpha = cos(eta1 / 2.0) * ddOmega_1_tmp;
  beta = ddOmega_1_tmp * ddOmega_1_tmp;
  b_s_tmp = beta / 2.0;
  n_xi6 = eta1 * eta1;
  n_xi2 = (1.0 - alpha) / n_xi6;
  cx = (alpha - beta) / n_xi6;
  ddOmega_1 = (cx * LR_U.dexp6_lambda[3] * LR_U.dexp6_lambda[0] + cx *
               LR_U.dexp6_lambda[4] * LR_U.dexp6_lambda[1]) + cx *
    LR_U.dexp6_lambda[5] * LR_U.dexp6_lambda[2];
  cx = ((1.0 - alpha) * 3.0 / n_xi6 - beta / 2.0) * (1.0 / n_xi6);
  cx = (cx * LR_U.dexp6_lambda[3] * LR_U.dexp6_lambda[0] + cx *
        LR_U.dexp6_lambda[4] * LR_U.dexp6_lambda[1]) + cx * LR_U.dexp6_lambda[5]
    * LR_U.dexp6_lambda[2];
  ceil_xi_0[0] = 0.0;
  ceil_xi_0[3] = -LR_U.dexp6_lambda[2];
  ceil_xi_0[6] = LR_U.dexp6_lambda[1];
  ceil_xi_0[1] = LR_U.dexp6_lambda[2];
  ceil_xi_0[4] = 0.0;
  ceil_xi_0[7] = -LR_U.dexp6_lambda[0];
  ceil_xi_0[2] = -LR_U.dexp6_lambda[1];
  ceil_xi_0[5] = LR_U.dexp6_lambda[0];
  ceil_xi_0[8] = 0.0;
  Cxi[0] = 0.0;
  Cxi[3] = -LR_U.dexp6_lambda[5];
  Cxi[6] = LR_U.dexp6_lambda[4];
  Cxi[1] = LR_U.dexp6_lambda[5];
  Cxi[4] = 0.0;
  Cxi[7] = -LR_U.dexp6_lambda[3];
  Cxi[2] = -LR_U.dexp6_lambda[4];
  Cxi[5] = LR_U.dexp6_lambda[3];
  Cxi[8] = 0.0;
  B[0] = 0.0;
  B[3] = -LR_U.dexp6_lambda[5];
  B[6] = LR_U.dexp6_lambda[4];
  B[1] = LR_U.dexp6_lambda[5];
  B[4] = 0.0;
  B[7] = -LR_U.dexp6_lambda[3];
  B[2] = -LR_U.dexp6_lambda[4];
  B[5] = LR_U.dexp6_lambda[3];
  B[8] = 0.0;
  Bdot[0] = 0.0;
  Bdot[3] = -LR_U.dexp6_lambda[2];
  Bdot[6] = LR_U.dexp6_lambda[1];
  Bdot[1] = LR_U.dexp6_lambda[2];
  Bdot[4] = 0.0;
  Bdot[7] = -LR_U.dexp6_lambda[0];
  Bdot[2] = -LR_U.dexp6_lambda[1];
  Bdot[5] = LR_U.dexp6_lambda[0];
  Bdot[8] = 0.0;
  for (i = 0; i < 3; i++) {
    e_i = 3 * i + 1;
    dddexp3__tmp = Bdot[e_i];
    dddexp3__tmp_0 = Bdot[3 * i];
    A6_tmp = 3 * i + 2;
    ddOmega_1_tmp = Bdot[A6_tmp];
    dddexp3__tmp_1 = Cxi[e_i];
    Cxi_3 = Cxi[3 * i];
    Bdot_4 = Cxi[A6_tmp];
    for (e_i = 0; e_i <= 0; e_i += 2) {
      tmp_b = _mm_loadu_pd(&B[e_i + 3]);
      tmp_c = _mm_loadu_pd(&B[e_i]);
      tmp_d = _mm_loadu_pd(&B[e_i + 6]);
      A6_tmp = 3 * i + e_i;
      _mm_storeu_pd(&ddBA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (dddexp3__tmp), tmp_b), _mm_mul_pd(_mm_set1_pd(dddexp3__tmp_0), tmp_c)),
        _mm_mul_pd(_mm_set1_pd(ddOmega_1_tmp), tmp_d)));
      tmp_b = _mm_loadu_pd(&ceil_xi_0[e_i + 3]);
      tmp_c = _mm_loadu_pd(&ceil_xi_0[e_i]);
      tmp_d = _mm_loadu_pd(&ceil_xi_0[e_i + 6]);
      _mm_storeu_pd(&etaddot_[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(dddexp3__tmp_1), tmp_b), _mm_mul_pd(_mm_set1_pd(Cxi_3),
        tmp_c)), _mm_mul_pd(_mm_set1_pd(Bdot_4), tmp_d)));
    }

    for (e_i = 2; e_i < 3; e_i++) {
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (dddexp3__tmp_1, dddexp3__tmp), _mm_set_pd(ceil_xi_0[e_i + 3], B[e_i + 3])),
        _mm_mul_pd(_mm_set_pd(Cxi_3, dddexp3__tmp_0), _mm_set_pd(ceil_xi_0[e_i],
        B[e_i]))), _mm_mul_pd(_mm_set_pd(Bdot_4, ddOmega_1_tmp), _mm_set_pd
        (ceil_xi_0[e_i + 6], B[e_i + 6]))));
      A6_tmp = 3 * i + e_i;
      ddBA[A6_tmp] = tmp_f[0];
      etaddot_[A6_tmp] = tmp_f[1];
    }
  }

  B[0] = b_s_tmp * 0.0;
  tmp_b = _mm_set1_pd(b_s_tmp);
  tmp_c = _mm_set_pd(LR_U.dexp6_lambda[1], -LR_U.dexp6_lambda[2]);
  _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, tmp_c));

  /* MATLAB Function: '<Root>/MATLAB Function2' incorporates:
   *  Inport: '<Root>/dexp6_lambda'
   */
  B[3] = tmp_f[0];
  B[6] = tmp_f[1];
  B[1] = b_s_tmp * LR_U.dexp6_lambda[2];
  B[4] = b_s_tmp * 0.0;
  tmp_d = _mm_set_pd(-LR_U.dexp6_lambda[1], -LR_U.dexp6_lambda[0]);
  _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, tmp_d));

  /* MATLAB Function: '<Root>/MATLAB Function2' incorporates:
   *  Inport: '<Root>/dexp6_lambda'
   */
  B[7] = tmp_f[0];
  B[2] = tmp_f[1];
  B[5] = b_s_tmp * LR_U.dexp6_lambda[0];
  B[8] = b_s_tmp * 0.0;
  Bdot[0] = ddOmega_1 * 0.0;
  tmp_b = _mm_set1_pd(ddOmega_1);
  _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(LR_U.dexp6_lambda[4],
    -LR_U.dexp6_lambda[5])));
  Bdot[3] = tmp_f[0];
  Bdot[6] = tmp_f[1];
  Bdot[1] = ddOmega_1 * LR_U.dexp6_lambda[5];
  Bdot[4] = ddOmega_1 * 0.0;
  _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(-LR_U.dexp6_lambda[4],
    -LR_U.dexp6_lambda[3])));
  Bdot[7] = tmp_f[0];
  Bdot[2] = tmp_f[1];
  Bdot[5] = ddOmega_1 * LR_U.dexp6_lambda[3];
  Bdot[8] = ddOmega_1 * 0.0;
  for (i = 0; i < 3; i++) {
    for (e_i = 0; e_i < 3; e_i++) {
      A6_tmp = 3 * e_i + i;
      Cxi[A6_tmp] = (((etaddot_[A6_tmp] + ddBA[A6_tmp]) * n_xi2 + B[A6_tmp]) +
                     Bdot[A6_tmp]) - ((ceil_xi[i + 3] * cx * ceil_xi[3 * e_i + 1]
        + cx * ceil_xi[i] * ceil_xi[3 * e_i]) + ceil_xi[i + 6] * cx * ceil_xi[3 *
        e_i + 2]);
    }
  }

  if (eta1 < 2.2204460492503131E-16) {
    Cxi[0] = 0.0;
    tmp_b = _mm_set1_pd(0.5);
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, tmp_c));
    Cxi[3] = tmp_f[0];
    Cxi[6] = tmp_f[1];
    Cxi[1] = 0.5 * LR_U.dexp6_lambda[2];
    Cxi[4] = 0.0;
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, tmp_d));
    Cxi[7] = tmp_f[0];
    Cxi[2] = tmp_f[1];
    Cxi[5] = 0.5 * LR_U.dexp6_lambda[0];
    Cxi[8] = 0.0;
    memset(&B[0], 0, 9U * sizeof(real_T));
    B[0] = 1.0;
    B[4] = 1.0;
    B[8] = 1.0;
    memset(&Bdot[0], 0, 9U * sizeof(real_T));
    Bdot[0] = 1.0;
    Bdot[4] = 1.0;
    Bdot[8] = 1.0;
  } else {
    for (i = 0; i < 9; i++) {
      b_I[i] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + i;
        B[f_i] = ((ceil_xi[i + 3] * n_xi2 * ceil_xi[3 * e_i + 1] + n_xi2 *
                   ceil_xi[i] * ceil_xi[3 * e_i]) + ceil_xi[i + 6] * n_xi2 *
                  ceil_xi[3 * e_i + 2]) + (ceil_xi[f_i] * b_s_tmp + (real_T)
          b_I[f_i]);
      }
    }

    for (i = 0; i < 9; i++) {
      b_I[i] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + i;
        Bdot[f_i] = ((ceil_xi[i + 3] * n_xi2 * ceil_xi[3 * e_i + 1] + n_xi2 *
                      ceil_xi[i] * ceil_xi[3 * e_i]) + ceil_xi[i + 6] * n_xi2 *
                     ceil_xi[3 * e_i + 2]) + (ceil_xi[f_i] * b_s_tmp + (real_T)
          b_I[f_i]);
      }
    }
  }

  /* Outport: '<Root>/dexp6' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function2'
   */
  for (i = 0; i < 3; i++) {
    LR_Y.dexp6[6 * i] = B[3 * i];

    /* MATLAB Function: '<Root>/MATLAB Function2' */
    e_i = (i + 3) * 6;
    LR_Y.dexp6[e_i] = Cxi[3 * i];
    LR_Y.dexp6[6 * i + 3] = 0.0;
    LR_Y.dexp6[e_i + 3] = Bdot[3 * i];

    /* MATLAB Function: '<Root>/MATLAB Function2' */
    A6_tmp = 3 * i + 1;
    LR_Y.dexp6[6 * i + 1] = B[A6_tmp];
    LR_Y.dexp6[e_i + 1] = Cxi[A6_tmp];
    LR_Y.dexp6[6 * i + 4] = 0.0;
    LR_Y.dexp6[e_i + 4] = Bdot[A6_tmp];

    /* MATLAB Function: '<Root>/MATLAB Function2' */
    A6_tmp = 3 * i + 2;
    LR_Y.dexp6[6 * i + 2] = B[A6_tmp];
    LR_Y.dexp6[e_i + 2] = Cxi[A6_tmp];
    LR_Y.dexp6[6 * i + 5] = 0.0;
    LR_Y.dexp6[e_i + 5] = Bdot[A6_tmp];
  }

  /* End of Outport: '<Root>/dexp6' */

  /* MATLAB Function: '<Root>/MATLAB Function3' incorporates:
   *  Inport: '<Root>/ddexp6_lambda'
   *  Inport: '<Root>/ddexp6_lambdadot'
   */
  ceil_xi[0] = 0.0;
  ceil_xi[3] = -LR_U.ddexp6_lambda[5];
  ceil_xi[6] = LR_U.ddexp6_lambda[4];
  ceil_xi[1] = LR_U.ddexp6_lambda[5];
  ceil_xi[4] = 0.0;
  ceil_xi[7] = -LR_U.ddexp6_lambda[3];
  ceil_xi[2] = -LR_U.ddexp6_lambda[4];
  ceil_xi[5] = LR_U.ddexp6_lambda[3];
  ceil_xi[8] = 0.0;
  B[0] = 0.0;
  B[3] = -LR_U.ddexp6_lambda[2];
  B[6] = LR_U.ddexp6_lambda[1];
  B[1] = LR_U.ddexp6_lambda[2];
  B[4] = 0.0;
  B[7] = -LR_U.ddexp6_lambda[0];
  B[2] = -LR_U.ddexp6_lambda[1];
  B[5] = LR_U.ddexp6_lambda[0];
  B[8] = 0.0;
  Cxi[0] = 0.0;
  Cxi[3] = -LR_U.ddexp6_lambdadot[5];
  Cxi[6] = LR_U.ddexp6_lambdadot[4];
  Cxi[1] = LR_U.ddexp6_lambdadot[5];
  Cxi[4] = 0.0;
  Cxi[7] = -LR_U.ddexp6_lambdadot[3];
  Cxi[2] = -LR_U.ddexp6_lambdadot[4];
  Cxi[5] = LR_U.ddexp6_lambdadot[3];
  Cxi[8] = 0.0;
  Bdot[0] = 0.0;
  Bdot[3] = -LR_U.ddexp6_lambdadot[2];
  Bdot[6] = LR_U.ddexp6_lambdadot[1];
  Bdot[1] = LR_U.ddexp6_lambdadot[2];
  Bdot[4] = 0.0;
  Bdot[7] = -LR_U.ddexp6_lambdadot[0];
  Bdot[2] = -LR_U.ddexp6_lambdadot[1];
  Bdot[5] = LR_U.ddexp6_lambdadot[0];
  Bdot[8] = 0.0;
  eta1 = LR_norm_e(&LR_U.ddexp6_lambda[3]);
  for (e_j = 0; e_j < 6; e_j++) {
    xn[e_j] = rt_powd_snf(eta1, (((real_T)e_j + 1.0) - 1.0) + 1.0);
  }

  d6 = sin(eta1);
  cx = cos(eta1);
  Gamma_1 = (eta1 - d6) / xn[2];
  Gamma_2 = ((2.0 * cx + xn[1]) - 2.0) / (2.0 * xn[3]);
  tmp_b = _mm_div_pd(_mm_set_pd(-(((4.0 * cx + eta1 * d6) + xn[1]) - 4.0),
    -((2.0 * eta1 - 3.0 * d6) + eta1 * cx)), _mm_loadu_pd(&xn[3]));
  _mm_storeu_pd(&tmp_f[0], tmp_b);

  /* MATLAB Function: '<Root>/MATLAB Function3' incorporates:
   *  Inport: '<Root>/ddexp6_lambda'
   *  Inport: '<Root>/ddexp6_lambdadot'
   */
  dGamma_1 = tmp_f[0];
  dGamma_2 = tmp_f[1];
  Gamma_3 = -tmp_f[0] / eta1;
  Gamma_4 = tmp_f[1] * eta1 + Gamma_2;
  for (i = 0; i < 3; i++) {
    f_i = 3 * i + 1;
    Bdot_3 = Bdot[f_i];
    Bdot_4 = Bdot[3 * i];
    e_j = 3 * i + 2;
    Bdot_5 = Bdot[e_j];
    for (e_i = 0; e_i <= 0; e_i += 2) {
      tmp_b = _mm_loadu_pd(&ceil_xi[e_i + 3]);
      tmp_c = _mm_loadu_pd(&ceil_xi[e_i]);
      tmp_d = _mm_loadu_pd(&ceil_xi[e_i + 6]);
      A6_tmp = 3 * i + e_i;
      _mm_storeu_pd(&ceil_xi_0[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(Bdot_3), tmp_b), _mm_mul_pd(_mm_set1_pd(Bdot_4), tmp_c)),
        _mm_mul_pd(_mm_set1_pd(Bdot_5), tmp_d)));
      tmp_b = _mm_loadu_pd(&Cxi[e_i + 3]);
      tmp_c = _mm_set1_pd(B[f_i]);
      tmp_d = _mm_loadu_pd(&Cxi[e_i]);
      tmp_e = _mm_set1_pd(B[3 * i]);
      tmp_3 = _mm_loadu_pd(&Cxi[e_i + 6]);
      tmp_i = _mm_set1_pd(B[e_j]);
      _mm_storeu_pd(&Cxi_0[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_c,
        tmp_b), _mm_mul_pd(tmp_e, tmp_d)), _mm_mul_pd(tmp_i, tmp_3)));
      tmp_b = _mm_loadu_pd(&ceil_xi[e_i + 3]);
      tmp_d = _mm_set1_pd(ceil_xi[f_i]);
      tmp_3 = _mm_loadu_pd(&ceil_xi[e_i]);
      tmp_g = _mm_set1_pd(ceil_xi[3 * i]);
      tmp_4 = _mm_loadu_pd(&ceil_xi[e_i + 6]);
      tmp_h = _mm_set1_pd(ceil_xi[e_j]);
      _mm_storeu_pd(&AA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_d, tmp_b),
        _mm_mul_pd(tmp_g, tmp_3)), _mm_mul_pd(tmp_h, tmp_4)));
      tmp_b = _mm_loadu_pd(&B[e_i + 3]);
      tmp_3 = _mm_loadu_pd(&B[e_i]);
      tmp_4 = _mm_loadu_pd(&B[e_i + 6]);
      _mm_storeu_pd(&BA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_d, tmp_b),
        _mm_mul_pd(tmp_g, tmp_3)), _mm_mul_pd(tmp_h, tmp_4)));
      tmp_b = _mm_loadu_pd(&ceil_xi[e_i + 3]);
      tmp_d = _mm_loadu_pd(&ceil_xi[e_i]);
      tmp_3 = _mm_loadu_pd(&ceil_xi[e_i + 6]);
      _mm_storeu_pd(&AB[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_c, tmp_b),
        _mm_mul_pd(tmp_e, tmp_d)), _mm_mul_pd(tmp_i, tmp_3)));
    }

    for (e_i = 2; e_i < 3; e_i++) {
      dddexp3__tmp = ceil_xi[e_i + 3];
      dddexp3__tmp_0 = ceil_xi[e_i + 6];
      ddOmega_1_tmp = B[f_i];
      dddexp3__tmp_1 = B[3 * i];
      Cxi_3 = B[e_j];
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (ddOmega_1_tmp, Bdot_3), _mm_set_pd(Cxi[e_i + 3], dddexp3__tmp)),
        _mm_mul_pd(_mm_set_pd(dddexp3__tmp_1, Bdot_4), _mm_set_pd(Cxi[e_i],
        ceil_xi[e_i]))), _mm_mul_pd(_mm_set_pd(Cxi_3, Bdot_5), _mm_set_pd
        (Cxi[e_i + 6], dddexp3__tmp_0))));
      A6_tmp = 3 * i + e_i;
      ceil_xi_0[A6_tmp] = tmp_f[0];
      Cxi_0[A6_tmp] = tmp_f[1];
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (ceil_xi[f_i]), _mm_set_pd(B[e_i + 3], dddexp3__tmp)), _mm_mul_pd
        (_mm_set1_pd(ceil_xi[3 * i]), _mm_set_pd(B[e_i], ceil_xi[e_i]))),
        _mm_mul_pd(_mm_set1_pd(ceil_xi[e_j]), _mm_set_pd(B[e_i + 6],
        dddexp3__tmp_0))));
      AA[A6_tmp] = tmp_f[0];
      BA[A6_tmp] = tmp_f[1];
      AB[A6_tmp] = (ddOmega_1_tmp * dddexp3__tmp + dddexp3__tmp_1 * ceil_xi[e_i])
        + Cxi_3 * dddexp3__tmp_0;
    }
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_b = _mm_loadu_pd(&Cxi_0[i]);
    tmp_c = _mm_loadu_pd(&ceil_xi_0[i]);
    _mm_storeu_pd(&dAB[i], _mm_add_pd(tmp_b, tmp_c));
  }

  for (i = 8; i < 9; i++) {
    dAB[i] = Cxi_0[i] + ceil_xi_0[i];
  }

  for (i = 0; i < 3; i++) {
    A6_tmp = 3 * i + 1;
    tmp = Cxi[A6_tmp];
    Cxi_2 = Cxi[3 * i];
    f_i = 3 * i + 2;
    Cxi_3 = Cxi[f_i];
    beta_2 = ceil_xi[A6_tmp];
    ceil_xi_2 = ceil_xi[3 * i];
    n_xi6 = ceil_xi[f_i];
    for (e_i = 0; e_i <= 0; e_i += 2) {
      tmp_b = _mm_loadu_pd(&B[e_i + 3]);
      tmp_c = _mm_loadu_pd(&B[e_i]);
      tmp_d = _mm_loadu_pd(&B[e_i + 6]);
      A6_tmp = 3 * i + e_i;
      _mm_storeu_pd(&B_0[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (tmp), tmp_b), _mm_mul_pd(_mm_set1_pd(Cxi_2), tmp_c)), _mm_mul_pd
        (_mm_set1_pd(Cxi_3), tmp_d)));
      tmp_b = _mm_loadu_pd(&Bdot[e_i + 3]);
      tmp_c = _mm_loadu_pd(&Bdot[e_i]);
      tmp_d = _mm_loadu_pd(&Bdot[e_i + 6]);
      _mm_storeu_pd(&Bdot_0[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(beta_2), tmp_b), _mm_mul_pd(_mm_set1_pd(ceil_xi_2), tmp_c)),
        _mm_mul_pd(_mm_set1_pd(n_xi6), tmp_d)));
    }

    for (e_i = 2; e_i < 3; e_i++) {
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (beta_2, tmp), _mm_set_pd(Bdot[e_i + 3], B[e_i + 3])), _mm_mul_pd
        (_mm_set_pd(ceil_xi_2, Cxi_2), _mm_set_pd(Bdot[e_i], B[e_i]))),
        _mm_mul_pd(_mm_set_pd(n_xi6, Cxi_3), _mm_set_pd(Bdot[e_i + 6], B[e_i + 6]))));
      f_i = 3 * i + e_i;
      B_0[f_i] = tmp_f[0];
      Bdot_0[f_i] = tmp_f[1];
    }
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_b = _mm_loadu_pd(&Bdot_0[i]);
    tmp_c = _mm_loadu_pd(&B_0[i]);
    _mm_storeu_pd(&dBA[i], _mm_add_pd(tmp_b, tmp_c));
  }

  for (i = 8; i < 9; i++) {
    dBA[i] = Bdot_0[i] + B_0[i];
  }

  Bdot_3 = (LR_U.ddexp6_lambda[3] * LR_U.ddexp6_lambdadot[3] +
            LR_U.ddexp6_lambda[4] * LR_U.ddexp6_lambdadot[4]) +
    LR_U.ddexp6_lambda[5] * LR_U.ddexp6_lambdadot[5];
  normxidot = Bdot_3 / eta1;
  if (eta1 < 1.0E-12) {
    B[0] = 0.0;
    tmp_b = _mm_set1_pd(0.5);
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(LR_U.ddexp6_lambdadot
      [4], -LR_U.ddexp6_lambdadot[5])));
    B[3] = tmp_f[0];
    B[6] = tmp_f[1];
    B[1] = 0.5 * LR_U.ddexp6_lambdadot[5];
    B[4] = 0.0;
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd
      (-LR_U.ddexp6_lambdadot[4], -LR_U.ddexp6_lambdadot[3])));
    B[7] = tmp_f[0];
    B[2] = tmp_f[1];
    B[5] = 0.5 * LR_U.ddexp6_lambdadot[3];
    B[8] = 0.0;
  } else {
    b_s = eta1 / 2.0;
    ddOmega_1 = sin(b_s) / b_s;
    alpha = ddOmega_1 * cos(b_s);
    beta = ddOmega_1 * ddOmega_1;
    beta_2 = beta / 2.0;
    theta_sqr = eta1 * eta1;
    tmp = (1.0 - alpha) / theta_sqr;
    ddOmega_1 = (alpha - beta) / theta_sqr * Bdot_3;
    xixidot = (beta_2 - 3.0 * tmp) / theta_sqr * Bdot_3;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + 1;
        e_j = 3 * e_i + 2;
        A6_tmp = 3 * e_i + i;
        Cxi_3 = ceil_xi[i + 3];
        Bdot_4 = ceil_xi[f_i];
        b_s_tmp = ceil_xi[3 * e_i];
        Bdot_5 = ceil_xi[i + 6];
        B_tmp = ceil_xi[e_j];
        B[A6_tmp] = (((((Cxi[3 * e_i] * ceil_xi[i] + Cxi[f_i] * Cxi_3) + Cxi[e_j]
                        * Bdot_5) + ((Cxi[i + 3] * Bdot_4 + b_s_tmp * Cxi[i]) +
          Cxi[i + 6] * B_tmp)) * tmp + Cxi[A6_tmp] * beta_2) + ceil_xi[A6_tmp] *
                     ddOmega_1) + ((Cxi_3 * xixidot * Bdot_4 + xixidot *
          ceil_xi[i] * b_s_tmp) + Bdot_5 * xixidot * B_tmp);
      }
    }
  }

  if (eta1 < 2.2204460492503131E-16) {
    /* Outport: '<Root>/ddexp6' */
    for (i = 0; i < 3; i++) {
      beta = dBA[i + 3];
      dBA_1 = dBA[i];
      alpha = dBA[i + 6];
      ddGamma_1 = BA[i + 3];
      B_tmp = BA[i];
      BA_4 = BA[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + 1;
        f_i = 3 * e_i + 2;
        e_j = 3 * e_i + i;
        Cxi_0[e_j] = (((Cxi[i + 3] * AB[A6_tmp] + AB[3 * e_i] * Cxi[i]) + Cxi[i
                       + 6] * AB[f_i]) + ((ceil_xi[i + 3] * dAB[A6_tmp] + dAB[3 *
          e_i] * ceil_xi[i]) + ceil_xi[i + 6] * dAB[f_i])) + ((ceil_xi[3 * e_i] *
          dBA_1 + ceil_xi[A6_tmp] * beta) + ceil_xi[f_i] * alpha);
        BA_0[e_j] = (Cxi[3 * e_i] * B_tmp + Cxi[A6_tmp] * ddGamma_1) + Cxi[f_i] *
          BA_4;
        LR_Y.ddexp6[e_i + 6 * i] = B[3 * i + e_i];
      }
    }

    for (i = 0; i < 3; i++) {
      e_i = (i + 3) * 6;
      LR_Y.ddexp6[e_i] = ((dAB[3 * i] + dBA[3 * i]) * 0.16666666666666666 +
                          Bdot[3 * i] * 0.5) + (Cxi_0[3 * i] + BA_0[3 * i]) *
        0.041666666666666664;
      LR_Y.ddexp6[6 * i + 3] = 0.0;
      LR_Y.ddexp6[e_i + 3] = B[3 * i];
      A6_tmp = 3 * i + 1;
      LR_Y.ddexp6[e_i + 1] = ((dAB[A6_tmp] + dBA[A6_tmp]) * 0.16666666666666666
        + Bdot[A6_tmp] * 0.5) + (Cxi_0[A6_tmp] + BA_0[A6_tmp]) *
        0.041666666666666664;
      LR_Y.ddexp6[6 * i + 4] = 0.0;
      LR_Y.ddexp6[e_i + 4] = B[A6_tmp];
      A6_tmp = 3 * i + 2;
      LR_Y.ddexp6[e_i + 2] = ((dAB[A6_tmp] + dBA[A6_tmp]) * 0.16666666666666666
        + Bdot[A6_tmp] * 0.5) + (Cxi_0[A6_tmp] + BA_0[A6_tmp]) *
        0.041666666666666664;
      LR_Y.ddexp6[6 * i + 5] = 0.0;
      LR_Y.ddexp6[e_i + 5] = B[A6_tmp];
    }
  } else {
    b_s = dGamma_1 * normxidot;
    n_xi6 = dGamma_2 * normxidot;
    xiTxidot = (-((((6.0 * eta1 - 12.0 * d6) + xn[1] * d6) + 6.0 * eta1 * cx) /
                  xn[4]) / eta1 + dGamma_1 / xn[1]) * normxidot;
    n_xi2 = (((((20.0 * cx - xn[1] * cx) + 8.0 * eta1 * d6) + 3.0 * xn[1]) -
              20.0) / xn[5] * eta1 + 2.0 * dGamma_2) * normxidot;
    for (i = 0; i < 3; i++) {
      cx = AA[i + 3];
      xn2sx = AA[i];
      b_dGamma_2 = AA[i + 6];
      beta = dBA[i + 3];
      dBA_1 = dBA[i];
      alpha = dBA[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        n_xi5 = AB[3 * e_i];
        beta_2 = ceil_xi[i] * n_xi5;
        ddGamma_1 = BA[i];
        dddexp3__tmp = ceil_xi[3 * e_i];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(Cxi[i], dddexp3__tmp),
          _mm_set_pd(n_xi5, ddGamma_1)));
        ddOmega_1 = Cxi[3 * e_i];
        B_tmp = ddOmega_1 * ddGamma_1;
        f_i = 3 * e_i + 1;
        n_xi5 = AB[f_i];
        ceil_xi_tmp = ceil_xi[i + 3];
        beta_2 += ceil_xi_tmp * n_xi5;
        ddGamma_1 = BA[i + 3];
        dddexp3__tmp_0 = ceil_xi[f_i];
        ddOmega_1_tmp = Cxi[i + 3];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(ddOmega_1_tmp,
          dddexp3__tmp_0), _mm_set_pd(n_xi5, ddGamma_1)), _mm_set_pd(tmp_f[1],
          tmp_f[0])));
        BA_4 = tmp_f[0];
        tmp = tmp_f[1];
        eta1 = Cxi[f_i];
        e_j = 3 * e_i + 2;
        n_xi5 = AB[e_j];
        dddexp3__tmp_1 = ceil_xi[e_j];
        tmp_b = _mm_set_pd(dddexp3__tmp_1, Cxi[e_j]);
        Cxi_3 = ceil_xi[i + 6];
        Bdot_4 = Cxi[i + 6];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
          (dddexp3__tmp_0, eta1), _mm_set_pd(ddOmega_1_tmp, ceil_xi_tmp)),
          _mm_mul_pd(_mm_set_pd(dddexp3__tmp, ddOmega_1), _mm_set_pd(Cxi[i],
          ceil_xi[i]))), _mm_mul_pd(tmp_b, _mm_set_pd(Bdot_4, Cxi_3))));
        A6_tmp = 3 * e_i + i;
        ceil_xi_1[A6_tmp] = tmp_f[0];
        Cxi_1[A6_tmp] = tmp_f[1];
        AA_0[A6_tmp] = (BA[3 * e_i] * xn2sx + BA[f_i] * cx) + BA[e_j] *
          b_dGamma_2;
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(tmp_b, _mm_set1_pd(BA[i +
          6])), _mm_set_pd(BA_4, eta1 * ddGamma_1 + B_tmp)));
        BA_1[A6_tmp] = tmp_f[0];
        BA_0[A6_tmp] = tmp_f[1];
        ceil_xi_0[A6_tmp] = Cxi_3 * n_xi5 + beta_2;
        Cxi_0[A6_tmp] = (((dAB[3 * e_i] * ceil_xi[i] + dAB[f_i] * ceil_xi_tmp) +
                          dAB[e_j] * Cxi_3) + (Bdot_4 * n_xi5 + tmp)) +
          ((dddexp3__tmp_0 * beta + dddexp3__tmp * dBA_1) + dddexp3__tmp_1 *
           alpha);
      }
    }

    for (i = 0; i <= 6; i += 2) {
      tmp_b = _mm_loadu_pd(&Cxi_1[i]);
      tmp_c = _mm_loadu_pd(&ceil_xi_1[i]);
      _mm_storeu_pd(&Bdot_0[i], _mm_add_pd(tmp_b, tmp_c));
    }

    for (i = 8; i < 9; i++) {
      Bdot_0[i] = Cxi_1[i] + ceil_xi_1[i];
    }

    /* Outport: '<Root>/ddexp6' */
    for (i = 0; i < 3; i++) {
      n_xi5 = AB[i];
      dBA_1 = AB[i + 3];
      AB_4 = AB[i + 6];
      dAB_1 = dAB[i];
      B_tmp = dAB[i + 3];
      BA_4 = dAB[i + 6];
      cx = AA[i + 3];
      xn2sx = AA[i];
      b_dGamma_2 = AA[i + 6];
      tmp = Bdot_0[i + 3];
      Cxi_2 = Bdot_0[i];
      Cxi_3 = Bdot_0[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        beta_2 = ceil_xi[3 * e_i];
        AB_3 = n_xi5 * beta_2;
        eta1 = dAB_1 * beta_2;
        A6_tmp = 3 * e_i + 1;
        beta_2 = ceil_xi[A6_tmp];
        AB_3 += dBA_1 * beta_2;
        eta1 += B_tmp * beta_2;
        e_j = 3 * e_i + 2;
        beta_2 = ceil_xi[e_j];
        f_i = 3 * e_i + i;
        AB_1[f_i] = (Cxi[3 * e_i] * n_xi5 + Cxi[A6_tmp] * dBA_1) + Cxi[e_j] *
          AB_4;
        dAB_0[f_i] = BA_4 * beta_2 + eta1;
        AB_0[f_i] = AB_4 * beta_2 + AB_3;
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
          (BA[A6_tmp], dBA[A6_tmp]), _mm_set_pd(tmp, cx)), _mm_mul_pd(_mm_set_pd
          (BA[3 * e_i], dBA[3 * e_i]), _mm_set_pd(Cxi_2, xn2sx))), _mm_mul_pd
          (_mm_set_pd(BA[e_j], dBA[e_j]), _mm_set_pd(Cxi_3, b_dGamma_2))));
        ceil_xi_1[f_i] = tmp_f[0];
        Cxi_1[f_i] = tmp_f[1];
        LR_Y.ddexp6[e_i + 6 * i] = B[3 * i + e_i];
      }
    }

    for (i = 0; i < 3; i++) {
      e_i = (i + 3) * 6;
      LR_Y.ddexp6[e_i] = ((((((((AB[3 * i] + BA[3 * i]) * b_s + Bdot[3 * i] *
        0.5) + (dAB[3 * i] + dBA[3 * i]) * Gamma_1) + (ceil_xi_0[3 * i] + BA_0[3
        * i]) * n_xi6) + (Cxi_0[3 * i] + BA_1[3 * i]) * Gamma_2) + AA_0[3 * i] *
                            xiTxidot) + (Cxi_1[3 * i] + ceil_xi_1[3 * i]) *
                           Gamma_3) + AB_0[3 * i] * n_xi2) + (dAB_0[3 * i] +
        AB_1[3 * i]) * Gamma_4;
      LR_Y.ddexp6[6 * i + 3] = 0.0;
      LR_Y.ddexp6[e_i + 3] = B[3 * i];
      A6_tmp = 3 * i + 1;
      LR_Y.ddexp6[e_i + 1] = ((((((((AB[A6_tmp] + BA[A6_tmp]) * b_s +
        Bdot[A6_tmp] * 0.5) + (dAB[A6_tmp] + dBA[A6_tmp]) * Gamma_1) +
        (ceil_xi_0[A6_tmp] + BA_0[A6_tmp]) * n_xi6) + (Cxi_0[A6_tmp] +
        BA_1[A6_tmp]) * Gamma_2) + AA_0[A6_tmp] * xiTxidot) + (Cxi_1[A6_tmp] +
        ceil_xi_1[A6_tmp]) * Gamma_3) + AB_0[A6_tmp] * n_xi2) + (dAB_0[A6_tmp] +
        AB_1[A6_tmp]) * Gamma_4;
      LR_Y.ddexp6[6 * i + 4] = 0.0;
      LR_Y.ddexp6[e_i + 4] = B[A6_tmp];
      A6_tmp = 3 * i + 2;
      LR_Y.ddexp6[e_i + 2] = ((((((((AB[A6_tmp] + BA[A6_tmp]) * b_s +
        Bdot[A6_tmp] * 0.5) + (dAB[A6_tmp] + dBA[A6_tmp]) * Gamma_1) +
        (ceil_xi_0[A6_tmp] + BA_0[A6_tmp]) * n_xi6) + (Cxi_0[A6_tmp] +
        BA_1[A6_tmp]) * Gamma_2) + AA_0[A6_tmp] * xiTxidot) + (Cxi_1[A6_tmp] +
        ceil_xi_1[A6_tmp]) * Gamma_3) + AB_0[A6_tmp] * n_xi2) + (dAB_0[A6_tmp] +
        AB_1[A6_tmp]) * Gamma_4;
      LR_Y.ddexp6[6 * i + 5] = 0.0;
      LR_Y.ddexp6[e_i + 5] = B[A6_tmp];
    }
  }

  /* MATLAB Function: '<Root>/MATLAB Function4' incorporates:
   *  Inport: '<Root>/dddexp6_lambda'
   *  Inport: '<Root>/dddexp6_lambdaddot'
   *  Inport: '<Root>/dddexp6_lambdadot'
   */
  ceil_xi[0] = 0.0;
  ceil_xi[3] = -LR_U.dddexp6_lambda[5];
  ceil_xi[6] = LR_U.dddexp6_lambda[4];
  ceil_xi[1] = LR_U.dddexp6_lambda[5];
  ceil_xi[4] = 0.0;
  ceil_xi[7] = -LR_U.dddexp6_lambda[3];
  ceil_xi[2] = -LR_U.dddexp6_lambda[4];
  ceil_xi[5] = LR_U.dddexp6_lambda[3];
  ceil_xi[8] = 0.0;
  eta_[0] = 0.0;
  eta_[3] = -LR_U.dddexp6_lambda[2];
  eta_[6] = LR_U.dddexp6_lambda[1];
  eta_[1] = LR_U.dddexp6_lambda[2];
  eta_[4] = 0.0;
  eta_[7] = -LR_U.dddexp6_lambda[0];
  eta_[2] = -LR_U.dddexp6_lambda[1];
  eta_[5] = LR_U.dddexp6_lambda[0];
  eta_[8] = 0.0;
  Cxi[0] = 0.0;
  Cxi[3] = -LR_U.dddexp6_lambdadot[5];
  Cxi[6] = LR_U.dddexp6_lambdadot[4];
  Cxi[1] = LR_U.dddexp6_lambdadot[5];
  Cxi[4] = 0.0;
  Cxi[7] = -LR_U.dddexp6_lambdadot[3];
  Cxi[2] = -LR_U.dddexp6_lambdadot[4];
  Cxi[5] = LR_U.dddexp6_lambdadot[3];
  Cxi[8] = 0.0;
  etadot_[0] = 0.0;
  etadot_[3] = -LR_U.dddexp6_lambdadot[2];
  etadot_[6] = LR_U.dddexp6_lambdadot[1];
  etadot_[1] = LR_U.dddexp6_lambdadot[2];
  etadot_[4] = 0.0;
  etadot_[7] = -LR_U.dddexp6_lambdadot[0];
  etadot_[2] = -LR_U.dddexp6_lambdadot[1];
  etadot_[5] = LR_U.dddexp6_lambdadot[0];
  etadot_[8] = 0.0;
  Bdot[0] = 0.0;
  Bdot[3] = -LR_U.dddexp6_lambdaddot[5];
  Bdot[6] = LR_U.dddexp6_lambdaddot[4];
  Bdot[1] = LR_U.dddexp6_lambdaddot[5];
  Bdot[4] = 0.0;
  Bdot[7] = -LR_U.dddexp6_lambdaddot[3];
  Bdot[2] = -LR_U.dddexp6_lambdaddot[4];
  Bdot[5] = LR_U.dddexp6_lambdaddot[3];
  Bdot[8] = 0.0;
  etaddot_[0] = 0.0;
  etaddot_[3] = -LR_U.dddexp6_lambdaddot[2];
  etaddot_[6] = LR_U.dddexp6_lambdaddot[1];
  etaddot_[1] = LR_U.dddexp6_lambdaddot[2];
  etaddot_[4] = 0.0;
  etaddot_[7] = -LR_U.dddexp6_lambdaddot[0];
  etaddot_[2] = -LR_U.dddexp6_lambdaddot[1];
  etaddot_[5] = LR_U.dddexp6_lambdaddot[0];
  etaddot_[8] = 0.0;
  eta1 = LR_norm_e(&LR_U.dddexp6_lambda[3]);
  for (e_j = 0; e_j < 7; e_j++) {
    xn_0[e_j] = rt_powd_snf(eta1, (((real_T)e_j + 1.0) - 1.0) + 1.0);
  }

  b_dGamma_2 = sin(eta1);
  cx = cos(eta1);
  xcx_tmp = eta1 * cx;
  Cxi_2 = eta1 * b_dGamma_2;
  beta_2 = eta1 * xcx_tmp;
  xn2sx = eta1 * Cxi_2;
  Gamma_1 = (eta1 - b_dGamma_2) / xn_0[2];
  dGamma_1 = -((2.0 * eta1 - 3.0 * b_dGamma_2) + xcx_tmp) / xn_0[3];
  ddGamma_1 = (((6.0 * eta1 - 12.0 * b_dGamma_2) + xn2sx) + 6.0 * xcx_tmp) /
    xn_0[4];
  Gamma_2 = ((2.0 * cx + xn_0[1]) - 2.0) / (2.0 * xn_0[3]);
  dGamma_2 = -(((4.0 * cx + Cxi_2) + xn_0[1]) - 4.0) / xn_0[4];
  tmp = ((((20.0 * cx - beta_2) + 8.0 * Cxi_2) + 3.0 * xn_0[1]) - 20.0) / xn_0[5];
  Gamma_3 = -dGamma_1 / eta1;
  xixidot = -ddGamma_1 / eta1 + dGamma_1 / xn_0[1];
  Gamma_4 = dGamma_2 * eta1 + Gamma_2;
  beta = tmp * eta1 + 2.0 * dGamma_2;
  for (i = 0; i < 3; i++) {
    f_i = 3 * i + 1;
    ddOmega_1 = etadot_[f_i];
    alpha = etadot_[3 * i];
    e_j = 3 * i + 2;
    dAB_1 = etadot_[e_j];
    for (e_i = 0; e_i <= 0; e_i += 2) {
      tmp_b = _mm_loadu_pd(&ceil_xi[e_i + 3]);
      tmp_c = _mm_loadu_pd(&ceil_xi[e_i]);
      tmp_d = _mm_loadu_pd(&ceil_xi[e_i + 6]);
      A6_tmp = 3 * i + e_i;
      _mm_storeu_pd(&ceil_xi_0[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(ddOmega_1), tmp_b), _mm_mul_pd(_mm_set1_pd(alpha), tmp_c)),
        _mm_mul_pd(_mm_set1_pd(dAB_1), tmp_d)));
      tmp_b = _mm_loadu_pd(&Cxi[e_i + 3]);
      tmp_c = _mm_set1_pd(eta_[f_i]);
      tmp_d = _mm_loadu_pd(&Cxi[e_i]);
      tmp_e = _mm_set1_pd(eta_[3 * i]);
      tmp_3 = _mm_loadu_pd(&Cxi[e_i + 6]);
      tmp_i = _mm_set1_pd(eta_[e_j]);
      _mm_storeu_pd(&Cxi_0[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_c,
        tmp_b), _mm_mul_pd(tmp_e, tmp_d)), _mm_mul_pd(tmp_i, tmp_3)));
      tmp_b = _mm_loadu_pd(&ceil_xi[e_i + 3]);
      tmp_d = _mm_set1_pd(ceil_xi[f_i]);
      tmp_3 = _mm_loadu_pd(&ceil_xi[e_i]);
      tmp_g = _mm_set1_pd(ceil_xi[3 * i]);
      tmp_4 = _mm_loadu_pd(&ceil_xi[e_i + 6]);
      tmp_h = _mm_set1_pd(ceil_xi[e_j]);
      _mm_storeu_pd(&AA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_d, tmp_b),
        _mm_mul_pd(tmp_g, tmp_3)), _mm_mul_pd(tmp_h, tmp_4)));
      tmp_b = _mm_loadu_pd(&eta_[e_i + 3]);
      tmp_3 = _mm_loadu_pd(&eta_[e_i]);
      tmp_4 = _mm_loadu_pd(&eta_[e_i + 6]);
      _mm_storeu_pd(&BA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_d, tmp_b),
        _mm_mul_pd(tmp_g, tmp_3)), _mm_mul_pd(tmp_h, tmp_4)));
      tmp_b = _mm_loadu_pd(&ceil_xi[e_i + 3]);
      tmp_d = _mm_loadu_pd(&ceil_xi[e_i]);
      tmp_3 = _mm_loadu_pd(&ceil_xi[e_i + 6]);
      _mm_storeu_pd(&AB[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_c, tmp_b),
        _mm_mul_pd(tmp_e, tmp_d)), _mm_mul_pd(tmp_i, tmp_3)));
    }

    for (e_i = 2; e_i < 3; e_i++) {
      dddexp3__tmp = ceil_xi[e_i + 3];
      dddexp3__tmp_0 = ceil_xi[e_i + 6];
      ddOmega_1_tmp = eta_[f_i];
      dddexp3__tmp_1 = eta_[3 * i];
      Cxi_3 = eta_[e_j];
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (ddOmega_1_tmp, ddOmega_1), _mm_set_pd(Cxi[e_i + 3], dddexp3__tmp)),
        _mm_mul_pd(_mm_set_pd(dddexp3__tmp_1, alpha), _mm_set_pd(Cxi[e_i],
        ceil_xi[e_i]))), _mm_mul_pd(_mm_set_pd(Cxi_3, dAB_1), _mm_set_pd(Cxi[e_i
        + 6], dddexp3__tmp_0))));
      A6_tmp = 3 * i + e_i;
      ceil_xi_0[A6_tmp] = tmp_f[0];
      Cxi_0[A6_tmp] = tmp_f[1];
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (ceil_xi[f_i]), _mm_set_pd(eta_[e_i + 3], dddexp3__tmp)), _mm_mul_pd
        (_mm_set1_pd(ceil_xi[3 * i]), _mm_set_pd(eta_[e_i], ceil_xi[e_i]))),
        _mm_mul_pd(_mm_set1_pd(ceil_xi[e_j]), _mm_set_pd(eta_[e_i + 6],
        dddexp3__tmp_0))));
      AA[A6_tmp] = tmp_f[0];
      BA[A6_tmp] = tmp_f[1];
      AB[A6_tmp] = (ddOmega_1_tmp * dddexp3__tmp + dddexp3__tmp_1 * ceil_xi[e_i])
        + Cxi_3 * dddexp3__tmp_0;
    }
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_b = _mm_loadu_pd(&Cxi_0[i]);
    tmp_c = _mm_loadu_pd(&ceil_xi_0[i]);
    _mm_storeu_pd(&dAB[i], _mm_add_pd(tmp_b, tmp_c));
  }

  for (i = 8; i < 9; i++) {
    dAB[i] = Cxi_0[i] + ceil_xi_0[i];
  }

  for (i = 0; i < 3; i++) {
    Bdot_3 = Bdot[i + 3];
    Bdot_4 = Bdot[i];
    Bdot_5 = Bdot[i + 6];
    for (e_i = 0; e_i < 3; e_i++) {
      f_i = 3 * e_i + 1;
      e_j = 3 * e_i + 2;
      B[i + 3 * e_i] = (((Cxi[i + 3] * 2.0 * etadot_[f_i] + 2.0 * Cxi[i] *
                          etadot_[3 * e_i]) + Cxi[i + 6] * 2.0 * etadot_[e_j]) +
                        ((eta_[3 * e_i] * Bdot_4 + eta_[f_i] * Bdot_3) +
                         eta_[e_j] * Bdot_5)) + ((ceil_xi[i + 3] * etaddot_[f_i]
        + etaddot_[3 * e_i] * ceil_xi[i]) + ceil_xi[i + 6] * etaddot_[e_j]);
      A6_tmp = 3 * i + 1;
      dAB_tmp = 3 * i + 2;
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (ceil_xi[A6_tmp], Cxi[A6_tmp]), _mm_set_pd(etadot_[e_i + 3], eta_[e_i +
        3])), _mm_mul_pd(_mm_set_pd(ceil_xi[3 * i], Cxi[3 * i]), _mm_set_pd
                         (etadot_[e_i], eta_[e_i]))), _mm_mul_pd(_mm_set_pd
        (ceil_xi[dAB_tmp], Cxi[dAB_tmp]), _mm_set_pd(etadot_[e_i + 6], eta_[e_i
        + 6]))));
      A6_tmp = 3 * i + e_i;
      eta__0[A6_tmp] = tmp_f[0];
      dAB_0[A6_tmp] = tmp_f[1];
    }
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_b = _mm_loadu_pd(&dAB_0[i]);
    tmp_c = _mm_loadu_pd(&eta__0[i]);
    _mm_storeu_pd(&dBA[i], _mm_add_pd(tmp_b, tmp_c));
  }

  for (i = 8; i < 9; i++) {
    dBA[i] = dAB_0[i] + eta__0[i];
  }

  for (i = 0; i < 3; i++) {
    n_xi6 = etaddot_[i + 3];
    d6 = etaddot_[i];
    b_s = etaddot_[i + 6];
    ddOmega_1 = etadot_[i + 3];
    alpha = etadot_[i];
    dAB_1 = etadot_[i + 6];
    normxidot = eta_[i + 3];
    b_s_tmp = eta_[i];
    theta_sqr = eta_[i + 6];
    for (e_i = 0; e_i < 3; e_i++) {
      A6_tmp = 3 * e_i + 1;
      f_i = 3 * e_i + 2;
      ddBA[i + 3 * e_i] = (((ceil_xi[3 * e_i] * d6 + ceil_xi[A6_tmp] * n_xi6) +
                            ceil_xi[f_i] * b_s) + ((ddOmega_1 * 2.0 * Cxi[A6_tmp]
        + 2.0 * alpha * Cxi[3 * e_i]) + dAB_1 * 2.0 * Cxi[f_i])) + ((Bdot[3 *
        e_i] * b_s_tmp + Bdot[A6_tmp] * normxidot) + Bdot[f_i] * theta_sqr);
      A6_tmp = 3 * i + 1;
      dAB_tmp = 3 * i + 2;
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (ceil_xi[A6_tmp], Cxi[A6_tmp]), _mm_set_pd(Cxi[e_i + 3], ceil_xi[e_i + 3])),
        _mm_mul_pd(_mm_set_pd(ceil_xi[3 * i], Cxi[3 * i]), _mm_set_pd(Cxi[e_i],
        ceil_xi[e_i]))), _mm_mul_pd(_mm_set_pd(ceil_xi[dAB_tmp], Cxi[dAB_tmp]),
        _mm_set_pd(Cxi[e_i + 6], ceil_xi[e_i + 6]))));
      A6_tmp = 3 * i + e_i;
      ceil_xi_0[A6_tmp] = tmp_f[0];
      Cxi_0[A6_tmp] = tmp_f[1];
    }
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_b = _mm_loadu_pd(&Cxi_0[i]);
    tmp_c = _mm_loadu_pd(&ceil_xi_0[i]);
    _mm_storeu_pd(&eta_[i], _mm_add_pd(tmp_b, tmp_c));
  }

  for (i = 8; i < 9; i++) {
    eta_[i] = Cxi_0[i] + ceil_xi_0[i];
  }

  Cxi_3 = 0.0;
  dddexp3__tmp = 0.0;
  dddexp3__tmp_0 = 0.0;
  ddOmega_1_tmp = 0.0;
  n_xi2 = eta1 * eta1;
  xiTxidot = rt_powd_snf(eta1, 4.0);
  n_xi5 = rt_powd_snf(eta1, 5.0);
  n_xi6 = rt_powd_snf(eta1, 6.0);
  dddexp3__tmp_1 = 0.0;
  for (i = 0; i < 3; i++) {
    dAB_1 = dAB[i + 3];
    B_tmp = dAB[i];
    BA_4 = dAB[i + 6];
    for (e_i = 0; e_i < 3; e_i++) {
      etadot_[i + 3 * e_i] = (Cxi[3 * e_i + 1] * dAB_1 + Cxi[3 * e_i] * B_tmp) +
        Cxi[3 * e_i + 2] * BA_4;
    }

    ddOmega_1 = LR_U.dddexp6_lambda[i + 3];
    d6 = LR_U.dddexp6_lambdadot[i + 3];
    Cxi_3 += ddOmega_1 * d6;
    dddexp3__tmp = Cxi_3;
    dddexp3__tmp_0 += d6 * d6;
    ddOmega_1_tmp += LR_U.dddexp6_lambdaddot[i + 3] * ddOmega_1;
    dddexp3__tmp_1 = Cxi_3;
  }

  normxidot = Cxi_3 / eta1;
  Cxi_3 = dddexp3__tmp_0 + ddOmega_1_tmp;
  alpha = dddexp3__tmp * dddexp3__tmp;
  theta_sqr = 1.0 / eta1 * Cxi_3 + alpha * -rt_powd_snf(eta1, -3.0);
  b_s_tmp = b_dGamma_2 / rt_powd_snf(eta1, 3.0);
  b_s = (2.0 * cx / xiTxidot + -2.0 / xiTxidot) + b_s_tmp;
  ddOmega_1_tmp = cx / xiTxidot;
  ddOmega_1 = (((8.0 / n_xi6 - 8.0 / n_xi6 * cx) - 5.0 * b_dGamma_2 / n_xi5) +
               ddOmega_1_tmp) * alpha + b_s * Cxi_3;
  xiTxidot = (3.0 / n_xi5 * b_dGamma_2 + -2.0 / xiTxidot) - ddOmega_1_tmp;
  n_xi5 = (((8.0 / n_xi6 - 15.0 * b_dGamma_2 / rt_powd_snf(eta1, 7.0)) + 7.0 *
            cx / n_xi6) + b_dGamma_2 / n_xi5) * alpha + xiTxidot * Cxi_3;
  d6 = fabs(eta1);
  if (d6 < 0.0001) {
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + i;
        dddexp3_[A6_tmp] = ((Cxi[i + 3] * 0.33333333333333331 * Cxi[3 * e_i + 1]
                             + 0.33333333333333331 * Cxi[i] * Cxi[3 * e_i]) +
                            Cxi[i + 6] * 0.33333333333333331 * Cxi[3 * e_i + 2])
          + Bdot[A6_tmp] * 0.5;
      }
    }
  } else {
    b_s = b_s * dddexp3__tmp_1 * 2.0;
    n_xi6 = 1.0 / n_xi2 - cx / n_xi2;
    xiTxidot = xiTxidot * dddexp3__tmp_1 * 2.0;
    n_xi2 = 1.0 / n_xi2 - b_s_tmp;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + 1;
        f_i = 3 * e_i + 2;
        e_j = 3 * e_i + i;
        b_s_tmp = ceil_xi[i + 3];
        dddexp3__tmp = ceil_xi[A6_tmp];
        dddexp3__tmp_0 = ceil_xi[3 * e_i];
        ddOmega_1_tmp = ceil_xi[i + 6];
        dddexp3__tmp_1 = ceil_xi[f_i];
        dddexp3_[e_j] = ((((Cxi[i + 3] * 2.0 * Cxi[A6_tmp] + 2.0 * Cxi[i] * Cxi
                            [3 * e_i]) + Cxi[i + 6] * 2.0 * Cxi[f_i]) + ((Bdot[i
          + 3] * dddexp3__tmp + dddexp3__tmp_0 * Bdot[i]) + Bdot[i + 6] *
          dddexp3__tmp_1)) + ((Bdot[3 * e_i] * ceil_xi[i] + Bdot[A6_tmp] *
          b_s_tmp) + Bdot[f_i] * ddOmega_1_tmp)) * n_xi2 + ((((b_s_tmp * n_xi5 *
          dddexp3__tmp + n_xi5 * ceil_xi[i] * dddexp3__tmp_0) + ddOmega_1_tmp *
          n_xi5 * dddexp3__tmp_1) + ((ceil_xi[e_j] * ddOmega_1 + Cxi[e_j] * b_s)
          + Bdot[e_j] * n_xi6)) + eta_[e_j] * xiTxidot);
      }
    }
  }

  if (d6 < 0.01) {
    n_xi2 = -((LR_U.dddexp6_lambda[3] * LR_U.dddexp6_lambdadot[3] +
               LR_U.dddexp6_lambda[4] * LR_U.dddexp6_lambdadot[4]) +
              LR_U.dddexp6_lambda[5] * LR_U.dddexp6_lambdadot[5]) / 60.0 * 2.0;

    /* Outport: '<Root>/dddexp6' */
    for (i = 0; i < 3; i++) {
      ddBA_0 = ddBA[i + 3];
      ddBA_1 = ddBA[i];
      ddBA_2 = ddBA[i + 6];
      ddGamma_1 = BA[i];
      B_tmp = BA[i + 3];
      BA_4 = BA[i + 6];
      beta = dBA[i + 3];
      dBA_1 = dBA[i];
      alpha = dBA[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + 1;
        e_j = 3 * e_i + 2;
        A6_tmp = 3 * e_i + i;
        ddOmega_1_tmp = ceil_xi[f_i];
        b_s = ceil_xi[3 * e_i];
        dddexp3__tmp = ceil_xi[e_j];
        Bdot_0[A6_tmp] = (((Bdot[i + 3] * AB[f_i] + AB[3 * e_i] * Bdot[i]) +
                           Bdot[i + 6] * AB[e_j]) + ((ceil_xi[i + 3] * B[f_i] +
          B[3 * e_i] * ceil_xi[i]) + ceil_xi[i + 6] * B[e_j])) + ((ddOmega_1_tmp
          * ddBA_0 + b_s * ddBA_1) + dddexp3__tmp * ddBA_2);
        Bdot_3 = Bdot[3 * e_i];
        BA_3 = ddGamma_1 * Bdot_3;
        n_xi5 = AB[i] * Bdot_3;
        Bdot_3 = Bdot[f_i];
        BA_3 += B_tmp * Bdot_3;
        n_xi5 += AB[i + 3] * Bdot_3;
        Bdot_3 = Bdot[e_j];
        AB_0[A6_tmp] = AB[i + 6] * Bdot_3 + n_xi5;
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
          (Cxi[f_i], dAB[f_i]), _mm_set_pd(beta, Cxi[i + 3])), _mm_mul_pd
          (_mm_set_pd(Cxi[3 * e_i], dAB[3 * e_i]), _mm_set_pd(dBA_1, Cxi[i]))),
          _mm_mul_pd(_mm_set_pd(Cxi[e_j], dAB[e_j]), _mm_set_pd(alpha, Cxi[i + 6]))));
        Cxi_0[A6_tmp] = tmp_f[0];
        dBA_0[A6_tmp] = tmp_f[1];
        BA_0[A6_tmp] = BA_4 * Bdot_3 + BA_3;
        Cxi_3 = etadot_[A6_tmp];
        B_0[A6_tmp] = (((B[i + 3] * ddOmega_1_tmp + b_s * B[i]) + B[i + 6] *
                        dddexp3__tmp) + Cxi_3) + Cxi_3;
        LR_Y.dddexp6[e_i + 6 * i] = dddexp3_[3 * i + e_i];
      }
    }

    for (i = 0; i < 3; i++) {
      e_i = (i + 3) * 6;
      LR_Y.dddexp6[e_i] = ((((dAB[3 * i] + dBA[3 * i]) * n_xi2 + etaddot_[3 * i]
        * 0.5) + (B[3 * i] + ddBA[3 * i]) * 0.16666666666666666) + ((dBA_0[3 * i]
        + Cxi_0[3 * i]) * 2.0 + (Bdot_0[3 * i] + BA_0[3 * i])) *
                           0.041666666666666664) + (B_0[3 * i] + AB_0[3 * i]) *
        0.041666666666666664;
      LR_Y.dddexp6[6 * i + 3] = 0.0;
      LR_Y.dddexp6[e_i + 3] = dddexp3_[3 * i];
      A6_tmp = 3 * i + 1;
      LR_Y.dddexp6[e_i + 1] = ((((dAB[A6_tmp] + dBA[A6_tmp]) * n_xi2 +
        etaddot_[A6_tmp] * 0.5) + (B[A6_tmp] + ddBA[A6_tmp]) *
        0.16666666666666666) + ((dBA_0[A6_tmp] + Cxi_0[A6_tmp]) * 2.0 +
        (Bdot_0[A6_tmp] + BA_0[A6_tmp])) * 0.041666666666666664) + (B_0[A6_tmp]
        + AB_0[A6_tmp]) * 0.041666666666666664;
      LR_Y.dddexp6[6 * i + 4] = 0.0;
      LR_Y.dddexp6[e_i + 4] = dddexp3_[A6_tmp];
      A6_tmp = 3 * i + 2;
      LR_Y.dddexp6[e_i + 2] = ((((dAB[A6_tmp] + dBA[A6_tmp]) * n_xi2 +
        etaddot_[A6_tmp] * 0.5) + (B[A6_tmp] + ddBA[A6_tmp]) *
        0.16666666666666666) + ((dBA_0[A6_tmp] + Cxi_0[A6_tmp]) * 2.0 +
        (Bdot_0[A6_tmp] + BA_0[A6_tmp])) * 0.041666666666666664) + (B_0[A6_tmp]
        + AB_0[A6_tmp]) * 0.041666666666666664;
      LR_Y.dddexp6[6 * i + 5] = 0.0;
      LR_Y.dddexp6[e_i + 5] = dddexp3_[A6_tmp];
    }
  } else {
    b_s_tmp = normxidot * normxidot;
    b_s = b_s_tmp * ddGamma_1 + dGamma_1 * theta_sqr;
    n_xi6 = 2.0 * dGamma_1 * normxidot;
    xiTxidot = b_s_tmp * tmp + dGamma_2 * theta_sqr;
    n_xi2 = 2.0 * dGamma_2 * normxidot;
    ddOmega_1 = ((-(-((((24.0 * eta1 - 60.0 * b_dGamma_2) - xn_0[2] * cx) + 9.0 *
                       xn2sx) + 36.0 * xcx_tmp) / xn_0[5]) / eta1 + 2.0 *
                  ddGamma_1 / xn_0[1]) - 2.0 * dGamma_1 / xn_0[2]) * b_s_tmp +
      xixidot * theta_sqr;
    xixidot = 2.0 * xixidot * normxidot;
    dGamma_1 = (-(((((120.0 * cx - 12.0 * beta_2) - xn_0[2] * b_dGamma_2) + 60.0
                    * Cxi_2) + 12.0 * xn_0[1]) - 120.0) / xn_0[6] * eta1 + 3.0 *
                tmp) * b_s_tmp + beta * theta_sqr;
    alpha = 2.0 * beta * normxidot;

    /* Outport: '<Root>/dddexp6' */
    for (i = 0; i < 3; i++) {
      cx = AA[i];
      xn2sx = AA[i + 3];
      b_dGamma_2 = AA[i + 6];
      normxidot = eta_[i + 3];
      b_s_tmp = eta_[i];
      theta_sqr = eta_[i + 6];
      ddBA_0 = ddBA[i + 3];
      ddBA_1 = ddBA[i];
      ddBA_2 = ddBA[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        n_xi5 = AB[3 * e_i];
        beta_2 = ceil_xi[i] * n_xi5;
        ddGamma_1 = BA[i];
        dddexp3__tmp = ceil_xi[3 * e_i];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(Cxi[i], dddexp3__tmp),
          _mm_set_pd(n_xi5, ddGamma_1)));
        B_tmp = tmp_f[0];
        tmp = tmp_f[1];
        dAB_1 = dAB[3 * e_i];
        ceil_xi_2 = ceil_xi[i] * dAB_1;
        beta = dBA[i];
        dddexp3__tmp_0 = Cxi[3 * e_i];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(dddexp3__tmp_0,
          dddexp3__tmp), _mm_set_pd(ddGamma_1, beta)));
        ddOmega_1_tmp = tmp_f[0];
        BA_4 = tmp_f[1];
        dddexp3__tmp_1 = Bdot[3 * e_i];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(dddexp3__tmp_1, Bdot[i]),
          _mm_set_pd(ddGamma_1, n_xi5)));
        Bdot_3 = tmp_f[0];
        BA_3 = tmp_f[1];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(Cxi[i], dddexp3__tmp_0),
          _mm_set_pd(dAB_1, beta)));
        dBA_1 = tmp_f[0];
        Cxi_2 = tmp_f[1];
        Cxi_3 = BA[3 * e_i];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(dBA[3 * e_i], Cxi_3),
          _mm_set1_pd(cx)));
        eta1 = tmp_f[0];
        d6 = tmp_f[1];
        f_i = 3 * e_i + 1;
        n_xi5 = AB[f_i];
        ceil_xi_tmp = ceil_xi[i + 3];
        beta_2 += ceil_xi_tmp * n_xi5;
        ddGamma_1 = BA[i + 3];
        Bdot_4 = ceil_xi[f_i];
        Bdot_5 = Cxi[i + 3];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Bdot_5, Bdot_4),
          _mm_set_pd(n_xi5, ddGamma_1)), _mm_set_pd(tmp, B_tmp)));
        B_tmp = tmp_f[0];
        tmp = tmp_f[1];
        dAB_1 = dAB[f_i];
        ceil_xi_2 += ceil_xi_tmp * dAB_1;
        beta = dBA[i + 3];
        xcx_tmp = Cxi[f_i];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(xcx_tmp,
          Bdot_4), _mm_set_pd(ddGamma_1, beta)), _mm_set_pd(BA_4, ddOmega_1_tmp)));
        ddOmega_1_tmp = tmp_f[0];
        BA_4 = tmp_f[1];
        tmp_k = Bdot[i + 3];
        tmp_l = Bdot[f_i];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(tmp_l, tmp_k),
          _mm_set_pd(ddGamma_1, n_xi5)), _mm_set_pd(BA_3, Bdot_3)));
        Bdot_3 = tmp_f[0];
        BA_3 = tmp_f[1];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Bdot_5,
          xcx_tmp), _mm_set_pd(dAB_1, beta)), _mm_set_pd(Cxi_2, dBA_1)));
        dBA_1 = tmp_f[0];
        Cxi_2 = tmp_f[1];
        tmp_j = BA[f_i];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(dBA[f_i],
          tmp_j), _mm_set1_pd(xn2sx)), _mm_set_pd(d6, eta1)));
        e_j = 3 * e_i + 2;
        n_xi5 = AB[e_j];
        ddGamma_1 = BA[i + 6];
        dAB_1 = dAB[e_j];
        beta = dBA[i + 6];
        A6_tmp = 3 * e_i + i;
        ceil_xi_1[A6_tmp] = dBA[e_j] * b_dGamma_2 + tmp_f[1];
        eta1 = BA[e_j];
        eta__0[A6_tmp] = (tmp_j * normxidot + Cxi_3 * b_s_tmp) + eta1 *
          theta_sqr;
        Cxi_3 = Cxi[i + 6];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Cxi_3, eta1),
          _mm_set_pd(dAB_1, b_dGamma_2)), _mm_set_pd(Cxi_2, tmp_f[0])));
        AA_0[A6_tmp] = tmp_f[0];
        Cxi_1[A6_tmp] = tmp_f[1];
        tmp_j = Cxi[e_j];
        Cxi_2 = Bdot[e_j];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Cxi_2, tmp_j),
          _mm_set_pd(ddGamma_1, beta)), _mm_set_pd(BA_3, dBA_1)));
        dBA_0[A6_tmp] = tmp_f[0];
        BA_2[A6_tmp] = tmp_f[1];
        AB_3 = ceil_xi[e_j];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(AB_3, tmp_j),
          _mm_set1_pd(ddGamma_1)), _mm_set_pd(B_tmp, BA_4)));
        BA_1[A6_tmp] = tmp_f[0];
        BA_0[A6_tmp] = tmp_f[1];
        dBA_1 = ceil_xi[i + 6];
        ceil_xi_0[A6_tmp] = dBA_1 * n_xi5 + beta_2;
        Cxi_0[A6_tmp] = ((Cxi_3 * n_xi5 + tmp) + (dBA_1 * dAB_1 + ceil_xi_2)) +
          (AB_3 * beta + ddOmega_1_tmp);
        ddOmega_1_tmp = Bdot[i + 6];
        Bdot_0[A6_tmp] = (((B[3 * e_i] * ceil_xi[i] + B[f_i] * ceil_xi_tmp) +
                           B[e_j] * dBA_1) + (ddOmega_1_tmp * n_xi5 + Bdot_3)) +
          ((Bdot_4 * ddBA_0 + dddexp3__tmp * ddBA_1) + AB_3 * ddBA_2);
        Bdot_1[A6_tmp] = (((Bdot_5 * 2.0 * xcx_tmp + 2.0 * Cxi[i] *
                            dddexp3__tmp_0) + Cxi_3 * 2.0 * tmp_j) + ((Bdot_4 *
          tmp_k + dddexp3__tmp * Bdot[i]) + AB_3 * ddOmega_1_tmp)) + ((tmp_l *
          ceil_xi_tmp + dddexp3__tmp_1 * ceil_xi[i]) + Cxi_2 * dBA_1);
      }

      n_xi5 = AB[i];
      dBA_1 = AB[i + 3];
      AB_4 = AB[i + 6];
      dAB_1 = dAB[i];
      B_tmp = dAB[i + 3];
      BA_4 = dAB[i + 6];
      dddexp3__tmp_1 = B[i];
      tmp = B[i + 3];
      beta = B[i + 6];
      Bdot_3 = Bdot_1[i + 3];
      Bdot_4 = Bdot_1[i];
      Bdot_5 = Bdot_1[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        beta_2 = ceil_xi[3 * e_i];
        AB_3 = n_xi5 * beta_2;
        eta1 = dAB_1 * beta_2;
        dddexp3__tmp = dddexp3__tmp_1 * beta_2;
        A6_tmp = 3 * e_i + 1;
        beta_2 = ceil_xi[A6_tmp];
        AB_3 += dBA_1 * beta_2;
        eta1 += B_tmp * beta_2;
        dddexp3__tmp += tmp * beta_2;
        e_j = 3 * e_i + 2;
        beta_2 = ceil_xi[e_j];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Cxi[e_j],
          Bdot[e_j]), _mm_set1_pd(AB_4)), _mm_set_pd(Cxi[A6_tmp] * dBA_1 + Cxi[3
          * e_i] * n_xi5, Bdot[A6_tmp] * dBA_1 + Bdot[3 * e_i] * n_xi5)));
        f_i = 3 * e_i + i;
        AB_2[f_i] = tmp_f[0];
        AB_1[f_i] = tmp_f[1];
        dAB_0[f_i] = BA_4 * beta_2 + eta1;
        AB_0[f_i] = AB_4 * beta_2 + AB_3;
        Bdot_2[f_i] = (((BA[3 * e_i] * Bdot_4 + BA[A6_tmp] * Bdot_3) + BA[e_j] *
                        Bdot_5) + ((normxidot * 2.0 * dBA[A6_tmp] + 2.0 *
          b_s_tmp * dBA[3 * e_i]) + theta_sqr * 2.0 * dBA[e_j])) + ((ddBA[3 *
          e_i] * cx + ddBA[A6_tmp] * xn2sx) + ddBA[e_j] * b_dGamma_2);
        Cxi_3 = etadot_[f_i];
        B_0[f_i] = ((beta * beta_2 + dddexp3__tmp) + Cxi_3) + Cxi_3;
        LR_Y.dddexp6[e_i + 6 * i] = dddexp3_[3 * i + e_i];
      }
    }

    for (i = 0; i < 3; i++) {
      e_i = (i + 3) * 6;
      LR_Y.dddexp6[e_i] = ((((((((((((AB[3 * i] + BA[3 * i]) * b_s + etaddot_[3 *
        i] * 0.5) + (dAB[3 * i] + dBA[3 * i]) * n_xi6) + (B[3 * i] + ddBA[3 * i])
        * Gamma_1) + (ceil_xi_0[3 * i] + BA_0[3 * i]) * xiTxidot) + (Cxi_0[3 * i]
        + BA_1[3 * i]) * n_xi2) + ((dBA_0[3 * i] + Cxi_1[3 * i]) * 2.0 +
        (Bdot_0[3 * i] + BA_2[3 * i])) * Gamma_2) + AA_0[3 * i] * ddOmega_1) +
        (eta__0[3 * i] + ceil_xi_1[3 * i]) * xixidot) + Bdot_2[3 * i] * Gamma_3)
                            + AB_0[3 * i] * dGamma_1) + (dAB_0[3 * i] + AB_1[3 *
        i]) * alpha) + (B_0[3 * i] + AB_2[3 * i]) * Gamma_4;
      LR_Y.dddexp6[6 * i + 3] = 0.0;
      LR_Y.dddexp6[e_i + 3] = dddexp3_[3 * i];
      A6_tmp = 3 * i + 1;
      LR_Y.dddexp6[e_i + 1] = ((((((((((((AB[A6_tmp] + BA[A6_tmp]) * b_s +
        etaddot_[A6_tmp] * 0.5) + (dAB[A6_tmp] + dBA[A6_tmp]) * n_xi6) +
        (B[A6_tmp] + ddBA[A6_tmp]) * Gamma_1) + (ceil_xi_0[A6_tmp] + BA_0[A6_tmp])
        * xiTxidot) + (Cxi_0[A6_tmp] + BA_1[A6_tmp]) * n_xi2) + ((dBA_0[A6_tmp]
        + Cxi_1[A6_tmp]) * 2.0 + (Bdot_0[A6_tmp] + BA_2[A6_tmp])) * Gamma_2) +
        AA_0[A6_tmp] * ddOmega_1) + (eta__0[A6_tmp] + ceil_xi_1[A6_tmp]) *
        xixidot) + Bdot_2[A6_tmp] * Gamma_3) + AB_0[A6_tmp] * dGamma_1) +
        (dAB_0[A6_tmp] + AB_1[A6_tmp]) * alpha) + (B_0[A6_tmp] + AB_2[A6_tmp]) *
        Gamma_4;
      LR_Y.dddexp6[6 * i + 4] = 0.0;
      LR_Y.dddexp6[e_i + 4] = dddexp3_[A6_tmp];
      A6_tmp = 3 * i + 2;
      LR_Y.dddexp6[e_i + 2] = ((((((((((((AB[A6_tmp] + BA[A6_tmp]) * b_s +
        etaddot_[A6_tmp] * 0.5) + (dAB[A6_tmp] + dBA[A6_tmp]) * n_xi6) +
        (B[A6_tmp] + ddBA[A6_tmp]) * Gamma_1) + (ceil_xi_0[A6_tmp] + BA_0[A6_tmp])
        * xiTxidot) + (Cxi_0[A6_tmp] + BA_1[A6_tmp]) * n_xi2) + ((dBA_0[A6_tmp]
        + Cxi_1[A6_tmp]) * 2.0 + (Bdot_0[A6_tmp] + BA_2[A6_tmp])) * Gamma_2) +
        AA_0[A6_tmp] * ddOmega_1) + (eta__0[A6_tmp] + ceil_xi_1[A6_tmp]) *
        xixidot) + Bdot_2[A6_tmp] * Gamma_3) + AB_0[A6_tmp] * dGamma_1) +
        (dAB_0[A6_tmp] + AB_1[A6_tmp]) * alpha) + (B_0[A6_tmp] + AB_2[A6_tmp]) *
        Gamma_4;
      LR_Y.dddexp6[6 * i + 5] = 0.0;
      LR_Y.dddexp6[e_i + 5] = dddexp3_[A6_tmp];
    }
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function4' */

  /* MATLAB Function: '<Root>/MATLAB Function5' incorporates:
   *  Inport: '<Root>/dexp3_xi'
   *  Outport: '<Root>/dexp3'
   */
  xiTxidot = LR_norm_e(LR_U.dexp3_xi);
  if (xiTxidot < 2.2204460492503131E-16) {
    /* Outport: '<Root>/dexp3' */
    memset(&LR_Y.dexp3[0], 0, 9U * sizeof(real_T));
    LR_Y.dexp3[0] = 1.0;
    LR_Y.dexp3[4] = 1.0;
    LR_Y.dexp3[8] = 1.0;
  } else {
    ceil_xi[0] = 0.0;
    ceil_xi[3] = -LR_U.dexp3_xi[2];
    ceil_xi[6] = LR_U.dexp3_xi[1];
    ceil_xi[1] = LR_U.dexp3_xi[2];
    ceil_xi[4] = 0.0;
    ceil_xi[7] = -LR_U.dexp3_xi[0];
    ceil_xi[2] = -LR_U.dexp3_xi[1];
    ceil_xi[5] = LR_U.dexp3_xi[0];
    ceil_xi[8] = 0.0;
    ddOmega_1 = sin(xiTxidot / 2.0) / (xiTxidot / 2.0);
    b_s = ddOmega_1 * ddOmega_1 / 2.0;
    n_xi6 = (1.0 - cos(xiTxidot / 2.0) * ddOmega_1) / (xiTxidot * xiTxidot);
    for (i = 0; i < 9; i++) {
      b_I[i] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;

    /* Outport: '<Root>/dexp3' */
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + i;
        LR_Y.dexp3[A6_tmp] = ((ceil_xi[i + 3] * n_xi6 * ceil_xi[3 * e_i + 1] +
          n_xi6 * ceil_xi[i] * ceil_xi[3 * e_i]) + ceil_xi[i + 6] * n_xi6 *
                              ceil_xi[3 * e_i + 2]) + (ceil_xi[A6_tmp] * b_s +
          (real_T)b_I[A6_tmp]);
      }
    }
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function5' */

  /* MATLAB Function: '<Root>/MATLAB Function6' incorporates:
   *  Inport: '<Root>/ddexp3_xi'
   *  Inport: '<Root>/ddexp3_xidot'
   *  Outport: '<Root>/ddexp3'
   */
  eta1 = 3.3121686421112381E-170;
  cx = fabs(LR_U.ddexp3_xi[0]);
  if (cx > 3.3121686421112381E-170) {
    d6 = 1.0;
    eta1 = cx;
  } else {
    Gamma_1 = cx / 3.3121686421112381E-170;
    d6 = Gamma_1 * Gamma_1;
  }

  cx = fabs(LR_U.ddexp3_xi[1]);
  if (cx > eta1) {
    Gamma_1 = eta1 / cx;
    d6 = d6 * Gamma_1 * Gamma_1 + 1.0;
    eta1 = cx;
  } else {
    Gamma_1 = cx / eta1;
    d6 += Gamma_1 * Gamma_1;
  }

  cx = fabs(LR_U.ddexp3_xi[2]);
  if (cx > eta1) {
    Gamma_1 = eta1 / cx;
    d6 = d6 * Gamma_1 * Gamma_1 + 1.0;
    eta1 = cx;
  } else {
    Gamma_1 = cx / eta1;
    d6 += Gamma_1 * Gamma_1;
  }

  d6 = eta1 * sqrt(d6);
  if (d6 < 1.0E-12) {
    /* Outport: '<Root>/ddexp3' */
    LR_Y.ddexp3[0] = 0.0;
    tmp_b = _mm_set1_pd(0.5);
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(LR_U.ddexp3_xidot[1],
      -LR_U.ddexp3_xidot[2])));
    LR_Y.ddexp3[3] = tmp_f[0];
    LR_Y.ddexp3[6] = tmp_f[1];

    /* Outport: '<Root>/ddexp3' incorporates:
     *  Inport: '<Root>/ddexp3_xidot'
     */
    LR_Y.ddexp3[1] = 0.5 * LR_U.ddexp3_xidot[2];
    LR_Y.ddexp3[4] = 0.0;
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(-LR_U.ddexp3_xidot[1],
      -LR_U.ddexp3_xidot[0])));
    LR_Y.ddexp3[7] = tmp_f[0];
    LR_Y.ddexp3[2] = tmp_f[1];

    /* Outport: '<Root>/ddexp3' incorporates:
     *  Inport: '<Root>/ddexp3_xidot'
     */
    LR_Y.ddexp3[5] = 0.5 * LR_U.ddexp3_xidot[0];
    LR_Y.ddexp3[8] = 0.0;
  } else {
    b_s = d6 / 2.0;
    ddOmega_1 = sin(b_s) / b_s;
    alpha = ddOmega_1 * cos(b_s);
    beta = ddOmega_1 * ddOmega_1;
    beta_2 = beta / 2.0;
    theta_sqr = d6 * d6;
    tmp = (1.0 - alpha) / theta_sqr;
    dddexp3__tmp = (LR_U.ddexp3_xi[0] * LR_U.ddexp3_xidot[0] + LR_U.ddexp3_xi[1]
                    * LR_U.ddexp3_xidot[1]) + LR_U.ddexp3_xi[2] *
      LR_U.ddexp3_xidot[2];
    ceil_xi[0] = 0.0;
    ceil_xi[3] = -LR_U.ddexp3_xi[2];
    ceil_xi[6] = LR_U.ddexp3_xi[1];
    ceil_xi[1] = LR_U.ddexp3_xi[2];
    ceil_xi[4] = 0.0;
    ceil_xi[7] = -LR_U.ddexp3_xi[0];
    ceil_xi[2] = -LR_U.ddexp3_xi[1];
    ceil_xi[5] = LR_U.ddexp3_xi[0];
    ceil_xi[8] = 0.0;
    AA[0] = 0.0;
    AA[3] = -LR_U.ddexp3_xidot[2];
    AA[6] = LR_U.ddexp3_xidot[1];
    AA[1] = LR_U.ddexp3_xidot[2];
    AA[4] = 0.0;
    AA[7] = -LR_U.ddexp3_xidot[0];
    AA[2] = -LR_U.ddexp3_xidot[1];
    AA[5] = LR_U.ddexp3_xidot[0];
    AA[8] = 0.0;
    b_s = (alpha - beta) / theta_sqr * dddexp3__tmp;
    n_xi6 = (beta_2 - 3.0 * tmp) / theta_sqr * dddexp3__tmp;

    /* Outport: '<Root>/ddexp3' incorporates:
     *  Inport: '<Root>/ddexp3_xidot'
     */
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + 1;
        f_i = 3 * e_i + 2;
        e_j = 3 * e_i + i;
        n_xi2 = ceil_xi[i + 3];
        ddOmega_1 = ceil_xi[A6_tmp];
        eta1 = ceil_xi[3 * e_i];
        d6 = ceil_xi[i + 6];
        dddexp3__tmp = ceil_xi[f_i];
        LR_Y.ddexp3[e_j] = (((((AA[3 * e_i] * ceil_xi[i] + AA[A6_tmp] * n_xi2) +
          AA[f_i] * d6) + ((AA[i + 3] * ddOmega_1 + eta1 * AA[i]) + AA[i + 6] *
                           dddexp3__tmp)) * tmp + AA[e_j] * beta_2) +
                            ceil_xi[e_j] * b_s) + ((n_xi2 * n_xi6 * ddOmega_1 +
          n_xi6 * ceil_xi[i] * eta1) + d6 * n_xi6 * dddexp3__tmp);
      }
    }
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function6' */

  /* MATLAB Function: '<Root>/MATLAB Function7' incorporates:
   *  Inport: '<Root>/dddexp3_xi'
   *  Inport: '<Root>/dddexp3_xiddot'
   *  Inport: '<Root>/dddexp3_xidot'
   */
  ceil_xi[0] = 0.0;
  ceil_xi[3] = -LR_U.dddexp3_xi[2];
  ceil_xi[6] = LR_U.dddexp3_xi[1];
  ceil_xi[1] = LR_U.dddexp3_xi[2];
  ceil_xi[4] = 0.0;
  ceil_xi[7] = -LR_U.dddexp3_xi[0];
  ceil_xi[2] = -LR_U.dddexp3_xi[1];
  ceil_xi[5] = LR_U.dddexp3_xi[0];
  ceil_xi[8] = 0.0;
  B[0] = 0.0;
  B[3] = -LR_U.dddexp3_xidot[2];
  B[6] = LR_U.dddexp3_xidot[1];
  B[1] = LR_U.dddexp3_xidot[2];
  B[4] = 0.0;
  B[7] = -LR_U.dddexp3_xidot[0];
  B[2] = -LR_U.dddexp3_xidot[1];
  B[5] = LR_U.dddexp3_xidot[0];
  B[8] = 0.0;
  Cxi[0] = 0.0;
  Cxi[3] = -LR_U.dddexp3_xiddot[2];
  Cxi[6] = LR_U.dddexp3_xiddot[1];
  Cxi[1] = LR_U.dddexp3_xiddot[2];
  Cxi[4] = 0.0;
  Cxi[7] = -LR_U.dddexp3_xiddot[0];
  Cxi[2] = -LR_U.dddexp3_xiddot[1];
  Cxi[5] = LR_U.dddexp3_xiddot[0];
  Cxi[8] = 0.0;
  eta1 = LR_norm_e(LR_U.dddexp3_xi);
  n_xi2 = eta1 * eta1;
  xiTxidot = rt_powd_snf(eta1, 4.0);
  n_xi5 = rt_powd_snf(eta1, 5.0);
  n_xi6 = rt_powd_snf(eta1, 6.0);
  d6 = sin(eta1);
  cx = cos(eta1);
  dddexp3__tmp = (LR_U.dddexp3_xi[0] * LR_U.dddexp3_xidot[0] + LR_U.dddexp3_xi[1]
                  * LR_U.dddexp3_xidot[1]) + LR_U.dddexp3_xi[2] *
    LR_U.dddexp3_xidot[2];
  dGamma_1 = ((LR_U.dddexp3_xidot[0] * LR_U.dddexp3_xidot[0] +
               LR_U.dddexp3_xidot[1] * LR_U.dddexp3_xidot[1]) +
              LR_U.dddexp3_xidot[2] * LR_U.dddexp3_xidot[2]) +
    ((LR_U.dddexp3_xi[0] * LR_U.dddexp3_xiddot[0] + LR_U.dddexp3_xi[1] *
      LR_U.dddexp3_xiddot[1]) + LR_U.dddexp3_xi[2] * LR_U.dddexp3_xiddot[2]);
  b_s_tmp = d6 / rt_powd_snf(eta1, 3.0);
  b_s = (2.0 * cx / xiTxidot + -2.0 / xiTxidot) + b_s_tmp;
  ddOmega_1_tmp = cx / xiTxidot;
  dddexp3__tmp_0 = dddexp3__tmp * dddexp3__tmp;
  ddOmega_1 = (((8.0 / n_xi6 - 8.0 / n_xi6 * cx) - 5.0 * d6 / n_xi5) +
               ddOmega_1_tmp) * dddexp3__tmp_0 + b_s * dGamma_1;
  xiTxidot = (3.0 / n_xi5 * d6 + -2.0 / xiTxidot) - ddOmega_1_tmp;
  n_xi5 = (((8.0 / n_xi6 - 15.0 * d6 / rt_powd_snf(eta1, 7.0)) + 7.0 * cx /
            n_xi6) + d6 / n_xi5) * dddexp3__tmp_0 + xiTxidot * dGamma_1;
  if (fabs(eta1) < 0.0001) {
    /* Outport: '<Root>/dddexp3' */
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + i;
        LR_Y.dddexp3[A6_tmp] = ((B[i + 3] * 0.33333333333333331 * B[3 * e_i + 1]
          + 0.33333333333333331 * B[i] * B[3 * e_i]) + B[i + 6] *
          0.33333333333333331 * B[3 * e_i + 2]) + Cxi[A6_tmp] * 0.5;
      }
    }
  } else {
    b_s = b_s * dddexp3__tmp * 2.0;
    n_xi6 = 1.0 / n_xi2 - cx / n_xi2;
    xiTxidot = xiTxidot * dddexp3__tmp * 2.0;
    n_xi2 = 1.0 / n_xi2 - b_s_tmp;

    /* Outport: '<Root>/dddexp3' */
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + 1;
        f_i = 3 * e_i + 2;
        e_j = 3 * e_i + i;
        eta1 = ceil_xi[i + 3];
        d6 = ceil_xi[A6_tmp];
        dddexp3__tmp = ceil_xi[3 * e_i];
        dGamma_1 = ceil_xi[i + 6];
        dddexp3__tmp_0 = ceil_xi[f_i];
        ddOmega_1_tmp = B[i + 3];
        Gamma_2 = B[A6_tmp];
        dddexp3__tmp_1 = B[3 * e_i];
        Gamma_1 = B[i + 6];
        Gamma_3 = B[f_i];
        LR_Y.dddexp3[e_j] = ((((Cxi[i + 3] * d6 + dddexp3__tmp * Cxi[i]) + Cxi[i
          + 6] * dddexp3__tmp_0) + ((ddOmega_1_tmp * 2.0 * Gamma_2 + 2.0 * B[i] *
          dddexp3__tmp_1) + Gamma_1 * 2.0 * Gamma_3)) + ((Cxi[3 * e_i] *
          ceil_xi[i] + Cxi[A6_tmp] * eta1) + Cxi[f_i] * dGamma_1)) * n_xi2 +
          ((((d6 * ddOmega_1_tmp + dddexp3__tmp * B[i]) + dddexp3__tmp_0 *
             Gamma_1) + ((Gamma_2 * eta1 + dddexp3__tmp_1 * ceil_xi[i]) +
                         Gamma_3 * dGamma_1)) * xiTxidot + (((eta1 * n_xi5 * d6
              + n_xi5 * ceil_xi[i] * dddexp3__tmp) + dGamma_1 * n_xi5 *
             dddexp3__tmp_0) + ((ceil_xi[e_j] * ddOmega_1 + B[e_j] * b_s) +
             Cxi[e_j] * n_xi6)));
      }
    }
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function7' */

  /* MATLAB Function: '<Root>/MATLAB Function8' incorporates:
   *  Inport: '<Root>/dexp3inv_xi'
   *  Outport: '<Root>/dexp3inv'
   */
  xiTxidot = LR_norm_jl(LR_U.dexp3inv_xi);
  if (xiTxidot < 2.2204460492503131E-16) {
    /* Outport: '<Root>/dexp3inv' */
    memset(&LR_Y.dexp3inv[0], 0, 9U * sizeof(real_T));
    LR_Y.dexp3inv[0] = 1.0;
    LR_Y.dexp3inv[4] = 1.0;
    LR_Y.dexp3inv[8] = 1.0;
  } else {
    ddOmega_1 = sin(xiTxidot / 2.0) / (xiTxidot / 2.0);
    b_s = (1.0 - cos(xiTxidot / 2.0) * ddOmega_1 / (ddOmega_1 * ddOmega_1)) /
      (xiTxidot * xiTxidot);
    ceil_xi[0] = 0.0;
    ceil_xi[3] = -LR_U.dexp3inv_xi[2];
    ceil_xi[6] = LR_U.dexp3inv_xi[1];
    ceil_xi[1] = LR_U.dexp3inv_xi[2];
    ceil_xi[4] = 0.0;
    ceil_xi[7] = -LR_U.dexp3inv_xi[0];
    ceil_xi[2] = -LR_U.dexp3inv_xi[1];
    ceil_xi[5] = LR_U.dexp3inv_xi[0];
    ceil_xi[8] = 0.0;
    for (i = 0; i < 9; i++) {
      b_I[i] = 0;
    }

    for (e_j = 0; e_j < 3; e_j++) {
      b_I[e_j + 3 * e_j] = 1;
    }

    for (i = 0; i < 3; i++) {
      for (e_j = 0; e_j <= 0; e_j += 2) {
        tmp_b = _mm_loadu_pd(&ceil_xi[e_j + 3]);
        tmp_c = _mm_loadu_pd(&ceil_xi[e_j]);
        tmp_d = _mm_loadu_pd(&ceil_xi[e_j + 6]);
        _mm_storeu_pd(&ceil_xi_0[e_j + 3 * i], _mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(ceil_xi[3 * i + 1]), tmp_b), _mm_mul_pd(_mm_set1_pd
          (ceil_xi[3 * i]), tmp_c)), _mm_mul_pd(_mm_set1_pd(ceil_xi[3 * i + 2]),
          tmp_d)));
      }

      for (e_j = 2; e_j < 3; e_j++) {
        ceil_xi_0[e_j + 3 * i] = (ceil_xi[3 * i + 1] * ceil_xi[e_j + 3] +
          ceil_xi[3 * i] * ceil_xi[e_j]) + ceil_xi[3 * i + 2] * ceil_xi[e_j + 6];
      }
    }

    B[0] = b_I[0];
    tmp_b = _mm_set1_pd(0.5);
    _mm_storeu_pd(&B[1], _mm_sub_pd(_mm_set_pd(b_I[2], b_I[1]), _mm_mul_pd(tmp_b,
      _mm_set_pd(-LR_U.dexp3inv_xi[1], LR_U.dexp3inv_xi[2]))));
    B[3] = (real_T)b_I[3] - 0.5 * -LR_U.dexp3inv_xi[2];
    B[4] = b_I[4];
    _mm_storeu_pd(&B[5], _mm_sub_pd(_mm_set_pd(b_I[6], b_I[5]), _mm_mul_pd(tmp_b,
      _mm_loadu_pd(&LR_U.dexp3inv_xi[0]))));
    B[7] = (real_T)b_I[7] - 0.5 * -LR_U.dexp3inv_xi[0];
    B[8] = b_I[8];

    /* Outport: '<Root>/dexp3inv' */
    for (i = 0; i <= 6; i += 2) {
      tmp_b = _mm_loadu_pd(&ceil_xi_0[i]);
      tmp_c = _mm_loadu_pd(&B[i]);
      _mm_storeu_pd(&LR_Y.dexp3inv[i], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(b_s),
        tmp_b), tmp_c));
    }

    for (i = 8; i < 9; i++) {
      LR_Y.dexp3inv[i] = b_s * ceil_xi_0[i] + B[i];
    }
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function8' */

  /* MATLAB Function: '<Root>/MATLAB Function9' incorporates:
   *  Inport: '<Root>/ddexp3inv_xi'
   *  Inport: '<Root>/ddexp3inv_xidot'
   *  Outport: '<Root>/ddexp3inv'
   */
  xiTxidot = LR_norm_e(LR_U.ddexp3inv_xi);
  if (xiTxidot < 2.2204460492503131E-16) {
    /* Outport: '<Root>/ddexp3inv' */
    LR_Y.ddexp3inv[0] = -0.0;
    tmp_b = _mm_set1_pd(-0.5);
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(LR_U.ddexp3inv_xidot[1],
      -LR_U.ddexp3inv_xidot[2])));
    LR_Y.ddexp3inv[3] = tmp_f[0];
    LR_Y.ddexp3inv[6] = tmp_f[1];

    /* Outport: '<Root>/ddexp3inv' incorporates:
     *  Inport: '<Root>/ddexp3inv_xidot'
     */
    LR_Y.ddexp3inv[1] = -0.5 * LR_U.ddexp3inv_xidot[2];
    LR_Y.ddexp3inv[4] = -0.0;
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(-LR_U.ddexp3inv_xidot
      [1], -LR_U.ddexp3inv_xidot[0])));
    LR_Y.ddexp3inv[7] = tmp_f[0];
    LR_Y.ddexp3inv[2] = tmp_f[1];

    /* Outport: '<Root>/ddexp3inv' incorporates:
     *  Inport: '<Root>/ddexp3inv_xidot'
     */
    LR_Y.ddexp3inv[5] = -0.5 * LR_U.ddexp3inv_xidot[0];
    LR_Y.ddexp3inv[8] = -0.0;
  } else {
    d6 = xiTxidot * xiTxidot;
    ddOmega_1 = sin(xiTxidot / 2.0) / (xiTxidot / 2.0);
    beta = ddOmega_1 * ddOmega_1;
    Gamma_1 = cos(xiTxidot / 2.0) * ddOmega_1 / beta;
    b_s = (1.0 - Gamma_1) / d6;
    cx = ((1.0 / beta + Gamma_1) - 2.0) * (1.0 / d6) / d6;
    cx = (cx * LR_U.ddexp3inv_xi[0] * LR_U.ddexp3inv_xidot[0] + cx *
          LR_U.ddexp3inv_xi[1] * LR_U.ddexp3inv_xidot[1]) + cx *
      LR_U.ddexp3inv_xi[2] * LR_U.ddexp3inv_xidot[2];
    ceil_xi_0[0] = 0.0;
    ceil_xi_0[3] = -LR_U.ddexp3inv_xidot[2];
    ceil_xi_0[6] = LR_U.ddexp3inv_xidot[1];
    ceil_xi_0[1] = LR_U.ddexp3inv_xidot[2];
    ceil_xi_0[4] = 0.0;
    ceil_xi_0[7] = -LR_U.ddexp3inv_xidot[0];
    ceil_xi_0[2] = -LR_U.ddexp3inv_xidot[1];
    ceil_xi_0[5] = LR_U.ddexp3inv_xidot[0];
    ceil_xi_0[8] = 0.0;
    Cxi[0] = 0.0;
    Cxi[3] = -LR_U.ddexp3inv_xi[2];
    Cxi[6] = LR_U.ddexp3inv_xi[1];
    Cxi[1] = LR_U.ddexp3inv_xi[2];
    Cxi[4] = 0.0;
    Cxi[7] = -LR_U.ddexp3inv_xi[0];
    Cxi[2] = -LR_U.ddexp3inv_xi[1];
    Cxi[5] = LR_U.ddexp3inv_xi[0];
    Cxi[8] = 0.0;
    B[0] = 0.0;
    B[3] = -LR_U.ddexp3inv_xi[2];
    B[6] = LR_U.ddexp3inv_xi[1];
    B[1] = LR_U.ddexp3inv_xi[2];
    B[4] = 0.0;
    B[7] = -LR_U.ddexp3inv_xi[0];
    B[2] = -LR_U.ddexp3inv_xi[1];
    B[5] = LR_U.ddexp3inv_xi[0];
    B[8] = 0.0;
    Bdot[0] = 0.0;
    Bdot[3] = -LR_U.ddexp3inv_xidot[2];
    Bdot[6] = LR_U.ddexp3inv_xidot[1];
    Bdot[1] = LR_U.ddexp3inv_xidot[2];
    Bdot[4] = 0.0;
    Bdot[7] = -LR_U.ddexp3inv_xidot[0];
    Bdot[2] = -LR_U.ddexp3inv_xidot[1];
    Bdot[5] = LR_U.ddexp3inv_xidot[0];
    Bdot[8] = 0.0;
    for (i = 0; i < 3; i++) {
      e_i = 3 * i + 1;
      dddexp3__tmp = Bdot[e_i];
      dddexp3__tmp_0 = Bdot[3 * i];
      A6_tmp = 3 * i + 2;
      ddOmega_1_tmp = Bdot[A6_tmp];
      dddexp3__tmp_1 = Cxi[e_i];
      Cxi_3 = Cxi[3 * i];
      Bdot_4 = Cxi[A6_tmp];
      for (e_i = 0; e_i <= 0; e_i += 2) {
        tmp_b = _mm_loadu_pd(&B[e_i + 3]);
        tmp_c = _mm_loadu_pd(&B[e_i]);
        tmp_d = _mm_loadu_pd(&B[e_i + 6]);
        A6_tmp = 3 * i + e_i;
        _mm_storeu_pd(&ddBA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(dddexp3__tmp), tmp_b), _mm_mul_pd(_mm_set1_pd
          (dddexp3__tmp_0), tmp_c)), _mm_mul_pd(_mm_set1_pd(ddOmega_1_tmp),
          tmp_d)));
        tmp_b = _mm_loadu_pd(&ceil_xi_0[e_i + 3]);
        tmp_c = _mm_loadu_pd(&ceil_xi_0[e_i]);
        tmp_d = _mm_loadu_pd(&ceil_xi_0[e_i + 6]);
        _mm_storeu_pd(&etaddot_[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(dddexp3__tmp_1), tmp_b), _mm_mul_pd(_mm_set1_pd(Cxi_3),
          tmp_c)), _mm_mul_pd(_mm_set1_pd(Bdot_4), tmp_d)));
      }

      for (e_i = 2; e_i < 3; e_i++) {
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
          (dddexp3__tmp_1, dddexp3__tmp), _mm_set_pd(ceil_xi_0[e_i + 3], B[e_i +
          3])), _mm_mul_pd(_mm_set_pd(Cxi_3, dddexp3__tmp_0), _mm_set_pd
                           (ceil_xi_0[e_i], B[e_i]))), _mm_mul_pd(_mm_set_pd
          (Bdot_4, ddOmega_1_tmp), _mm_set_pd(ceil_xi_0[e_i + 6], B[e_i + 6]))));
        A6_tmp = 3 * i + e_i;
        ddBA[A6_tmp] = tmp_f[0];
        etaddot_[A6_tmp] = tmp_f[1];
      }
    }

    ceil_xi_0[0] = -0.0;
    tmp_b = _mm_set1_pd(-0.5);
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(LR_U.ddexp3inv_xidot[1],
      -LR_U.ddexp3inv_xidot[2])));
    ceil_xi_0[3] = tmp_f[0];
    ceil_xi_0[6] = tmp_f[1];
    ceil_xi_0[1] = -0.5 * LR_U.ddexp3inv_xidot[2];
    ceil_xi_0[4] = -0.0;
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(-LR_U.ddexp3inv_xidot
      [1], -LR_U.ddexp3inv_xidot[0])));
    ceil_xi_0[7] = tmp_f[0];
    ceil_xi_0[2] = tmp_f[1];
    ceil_xi_0[5] = -0.5 * LR_U.ddexp3inv_xidot[0];
    ceil_xi_0[8] = -0.0;
    Bdot[0] = cx * 0.0;
    tmp_b = _mm_set1_pd(cx);
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(LR_U.ddexp3inv_xi[1],
      -LR_U.ddexp3inv_xi[2])));
    Bdot[3] = tmp_f[0];
    Bdot[6] = tmp_f[1];
    Bdot[1] = cx * LR_U.ddexp3inv_xi[2];
    Bdot[4] = cx * 0.0;
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(-LR_U.ddexp3inv_xi[1],
      -LR_U.ddexp3inv_xi[0])));
    Bdot[7] = tmp_f[0];
    Bdot[2] = tmp_f[1];
    Bdot[5] = cx * LR_U.ddexp3inv_xi[0];
    Bdot[8] = cx * 0.0;
    Cxi[0] = 0.0;
    Cxi[3] = -LR_U.ddexp3inv_xi[2];
    Cxi[6] = LR_U.ddexp3inv_xi[1];
    Cxi[1] = LR_U.ddexp3inv_xi[2];
    Cxi[4] = 0.0;
    Cxi[7] = -LR_U.ddexp3inv_xi[0];
    Cxi[2] = -LR_U.ddexp3inv_xi[1];
    Cxi[5] = LR_U.ddexp3inv_xi[0];
    Cxi[8] = 0.0;

    /* Outport: '<Root>/ddexp3inv' incorporates:
     *  Inport: '<Root>/ddexp3inv_xidot'
     */
    for (i = 0; i < 3; i++) {
      n_xi6 = Bdot[i + 3];
      d6 = Bdot[i];
      dddexp3__tmp = Bdot[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + i;
        LR_Y.ddexp3inv[A6_tmp] = ((Cxi[3 * e_i + 1] * n_xi6 + Cxi[3 * e_i] * d6)
          + Cxi[3 * e_i + 2] * dddexp3__tmp) + ((etaddot_[A6_tmp] + ddBA[A6_tmp])
          * b_s + ceil_xi_0[A6_tmp]);
      }
    }
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function9' */

  /* MATLAB Function: '<Root>/MATLAB Function10' incorporates:
   *  Inport: '<Root>/dddexp3inv_xi'
   *  Inport: '<Root>/dddexp3inv_xiddot'
   *  Inport: '<Root>/dddexp3inv_xidot'
   */
  eta1 = LR_norm_e(LR_U.dddexp3inv_xi);
  d6 = sin(eta1);
  dGamma_1 = fabs(d6);
  if (dGamma_1 < 0.001) {
    if (fabs(eta1 - 6.2831853071795862) < 0.001) {
      ceil_xi[0] = 0.0;
      ceil_xi[3] = -LR_U.dddexp3inv_xi[2];
      ceil_xi[6] = LR_U.dddexp3inv_xi[1];
      ceil_xi[1] = LR_U.dddexp3inv_xi[2];
      ceil_xi[4] = 0.0;
      ceil_xi[7] = -LR_U.dddexp3inv_xi[0];
      ceil_xi[2] = -LR_U.dddexp3inv_xi[1];
      ceil_xi[5] = LR_U.dddexp3inv_xi[0];
      ceil_xi[8] = 0.0;
      memset(&AA[0], 0, 9U * sizeof(real_T));
      AA[0] = 1.0;
      AA[4] = 1.0;
      AA[8] = 1.0;
      for (i = 0; i < 3; i++) {
        for (e_i = 0; e_i < 3; e_i++) {
          f_i = 3 * e_i + i;
          B[f_i] = ((ceil_xi[i + 3] * 0.083333333333333329 * ceil_xi[3 * e_i + 1]
                     + 0.083333333333333329 * ceil_xi[i] * ceil_xi[3 * e_i]) +
                    ceil_xi[i + 6] * 0.083333333333333329 * ceil_xi[3 * e_i + 2])
            + (AA[f_i] - ceil_xi[f_i] * 0.5);
        }
      }
    } else {
      memset(&B[0], 0, 9U * sizeof(real_T));
      B[0] = 1.0;
      B[4] = 1.0;
      B[8] = 1.0;
    }
  } else {
    b_s = eta1 / 2.0;
    ceil_xi[0] = 0.0;
    ceil_xi[3] = -LR_U.dddexp3inv_xi[2];
    ceil_xi[6] = LR_U.dddexp3inv_xi[1];
    ceil_xi[1] = LR_U.dddexp3inv_xi[2];
    ceil_xi[4] = 0.0;
    ceil_xi[7] = -LR_U.dddexp3inv_xi[0];
    ceil_xi[2] = -LR_U.dddexp3inv_xi[1];
    ceil_xi[5] = LR_U.dddexp3inv_xi[0];
    ceil_xi[8] = 0.0;
    b_s = (1.0 - cos(b_s) / (sin(b_s) / b_s)) / (eta1 * eta1);
    memset(&AA[0], 0, 9U * sizeof(real_T));
    AA[0] = 1.0;
    AA[4] = 1.0;
    AA[8] = 1.0;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + i;
        B[f_i] = ((ceil_xi[i + 3] * b_s * ceil_xi[3 * e_i + 1] + b_s * ceil_xi[i]
                   * ceil_xi[3 * e_i]) + ceil_xi[i + 6] * b_s * ceil_xi[3 * e_i
                  + 2]) + (AA[f_i] - ceil_xi[f_i] * 0.5);
      }
    }
  }

  if (eta1 < 1.0E-12) {
    Bdot[0] = 0.0;
    tmp_b = _mm_set1_pd(0.5);
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd(LR_U.dddexp3inv_xidot
      [1], -LR_U.dddexp3inv_xidot[2])));
    Bdot[3] = tmp_f[0];
    Bdot[6] = tmp_f[1];
    Bdot[1] = 0.5 * LR_U.dddexp3inv_xidot[2];
    Bdot[4] = 0.0;
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd
      (-LR_U.dddexp3inv_xidot[1], -LR_U.dddexp3inv_xidot[0])));
    Bdot[7] = tmp_f[0];
    Bdot[2] = tmp_f[1];
    Bdot[5] = 0.5 * LR_U.dddexp3inv_xidot[0];
    Bdot[8] = 0.0;
  } else {
    cx = eta1 / 2.0;
    ddOmega_1 = sin(cx) / cx;
    alpha = ddOmega_1 * cos(cx);
    beta = ddOmega_1 * ddOmega_1;
    beta_2 = beta / 2.0;
    theta_sqr = eta1 * eta1;
    tmp = (1.0 - alpha) / theta_sqr;
    dddexp3__tmp = (LR_U.dddexp3inv_xi[0] * LR_U.dddexp3inv_xidot[0] +
                    LR_U.dddexp3inv_xi[1] * LR_U.dddexp3inv_xidot[1]) +
      LR_U.dddexp3inv_xi[2] * LR_U.dddexp3inv_xidot[2];
    ceil_xi[0] = 0.0;
    ceil_xi[3] = -LR_U.dddexp3inv_xi[2];
    ceil_xi[6] = LR_U.dddexp3inv_xi[1];
    ceil_xi[1] = LR_U.dddexp3inv_xi[2];
    ceil_xi[4] = 0.0;
    ceil_xi[7] = -LR_U.dddexp3inv_xi[0];
    ceil_xi[2] = -LR_U.dddexp3inv_xi[1];
    ceil_xi[5] = LR_U.dddexp3inv_xi[0];
    ceil_xi[8] = 0.0;
    AA[0] = 0.0;
    AA[3] = -LR_U.dddexp3inv_xidot[2];
    AA[6] = LR_U.dddexp3inv_xidot[1];
    AA[1] = LR_U.dddexp3inv_xidot[2];
    AA[4] = 0.0;
    AA[7] = -LR_U.dddexp3inv_xidot[0];
    AA[2] = -LR_U.dddexp3inv_xidot[1];
    AA[5] = LR_U.dddexp3inv_xidot[0];
    AA[8] = 0.0;
    n_xi6 = (alpha - beta) / theta_sqr * dddexp3__tmp;
    xiTxidot = (beta_2 - 3.0 * tmp) / theta_sqr * dddexp3__tmp;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + 1;
        e_j = 3 * e_i + 2;
        A6_tmp = 3 * e_i + i;
        ddOmega_1_tmp = ceil_xi[i + 3];
        b_s = ceil_xi[f_i];
        dddexp3__tmp = ceil_xi[3 * e_i];
        n_xi2 = ceil_xi[i + 6];
        ddOmega_1 = ceil_xi[e_j];
        Bdot[A6_tmp] = (((((AA[3 * e_i] * ceil_xi[i] + AA[f_i] * ddOmega_1_tmp)
                           + AA[e_j] * n_xi2) + ((AA[i + 3] * b_s + dddexp3__tmp
          * AA[i]) + AA[i + 6] * ddOmega_1)) * tmp + AA[A6_tmp] * beta_2) +
                        ceil_xi[A6_tmp] * n_xi6) + ((ddOmega_1_tmp * xiTxidot *
          b_s + xiTxidot * ceil_xi[i] * dddexp3__tmp) + n_xi2 * xiTxidot *
          ddOmega_1);
      }
    }
  }

  if (dGamma_1 < 1.0E-7) {
    if (eta1 < 1.0E-7) {
      etaddot_[0] = -0.0;
      tmp_b = _mm_set1_pd(-0.5);
      _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd
        (LR_U.dddexp3inv_xidot[1], -LR_U.dddexp3inv_xidot[2])));
      etaddot_[3] = tmp_f[0];
      etaddot_[6] = tmp_f[1];
      etaddot_[1] = -0.5 * LR_U.dddexp3inv_xidot[2];
      etaddot_[4] = -0.0;
      _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd
        (-LR_U.dddexp3inv_xidot[1], -LR_U.dddexp3inv_xidot[0])));
      etaddot_[7] = tmp_f[0];
      etaddot_[2] = tmp_f[1];
      etaddot_[5] = -0.5 * LR_U.dddexp3inv_xidot[0];
      etaddot_[8] = -0.0;
    } else {
      ceil_xi[0] = 0.0;
      ceil_xi[3] = -LR_U.dddexp3inv_xi[2];
      ceil_xi[6] = LR_U.dddexp3inv_xi[1];
      ceil_xi[1] = LR_U.dddexp3inv_xi[2];
      ceil_xi[4] = 0.0;
      ceil_xi[7] = -LR_U.dddexp3inv_xi[0];
      ceil_xi[2] = -LR_U.dddexp3inv_xi[1];
      ceil_xi[5] = LR_U.dddexp3inv_xi[0];
      ceil_xi[8] = 0.0;
      dddexp3__tmp = (0.0027777777777777779 * LR_U.dddexp3inv_xi[0] *
                      LR_U.dddexp3inv_xidot[0] + 0.0027777777777777779 *
                      LR_U.dddexp3inv_xi[1] * LR_U.dddexp3inv_xidot[1]) +
        0.0027777777777777779 * LR_U.dddexp3inv_xi[2] * LR_U.dddexp3inv_xidot[2];
      ceil_xi_0[0] = -0.0;
      tmp_b = _mm_set1_pd(-0.5);
      _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd
        (LR_U.dddexp3inv_xidot[1], -LR_U.dddexp3inv_xidot[2])));
      ceil_xi_0[3] = tmp_f[0];
      ceil_xi_0[6] = tmp_f[1];
      ceil_xi_0[1] = -0.5 * LR_U.dddexp3inv_xidot[2];
      ceil_xi_0[4] = -0.0;
      _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd
        (-LR_U.dddexp3inv_xidot[1], -LR_U.dddexp3inv_xidot[0])));
      ceil_xi_0[7] = tmp_f[0];
      ceil_xi_0[2] = tmp_f[1];
      ceil_xi_0[5] = -0.5 * LR_U.dddexp3inv_xidot[0];
      ceil_xi_0[8] = -0.0;
      for (i = 0; i < 3; i++) {
        for (e_i = 0; e_i <= 0; e_i += 2) {
          tmp_b = _mm_loadu_pd(&ceil_xi[e_i + 3]);
          tmp_c = _mm_set1_pd(0.083333333333333329);
          tmp_d = _mm_loadu_pd(&ceil_xi[e_i]);
          tmp_e = _mm_loadu_pd(&ceil_xi[e_i + 6]);
          _mm_storeu_pd(&Cxi[e_i + 3 * i], _mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_mul_pd(tmp_b, tmp_c), _mm_set1_pd(ceil_xi[3 * i + 1])),
            _mm_mul_pd(_mm_mul_pd(tmp_c, tmp_d), _mm_set1_pd(ceil_xi[3 * i]))),
            _mm_mul_pd(_mm_mul_pd(tmp_e, tmp_c), _mm_set1_pd(ceil_xi[3 * i + 2]))));
        }

        for (e_i = 2; e_i < 3; e_i++) {
          Cxi[e_i + 3 * i] = (ceil_xi[e_i + 3] * 0.083333333333333329 * ceil_xi
                              [3 * i + 1] + 0.083333333333333329 * ceil_xi[e_i] *
                              ceil_xi[3 * i]) + ceil_xi[e_i + 6] *
            0.083333333333333329 * ceil_xi[3 * i + 2];
        }
      }

      for (i = 0; i <= 6; i += 2) {
        tmp_b = _mm_loadu_pd(&ceil_xi_0[i]);
        tmp_c = _mm_loadu_pd(&Cxi[i]);
        _mm_storeu_pd(&etaddot_[i], _mm_sub_pd(_mm_add_pd(tmp_b, tmp_c),
          _mm_set1_pd(dddexp3__tmp)));
      }

      for (i = 8; i < 9; i++) {
        etaddot_[i] = (ceil_xi_0[i] + Cxi[i]) - dddexp3__tmp;
      }
    }
  } else {
    beta = eta1 / 2.0;
    b_s = sin(beta) / beta;
    Gamma_1 = cos(beta) / b_s;
    beta = eta1 * eta1;
    ceil_xi[0] = 0.0;
    ceil_xi[3] = -LR_U.dddexp3inv_xi[2];
    ceil_xi[6] = LR_U.dddexp3inv_xi[1];
    ceil_xi[1] = LR_U.dddexp3inv_xi[2];
    ceil_xi[4] = 0.0;
    ceil_xi[7] = -LR_U.dddexp3inv_xi[0];
    ceil_xi[2] = -LR_U.dddexp3inv_xi[1];
    ceil_xi[5] = LR_U.dddexp3inv_xi[0];
    ceil_xi[8] = 0.0;
    AA[0] = 0.0;
    AA[3] = -LR_U.dddexp3inv_xidot[2];
    AA[6] = LR_U.dddexp3inv_xidot[1];
    AA[1] = LR_U.dddexp3inv_xidot[2];
    AA[4] = 0.0;
    AA[7] = -LR_U.dddexp3inv_xidot[0];
    AA[2] = -LR_U.dddexp3inv_xidot[1];
    AA[5] = LR_U.dddexp3inv_xidot[0];
    AA[8] = 0.0;
    n_xi2 = (1.0 - Gamma_1) / beta;
    ddOmega_1 = ((1.0 / (b_s * b_s) + Gamma_1) - 2.0) * (1.0 / beta) / beta *
      ((LR_U.dddexp3inv_xi[0] * LR_U.dddexp3inv_xidot[0] + LR_U.dddexp3inv_xi[1]
        * LR_U.dddexp3inv_xidot[1]) + LR_U.dddexp3inv_xi[2] *
       LR_U.dddexp3inv_xidot[2]);
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + 1;
        e_j = 3 * e_i + 2;
        n_xi6 = ceil_xi[i + 3];
        b_s = ceil_xi[f_i];
        dddexp3__tmp = ceil_xi[3 * e_i];
        dGamma_1 = ceil_xi[i + 6];
        dddexp3__tmp_0 = ceil_xi[e_j];
        A6_tmp = 3 * e_i + i;
        etaddot_[A6_tmp] = ((((AA[3 * e_i] * ceil_xi[i] + AA[f_i] * n_xi6) +
                              AA[e_j] * dGamma_1) + ((AA[i + 3] * b_s +
          dddexp3__tmp * AA[i]) + AA[i + 6] * dddexp3__tmp_0)) * n_xi2 +
                            AA[A6_tmp] * -0.5) + ((n_xi6 * ddOmega_1 * b_s +
          ddOmega_1 * ceil_xi[i] * dddexp3__tmp) + dGamma_1 * ddOmega_1 *
          dddexp3__tmp_0);
      }
    }
  }

  AA[0] = 0.0;
  AA[3] = -LR_U.dddexp3inv_xi[2];
  AA[6] = LR_U.dddexp3inv_xi[1];
  AA[1] = LR_U.dddexp3inv_xi[2];
  AA[4] = 0.0;
  AA[7] = -LR_U.dddexp3inv_xi[0];
  AA[2] = -LR_U.dddexp3inv_xi[1];
  AA[5] = LR_U.dddexp3inv_xi[0];
  AA[8] = 0.0;
  ceil_xi[0] = 0.0;
  ceil_xi[3] = -LR_U.dddexp3inv_xidot[2];
  ceil_xi[6] = LR_U.dddexp3inv_xidot[1];
  ceil_xi[1] = LR_U.dddexp3inv_xidot[2];
  ceil_xi[4] = 0.0;
  ceil_xi[7] = -LR_U.dddexp3inv_xidot[0];
  ceil_xi[2] = -LR_U.dddexp3inv_xidot[1];
  ceil_xi[5] = LR_U.dddexp3inv_xidot[0];
  ceil_xi[8] = 0.0;
  Cxi[0] = 0.0;
  Cxi[3] = -LR_U.dddexp3inv_xiddot[2];
  Cxi[6] = LR_U.dddexp3inv_xiddot[1];
  Cxi[1] = LR_U.dddexp3inv_xiddot[2];
  Cxi[4] = 0.0;
  Cxi[7] = -LR_U.dddexp3inv_xiddot[0];
  Cxi[2] = -LR_U.dddexp3inv_xiddot[1];
  Cxi[5] = LR_U.dddexp3inv_xiddot[0];
  Cxi[8] = 0.0;
  n_xi2 = eta1 * eta1;
  xiTxidot = rt_powd_snf(eta1, 4.0);
  n_xi5 = rt_powd_snf(eta1, 5.0);
  n_xi6 = rt_powd_snf(eta1, 6.0);
  cx = cos(eta1);
  dddexp3__tmp = (LR_U.dddexp3inv_xi[0] * LR_U.dddexp3inv_xidot[0] +
                  LR_U.dddexp3inv_xi[1] * LR_U.dddexp3inv_xidot[1]) +
    LR_U.dddexp3inv_xi[2] * LR_U.dddexp3inv_xidot[2];
  dGamma_1 = ((LR_U.dddexp3inv_xidot[0] * LR_U.dddexp3inv_xidot[0] +
               LR_U.dddexp3inv_xidot[1] * LR_U.dddexp3inv_xidot[1]) +
              LR_U.dddexp3inv_xidot[2] * LR_U.dddexp3inv_xidot[2]) +
    ((LR_U.dddexp3inv_xi[0] * LR_U.dddexp3inv_xiddot[0] + LR_U.dddexp3inv_xi[1] *
      LR_U.dddexp3inv_xiddot[1]) + LR_U.dddexp3inv_xi[2] *
     LR_U.dddexp3inv_xiddot[2]);
  b_s_tmp = d6 / rt_powd_snf(eta1, 3.0);
  b_s = (2.0 * cx / xiTxidot + -2.0 / xiTxidot) + b_s_tmp;
  ddOmega_1_tmp = cx / xiTxidot;
  dddexp3__tmp_0 = dddexp3__tmp * dddexp3__tmp;
  ddOmega_1 = (((8.0 / n_xi6 - 8.0 / n_xi6 * cx) - 5.0 * d6 / n_xi5) +
               ddOmega_1_tmp) * dddexp3__tmp_0 + b_s * dGamma_1;
  xiTxidot = (3.0 / n_xi5 * d6 + -2.0 / xiTxidot) - ddOmega_1_tmp;
  n_xi5 = (((8.0 / n_xi6 - 15.0 * d6 / rt_powd_snf(eta1, 7.0)) + 7.0 * cx /
            n_xi6) + d6 / n_xi5) * dddexp3__tmp_0 + xiTxidot * dGamma_1;
  if (fabs(eta1) < 0.0001) {
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + i;
        ddBA[A6_tmp] = ((ceil_xi[i + 3] * 0.33333333333333331 * ceil_xi[3 * e_i
                         + 1] + 0.33333333333333331 * ceil_xi[i] * ceil_xi[3 *
                         e_i]) + ceil_xi[i + 6] * 0.33333333333333331 * ceil_xi
                        [3 * e_i + 2]) + Cxi[A6_tmp] * 0.5;
      }
    }
  } else {
    xixidot = b_s * dddexp3__tmp * 2.0;
    dGamma_1 = 1.0 / n_xi2 - cx / n_xi2;
    alpha = xiTxidot * dddexp3__tmp * 2.0;
    n_xi2 = 1.0 / n_xi2 - b_s_tmp;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + 1;
        f_i = 3 * e_i + 2;
        e_j = 3 * e_i + i;
        eta1 = AA[i + 3];
        d6 = AA[A6_tmp];
        n_xi6 = AA[3 * e_i];
        b_s = AA[i + 6];
        dddexp3__tmp = AA[f_i];
        dddexp3__tmp_0 = ceil_xi[i + 3];
        ddOmega_1_tmp = ceil_xi[A6_tmp];
        Gamma_2 = ceil_xi[3 * e_i];
        dddexp3__tmp_1 = ceil_xi[i + 6];
        Gamma_1 = ceil_xi[f_i];
        ddBA[e_j] = ((((Cxi[i + 3] * d6 + n_xi6 * Cxi[i]) + Cxi[i + 6] *
                       dddexp3__tmp) + ((dddexp3__tmp_0 * 2.0 * ddOmega_1_tmp +
          2.0 * ceil_xi[i] * Gamma_2) + dddexp3__tmp_1 * 2.0 * Gamma_1)) +
                     ((Cxi[3 * e_i] * AA[i] + Cxi[A6_tmp] * eta1) + Cxi[f_i] *
                      b_s)) * n_xi2 + ((((d6 * dddexp3__tmp_0 + n_xi6 *
          ceil_xi[i]) + dddexp3__tmp * dddexp3__tmp_1) + ((ddOmega_1_tmp * eta1
          + Gamma_2 * AA[i]) + Gamma_1 * b_s)) * alpha + (((eta1 * n_xi5 * d6 +
          n_xi5 * AA[i] * n_xi6) + b_s * n_xi5 * dddexp3__tmp) + ((AA[e_j] *
          ddOmega_1 + ceil_xi[e_j] * xixidot) + Cxi[e_j] * dGamma_1)));
      }
    }
  }

  for (i = 0; i < 3; i++) {
    dddexp3__tmp_1 = B[i];
    tmp = B[i + 3];
    beta = B[i + 6];
    for (e_i = 0; e_i < 3; e_i++) {
      A6_tmp = 3 * e_i + 1;
      f_i = 3 * e_i + 2;
      e_j = 3 * e_i + i;
      B_0[e_j] = (ddBA[3 * e_i] * dddexp3__tmp_1 + ddBA[A6_tmp] * tmp) +
        ddBA[f_i] * beta;
      ceil_xi_0[e_j] = (-2.0 * tmp * Bdot[A6_tmp] + -2.0 * dddexp3__tmp_1 *
                        Bdot[3 * e_i]) + -2.0 * beta * Bdot[f_i];
    }

    dddexp3__tmp_1 = B_0[i + 3];
    tmp = B_0[i];
    beta = B_0[i + 6];
    dddexp3__tmp = ceil_xi_0[i + 3];
    dddexp3__tmp_0 = ceil_xi_0[i];
    ddOmega_1_tmp = ceil_xi_0[i + 6];
    for (e_i = 0; e_i < 3; e_i++) {
      A6_tmp = 3 * e_i + 1;
      dAB_tmp = 3 * e_i + 2;
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (etaddot_[A6_tmp], B[A6_tmp]), _mm_set_pd(dddexp3__tmp, dddexp3__tmp_1)),
        _mm_mul_pd(_mm_set_pd(etaddot_[3 * e_i], B[3 * e_i]), _mm_set_pd
                   (dddexp3__tmp_0, tmp))), _mm_mul_pd(_mm_set_pd
        (etaddot_[dAB_tmp], B[dAB_tmp]), _mm_set_pd(ddOmega_1_tmp, beta))));
      f_i = 3 * e_i + i;
      ceil_xi[f_i] = tmp_f[0];
      Cxi[f_i] = tmp_f[1];
    }
  }

  /* Outport: '<Root>/dddexp3inv' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function10'
   */
  for (i = 0; i <= 6; i += 2) {
    /* MATLAB Function: '<Root>/MATLAB Function10' */
    tmp_b = _mm_loadu_pd(&Cxi[i]);
    tmp_c = _mm_loadu_pd(&ceil_xi[i]);
    _mm_storeu_pd(&LR_Y.dddexp3inv[i], _mm_sub_pd(tmp_b, tmp_c));
  }

  for (i = 8; i < 9; i++) {
    LR_Y.dddexp3inv[i] = Cxi[i] - ceil_xi[i];
  }

  /* End of Outport: '<Root>/dddexp3inv' */

  /* MATLAB Function: '<Root>/MATLAB Function11' incorporates:
   *  Inport: '<Root>/dexp6inv_lambda'
   */
  eta1 = LR_norm_e(&LR_U.dexp6inv_lambda[3]);
  ddOmega_1_tmp = sin(eta1 / 2.0) / (eta1 / 2.0);
  beta = ddOmega_1_tmp * ddOmega_1_tmp;
  xixidot = cos(eta1 / 2.0) * ddOmega_1_tmp / beta;
  if (eta1 < 2.2204460492503131E-16) {
    memset(&ceil_xi[0], 0, 9U * sizeof(real_T));
    ceil_xi[0] = 1.0;
    ceil_xi[4] = 1.0;
    ceil_xi[8] = 1.0;
  } else {
    xiTxidot = (1.0 - xixidot) / (eta1 * eta1);
    B[0] = 0.0;
    B[3] = -LR_U.dexp6inv_lambda[5];
    B[6] = LR_U.dexp6inv_lambda[4];
    B[1] = LR_U.dexp6inv_lambda[5];
    B[4] = 0.0;
    B[7] = -LR_U.dexp6inv_lambda[3];
    B[2] = -LR_U.dexp6inv_lambda[4];
    B[5] = LR_U.dexp6inv_lambda[3];
    B[8] = 0.0;
    for (i = 0; i < 9; i++) {
      b_I[i] = 0;
    }

    for (e_j = 0; e_j < 3; e_j++) {
      b_I[e_j + 3 * e_j] = 1;
    }

    for (i = 0; i < 3; i++) {
      for (e_j = 0; e_j <= 0; e_j += 2) {
        tmp_b = _mm_loadu_pd(&B[e_j + 3]);
        tmp_c = _mm_loadu_pd(&B[e_j]);
        tmp_d = _mm_loadu_pd(&B[e_j + 6]);
        _mm_storeu_pd(&B_0[e_j + 3 * i], _mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(B[3 * i + 1]), tmp_b), _mm_mul_pd(_mm_set1_pd(B[3 * i]),
          tmp_c)), _mm_mul_pd(_mm_set1_pd(B[3 * i + 2]), tmp_d)));
      }

      for (e_j = 2; e_j < 3; e_j++) {
        B_0[e_j + 3 * i] = (B[3 * i + 1] * B[e_j + 3] + B[3 * i] * B[e_j]) + B[3
          * i + 2] * B[e_j + 6];
      }
    }

    B[0] = b_I[0];
    tmp_b = _mm_set1_pd(0.5);
    _mm_storeu_pd(&B[1], _mm_sub_pd(_mm_set_pd(b_I[2], b_I[1]), _mm_mul_pd(tmp_b,
      _mm_set_pd(-LR_U.dexp6inv_lambda[4], LR_U.dexp6inv_lambda[5]))));
    B[3] = (real_T)b_I[3] - 0.5 * -LR_U.dexp6inv_lambda[5];
    B[4] = b_I[4];
    _mm_storeu_pd(&B[5], _mm_sub_pd(_mm_set_pd(b_I[6], b_I[5]), _mm_mul_pd(tmp_b,
      _mm_loadu_pd(&LR_U.dexp6inv_lambda[3]))));
    B[7] = (real_T)b_I[7] - 0.5 * -LR_U.dexp6inv_lambda[3];
    B[8] = b_I[8];
    for (i = 0; i <= 6; i += 2) {
      tmp_b = _mm_loadu_pd(&B_0[i]);
      tmp_c = _mm_loadu_pd(&B[i]);
      _mm_storeu_pd(&ceil_xi[i], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(xiTxidot),
        tmp_b), tmp_c));
    }

    for (i = 8; i < 9; i++) {
      ceil_xi[i] = xiTxidot * B_0[i] + B[i];
    }
  }

  d6 = eta1 * eta1;
  b_s = (1.0 - xixidot) / d6;
  cx = ((1.0 / beta + xixidot) - 2.0) * (1.0 / d6) / d6;
  cx = (cx * LR_U.dexp6inv_lambda[3] * LR_U.dexp6inv_lambda[0] + cx *
        LR_U.dexp6inv_lambda[4] * LR_U.dexp6inv_lambda[1]) + cx *
    LR_U.dexp6inv_lambda[5] * LR_U.dexp6inv_lambda[2];
  ceil_xi_0[0] = 0.0;
  ceil_xi_0[3] = -LR_U.dexp6inv_lambda[2];
  ceil_xi_0[6] = LR_U.dexp6inv_lambda[1];
  ceil_xi_0[1] = LR_U.dexp6inv_lambda[2];
  ceil_xi_0[4] = 0.0;
  ceil_xi_0[7] = -LR_U.dexp6inv_lambda[0];
  ceil_xi_0[2] = -LR_U.dexp6inv_lambda[1];
  ceil_xi_0[5] = LR_U.dexp6inv_lambda[0];
  ceil_xi_0[8] = 0.0;
  Cxi[0] = 0.0;
  Cxi[3] = -LR_U.dexp6inv_lambda[5];
  Cxi[6] = LR_U.dexp6inv_lambda[4];
  Cxi[1] = LR_U.dexp6inv_lambda[5];
  Cxi[4] = 0.0;
  Cxi[7] = -LR_U.dexp6inv_lambda[3];
  Cxi[2] = -LR_U.dexp6inv_lambda[4];
  Cxi[5] = LR_U.dexp6inv_lambda[3];
  Cxi[8] = 0.0;
  B[0] = 0.0;
  B[3] = -LR_U.dexp6inv_lambda[5];
  B[6] = LR_U.dexp6inv_lambda[4];
  B[1] = LR_U.dexp6inv_lambda[5];
  B[4] = 0.0;
  B[7] = -LR_U.dexp6inv_lambda[3];
  B[2] = -LR_U.dexp6inv_lambda[4];
  B[5] = LR_U.dexp6inv_lambda[3];
  B[8] = 0.0;
  Bdot[0] = 0.0;
  Bdot[3] = -LR_U.dexp6inv_lambda[2];
  Bdot[6] = LR_U.dexp6inv_lambda[1];
  Bdot[1] = LR_U.dexp6inv_lambda[2];
  Bdot[4] = 0.0;
  Bdot[7] = -LR_U.dexp6inv_lambda[0];
  Bdot[2] = -LR_U.dexp6inv_lambda[1];
  Bdot[5] = LR_U.dexp6inv_lambda[0];
  Bdot[8] = 0.0;
  for (i = 0; i < 3; i++) {
    e_i = 3 * i + 1;
    dddexp3__tmp = Bdot[e_i];
    dddexp3__tmp_0 = Bdot[3 * i];
    A6_tmp = 3 * i + 2;
    ddOmega_1_tmp = Bdot[A6_tmp];
    dddexp3__tmp_1 = Cxi[e_i];
    Cxi_3 = Cxi[3 * i];
    Bdot_4 = Cxi[A6_tmp];
    for (e_i = 0; e_i <= 0; e_i += 2) {
      tmp_b = _mm_loadu_pd(&B[e_i + 3]);
      tmp_c = _mm_loadu_pd(&B[e_i]);
      tmp_d = _mm_loadu_pd(&B[e_i + 6]);
      A6_tmp = 3 * i + e_i;
      _mm_storeu_pd(&ddBA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (dddexp3__tmp), tmp_b), _mm_mul_pd(_mm_set1_pd(dddexp3__tmp_0), tmp_c)),
        _mm_mul_pd(_mm_set1_pd(ddOmega_1_tmp), tmp_d)));
      tmp_b = _mm_loadu_pd(&ceil_xi_0[e_i + 3]);
      tmp_c = _mm_loadu_pd(&ceil_xi_0[e_i]);
      tmp_d = _mm_loadu_pd(&ceil_xi_0[e_i + 6]);
      _mm_storeu_pd(&etaddot_[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(dddexp3__tmp_1), tmp_b), _mm_mul_pd(_mm_set1_pd(Cxi_3),
        tmp_c)), _mm_mul_pd(_mm_set1_pd(Bdot_4), tmp_d)));
    }

    for (e_i = 2; e_i < 3; e_i++) {
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (dddexp3__tmp_1, dddexp3__tmp), _mm_set_pd(ceil_xi_0[e_i + 3], B[e_i + 3])),
        _mm_mul_pd(_mm_set_pd(Cxi_3, dddexp3__tmp_0), _mm_set_pd(ceil_xi_0[e_i],
        B[e_i]))), _mm_mul_pd(_mm_set_pd(Bdot_4, ddOmega_1_tmp), _mm_set_pd
        (ceil_xi_0[e_i + 6], B[e_i + 6]))));
      A6_tmp = 3 * i + e_i;
      ddBA[A6_tmp] = tmp_f[0];
      etaddot_[A6_tmp] = tmp_f[1];
    }
  }

  ceil_xi_0[0] = -0.0;
  tmp_b = _mm_set1_pd(-0.5);
  tmp_c = _mm_mul_pd(tmp_b, _mm_set_pd(LR_U.dexp6inv_lambda[1],
    -LR_U.dexp6inv_lambda[2]));
  _mm_storeu_pd(&tmp_f[0], tmp_c);

  /* MATLAB Function: '<Root>/MATLAB Function11' incorporates:
   *  Inport: '<Root>/dexp6inv_lambda'
   */
  ceil_xi_0[3] = tmp_f[0];
  ceil_xi_0[6] = tmp_f[1];
  ceil_xi_0[1] = -0.5 * LR_U.dexp6inv_lambda[2];
  ceil_xi_0[4] = -0.0;
  tmp_d = _mm_mul_pd(tmp_b, _mm_set_pd(-LR_U.dexp6inv_lambda[1],
    -LR_U.dexp6inv_lambda[0]));
  _mm_storeu_pd(&tmp_f[0], tmp_d);

  /* MATLAB Function: '<Root>/MATLAB Function11' incorporates:
   *  Inport: '<Root>/dexp6inv_lambda'
   */
  ceil_xi_0[7] = tmp_f[0];
  ceil_xi_0[2] = tmp_f[1];
  ceil_xi_0[5] = -0.5 * LR_U.dexp6inv_lambda[0];
  ceil_xi_0[8] = -0.0;
  Bdot[0] = cx * 0.0;
  tmp_e = _mm_set1_pd(cx);
  _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_e, _mm_set_pd(LR_U.dexp6inv_lambda[4],
    -LR_U.dexp6inv_lambda[5])));
  Bdot[3] = tmp_f[0];
  Bdot[6] = tmp_f[1];
  Bdot[1] = cx * LR_U.dexp6inv_lambda[5];
  Bdot[4] = cx * 0.0;
  _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_e, _mm_set_pd(-LR_U.dexp6inv_lambda[4],
    -LR_U.dexp6inv_lambda[3])));
  Bdot[7] = tmp_f[0];
  Bdot[2] = tmp_f[1];
  Bdot[5] = cx * LR_U.dexp6inv_lambda[3];
  Bdot[8] = cx * 0.0;
  Cxi[0] = 0.0;
  Cxi[3] = -LR_U.dexp6inv_lambda[5];
  Cxi[6] = LR_U.dexp6inv_lambda[4];
  Cxi[1] = LR_U.dexp6inv_lambda[5];
  Cxi[4] = 0.0;
  Cxi[7] = -LR_U.dexp6inv_lambda[3];
  Cxi[2] = -LR_U.dexp6inv_lambda[4];
  Cxi[5] = LR_U.dexp6inv_lambda[3];
  Cxi[8] = 0.0;
  for (i = 0; i < 3; i++) {
    n_xi6 = Bdot[i + 3];
    d6 = Bdot[i];
    dddexp3__tmp = Bdot[i + 6];
    for (e_i = 0; e_i < 3; e_i++) {
      f_i = 3 * e_i + i;
      B[f_i] = ((Cxi[3 * e_i + 1] * n_xi6 + Cxi[3 * e_i] * d6) + Cxi[3 * e_i + 2]
                * dddexp3__tmp) + ((etaddot_[f_i] + ddBA[f_i]) * b_s +
        ceil_xi_0[f_i]);
    }
  }

  if (eta1 < 2.2204460492503131E-16) {
    B[0] = -0.0;
    _mm_storeu_pd(&tmp_f[0], tmp_c);
    B[3] = tmp_f[0];
    B[6] = tmp_f[1];
    B[1] = -0.5 * LR_U.dexp6inv_lambda[2];
    B[4] = -0.0;
    _mm_storeu_pd(&tmp_f[0], tmp_d);
    B[7] = tmp_f[0];
    B[2] = tmp_f[1];
    B[5] = -0.5 * LR_U.dexp6inv_lambda[0];
    B[8] = -0.0;
  }

  /* Outport: '<Root>/dexp6inv' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function11'
   */
  for (i = 0; i < 3; i++) {
    /* MATLAB Function: '<Root>/MATLAB Function11' */
    beta_2 = ceil_xi[3 * i];
    LR_Y.dexp6inv[6 * i] = beta_2;

    /* MATLAB Function: '<Root>/MATLAB Function11' */
    e_i = (i + 3) * 6;
    LR_Y.dexp6inv[e_i] = B[3 * i];
    LR_Y.dexp6inv[6 * i + 3] = 0.0;
    LR_Y.dexp6inv[e_i + 3] = beta_2;

    /* MATLAB Function: '<Root>/MATLAB Function11' */
    A6_tmp = 3 * i + 1;
    beta_2 = ceil_xi[A6_tmp];
    LR_Y.dexp6inv[6 * i + 1] = beta_2;
    LR_Y.dexp6inv[e_i + 1] = B[A6_tmp];
    LR_Y.dexp6inv[6 * i + 4] = 0.0;
    LR_Y.dexp6inv[e_i + 4] = beta_2;

    /* MATLAB Function: '<Root>/MATLAB Function11' */
    A6_tmp = 3 * i + 2;
    beta_2 = ceil_xi[A6_tmp];
    LR_Y.dexp6inv[6 * i + 2] = beta_2;
    LR_Y.dexp6inv[e_i + 2] = B[A6_tmp];
    LR_Y.dexp6inv[6 * i + 5] = 0.0;
    LR_Y.dexp6inv[e_i + 5] = beta_2;
  }

  /* End of Outport: '<Root>/dexp6inv' */

  /* MATLAB Function: '<Root>/MATLAB Function12' incorporates:
   *  Inport: '<Root>/ddexp6inv_lambda'
   */
  eta1 = LR_norm_e(&LR_U.ddexp6inv_lambda[3]);
  ddOmega_1_tmp = sin(eta1 / 2.0) / (eta1 / 2.0);
  n_xi2 = ddOmega_1_tmp * ddOmega_1_tmp;
  ddOmega_1 = cos(eta1 / 2.0) * ddOmega_1_tmp;
  xixidot = ddOmega_1 / n_xi2;
  if (eta1 < 2.2204460492503131E-16) {
    memset(&ceil_xi[0], 0, 9U * sizeof(real_T));
    ceil_xi[0] = 1.0;
    ceil_xi[4] = 1.0;
    ceil_xi[8] = 1.0;
  } else {
    xiTxidot = (1.0 - xixidot) / (eta1 * eta1);
    B[0] = 0.0;
    B[3] = -LR_U.ddexp6inv_lambda[5];
    B[6] = LR_U.ddexp6inv_lambda[4];
    B[1] = LR_U.ddexp6inv_lambda[5];
    B[4] = 0.0;
    B[7] = -LR_U.ddexp6inv_lambda[3];
    B[2] = -LR_U.ddexp6inv_lambda[4];
    B[5] = LR_U.ddexp6inv_lambda[3];
    B[8] = 0.0;
    memset(&Cxi[0], 0, 9U * sizeof(real_T));
    for (e_j = 0; e_j < 3; e_j++) {
      Cxi[e_j + 3 * e_j] = 1.0;
    }

    for (i = 0; i < 3; i++) {
      for (e_j = 0; e_j <= 0; e_j += 2) {
        tmp_c = _mm_loadu_pd(&B[e_j + 3]);
        tmp_d = _mm_loadu_pd(&B[e_j]);
        tmp_e = _mm_loadu_pd(&B[e_j + 6]);
        _mm_storeu_pd(&B_0[e_j + 3 * i], _mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(B[3 * i + 1]), tmp_c), _mm_mul_pd(_mm_set1_pd(B[3 * i]),
          tmp_d)), _mm_mul_pd(_mm_set1_pd(B[3 * i + 2]), tmp_e)));
      }

      for (e_j = 2; e_j < 3; e_j++) {
        B_0[e_j + 3 * i] = (B[3 * i + 1] * B[e_j + 3] + B[3 * i] * B[e_j]) + B[3
          * i + 2] * B[e_j + 6];
      }
    }

    Cxi_0[0] = Cxi[0];
    tmp_c = _mm_set1_pd(0.5);
    tmp_d = _mm_sub_pd(_mm_loadu_pd(&Cxi[1]), _mm_mul_pd(tmp_c, _mm_set_pd
      (-LR_U.ddexp6inv_lambda[4], LR_U.ddexp6inv_lambda[5])));
    _mm_storeu_pd(&Cxi_0[1], tmp_d);
    Cxi_0[3] = Cxi[3] - 0.5 * -LR_U.ddexp6inv_lambda[5];
    Cxi_0[4] = Cxi[4];
    tmp_c = _mm_sub_pd(_mm_loadu_pd(&Cxi[5]), _mm_mul_pd(tmp_c, _mm_loadu_pd
      (&LR_U.ddexp6inv_lambda[3])));
    _mm_storeu_pd(&Cxi_0[5], tmp_c);
    Cxi_0[7] = Cxi[7] - 0.5 * -LR_U.ddexp6inv_lambda[3];
    Cxi_0[8] = Cxi[8];
    for (i = 0; i <= 6; i += 2) {
      tmp_c = _mm_loadu_pd(&B_0[i]);
      tmp_d = _mm_loadu_pd(&Cxi_0[i]);
      _mm_storeu_pd(&ceil_xi[i], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(xiTxidot),
        tmp_c), tmp_d));
    }

    for (i = 8; i < 9; i++) {
      ceil_xi[i] = xiTxidot * B_0[i] + Cxi_0[i];
    }
  }

  xiTxidot = eta1 * eta1;
  b_s = (1.0 - xixidot) / xiTxidot;
  cx = ((1.0 / n_xi2 + xixidot) - 2.0) * (1.0 / xiTxidot) / xiTxidot;
  cx = (cx * LR_U.ddexp6inv_lambda[3] * LR_U.ddexp6inv_lambda[0] + cx *
        LR_U.ddexp6inv_lambda[4] * LR_U.ddexp6inv_lambda[1]) + cx *
    LR_U.ddexp6inv_lambda[5] * LR_U.ddexp6inv_lambda[2];
  ceil_xi_0[0] = 0.0;
  ceil_xi_0[3] = -LR_U.ddexp6inv_lambda[5];
  ceil_xi_0[6] = LR_U.ddexp6inv_lambda[4];
  ceil_xi_0[1] = LR_U.ddexp6inv_lambda[5];
  ceil_xi_0[4] = 0.0;
  ceil_xi_0[7] = -LR_U.ddexp6inv_lambda[3];
  ceil_xi_0[2] = -LR_U.ddexp6inv_lambda[4];
  ceil_xi_0[5] = LR_U.ddexp6inv_lambda[3];
  ceil_xi_0[8] = 0.0;
  Cxi[0] = 0.0;
  Cxi[3] = -LR_U.ddexp6inv_lambda[2];
  Cxi[6] = LR_U.ddexp6inv_lambda[1];
  Cxi[1] = LR_U.ddexp6inv_lambda[2];
  Cxi[4] = 0.0;
  Cxi[7] = -LR_U.ddexp6inv_lambda[0];
  Cxi[2] = -LR_U.ddexp6inv_lambda[1];
  Cxi[5] = LR_U.ddexp6inv_lambda[0];
  Cxi[8] = 0.0;
  B[0] = 0.0;
  B[3] = -LR_U.ddexp6inv_lambda[2];
  B[6] = LR_U.ddexp6inv_lambda[1];
  B[1] = LR_U.ddexp6inv_lambda[2];
  B[4] = 0.0;
  B[7] = -LR_U.ddexp6inv_lambda[0];
  B[2] = -LR_U.ddexp6inv_lambda[1];
  B[5] = LR_U.ddexp6inv_lambda[0];
  B[8] = 0.0;
  Bdot[0] = 0.0;
  Bdot[3] = -LR_U.ddexp6inv_lambda[5];
  Bdot[6] = LR_U.ddexp6inv_lambda[4];
  Bdot[1] = LR_U.ddexp6inv_lambda[5];
  Bdot[4] = 0.0;
  Bdot[7] = -LR_U.ddexp6inv_lambda[3];
  Bdot[2] = -LR_U.ddexp6inv_lambda[4];
  Bdot[5] = LR_U.ddexp6inv_lambda[3];
  Bdot[8] = 0.0;
  for (i = 0; i < 3; i++) {
    e_i = 3 * i + 1;
    dddexp3__tmp = Bdot[e_i];
    dddexp3__tmp_0 = Bdot[3 * i];
    A6_tmp = 3 * i + 2;
    ddOmega_1_tmp = Bdot[A6_tmp];
    dddexp3__tmp_1 = Cxi[e_i];
    Cxi_3 = Cxi[3 * i];
    Bdot_4 = Cxi[A6_tmp];
    for (e_i = 0; e_i <= 0; e_i += 2) {
      tmp_c = _mm_loadu_pd(&B[e_i + 3]);
      tmp_d = _mm_loadu_pd(&B[e_i]);
      tmp_e = _mm_loadu_pd(&B[e_i + 6]);
      A6_tmp = 3 * i + e_i;
      _mm_storeu_pd(&ddBA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (dddexp3__tmp), tmp_c), _mm_mul_pd(_mm_set1_pd(dddexp3__tmp_0), tmp_d)),
        _mm_mul_pd(_mm_set1_pd(ddOmega_1_tmp), tmp_e)));
      tmp_c = _mm_loadu_pd(&ceil_xi_0[e_i + 3]);
      tmp_d = _mm_loadu_pd(&ceil_xi_0[e_i]);
      tmp_e = _mm_loadu_pd(&ceil_xi_0[e_i + 6]);
      _mm_storeu_pd(&etaddot_[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(dddexp3__tmp_1), tmp_c), _mm_mul_pd(_mm_set1_pd(Cxi_3),
        tmp_d)), _mm_mul_pd(_mm_set1_pd(Bdot_4), tmp_e)));
    }

    for (e_i = 2; e_i < 3; e_i++) {
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (dddexp3__tmp_1, dddexp3__tmp), _mm_set_pd(ceil_xi_0[e_i + 3], B[e_i + 3])),
        _mm_mul_pd(_mm_set_pd(Cxi_3, dddexp3__tmp_0), _mm_set_pd(ceil_xi_0[e_i],
        B[e_i]))), _mm_mul_pd(_mm_set_pd(Bdot_4, ddOmega_1_tmp), _mm_set_pd
        (ceil_xi_0[e_i + 6], B[e_i + 6]))));
      A6_tmp = 3 * i + e_i;
      ddBA[A6_tmp] = tmp_f[0];
      etaddot_[A6_tmp] = tmp_f[1];
    }
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_c = _mm_loadu_pd(&ddBA[i]);
    tmp_d = _mm_loadu_pd(&etaddot_[i]);
    _mm_storeu_pd(&AB[i], _mm_add_pd(tmp_c, tmp_d));
  }

  for (i = 8; i < 9; i++) {
    AB[i] = ddBA[i] + etaddot_[i];
  }

  ceil_xi_0[0] = -0.0;
  tmp_c = _mm_mul_pd(tmp_b, _mm_set_pd(LR_U.ddexp6inv_lambda[1],
    -LR_U.ddexp6inv_lambda[2]));
  _mm_storeu_pd(&tmp_f[0], tmp_c);

  /* MATLAB Function: '<Root>/MATLAB Function12' incorporates:
   *  Inport: '<Root>/ddexp6inv_lambda'
   */
  ceil_xi_0[3] = tmp_f[0];
  ceil_xi_0[6] = tmp_f[1];
  ceil_xi_0[1] = -0.5 * LR_U.ddexp6inv_lambda[2];
  ceil_xi_0[4] = -0.0;
  tmp_d = _mm_mul_pd(tmp_b, _mm_set_pd(-LR_U.ddexp6inv_lambda[1],
    -LR_U.ddexp6inv_lambda[0]));
  _mm_storeu_pd(&tmp_f[0], tmp_d);

  /* MATLAB Function: '<Root>/MATLAB Function12' incorporates:
   *  Inport: '<Root>/ddexp6inv_lambda'
   *  Inport: '<Root>/ddexp6inv_lambdadot'
   */
  ceil_xi_0[7] = tmp_f[0];
  ceil_xi_0[2] = tmp_f[1];
  ceil_xi_0[5] = -0.5 * LR_U.ddexp6inv_lambda[0];
  ceil_xi_0[8] = -0.0;
  Bdot[0] = cx * 0.0;
  tmp_e = _mm_set1_pd(cx);
  _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_e, _mm_set_pd(LR_U.ddexp6inv_lambda[4],
    -LR_U.ddexp6inv_lambda[5])));
  Bdot[3] = tmp_f[0];
  Bdot[6] = tmp_f[1];
  Bdot[1] = cx * LR_U.ddexp6inv_lambda[5];
  Bdot[4] = cx * 0.0;
  _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_e, _mm_set_pd(-LR_U.ddexp6inv_lambda[4],
    -LR_U.ddexp6inv_lambda[3])));
  Bdot[7] = tmp_f[0];
  Bdot[2] = tmp_f[1];
  Bdot[5] = cx * LR_U.ddexp6inv_lambda[3];
  Bdot[8] = cx * 0.0;
  Cxi[0] = 0.0;
  Cxi[3] = -LR_U.ddexp6inv_lambda[5];
  Cxi[6] = LR_U.ddexp6inv_lambda[4];
  Cxi[1] = LR_U.ddexp6inv_lambda[5];
  Cxi[4] = 0.0;
  Cxi[7] = -LR_U.ddexp6inv_lambda[3];
  Cxi[2] = -LR_U.ddexp6inv_lambda[4];
  Cxi[5] = LR_U.ddexp6inv_lambda[3];
  Cxi[8] = 0.0;
  for (i = 0; i < 3; i++) {
    n_xi6 = Bdot[i + 3];
    d6 = Bdot[i];
    dddexp3__tmp = Bdot[i + 6];
    for (e_i = 0; e_i < 3; e_i++) {
      f_i = 3 * e_i + i;
      B[f_i] = ((Cxi[3 * e_i + 1] * n_xi6 + Cxi[3 * e_i] * d6) + Cxi[3 * e_i + 2]
                * dddexp3__tmp) + (AB[f_i] * b_s + ceil_xi_0[f_i]);
    }
  }

  if (eta1 < 2.2204460492503131E-16) {
    B[0] = -0.0;
    _mm_storeu_pd(&tmp_f[0], tmp_c);
    B[3] = tmp_f[0];
    B[6] = tmp_f[1];
    B[1] = -0.5 * LR_U.ddexp6inv_lambda[2];
    B[4] = -0.0;
    _mm_storeu_pd(&tmp_f[0], tmp_d);
    B[7] = tmp_f[0];
    B[2] = tmp_f[1];
    B[5] = -0.5 * LR_U.ddexp6inv_lambda[0];
    B[8] = -0.0;
  }

  for (i = 0; i < 3; i++) {
    beta_2 = ceil_xi[3 * i];
    dlog6mat[6 * i] = beta_2;
    f_i = (i + 3) * 6;
    dlog6mat[f_i] = B[3 * i];
    dlog6mat[6 * i + 3] = 0.0;
    dlog6mat[f_i + 3] = beta_2;
    A6_tmp = 3 * i + 1;
    beta_2 = ceil_xi[A6_tmp];
    dlog6mat[6 * i + 1] = beta_2;
    dlog6mat[f_i + 1] = B[A6_tmp];
    dlog6mat[6 * i + 4] = 0.0;
    dlog6mat[f_i + 4] = beta_2;
    A6_tmp = 3 * i + 2;
    beta_2 = ceil_xi[A6_tmp];
    dlog6mat[6 * i + 2] = beta_2;
    dlog6mat[f_i + 2] = B[A6_tmp];
    dlog6mat[6 * i + 5] = 0.0;
    dlog6mat[f_i + 5] = beta_2;
  }

  ceil_xi[0] = 0.0;
  ceil_xi[3] = -LR_U.ddexp6inv_lambda[5];
  ceil_xi[6] = LR_U.ddexp6inv_lambda[4];
  ceil_xi[1] = LR_U.ddexp6inv_lambda[5];
  ceil_xi[4] = 0.0;
  ceil_xi[7] = -LR_U.ddexp6inv_lambda[3];
  ceil_xi[2] = -LR_U.ddexp6inv_lambda[4];
  ceil_xi[5] = LR_U.ddexp6inv_lambda[3];
  ceil_xi[8] = 0.0;
  B[0] = 0.0;
  B[3] = -LR_U.ddexp6inv_lambda[2];
  B[6] = LR_U.ddexp6inv_lambda[1];
  B[1] = LR_U.ddexp6inv_lambda[2];
  B[4] = 0.0;
  B[7] = -LR_U.ddexp6inv_lambda[0];
  B[2] = -LR_U.ddexp6inv_lambda[1];
  B[5] = LR_U.ddexp6inv_lambda[0];
  B[8] = 0.0;
  Cxi[0] = 0.0;
  Cxi[3] = -LR_U.ddexp6inv_lambdadot[5];
  Cxi[6] = LR_U.ddexp6inv_lambdadot[4];
  Cxi[1] = LR_U.ddexp6inv_lambdadot[5];
  Cxi[4] = 0.0;
  Cxi[7] = -LR_U.ddexp6inv_lambdadot[3];
  Cxi[2] = -LR_U.ddexp6inv_lambdadot[4];
  Cxi[5] = LR_U.ddexp6inv_lambdadot[3];
  Cxi[8] = 0.0;
  Bdot[0] = 0.0;
  Bdot[3] = -LR_U.ddexp6inv_lambdadot[2];
  Bdot[6] = LR_U.ddexp6inv_lambdadot[1];
  Bdot[1] = LR_U.ddexp6inv_lambdadot[2];
  Bdot[4] = 0.0;
  Bdot[7] = -LR_U.ddexp6inv_lambdadot[0];
  Bdot[2] = -LR_U.ddexp6inv_lambdadot[1];
  Bdot[5] = LR_U.ddexp6inv_lambdadot[0];
  Bdot[8] = 0.0;
  for (e_j = 0; e_j < 6; e_j++) {
    xn[e_j] = rt_powd_snf(eta1, (((real_T)e_j + 1.0) - 1.0) + 1.0);
  }

  d6 = sin(eta1);
  cx = cos(eta1);
  Gamma_1 = (eta1 - d6) / xn[2];
  Gamma_2 = ((2.0 * cx + xn[1]) - 2.0) / (2.0 * xn[3]);
  tmp_c = _mm_div_pd(_mm_set_pd(-(((4.0 * cx + eta1 * d6) + xn[1]) - 4.0),
    -((2.0 * eta1 - 3.0 * d6) + eta1 * cx)), _mm_loadu_pd(&xn[3]));
  _mm_storeu_pd(&tmp_f[0], tmp_c);

  /* MATLAB Function: '<Root>/MATLAB Function12' incorporates:
   *  Inport: '<Root>/ddexp6inv_lambda'
   *  Inport: '<Root>/ddexp6inv_lambdadot'
   */
  dGamma_1 = tmp_f[0];
  dGamma_2 = tmp_f[1];
  Gamma_3 = -tmp_f[0] / eta1;
  Gamma_4 = tmp_f[1] * eta1 + Gamma_2;
  for (i = 0; i < 3; i++) {
    f_i = 3 * i + 1;
    Bdot_3 = Bdot[f_i];
    Bdot_4 = Bdot[3 * i];
    e_j = 3 * i + 2;
    Bdot_5 = Bdot[e_j];
    dddexp3__tmp_1 = B[f_i];
    tmp = B[3 * i];
    beta = B[e_j];
    for (e_i = 0; e_i <= 0; e_i += 2) {
      tmp_c = _mm_loadu_pd(&ceil_xi[e_i + 3]);
      tmp_d = _mm_loadu_pd(&ceil_xi[e_i]);
      tmp_e = _mm_loadu_pd(&ceil_xi[e_i + 6]);
      A6_tmp = 3 * i + e_i;
      _mm_storeu_pd(&ceil_xi_0[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(Bdot_3), tmp_c), _mm_mul_pd(_mm_set1_pd(Bdot_4), tmp_d)),
        _mm_mul_pd(_mm_set1_pd(Bdot_5), tmp_e)));
      tmp_c = _mm_loadu_pd(&Cxi[e_i + 3]);
      tmp_d = _mm_loadu_pd(&Cxi[e_i]);
      tmp_e = _mm_loadu_pd(&Cxi[e_i + 6]);
      _mm_storeu_pd(&Cxi_0[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (dddexp3__tmp_1), tmp_c), _mm_mul_pd(_mm_set1_pd(tmp), tmp_d)),
        _mm_mul_pd(_mm_set1_pd(beta), tmp_e)));
      tmp_c = _mm_loadu_pd(&ceil_xi[e_i + 3]);
      tmp_d = _mm_loadu_pd(&ceil_xi[e_i]);
      tmp_e = _mm_loadu_pd(&ceil_xi[e_i + 6]);
      _mm_storeu_pd(&AA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (ceil_xi[f_i]), tmp_c), _mm_mul_pd(_mm_set1_pd(ceil_xi[3 * i]), tmp_d)),
        _mm_mul_pd(_mm_set1_pd(ceil_xi[e_j]), tmp_e)));
    }

    for (e_i = 2; e_i < 3; e_i++) {
      dddexp3__tmp = ceil_xi[e_i + 3];
      dddexp3__tmp_0 = ceil_xi[e_i + 6];
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (dddexp3__tmp_1, Bdot_3), _mm_set_pd(Cxi[e_i + 3], dddexp3__tmp)),
        _mm_mul_pd(_mm_set_pd(tmp, Bdot_4), _mm_set_pd(Cxi[e_i], ceil_xi[e_i]))),
        _mm_mul_pd(_mm_set_pd(beta, Bdot_5), _mm_set_pd(Cxi[e_i + 6],
        dddexp3__tmp_0))));
      A6_tmp = 3 * i + e_i;
      ceil_xi_0[A6_tmp] = tmp_f[0];
      Cxi_0[A6_tmp] = tmp_f[1];
      AA[A6_tmp] = (ceil_xi[3 * i] * ceil_xi[e_i] + ceil_xi[f_i] * dddexp3__tmp)
        + ceil_xi[e_j] * dddexp3__tmp_0;
    }
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_c = _mm_loadu_pd(&Cxi_0[i]);
    tmp_d = _mm_loadu_pd(&ceil_xi_0[i]);
    _mm_storeu_pd(&dAB[i], _mm_add_pd(tmp_c, tmp_d));
  }

  for (i = 8; i < 9; i++) {
    dAB[i] = Cxi_0[i] + ceil_xi_0[i];
  }

  for (i = 0; i < 3; i++) {
    A6_tmp = 3 * i + 1;
    tmp = Cxi[A6_tmp];
    Cxi_2 = Cxi[3 * i];
    f_i = 3 * i + 2;
    Cxi_3 = Cxi[f_i];
    beta_2 = ceil_xi[A6_tmp];
    ceil_xi_2 = ceil_xi[3 * i];
    n_xi6 = ceil_xi[f_i];
    for (e_i = 0; e_i <= 0; e_i += 2) {
      tmp_c = _mm_loadu_pd(&B[e_i + 3]);
      tmp_d = _mm_loadu_pd(&B[e_i]);
      tmp_e = _mm_loadu_pd(&B[e_i + 6]);
      A6_tmp = 3 * i + e_i;
      _mm_storeu_pd(&B_0[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (tmp), tmp_c), _mm_mul_pd(_mm_set1_pd(Cxi_2), tmp_d)), _mm_mul_pd
        (_mm_set1_pd(Cxi_3), tmp_e)));
      tmp_c = _mm_loadu_pd(&Bdot[e_i + 3]);
      tmp_d = _mm_loadu_pd(&Bdot[e_i]);
      tmp_e = _mm_loadu_pd(&Bdot[e_i + 6]);
      _mm_storeu_pd(&Bdot_0[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(beta_2), tmp_c), _mm_mul_pd(_mm_set1_pd(ceil_xi_2), tmp_d)),
        _mm_mul_pd(_mm_set1_pd(n_xi6), tmp_e)));
    }

    for (e_i = 2; e_i < 3; e_i++) {
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (beta_2, tmp), _mm_set_pd(Bdot[e_i + 3], B[e_i + 3])), _mm_mul_pd
        (_mm_set_pd(ceil_xi_2, Cxi_2), _mm_set_pd(Bdot[e_i], B[e_i]))),
        _mm_mul_pd(_mm_set_pd(n_xi6, Cxi_3), _mm_set_pd(Bdot[e_i + 6], B[e_i + 6]))));
      f_i = 3 * i + e_i;
      B_0[f_i] = tmp_f[0];
      Bdot_0[f_i] = tmp_f[1];
    }
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_c = _mm_loadu_pd(&Bdot_0[i]);
    tmp_d = _mm_loadu_pd(&B_0[i]);
    _mm_storeu_pd(&dBA[i], _mm_add_pd(tmp_c, tmp_d));
  }

  for (i = 8; i < 9; i++) {
    dBA[i] = Bdot_0[i] + B_0[i];
  }

  Bdot_3 = (LR_U.ddexp6inv_lambda[3] * LR_U.ddexp6inv_lambdadot[3] +
            LR_U.ddexp6inv_lambda[4] * LR_U.ddexp6inv_lambdadot[4]) +
    LR_U.ddexp6inv_lambda[5] * LR_U.ddexp6inv_lambdadot[5];
  normxidot = Bdot_3 / eta1;
  if (eta1 < 1.0E-12) {
    B[0] = 0.0;
    tmp_c = _mm_set1_pd(0.5);
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_c, _mm_set_pd
      (LR_U.ddexp6inv_lambdadot[4], -LR_U.ddexp6inv_lambdadot[5])));
    B[3] = tmp_f[0];
    B[6] = tmp_f[1];
    B[1] = 0.5 * LR_U.ddexp6inv_lambdadot[5];
    B[4] = 0.0;
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_c, _mm_set_pd
      (-LR_U.ddexp6inv_lambdadot[4], -LR_U.ddexp6inv_lambdadot[3])));
    B[7] = tmp_f[0];
    B[2] = tmp_f[1];
    B[5] = 0.5 * LR_U.ddexp6inv_lambdadot[3];
    B[8] = 0.0;
  } else {
    beta_2 = n_xi2 / 2.0;
    tmp = (1.0 - ddOmega_1) / xiTxidot;
    alpha = (ddOmega_1 - n_xi2) / xiTxidot * Bdot_3;
    n_xi2 = (beta_2 - 3.0 * tmp) / xiTxidot * Bdot_3;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + 1;
        e_j = 3 * e_i + 2;
        A6_tmp = 3 * e_i + i;
        Cxi_3 = ceil_xi[i + 3];
        Bdot_4 = ceil_xi[f_i];
        b_s_tmp = ceil_xi[3 * e_i];
        Bdot_5 = ceil_xi[i + 6];
        B_tmp = ceil_xi[e_j];
        B[A6_tmp] = (((((Cxi[3 * e_i] * ceil_xi[i] + Cxi[f_i] * Cxi_3) + Cxi[e_j]
                        * Bdot_5) + ((Cxi[i + 3] * Bdot_4 + b_s_tmp * Cxi[i]) +
          Cxi[i + 6] * B_tmp)) * tmp + Cxi[A6_tmp] * beta_2) + ceil_xi[A6_tmp] *
                     alpha) + ((Cxi_3 * n_xi2 * Bdot_4 + n_xi2 * ceil_xi[i] *
          b_s_tmp) + Bdot_5 * n_xi2 * B_tmp);
      }
    }
  }

  if (eta1 < 2.2204460492503131E-16) {
    for (i = 0; i < 3; i++) {
      beta = dBA[i + 3];
      dBA_1 = dBA[i];
      alpha = dBA[i + 6];
      ddBA_0 = ddBA[i + 3];
      ddBA_1 = ddBA[i];
      ddBA_2 = ddBA[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + 1;
        f_i = 3 * e_i + 2;
        e_j = 3 * e_i + i;
        Cxi_0[e_j] = (((Cxi[i + 3] * etaddot_[A6_tmp] + etaddot_[3 * e_i] *
                        Cxi[i]) + Cxi[i + 6] * etaddot_[f_i]) + ((ceil_xi[i + 3]
          * dAB[A6_tmp] + dAB[3 * e_i] * ceil_xi[i]) + ceil_xi[i + 6] * dAB[f_i]))
          + ((ceil_xi[3 * e_i] * dBA_1 + ceil_xi[A6_tmp] * beta) + ceil_xi[f_i] *
             alpha);
        BA[e_j] = (Cxi[3 * e_i] * ddBA_1 + Cxi[A6_tmp] * ddBA_0) + Cxi[f_i] *
          ddBA_2;
        b[e_i + 6 * i] = B[3 * i + e_i];
      }
    }

    for (i = 0; i < 3; i++) {
      e_i = (i + 3) * 6;
      b[e_i] = ((dAB[3 * i] + dBA[3 * i]) * 0.16666666666666666 + Bdot[3 * i] *
                0.5) + (Cxi_0[3 * i] + BA[3 * i]) * 0.041666666666666664;
      b[6 * i + 3] = 0.0;
      b[e_i + 3] = B[3 * i];
      A6_tmp = 3 * i + 1;
      b[e_i + 1] = ((dAB[A6_tmp] + dBA[A6_tmp]) * 0.16666666666666666 +
                    Bdot[A6_tmp] * 0.5) + (Cxi_0[A6_tmp] + BA[A6_tmp]) *
        0.041666666666666664;
      b[6 * i + 4] = 0.0;
      b[e_i + 4] = B[A6_tmp];
      A6_tmp = 3 * i + 2;
      b[e_i + 2] = ((dAB[A6_tmp] + dBA[A6_tmp]) * 0.16666666666666666 +
                    Bdot[A6_tmp] * 0.5) + (Cxi_0[A6_tmp] + BA[A6_tmp]) *
        0.041666666666666664;
      b[6 * i + 5] = 0.0;
      b[e_i + 5] = B[A6_tmp];
    }
  } else {
    n_xi2 = dGamma_1 * normxidot;
    ddOmega_1 = dGamma_2 * normxidot;
    xixidot = (-((((6.0 * eta1 - 12.0 * d6) + xn[1] * d6) + 6.0 * eta1 * cx) /
                 xn[4]) / eta1 + dGamma_1 / xn[1]) * normxidot;
    dGamma_1 = (((((20.0 * cx - xn[1] * cx) + 8.0 * eta1 * d6) + 3.0 * xn[1]) -
                 20.0) / xn[5] * eta1 + 2.0 * dGamma_2) * normxidot;
    for (i = 0; i < 3; i++) {
      cx = AA[i + 3];
      xn2sx = AA[i];
      b_dGamma_2 = AA[i + 6];
      beta = dBA[i + 3];
      dBA_1 = dBA[i];
      alpha = dBA[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        n_xi6 = etaddot_[3 * e_i];
        beta_2 = ceil_xi[i] * n_xi6;
        ddBA_0 = ddBA[i];
        dddexp3__tmp = ceil_xi[3 * e_i];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(Cxi[i], dddexp3__tmp),
          _mm_set_pd(n_xi6, ddBA_0)));
        eta1 = Cxi[3 * e_i];
        ddBA_1 = eta1 * ddBA_0;
        f_i = 3 * e_i + 1;
        n_xi6 = etaddot_[f_i];
        ceil_xi_tmp = ceil_xi[i + 3];
        beta_2 += ceil_xi_tmp * n_xi6;
        ddBA_0 = ddBA[i + 3];
        dddexp3__tmp_0 = ceil_xi[f_i];
        ddOmega_1_tmp = Cxi[i + 3];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(ddOmega_1_tmp,
          dddexp3__tmp_0), _mm_set_pd(n_xi6, ddBA_0)), _mm_set_pd(tmp_f[1],
          tmp_f[0])));
        ddBA_2 = tmp_f[0];
        tmp = tmp_f[1];
        d6 = Cxi[f_i];
        e_j = 3 * e_i + 2;
        n_xi6 = etaddot_[e_j];
        dddexp3__tmp_1 = ceil_xi[e_j];
        tmp_c = _mm_set_pd(dddexp3__tmp_1, Cxi[e_j]);
        Cxi_3 = ceil_xi[i + 6];
        Bdot_4 = Cxi[i + 6];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
          (dddexp3__tmp_0, d6), _mm_set_pd(ddOmega_1_tmp, ceil_xi_tmp)),
          _mm_mul_pd(_mm_set_pd(dddexp3__tmp, eta1), _mm_set_pd(Cxi[i],
          ceil_xi[i]))), _mm_mul_pd(tmp_c, _mm_set_pd(Bdot_4, Cxi_3))));
        A6_tmp = 3 * e_i + i;
        ceil_xi_1[A6_tmp] = tmp_f[0];
        Cxi_1[A6_tmp] = tmp_f[1];
        AA_0[A6_tmp] = (ddBA[3 * e_i] * xn2sx + ddBA[f_i] * cx) + ddBA[e_j] *
          b_dGamma_2;
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(tmp_c, _mm_set1_pd(ddBA[i
          + 6])), _mm_set_pd(ddBA_2, d6 * ddBA_0 + ddBA_1)));
        eta_[A6_tmp] = tmp_f[0];
        BA[A6_tmp] = tmp_f[1];
        ceil_xi_0[A6_tmp] = Cxi_3 * n_xi6 + beta_2;
        Cxi_0[A6_tmp] = (((dAB[3 * e_i] * ceil_xi[i] + dAB[f_i] * ceil_xi_tmp) +
                          dAB[e_j] * Cxi_3) + (Bdot_4 * n_xi6 + tmp)) +
          ((dddexp3__tmp_0 * beta + dddexp3__tmp * dBA_1) + dddexp3__tmp_1 *
           alpha);
      }
    }

    for (i = 0; i <= 6; i += 2) {
      tmp_c = _mm_loadu_pd(&Cxi_1[i]);
      tmp_d = _mm_loadu_pd(&ceil_xi_1[i]);
      _mm_storeu_pd(&Bdot_0[i], _mm_add_pd(tmp_c, tmp_d));
    }

    for (i = 8; i < 9; i++) {
      Bdot_0[i] = Cxi_1[i] + ceil_xi_1[i];
    }

    for (i = 0; i < 3; i++) {
      n_xi6 = etaddot_[i];
      d6 = etaddot_[i + 3];
      b_s = etaddot_[i + 6];
      dAB_1 = dAB[i];
      B_tmp = dAB[i + 3];
      BA_4 = dAB[i + 6];
      cx = AA[i + 3];
      xn2sx = AA[i];
      b_dGamma_2 = AA[i + 6];
      tmp = Bdot_0[i + 3];
      Cxi_2 = Bdot_0[i];
      Cxi_3 = Bdot_0[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        beta_2 = ceil_xi[3 * e_i];
        dddexp3__tmp = n_xi6 * beta_2;
        eta1 = dAB_1 * beta_2;
        A6_tmp = 3 * e_i + 1;
        beta_2 = ceil_xi[A6_tmp];
        dddexp3__tmp += d6 * beta_2;
        eta1 += B_tmp * beta_2;
        e_j = 3 * e_i + 2;
        beta_2 = ceil_xi[e_j];
        f_i = 3 * e_i + i;
        dddexp3_[f_i] = (Cxi[3 * e_i] * n_xi6 + Cxi[A6_tmp] * d6) + Cxi[e_j] *
          b_s;
        dAB_0[f_i] = BA_4 * beta_2 + eta1;
        etadot_[f_i] = b_s * beta_2 + dddexp3__tmp;
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
          (ddBA[A6_tmp], dBA[A6_tmp]), _mm_set_pd(tmp, cx)), _mm_mul_pd
          (_mm_set_pd(ddBA[3 * e_i], dBA[3 * e_i]), _mm_set_pd(Cxi_2, xn2sx))),
          _mm_mul_pd(_mm_set_pd(ddBA[e_j], dBA[e_j]), _mm_set_pd(Cxi_3,
          b_dGamma_2))));
        ceil_xi_1[f_i] = tmp_f[0];
        Cxi_1[f_i] = tmp_f[1];
        b[e_i + 6 * i] = B[3 * i + e_i];
      }
    }

    for (i = 0; i < 3; i++) {
      e_i = (i + 3) * 6;
      b[e_i] = (((((((Bdot[3 * i] * 0.5 + AB[3 * i] * n_xi2) + (dAB[3 * i] +
        dBA[3 * i]) * Gamma_1) + (ceil_xi_0[3 * i] + BA[3 * i]) * ddOmega_1) +
                   (Cxi_0[3 * i] + eta_[3 * i]) * Gamma_2) + AA_0[3 * i] *
                  xixidot) + (Cxi_1[3 * i] + ceil_xi_1[3 * i]) * Gamma_3) +
                etadot_[3 * i] * dGamma_1) + (dAB_0[3 * i] + dddexp3_[3 * i]) *
        Gamma_4;
      b[6 * i + 3] = 0.0;
      b[e_i + 3] = B[3 * i];
      A6_tmp = 3 * i + 1;
      b[e_i + 1] = (((((((Bdot[A6_tmp] * 0.5 + AB[A6_tmp] * n_xi2) + (dAB[A6_tmp]
        + dBA[A6_tmp]) * Gamma_1) + (ceil_xi_0[A6_tmp] + BA[A6_tmp]) * ddOmega_1)
                       + (Cxi_0[A6_tmp] + eta_[A6_tmp]) * Gamma_2) + AA_0[A6_tmp]
                      * xixidot) + (Cxi_1[A6_tmp] + ceil_xi_1[A6_tmp]) * Gamma_3)
                    + etadot_[A6_tmp] * dGamma_1) + (dAB_0[A6_tmp] +
        dddexp3_[A6_tmp]) * Gamma_4;
      b[6 * i + 4] = 0.0;
      b[e_i + 4] = B[A6_tmp];
      A6_tmp = 3 * i + 2;
      b[e_i + 2] = (((((((Bdot[A6_tmp] * 0.5 + AB[A6_tmp] * n_xi2) + (dAB[A6_tmp]
        + dBA[A6_tmp]) * Gamma_1) + (ceil_xi_0[A6_tmp] + BA[A6_tmp]) * ddOmega_1)
                       + (Cxi_0[A6_tmp] + eta_[A6_tmp]) * Gamma_2) + AA_0[A6_tmp]
                      * xixidot) + (Cxi_1[A6_tmp] + ceil_xi_1[A6_tmp]) * Gamma_3)
                    + etadot_[A6_tmp] * dGamma_1) + (dAB_0[A6_tmp] +
        dddexp3_[A6_tmp]) * Gamma_4;
      b[6 * i + 5] = 0.0;
      b[e_i + 5] = B[A6_tmp];
    }
  }

  for (i = 0; i < 6; i++) {
    for (e_i = 0; e_i < 6; e_i++) {
      n_xi2 = 0.0;
      for (A6_tmp = 0; A6_tmp < 6; A6_tmp++) {
        n_xi2 += dlog6mat[6 * A6_tmp + i] * b[6 * e_i + A6_tmp];
      }

      dlog6mat_0[i + 6 * e_i] = n_xi2;
    }

    /* Outport: '<Root>/ddexp6inv' */
    for (e_i = 0; e_i < 6; e_i++) {
      n_xi2 = 0.0;
      for (A6_tmp = 0; A6_tmp < 6; A6_tmp++) {
        n_xi2 += dlog6mat_0[6 * A6_tmp + i] * dlog6mat[6 * e_i + A6_tmp];
      }

      LR_Y.ddexp6inv[i + 6 * e_i] = n_xi2;
    }

    /* End of Outport: '<Root>/ddexp6inv' */
  }

  /* MATLAB Function: '<Root>/MATLAB Function13' incorporates:
   *  Inport: '<Root>/dddexp6inv_lambda'
   *  Inport: '<Root>/dddexp6inv_lambdadot'
   */
  memset(&dlog6mat[0], 0, 36U * sizeof(real_T));
  eta1 = LR_norm_e(&LR_U.dddexp6inv_lambda[3]);
  d6 = sin(eta1);
  if (fabs(d6) < 0.001) {
    if (fabs(eta1 - 6.2831853071795862) < 0.001) {
      ceil_xi[0] = 0.0;
      ceil_xi[3] = -LR_U.dddexp6inv_lambda[5];
      ceil_xi[6] = LR_U.dddexp6inv_lambda[4];
      ceil_xi[1] = LR_U.dddexp6inv_lambda[5];
      ceil_xi[4] = 0.0;
      ceil_xi[7] = -LR_U.dddexp6inv_lambda[3];
      ceil_xi[2] = -LR_U.dddexp6inv_lambda[4];
      ceil_xi[5] = LR_U.dddexp6inv_lambda[3];
      ceil_xi[8] = 0.0;
      memset(&Cxi[0], 0, 9U * sizeof(real_T));
      Cxi[0] = 1.0;
      Cxi[4] = 1.0;
      Cxi[8] = 1.0;
      for (i = 0; i < 3; i++) {
        for (e_i = 0; e_i < 3; e_i++) {
          f_i = 3 * e_i + i;
          B[f_i] = ((ceil_xi[i + 3] * 0.083333333333333329 * ceil_xi[3 * e_i + 1]
                     + 0.083333333333333329 * ceil_xi[i] * ceil_xi[3 * e_i]) +
                    ceil_xi[i + 6] * 0.083333333333333329 * ceil_xi[3 * e_i + 2])
            + (Cxi[f_i] - ceil_xi[f_i] * 0.5);
        }
      }
    } else {
      memset(&B[0], 0, 9U * sizeof(real_T));
      B[0] = 1.0;
      B[4] = 1.0;
      B[8] = 1.0;
    }
  } else {
    cx = eta1 / 2.0;
    ceil_xi[0] = 0.0;
    ceil_xi[3] = -LR_U.dddexp6inv_lambda[5];
    ceil_xi[6] = LR_U.dddexp6inv_lambda[4];
    ceil_xi[1] = LR_U.dddexp6inv_lambda[5];
    ceil_xi[4] = 0.0;
    ceil_xi[7] = -LR_U.dddexp6inv_lambda[3];
    ceil_xi[2] = -LR_U.dddexp6inv_lambda[4];
    ceil_xi[5] = LR_U.dddexp6inv_lambda[3];
    ceil_xi[8] = 0.0;
    cx = (1.0 - cos(cx) / (sin(cx) / cx)) / (eta1 * eta1);
    memset(&Cxi[0], 0, 9U * sizeof(real_T));
    Cxi[0] = 1.0;
    Cxi[4] = 1.0;
    Cxi[8] = 1.0;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + i;
        B[f_i] = ((ceil_xi[i + 3] * cx * ceil_xi[3 * e_i + 1] + cx * ceil_xi[i] *
                   ceil_xi[3 * e_i]) + ceil_xi[i + 6] * cx * ceil_xi[3 * e_i + 2])
          + (Cxi[f_i] - ceil_xi[f_i] * 0.5);
      }
    }
  }

  LR_ddexpInvSo3(&LR_U.dddexp6inv_lambda[3], &LR_U.dddexp6inv_lambda[0],
                 ceil_xi_0);
  for (i = 0; i < 3; i++) {
    n_xi2 = B[3 * i];
    dlog6mat[6 * i] = n_xi2;
    f_i = 3 * i + 1;
    ddOmega_1 = B[f_i];
    dlog6mat[6 * i + 1] = ddOmega_1;
    e_i = 3 * i + 2;
    n_xi6 = B[e_i];
    dlog6mat[6 * i + 2] = n_xi6;
    A6_tmp = (i + 3) * 6;
    dlog6mat[A6_tmp] = ceil_xi_0[3 * i];
    dlog6mat[6 * i + 3] = 0.0;
    dlog6mat[A6_tmp + 3] = n_xi2;
    dlog6mat[A6_tmp + 1] = ceil_xi_0[f_i];
    dlog6mat[6 * i + 4] = 0.0;
    dlog6mat[A6_tmp + 4] = ddOmega_1;
    dlog6mat[A6_tmp + 2] = ceil_xi_0[e_i];
    dlog6mat[6 * i + 5] = 0.0;
    dlog6mat[A6_tmp + 5] = n_xi6;
  }

  ceil_xi[0] = 0.0;
  ceil_xi[3] = -LR_U.dddexp6inv_lambda[5];
  ceil_xi[6] = LR_U.dddexp6inv_lambda[4];
  ceil_xi[1] = LR_U.dddexp6inv_lambda[5];
  ceil_xi[4] = 0.0;
  ceil_xi[7] = -LR_U.dddexp6inv_lambda[3];
  ceil_xi[2] = -LR_U.dddexp6inv_lambda[4];
  ceil_xi[5] = LR_U.dddexp6inv_lambda[3];
  ceil_xi[8] = 0.0;
  dddexp3_[0] = 0.0;
  dddexp3_[3] = -LR_U.dddexp6inv_lambda[2];
  dddexp3_[6] = LR_U.dddexp6inv_lambda[1];
  dddexp3_[1] = LR_U.dddexp6inv_lambda[2];
  dddexp3_[4] = 0.0;
  dddexp3_[7] = -LR_U.dddexp6inv_lambda[0];
  dddexp3_[2] = -LR_U.dddexp6inv_lambda[1];
  dddexp3_[5] = LR_U.dddexp6inv_lambda[0];
  dddexp3_[8] = 0.0;
  Cxi[0] = 0.0;
  Cxi[3] = -LR_U.dddexp6inv_lambdadot[5];
  Cxi[6] = LR_U.dddexp6inv_lambdadot[4];
  Cxi[1] = LR_U.dddexp6inv_lambdadot[5];
  Cxi[4] = 0.0;
  Cxi[7] = -LR_U.dddexp6inv_lambdadot[3];
  Cxi[2] = -LR_U.dddexp6inv_lambdadot[4];
  Cxi[5] = LR_U.dddexp6inv_lambdadot[3];
  Cxi[8] = 0.0;
  dAB[0] = 0.0;
  dAB[3] = -LR_U.dddexp6inv_lambdadot[2];
  dAB[6] = LR_U.dddexp6inv_lambdadot[1];
  dAB[1] = LR_U.dddexp6inv_lambdadot[2];
  dAB[4] = 0.0;
  dAB[7] = -LR_U.dddexp6inv_lambdadot[0];
  dAB[2] = -LR_U.dddexp6inv_lambdadot[1];
  dAB[5] = LR_U.dddexp6inv_lambdadot[0];
  dAB[8] = 0.0;
  for (e_j = 0; e_j < 6; e_j++) {
    xn[e_j] = rt_powd_snf(eta1, (((real_T)e_j + 1.0) - 1.0) + 1.0);
  }

  cx = cos(eta1);
  xixidot = eta1 - d6;
  Gamma_1 = xixidot / xn[2];
  Gamma_2 = ((2.0 * cx + xn[1]) - 2.0) / (2.0 * xn[3]);
  dddexp3__tmp = eta1 * cx;
  dddexp3__tmp_0 = eta1 * d6;
  ddOmega_1_tmp = -((2.0 * eta1 - 3.0 * d6) + dddexp3__tmp);
  dddexp3__tmp_1 = 4.0 * cx + dddexp3__tmp_0;
  tmp_c = _mm_div_pd(_mm_set_pd(-((dddexp3__tmp_1 + xn[1]) - 4.0), ddOmega_1_tmp),
                     _mm_loadu_pd(&xn[3]));
  _mm_storeu_pd(&tmp_f[0], tmp_c);

  /* MATLAB Function: '<Root>/MATLAB Function13' incorporates:
   *  Inport: '<Root>/dddexp6inv_lambda'
   *  Inport: '<Root>/dddexp6inv_lambdaddot'
   *  Inport: '<Root>/dddexp6inv_lambdadot'
   */
  dGamma_1 = tmp_f[0];
  dGamma_2 = tmp_f[1];
  Gamma_3 = -tmp_f[0] / eta1;
  Gamma_4 = tmp_f[1] * eta1 + Gamma_2;
  for (i = 0; i < 3; i++) {
    e_j = 3 * i + 1;
    dAB_1 = dAB[e_j];
    B_tmp = dAB[3 * i];
    dAB_tmp = 3 * i + 2;
    BA_4 = dAB[dAB_tmp];
    for (e_i = 0; e_i <= 0; e_i += 2) {
      tmp_c = _mm_loadu_pd(&Cxi[e_i + 3]);
      tmp_d = _mm_set1_pd(dddexp3_[e_j]);
      tmp_e = _mm_loadu_pd(&Cxi[e_i]);
      tmp_3 = _mm_set1_pd(dddexp3_[3 * i]);
      tmp_i = _mm_loadu_pd(&Cxi[e_i + 6]);
      tmp_g = _mm_set1_pd(dddexp3_[dAB_tmp]);
      A6_tmp = 3 * i + e_i;
      _mm_storeu_pd(&etaddot_[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_d,
        tmp_c), _mm_mul_pd(tmp_3, tmp_e)), _mm_mul_pd(tmp_g, tmp_i)));
      tmp_c = _mm_loadu_pd(&ceil_xi[e_i + 3]);
      tmp_e = _mm_loadu_pd(&ceil_xi[e_i]);
      tmp_i = _mm_loadu_pd(&ceil_xi[e_i + 6]);
      _mm_storeu_pd(&Bdot[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (dAB_1), tmp_c), _mm_mul_pd(_mm_set1_pd(B_tmp), tmp_e)), _mm_mul_pd
        (_mm_set1_pd(BA_4), tmp_i)));
      tmp_c = _mm_loadu_pd(&ceil_xi[e_i + 3]);
      tmp_e = _mm_set1_pd(ceil_xi[e_j]);
      tmp_i = _mm_loadu_pd(&ceil_xi[e_i]);
      tmp_4 = _mm_set1_pd(ceil_xi[3 * i]);
      tmp_h = _mm_loadu_pd(&ceil_xi[e_i + 6]);
      tmp_m = _mm_set1_pd(ceil_xi[dAB_tmp]);
      _mm_storeu_pd(&BA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_e, tmp_c),
        _mm_mul_pd(tmp_4, tmp_i)), _mm_mul_pd(tmp_m, tmp_h)));
      tmp_c = _mm_loadu_pd(&dddexp3_[e_i + 3]);
      tmp_i = _mm_loadu_pd(&dddexp3_[e_i]);
      tmp_h = _mm_loadu_pd(&dddexp3_[e_i + 6]);
      _mm_storeu_pd(&AB[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_e, tmp_c),
        _mm_mul_pd(tmp_4, tmp_i)), _mm_mul_pd(tmp_m, tmp_h)));
      tmp_c = _mm_loadu_pd(&ceil_xi[e_i + 3]);
      tmp_e = _mm_loadu_pd(&ceil_xi[e_i]);
      tmp_i = _mm_loadu_pd(&ceil_xi[e_i + 6]);
      _mm_storeu_pd(&dBA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_d, tmp_c),
        _mm_mul_pd(tmp_3, tmp_e)), _mm_mul_pd(tmp_g, tmp_i)));
    }

    for (e_i = 2; e_i < 3; e_i++) {
      Cxi_3 = ceil_xi[e_i + 3];
      Bdot_4 = ceil_xi[e_i + 6];
      Bdot_5 = dddexp3_[e_j];
      xcx_tmp = dddexp3_[3 * i];
      tmp_k = dddexp3_[dAB_tmp];
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd(dAB_1,
        Bdot_5), _mm_set_pd(Cxi_3, Cxi[e_i + 3])), _mm_mul_pd(_mm_set_pd(B_tmp,
        xcx_tmp), _mm_set_pd(ceil_xi[e_i], Cxi[e_i]))), _mm_mul_pd(_mm_set_pd
        (BA_4, tmp_k), _mm_set_pd(Bdot_4, Cxi[e_i + 6]))));
      f_i = 3 * i + e_i;
      etaddot_[f_i] = tmp_f[0];
      Bdot[f_i] = tmp_f[1];
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (ceil_xi[e_j]), _mm_set_pd(dddexp3_[e_i + 3], Cxi_3)), _mm_mul_pd
        (_mm_set1_pd(ceil_xi[3 * i]), _mm_set_pd(dddexp3_[e_i], ceil_xi[e_i]))),
        _mm_mul_pd(_mm_set1_pd(ceil_xi[dAB_tmp]), _mm_set_pd(dddexp3_[e_i + 6],
        Bdot_4))));
      BA[f_i] = tmp_f[0];
      AB[f_i] = tmp_f[1];
      dBA[f_i] = (Bdot_5 * Cxi_3 + xcx_tmp * ceil_xi[e_i]) + tmp_k * Bdot_4;
    }
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_c = _mm_loadu_pd(&etaddot_[i]);
    tmp_d = _mm_loadu_pd(&Bdot[i]);
    _mm_storeu_pd(&eta_[i], _mm_add_pd(tmp_c, tmp_d));
  }

  for (i = 8; i < 9; i++) {
    eta_[i] = etaddot_[i] + Bdot[i];
  }

  for (i = 0; i < 3; i++) {
    A6_tmp = 3 * i + 1;
    tmp = Cxi[A6_tmp];
    Cxi_2 = Cxi[3 * i];
    f_i = 3 * i + 2;
    Cxi_3 = Cxi[f_i];
    beta_2 = ceil_xi[A6_tmp];
    ceil_xi_2 = ceil_xi[3 * i];
    n_xi6 = ceil_xi[f_i];
    for (e_i = 0; e_i <= 0; e_i += 2) {
      tmp_c = _mm_loadu_pd(&dddexp3_[e_i + 3]);
      tmp_d = _mm_loadu_pd(&dddexp3_[e_i]);
      tmp_e = _mm_loadu_pd(&dddexp3_[e_i + 6]);
      A6_tmp = 3 * i + e_i;
      _mm_storeu_pd(&AA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (tmp), tmp_c), _mm_mul_pd(_mm_set1_pd(Cxi_2), tmp_d)), _mm_mul_pd
        (_mm_set1_pd(Cxi_3), tmp_e)));
      tmp_c = _mm_loadu_pd(&dAB[e_i + 3]);
      tmp_d = _mm_loadu_pd(&dAB[e_i]);
      tmp_e = _mm_loadu_pd(&dAB[e_i + 6]);
      _mm_storeu_pd(&ddBA[A6_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (beta_2), tmp_c), _mm_mul_pd(_mm_set1_pd(ceil_xi_2), tmp_d)), _mm_mul_pd
        (_mm_set1_pd(n_xi6), tmp_e)));
    }

    for (e_i = 2; e_i < 3; e_i++) {
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (beta_2, tmp), _mm_set_pd(dAB[e_i + 3], dddexp3_[e_i + 3])), _mm_mul_pd
        (_mm_set_pd(ceil_xi_2, Cxi_2), _mm_set_pd(dAB[e_i], dddexp3_[e_i]))),
        _mm_mul_pd(_mm_set_pd(n_xi6, Cxi_3), _mm_set_pd(dAB[e_i + 6],
        dddexp3_[e_i + 6]))));
      A6_tmp = 3 * i + e_i;
      AA[A6_tmp] = tmp_f[0];
      ddBA[A6_tmp] = tmp_f[1];
    }
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_c = _mm_loadu_pd(&ddBA[i]);
    tmp_d = _mm_loadu_pd(&AA[i]);
    _mm_storeu_pd(&etadot_[i], _mm_add_pd(tmp_c, tmp_d));
  }

  for (i = 8; i < 9; i++) {
    etadot_[i] = ddBA[i] + AA[i];
  }

  Bdot_3 = (LR_U.dddexp6inv_lambda[3] * LR_U.dddexp6inv_lambdadot[3] +
            LR_U.dddexp6inv_lambda[4] * LR_U.dddexp6inv_lambdadot[4]) +
    LR_U.dddexp6inv_lambda[5] * LR_U.dddexp6inv_lambdadot[5];
  normxidot = Bdot_3 / eta1;
  if (eta1 < 1.0E-12) {
    B[0] = 0.0;
    tmp_c = _mm_set1_pd(0.5);
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_c, _mm_set_pd
      (LR_U.dddexp6inv_lambdadot[4], -LR_U.dddexp6inv_lambdadot[5])));
    B[3] = tmp_f[0];
    B[6] = tmp_f[1];
    B[1] = 0.5 * LR_U.dddexp6inv_lambdadot[5];
    B[4] = 0.0;
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_c, _mm_set_pd
      (-LR_U.dddexp6inv_lambdadot[4], -LR_U.dddexp6inv_lambdadot[3])));
    B[7] = tmp_f[0];
    B[2] = tmp_f[1];
    B[5] = 0.5 * LR_U.dddexp6inv_lambdadot[3];
    B[8] = 0.0;
  } else {
    beta = eta1 / 2.0;
    b_s = sin(beta) / beta;
    alpha = b_s * cos(beta);
    b_s *= b_s;
    beta_2 = b_s / 2.0;
    beta = eta1 * eta1;
    ddOmega_1 = (1.0 - alpha) / beta;
    b_s = (alpha - b_s) / beta * Bdot_3;
    beta = (beta_2 - 3.0 * ddOmega_1) / beta * Bdot_3;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + 1;
        e_j = 3 * e_i + 2;
        A6_tmp = 3 * e_i + i;
        Cxi_3 = ceil_xi[i + 3];
        Bdot_4 = ceil_xi[f_i];
        b_s_tmp = ceil_xi[3 * e_i];
        Bdot_5 = ceil_xi[i + 6];
        B_tmp = ceil_xi[e_j];
        B[A6_tmp] = (((((Cxi[3 * e_i] * ceil_xi[i] + Cxi[f_i] * Cxi_3) + Cxi[e_j]
                        * Bdot_5) + ((Cxi[i + 3] * Bdot_4 + b_s_tmp * Cxi[i]) +
          Cxi[i + 6] * B_tmp)) * ddOmega_1 + Cxi[A6_tmp] * beta_2) +
                     ceil_xi[A6_tmp] * b_s) + ((Cxi_3 * beta * Bdot_4 + beta *
          ceil_xi[i] * b_s_tmp) + Bdot_5 * beta * B_tmp);
      }
    }
  }

  if (eta1 < 2.2204460492503131E-16) {
    for (i = 0; i < 3; i++) {
      ddOmega_1 = etadot_[i + 3];
      alpha = etadot_[i];
      dAB_1 = etadot_[i + 6];
      n_xi5 = AB[i + 3];
      dBA_1 = AB[i];
      AB_4 = AB[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + 1;
        f_i = 3 * e_i + 2;
        e_j = 3 * e_i + i;
        Cxi_0[e_j] = (((Cxi[i + 3] * dBA[A6_tmp] + dBA[3 * e_i] * Cxi[i]) +
                       Cxi[i + 6] * dBA[f_i]) + ((ceil_xi[i + 3] * eta_[A6_tmp]
          + eta_[3 * e_i] * ceil_xi[i]) + ceil_xi[i + 6] * eta_[f_i])) +
          ((ceil_xi[3 * e_i] * alpha + ceil_xi[A6_tmp] * ddOmega_1) +
           ceil_xi[f_i] * dAB_1);
        AB_0[e_j] = (Cxi[3 * e_i] * dBA_1 + Cxi[A6_tmp] * n_xi5) + Cxi[f_i] *
          AB_4;
        b[e_i + 6 * i] = B[3 * i + e_i];
      }
    }

    for (i = 0; i < 3; i++) {
      e_i = (i + 3) * 6;
      b[e_i] = ((eta_[3 * i] + etadot_[3 * i]) * 0.16666666666666666 + dAB[3 * i]
                * 0.5) + (Cxi_0[3 * i] + AB_0[3 * i]) * 0.041666666666666664;
      b[6 * i + 3] = 0.0;
      b[e_i + 3] = B[3 * i];
      A6_tmp = 3 * i + 1;
      b[e_i + 1] = ((eta_[A6_tmp] + etadot_[A6_tmp]) * 0.16666666666666666 +
                    dAB[A6_tmp] * 0.5) + (Cxi_0[A6_tmp] + AB_0[A6_tmp]) *
        0.041666666666666664;
      b[6 * i + 4] = 0.0;
      b[e_i + 4] = B[A6_tmp];
      A6_tmp = 3 * i + 2;
      b[e_i + 2] = ((eta_[A6_tmp] + etadot_[A6_tmp]) * 0.16666666666666666 +
                    dAB[A6_tmp] * 0.5) + (Cxi_0[A6_tmp] + AB_0[A6_tmp]) *
        0.041666666666666664;
      b[6 * i + 5] = 0.0;
      b[e_i + 5] = B[A6_tmp];
    }
  } else {
    b_s = dGamma_1 * normxidot;
    n_xi6 = dGamma_2 * normxidot;
    xiTxidot = (-((((6.0 * eta1 - 12.0 * d6) + xn[1] * d6) + 6.0 * eta1 * cx) /
                  xn[4]) / eta1 + dGamma_1 / xn[1]) * normxidot;
    n_xi2 = (((((20.0 * cx - xn[1] * cx) + 8.0 * eta1 * d6) + 3.0 * xn[1]) -
              20.0) / xn[5] * eta1 + 2.0 * dGamma_2) * normxidot;
    for (i = 0; i < 3; i++) {
      ddGamma_1 = BA[i + 3];
      B_tmp = BA[i];
      BA_4 = BA[i + 6];
      ddOmega_1 = etadot_[i + 3];
      alpha = etadot_[i];
      dAB_1 = etadot_[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        beta = dBA[3 * e_i];
        beta_2 = ceil_xi[i] * beta;
        n_xi5 = AB[i];
        Cxi_3 = ceil_xi[3 * e_i];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(Cxi[i], Cxi_3),
          _mm_set_pd(beta, n_xi5)));
        dGamma_1 = Cxi[3 * e_i];
        dBA_1 = dGamma_1 * n_xi5;
        f_i = 3 * e_i + 1;
        beta = dBA[f_i];
        ceil_xi_tmp = ceil_xi[i + 3];
        beta_2 += ceil_xi_tmp * beta;
        n_xi5 = AB[i + 3];
        Bdot_4 = ceil_xi[f_i];
        Bdot_5 = Cxi[i + 3];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Bdot_5, Bdot_4),
          _mm_set_pd(beta, n_xi5)), _mm_set_pd(tmp_f[1], tmp_f[0])));
        AB_4 = tmp_f[0];
        tmp = tmp_f[1];
        dGamma_2 = Cxi[f_i];
        e_j = 3 * e_i + 2;
        beta = dBA[e_j];
        xcx_tmp = ceil_xi[e_j];
        tmp_c = _mm_set_pd(xcx_tmp, Cxi[e_j]);
        tmp_k = ceil_xi[i + 6];
        tmp_l = Cxi[i + 6];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
          (Bdot_4, dGamma_2), _mm_set_pd(Bdot_5, ceil_xi_tmp)), _mm_mul_pd
          (_mm_set_pd(Cxi_3, dGamma_1), _mm_set_pd(Cxi[i], ceil_xi[i]))),
          _mm_mul_pd(tmp_c, _mm_set_pd(tmp_l, tmp_k))));
        A6_tmp = 3 * e_i + i;
        ceil_xi_1[A6_tmp] = tmp_f[0];
        Cxi_1[A6_tmp] = tmp_f[1];
        BA_0[A6_tmp] = (AB[3 * e_i] * B_tmp + AB[f_i] * ddGamma_1) + AB[e_j] *
          BA_4;
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(tmp_c, _mm_set1_pd(AB[i +
          6])), _mm_set_pd(AB_4, dGamma_2 * n_xi5 + dBA_1)));
        AB_1[A6_tmp] = tmp_f[0];
        AB_0[A6_tmp] = tmp_f[1];
        ceil_xi_0[A6_tmp] = tmp_k * beta + beta_2;
        Cxi_0[A6_tmp] = (((eta_[3 * e_i] * ceil_xi[i] + eta_[f_i] * ceil_xi_tmp)
                          + eta_[e_j] * tmp_k) + (tmp_l * beta + tmp)) +
          ((Bdot_4 * ddOmega_1 + Cxi_3 * alpha) + xcx_tmp * dAB_1);
      }
    }

    for (i = 0; i <= 6; i += 2) {
      tmp_c = _mm_loadu_pd(&Cxi_1[i]);
      tmp_d = _mm_loadu_pd(&ceil_xi_1[i]);
      _mm_storeu_pd(&Bdot_0[i], _mm_add_pd(tmp_c, tmp_d));
    }

    for (i = 8; i < 9; i++) {
      Bdot_0[i] = Cxi_1[i] + ceil_xi_1[i];
    }

    for (i = 0; i < 3; i++) {
      beta = dBA[i];
      dBA_1 = dBA[i + 3];
      alpha = dBA[i + 6];
      normxidot = eta_[i];
      b_s_tmp = eta_[i + 3];
      theta_sqr = eta_[i + 6];
      ddGamma_1 = BA[i + 3];
      B_tmp = BA[i];
      BA_4 = BA[i + 6];
      tmp = Bdot_0[i + 3];
      Cxi_2 = Bdot_0[i];
      Cxi_3 = Bdot_0[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        beta_2 = ceil_xi[3 * e_i];
        ddOmega_1 = beta * beta_2;
        n_xi5 = normxidot * beta_2;
        A6_tmp = 3 * e_i + 1;
        beta_2 = ceil_xi[A6_tmp];
        ddOmega_1 += dBA_1 * beta_2;
        n_xi5 += b_s_tmp * beta_2;
        e_j = 3 * e_i + 2;
        beta_2 = ceil_xi[e_j];
        f_i = 3 * e_i + i;
        ceil_xi_1[f_i] = (Cxi[3 * e_i] * beta + Cxi[A6_tmp] * dBA_1) + Cxi[e_j] *
          alpha;
        eta__0[f_i] = theta_sqr * beta_2 + n_xi5;
        dBA_0[f_i] = alpha * beta_2 + ddOmega_1;
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
          (AB[A6_tmp], etadot_[A6_tmp]), _mm_set_pd(tmp, ddGamma_1)), _mm_mul_pd
          (_mm_set_pd(AB[3 * e_i], etadot_[3 * e_i]), _mm_set_pd(Cxi_2, B_tmp))),
          _mm_mul_pd(_mm_set_pd(AB[e_j], etadot_[e_j]), _mm_set_pd(Cxi_3, BA_4))));
        BA_1[f_i] = tmp_f[0];
        Cxi_1[f_i] = tmp_f[1];
        b[e_i + 6 * i] = B[3 * i + e_i];
      }
    }

    for (i = 0; i < 3; i++) {
      e_i = (i + 3) * 6;
      b[e_i] = ((((((((dBA[3 * i] + AB[3 * i]) * b_s + dAB[3 * i] * 0.5) +
                     (eta_[3 * i] + etadot_[3 * i]) * Gamma_1) + (ceil_xi_0[3 *
        i] + AB_0[3 * i]) * n_xi6) + (Cxi_0[3 * i] + AB_1[3 * i]) * Gamma_2) +
                  BA_0[3 * i] * xiTxidot) + (Cxi_1[3 * i] + BA_1[3 * i]) *
                 Gamma_3) + dBA_0[3 * i] * n_xi2) + (eta__0[3 * i] + ceil_xi_1[3
        * i]) * Gamma_4;
      b[6 * i + 3] = 0.0;
      b[e_i + 3] = B[3 * i];
      A6_tmp = 3 * i + 1;
      b[e_i + 1] = ((((((((dBA[A6_tmp] + AB[A6_tmp]) * b_s + dAB[A6_tmp] * 0.5)
                         + (eta_[A6_tmp] + etadot_[A6_tmp]) * Gamma_1) +
                        (ceil_xi_0[A6_tmp] + AB_0[A6_tmp]) * n_xi6) +
                       (Cxi_0[A6_tmp] + AB_1[A6_tmp]) * Gamma_2) + BA_0[A6_tmp] *
                      xiTxidot) + (Cxi_1[A6_tmp] + BA_1[A6_tmp]) * Gamma_3) +
                    dBA_0[A6_tmp] * n_xi2) + (eta__0[A6_tmp] + ceil_xi_1[A6_tmp])
        * Gamma_4;
      b[6 * i + 4] = 0.0;
      b[e_i + 4] = B[A6_tmp];
      A6_tmp = 3 * i + 2;
      b[e_i + 2] = ((((((((dBA[A6_tmp] + AB[A6_tmp]) * b_s + dAB[A6_tmp] * 0.5)
                         + (eta_[A6_tmp] + etadot_[A6_tmp]) * Gamma_1) +
                        (ceil_xi_0[A6_tmp] + AB_0[A6_tmp]) * n_xi6) +
                       (Cxi_0[A6_tmp] + AB_1[A6_tmp]) * Gamma_2) + BA_0[A6_tmp] *
                      xiTxidot) + (Cxi_1[A6_tmp] + BA_1[A6_tmp]) * Gamma_3) +
                    dBA_0[A6_tmp] * n_xi2) + (eta__0[A6_tmp] + ceil_xi_1[A6_tmp])
        * Gamma_4;
      b[6 * i + 5] = 0.0;
      b[e_i + 5] = B[A6_tmp];
    }
  }

  memset(&b_b[0], 0, 36U * sizeof(real_T));
  LR_ddexpInvSo3(&LR_U.dddexp6inv_lambda[3], &LR_U.dddexp6inv_lambdadot[3],
                 ceil_xi_0);
  for (i = 0; i < 3; i++) {
    n_xi2 = ceil_xi_0[3 * i];
    b_b[6 * i] = n_xi2;
    b_b[6 * i + 3] = 0.0;
    A6_tmp = (i + 3) * 6;
    b_b[A6_tmp + 3] = n_xi2;
    n_xi2 = ceil_xi_0[3 * i + 1];
    b_b[6 * i + 1] = n_xi2;
    b_b[6 * i + 4] = 0.0;
    b_b[A6_tmp + 4] = n_xi2;
    n_xi2 = ceil_xi_0[3 * i + 2];
    b_b[6 * i + 2] = n_xi2;
    b_b[6 * i + 5] = 0.0;
    b_b[A6_tmp + 5] = n_xi2;
  }

  if (eta1 < 1.0E-12) {
    ceil_xi_0[0] = -0.0;
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd
      (LR_U.dddexp6inv_lambdadot[1], -LR_U.dddexp6inv_lambdadot[2])));
    ceil_xi_0[3] = tmp_f[0];
    ceil_xi_0[6] = tmp_f[1];
    ceil_xi_0[1] = -0.5 * LR_U.dddexp6inv_lambdadot[2];
    ceil_xi_0[4] = -0.0;
    _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(tmp_b, _mm_set_pd
      (-LR_U.dddexp6inv_lambdadot[1], -LR_U.dddexp6inv_lambdadot[0])));
    ceil_xi_0[7] = tmp_f[0];
    ceil_xi_0[2] = tmp_f[1];
    ceil_xi_0[5] = -0.5 * LR_U.dddexp6inv_lambdadot[0];
    ceil_xi_0[8] = -0.0;
    for (i = 0; i < 3; i++) {
      tmp_b = _mm_add_pd(_mm_mul_pd(_mm_add_pd(_mm_loadu_pd(&AA[3 * i]),
        _mm_loadu_pd(&etaddot_[3 * i])), _mm_set1_pd(0.083333333333333329)),
                         _mm_loadu_pd(&ceil_xi_0[3 * i]));
      e_i = (i + 3) * 6;
      _mm_storeu_pd(&b_b[e_i], tmp_b);
      A6_tmp = 3 * i + 2;
      b_b[e_i + 2] = (AA[A6_tmp] + etaddot_[A6_tmp]) * 0.083333333333333329 +
        ceil_xi_0[A6_tmp];
    }
  } else {
    b_s = eta1 / 2.0;
    ddOmega_1 = sin(b_s) / b_s;
    beta = ddOmega_1 * ddOmega_1;
    Gamma_1 = cos(b_s) / ddOmega_1;
    theta_sqr = eta1 * eta1;
    ddOmega_1 = (1.0 - Gamma_1) / theta_sqr;
    _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
      (LR_U.dddexp6inv_lambda[3], LR_U.dddexp6inv_lambda[0]), _mm_set_pd
      (LR_U.dddexp6inv_lambdadot[3], LR_U.dddexp6inv_lambda[3])), _mm_mul_pd
      (_mm_set_pd(LR_U.dddexp6inv_lambda[4], LR_U.dddexp6inv_lambda[1]),
       _mm_set_pd(LR_U.dddexp6inv_lambdadot[4], LR_U.dddexp6inv_lambda[4]))),
      _mm_mul_pd(_mm_set_pd(LR_U.dddexp6inv_lambda[5], LR_U.dddexp6inv_lambda[2]),
                 _mm_set_pd(LR_U.dddexp6inv_lambdadot[5],
      LR_U.dddexp6inv_lambda[5]))));
    Cxi_3 = tmp_f[0];
    Bdot_4 = tmp_f[1];
    tmp = ((LR_U.dddexp6inv_lambda[0] * LR_U.dddexp6inv_lambda[3] +
            LR_U.dddexp6inv_lambda[1] * LR_U.dddexp6inv_lambda[4]) +
           LR_U.dddexp6inv_lambda[2] * LR_U.dddexp6inv_lambda[5]) * Bdot_3 /
      theta_sqr;
    dGamma_1 = (((LR_U.dddexp6inv_lambda[0] * LR_U.dddexp6inv_lambdadot[3] +
                  LR_U.dddexp6inv_lambda[1] * LR_U.dddexp6inv_lambdadot[4]) +
                 LR_U.dddexp6inv_lambda[2] * LR_U.dddexp6inv_lambdadot[5]) +
                ((LR_U.dddexp6inv_lambdadot[0] * LR_U.dddexp6inv_lambda[3] +
                  LR_U.dddexp6inv_lambdadot[1] * LR_U.dddexp6inv_lambda[4]) +
                 LR_U.dddexp6inv_lambdadot[2] * LR_U.dddexp6inv_lambda[5])) -
      3.0 * tmp;
    n_xi2 = ((1.0 / beta + Gamma_1) - 2.0) * (1.0 / theta_sqr) / theta_sqr;
    b_s = (1.0 - Gamma_1 / beta) * (2.0 / theta_sqr) / theta_sqr * tmp;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        A6_tmp = 3 * e_i + 1;
        f_i = 3 * e_i + 2;
        n_xi6 = ceil_xi[i + 3];
        Gamma_2 = ceil_xi[A6_tmp];
        Gamma_1 = ceil_xi[3 * e_i];
        Gamma_3 = ceil_xi[i + 6];
        Gamma_4 = ceil_xi[f_i];
        e_j = 3 * e_i + i;
        b_b[i + 6 * (e_i + 3)] = ((((((Cxi[3 * e_i] * ceil_xi[i] + Cxi[A6_tmp] *
          n_xi6) + Cxi[f_i] * Gamma_3) + ((Cxi[i + 3] * Gamma_2 + Gamma_1 *
          Cxi[i]) + Cxi[i + 6] * Gamma_4)) * Cxi_3 + ((n_xi6 * dGamma_1 *
          Gamma_2 + dGamma_1 * ceil_xi[i] * Gamma_1) + Gamma_3 * dGamma_1 *
          Gamma_4)) + (dBA[e_j] + AB[e_j]) * Bdot_4) * n_xi2 + (((Bdot[e_j] +
          ddBA[e_j]) + (AA[e_j] + etaddot_[e_j])) * ddOmega_1 + dAB[e_j] * -0.5))
          + ((n_xi6 * b_s * Gamma_2 + b_s * ceil_xi[i] * Gamma_1) + Gamma_3 *
             b_s * Gamma_4);
      }
    }
  }

  Bdot[0] = 0.0;
  Bdot[3] = -LR_U.dddexp6inv_lambdaddot[5];
  Bdot[6] = LR_U.dddexp6inv_lambdaddot[4];
  Bdot[1] = LR_U.dddexp6inv_lambdaddot[5];
  Bdot[4] = 0.0;
  Bdot[7] = -LR_U.dddexp6inv_lambdaddot[3];
  Bdot[2] = -LR_U.dddexp6inv_lambdaddot[4];
  Bdot[5] = LR_U.dddexp6inv_lambdaddot[3];
  Bdot[8] = 0.0;
  etaddot_[0] = 0.0;
  etaddot_[3] = -LR_U.dddexp6inv_lambdaddot[2];
  etaddot_[6] = LR_U.dddexp6inv_lambdaddot[1];
  etaddot_[1] = LR_U.dddexp6inv_lambdaddot[2];
  etaddot_[4] = 0.0;
  etaddot_[7] = -LR_U.dddexp6inv_lambdaddot[0];
  etaddot_[2] = -LR_U.dddexp6inv_lambdaddot[1];
  etaddot_[5] = LR_U.dddexp6inv_lambdaddot[0];
  etaddot_[8] = 0.0;
  for (e_j = 0; e_j < 7; e_j++) {
    xn_0[e_j] = rt_powd_snf(eta1, (((real_T)e_j + 1.0) - 1.0) + 1.0);
  }

  beta_2 = eta1 * dddexp3__tmp;
  xn2sx = eta1 * dddexp3__tmp_0;
  Gamma_1 = xixidot / xn_0[2];
  dGamma_2 = ddOmega_1_tmp / xn_0[3];
  ddGamma_1 = (((6.0 * eta1 - 12.0 * d6) + xn2sx) + 6.0 * dddexp3__tmp) / xn_0[4];
  Gamma_2 = ((2.0 * cx + xn_0[1]) - 2.0) / (2.0 * xn_0[3]);
  b_dGamma_2 = -((dddexp3__tmp_1 + xn_0[1]) - 4.0) / xn_0[4];
  tmp = ((((20.0 * cx - beta_2) + 8.0 * dddexp3__tmp_0) + 3.0 * xn_0[1]) - 20.0)
    / xn_0[5];
  Gamma_3 = -dGamma_2 / eta1;
  xixidot = -ddGamma_1 / eta1 + dGamma_2 / xn_0[1];
  Gamma_4 = b_dGamma_2 * eta1 + Gamma_2;
  beta = tmp * eta1 + 2.0 * b_dGamma_2;
  for (i = 0; i < 3; i++) {
    for (e_i = 0; e_i < 3; e_i++) {
      A6_tmp = 3 * e_i + 1;
      dAB_tmp = 3 * e_i + 2;
      tmp_b = _mm_set1_pd(2.0);
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd
        (_mm_mul_pd(_mm_set_pd(ceil_xi[A6_tmp], dddexp3_[A6_tmp]), _mm_set_pd
                    (etaddot_[i + 3], Bdot[i + 3])), _mm_mul_pd(_mm_set_pd
        (ceil_xi[3 * e_i], dddexp3_[3 * e_i]), _mm_set_pd(etaddot_[i], Bdot[i]))),
        _mm_mul_pd(_mm_set_pd(ceil_xi[dAB_tmp], dddexp3_[dAB_tmp]), _mm_set_pd
                   (etaddot_[i + 6], Bdot[i + 6]))), _mm_add_pd(_mm_add_pd
        (_mm_mul_pd(_mm_mul_pd(_mm_set_pd(dAB[i + 3], Cxi[i + 3]), tmp_b),
                    _mm_set_pd(Cxi[A6_tmp], dAB[A6_tmp])), _mm_mul_pd(_mm_mul_pd
        (tmp_b, _mm_set_pd(dAB[i], Cxi[i])), _mm_set_pd(Cxi[3 * e_i], dAB[3 *
        e_i]))), _mm_mul_pd(_mm_mul_pd(_mm_set_pd(dAB[i + 6], Cxi[i + 6]), tmp_b),
                            _mm_set_pd(Cxi[dAB_tmp], dAB[dAB_tmp])))),
        _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd(Bdot[A6_tmp],
        etaddot_[A6_tmp]), _mm_set_pd(dddexp3_[i + 3], ceil_xi[i + 3])),
        _mm_mul_pd(_mm_set_pd(Bdot[3 * e_i], etaddot_[3 * e_i]), _mm_set_pd
                   (dddexp3_[i], ceil_xi[i]))), _mm_mul_pd(_mm_set_pd
        (Bdot[dAB_tmp], etaddot_[dAB_tmp]), _mm_set_pd(dddexp3_[i + 6],
        ceil_xi[i + 6])))));
      A6_tmp = 3 * e_i + i;
      AA[A6_tmp] = tmp_f[0];
      ddBA[A6_tmp] = tmp_f[1];
      A6_tmp = 3 * i + 1;
      dAB_tmp = 3 * i + 2;
      _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
        (ceil_xi[A6_tmp], Cxi[A6_tmp]), _mm_set_pd(Cxi[e_i + 3], ceil_xi[e_i + 3])),
        _mm_mul_pd(_mm_set_pd(ceil_xi[3 * i], Cxi[3 * i]), _mm_set_pd(Cxi[e_i],
        ceil_xi[e_i]))), _mm_mul_pd(_mm_set_pd(ceil_xi[dAB_tmp], Cxi[dAB_tmp]),
        _mm_set_pd(Cxi[e_i + 6], ceil_xi[e_i + 6]))));
      A6_tmp = 3 * i + e_i;
      ceil_xi_0[A6_tmp] = tmp_f[0];
      Cxi_0[A6_tmp] = tmp_f[1];
    }
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_b = _mm_loadu_pd(&Cxi_0[i]);
    tmp_c = _mm_loadu_pd(&ceil_xi_0[i]);
    _mm_storeu_pd(&dddexp3_[i], _mm_add_pd(tmp_b, tmp_c));
  }

  for (i = 8; i < 9; i++) {
    dddexp3_[i] = Cxi_0[i] + ceil_xi_0[i];
  }

  Bdot_5 = 0.0;
  ddOmega_1_tmp = 0.0;
  dddexp3__tmp_1 = 0.0;
  Cxi_3 = 0.0;
  n_xi2 = eta1 * eta1;
  xiTxidot = rt_powd_snf(eta1, 4.0);
  n_xi5 = rt_powd_snf(eta1, 5.0);
  n_xi6 = rt_powd_snf(eta1, 6.0);
  Bdot_4 = 0.0;
  for (i = 0; i < 3; i++) {
    normxidot = eta_[i + 3];
    b_s_tmp = eta_[i];
    theta_sqr = eta_[i + 6];
    for (e_i = 0; e_i < 3; e_i++) {
      dAB[i + 3 * e_i] = (Cxi[3 * e_i + 1] * normxidot + Cxi[3 * e_i] * b_s_tmp)
        + Cxi[3 * e_i + 2] * theta_sqr;
    }

    ddOmega_1 = LR_U.dddexp6inv_lambda[i + 3];
    b_s = LR_U.dddexp6inv_lambdadot[i + 3];
    Bdot_5 += ddOmega_1 * b_s;
    ddOmega_1_tmp = Bdot_5;
    dddexp3__tmp_1 += b_s * b_s;
    Cxi_3 += LR_U.dddexp6inv_lambdaddot[i + 3] * ddOmega_1;
    Bdot_4 = Bdot_5;
  }

  normxidot = Bdot_5 / eta1;
  Cxi_3 += dddexp3__tmp_1;
  alpha = ddOmega_1_tmp * ddOmega_1_tmp;
  theta_sqr = 1.0 / eta1 * Cxi_3 + alpha * -rt_powd_snf(eta1, -3.0);
  b_s_tmp = d6 / rt_powd_snf(eta1, 3.0);
  b_s = (2.0 * cx / xiTxidot + -2.0 / xiTxidot) + b_s_tmp;
  ddOmega_1_tmp = cx / xiTxidot;
  ddOmega_1 = (((8.0 / n_xi6 - 8.0 / n_xi6 * cx) - 5.0 * d6 / n_xi5) +
               ddOmega_1_tmp) * alpha + b_s * Cxi_3;
  xiTxidot = (3.0 / n_xi5 * d6 + -2.0 / xiTxidot) - ddOmega_1_tmp;
  n_xi5 = (((8.0 / n_xi6 - 15.0 * d6 / rt_powd_snf(eta1, 7.0)) + 7.0 * cx /
            n_xi6) + d6 / n_xi5) * alpha + xiTxidot * Cxi_3;
  dGamma_1 = fabs(eta1);
  if (dGamma_1 < 0.0001) {
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + i;
        B[f_i] = ((Cxi[i + 3] * 0.33333333333333331 * Cxi[3 * e_i + 1] +
                   0.33333333333333331 * Cxi[i] * Cxi[3 * e_i]) + Cxi[i + 6] *
                  0.33333333333333331 * Cxi[3 * e_i + 2]) + Bdot[f_i] * 0.5;
      }
    }
  } else {
    b_s = b_s * Bdot_4 * 2.0;
    n_xi6 = 1.0 / n_xi2 - cx / n_xi2;
    xiTxidot = xiTxidot * Bdot_4 * 2.0;
    n_xi2 = 1.0 / n_xi2 - b_s_tmp;
    for (i = 0; i < 3; i++) {
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + 1;
        e_j = 3 * e_i + 2;
        A6_tmp = 3 * e_i + i;
        Cxi_3 = ceil_xi[i + 3];
        Bdot_4 = ceil_xi[f_i];
        b_s_tmp = ceil_xi[3 * e_i];
        Bdot_5 = ceil_xi[i + 6];
        B_tmp = ceil_xi[e_j];
        B[A6_tmp] = ((((Cxi[i + 3] * 2.0 * Cxi[f_i] + 2.0 * Cxi[i] * Cxi[3 * e_i])
                       + Cxi[i + 6] * 2.0 * Cxi[e_j]) + ((Bdot[i + 3] * Bdot_4 +
          b_s_tmp * Bdot[i]) + Bdot[i + 6] * B_tmp)) + ((Bdot[3 * e_i] *
          ceil_xi[i] + Bdot[f_i] * Cxi_3) + Bdot[e_j] * Bdot_5)) * n_xi2 +
          ((((Cxi_3 * n_xi5 * Bdot_4 + n_xi5 * ceil_xi[i] * b_s_tmp) + Bdot_5 *
             n_xi5 * B_tmp) + ((ceil_xi[A6_tmp] * ddOmega_1 + Cxi[A6_tmp] * b_s)
             + Bdot[A6_tmp] * n_xi6)) + dddexp3_[A6_tmp] * xiTxidot);
      }
    }
  }

  if (dGamma_1 < 0.01) {
    d6 = -Bdot_3 / 60.0 * 2.0;
    for (i = 0; i < 3; i++) {
      ddBA_0 = ddBA[i + 3];
      ddBA_1 = ddBA[i];
      ddBA_2 = ddBA[i + 6];
      n_xi5 = AB[i];
      dBA_1 = AB[i + 3];
      AB_4 = AB[i + 6];
      ddOmega_1 = etadot_[i + 3];
      alpha = etadot_[i];
      dAB_1 = etadot_[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        f_i = 3 * e_i + 1;
        e_j = 3 * e_i + 2;
        A6_tmp = 3 * e_i + i;
        ddOmega_1_tmp = ceil_xi[f_i];
        b_s = ceil_xi[3 * e_i];
        dddexp3__tmp = ceil_xi[e_j];
        Bdot_0[A6_tmp] = (((Bdot[i + 3] * dBA[f_i] + dBA[3 * e_i] * Bdot[i]) +
                           Bdot[i + 6] * dBA[e_j]) + ((ceil_xi[i + 3] * AA[f_i]
          + AA[3 * e_i] * ceil_xi[i]) + ceil_xi[i + 6] * AA[e_j])) +
          ((ddOmega_1_tmp * ddBA_0 + b_s * ddBA_1) + dddexp3__tmp * ddBA_2);
        Bdot_3 = Bdot[3 * e_i];
        AB_3 = n_xi5 * Bdot_3;
        beta = dBA[i] * Bdot_3;
        Bdot_3 = Bdot[f_i];
        AB_3 += dBA_1 * Bdot_3;
        beta += dBA[i + 3] * Bdot_3;
        Bdot_3 = Bdot[e_j];
        dBA_0[A6_tmp] = dBA[i + 6] * Bdot_3 + beta;
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd
          (Cxi[f_i], eta_[f_i]), _mm_set_pd(ddOmega_1, Cxi[i + 3])), _mm_mul_pd
          (_mm_set_pd(Cxi[3 * e_i], eta_[3 * e_i]), _mm_set_pd(alpha, Cxi[i]))),
          _mm_mul_pd(_mm_set_pd(Cxi[e_j], eta_[e_j]), _mm_set_pd(dAB_1, Cxi[i +
          6]))));
        Cxi_0[A6_tmp] = tmp_f[0];
        dAB_0[A6_tmp] = tmp_f[1];
        AB_0[A6_tmp] = AB_4 * Bdot_3 + AB_3;
        dddexp3__tmp_0 = dAB[A6_tmp];
        AA_0[A6_tmp] = (((AA[i + 3] * ddOmega_1_tmp + b_s * AA[i]) + AA[i + 6] *
                         dddexp3__tmp) + dddexp3__tmp_0) + dddexp3__tmp_0;
        c_b[e_i + 6 * i] = B[3 * i + e_i];
      }
    }

    for (i = 0; i < 3; i++) {
      e_i = (i + 3) * 6;
      c_b[e_i] = ((((eta_[3 * i] + etadot_[3 * i]) * d6 + etaddot_[3 * i] * 0.5)
                   + (AA[3 * i] + ddBA[3 * i]) * 0.16666666666666666) + ((dAB_0
        [3 * i] + Cxi_0[3 * i]) * 2.0 + (Bdot_0[3 * i] + AB_0[3 * i])) *
                  0.041666666666666664) + (AA_0[3 * i] + dBA_0[3 * i]) *
        0.041666666666666664;
      c_b[6 * i + 3] = 0.0;
      c_b[e_i + 3] = B[3 * i];
      A6_tmp = 3 * i + 1;
      c_b[e_i + 1] = ((((eta_[A6_tmp] + etadot_[A6_tmp]) * d6 + etaddot_[A6_tmp]
                        * 0.5) + (AA[A6_tmp] + ddBA[A6_tmp]) *
                       0.16666666666666666) + ((dAB_0[A6_tmp] + Cxi_0[A6_tmp]) *
        2.0 + (Bdot_0[A6_tmp] + AB_0[A6_tmp])) * 0.041666666666666664) +
        (AA_0[A6_tmp] + dBA_0[A6_tmp]) * 0.041666666666666664;
      c_b[6 * i + 4] = 0.0;
      c_b[e_i + 4] = B[A6_tmp];
      A6_tmp = 3 * i + 2;
      c_b[e_i + 2] = ((((eta_[A6_tmp] + etadot_[A6_tmp]) * d6 + etaddot_[A6_tmp]
                        * 0.5) + (AA[A6_tmp] + ddBA[A6_tmp]) *
                       0.16666666666666666) + ((dAB_0[A6_tmp] + Cxi_0[A6_tmp]) *
        2.0 + (Bdot_0[A6_tmp] + AB_0[A6_tmp])) * 0.041666666666666664) +
        (AA_0[A6_tmp] + dBA_0[A6_tmp]) * 0.041666666666666664;
      c_b[6 * i + 5] = 0.0;
      c_b[e_i + 5] = B[A6_tmp];
    }
  } else {
    b_s_tmp = normxidot * normxidot;
    b_s = b_s_tmp * ddGamma_1 + dGamma_2 * theta_sqr;
    n_xi6 = 2.0 * dGamma_2 * normxidot;
    xiTxidot = b_s_tmp * tmp + b_dGamma_2 * theta_sqr;
    n_xi2 = 2.0 * b_dGamma_2 * normxidot;
    dGamma_1 = ((-(-((((24.0 * eta1 - 60.0 * d6) - xn_0[2] * cx) + 9.0 * xn2sx)
                     + 36.0 * dddexp3__tmp) / xn_0[5]) / eta1 + 2.0 * ddGamma_1 /
                 xn_0[1]) - 2.0 * dGamma_2 / xn_0[2]) * b_s_tmp + xixidot *
      theta_sqr;
    dGamma_2 = 2.0 * xixidot * normxidot;
    d6 = (-(((((120.0 * cx - 12.0 * beta_2) - xn_0[2] * d6) + 60.0 *
              dddexp3__tmp_0) + 12.0 * xn_0[1]) - 120.0) / xn_0[6] * eta1 + 3.0 *
          tmp) * b_s_tmp + beta * theta_sqr;
    eta1 = 2.0 * beta * normxidot;
    for (i = 0; i < 3; i++) {
      ddGamma_1 = BA[i];
      B_tmp = BA[i + 3];
      BA_4 = BA[i + 6];
      xixidot = dddexp3_[i + 3];
      dAB_1 = dddexp3_[i];
      dddexp3__0 = dddexp3_[i + 6];
      ddBA_0 = ddBA[i + 3];
      ddBA_1 = ddBA[i];
      ddBA_2 = ddBA[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        beta = dBA[3 * e_i];
        beta_2 = ceil_xi[i] * beta;
        n_xi5 = AB[i];
        dddexp3__tmp = ceil_xi[3 * e_i];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(Cxi[i], dddexp3__tmp),
          _mm_set_pd(beta, n_xi5)));
        dBA_1 = tmp_f[0];
        tmp = tmp_f[1];
        normxidot = eta_[3 * e_i];
        ceil_xi_2 = ceil_xi[i] * normxidot;
        ddOmega_1 = etadot_[i];
        dddexp3__tmp_0 = Cxi[3 * e_i];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(dddexp3__tmp_0,
          dddexp3__tmp), _mm_set_pd(n_xi5, ddOmega_1)));
        ddOmega_1_tmp = tmp_f[0];
        AB_4 = tmp_f[1];
        dddexp3__tmp_1 = Bdot[3 * e_i];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(dddexp3__tmp_1, Bdot[i]),
          _mm_set_pd(n_xi5, beta)));
        Bdot_3 = tmp_f[0];
        AB_3 = tmp_f[1];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(Cxi[i], dddexp3__tmp_0),
          _mm_set_pd(normxidot, ddOmega_1)));
        alpha = tmp_f[0];
        Cxi_2 = tmp_f[1];
        Cxi_3 = AB[3 * e_i];
        _mm_storeu_pd(&tmp_f[0], _mm_mul_pd(_mm_set_pd(etadot_[3 * e_i], Cxi_3),
          _mm_set1_pd(ddGamma_1)));
        BA_3 = tmp_f[0];
        cx = tmp_f[1];
        f_i = 3 * e_i + 1;
        beta = dBA[f_i];
        ceil_xi_tmp = ceil_xi[i + 3];
        beta_2 += ceil_xi_tmp * beta;
        n_xi5 = AB[i + 3];
        Bdot_4 = ceil_xi[f_i];
        Bdot_5 = Cxi[i + 3];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Bdot_5, Bdot_4),
          _mm_set_pd(beta, n_xi5)), _mm_set_pd(tmp, dBA_1)));
        dBA_1 = tmp_f[0];
        tmp = tmp_f[1];
        normxidot = eta_[f_i];
        ceil_xi_2 += ceil_xi_tmp * normxidot;
        ddOmega_1 = etadot_[i + 3];
        xcx_tmp = Cxi[f_i];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(xcx_tmp,
          Bdot_4), _mm_set_pd(n_xi5, ddOmega_1)), _mm_set_pd(AB_4, ddOmega_1_tmp)));
        ddOmega_1_tmp = tmp_f[0];
        AB_4 = tmp_f[1];
        tmp_k = Bdot[i + 3];
        tmp_l = Bdot[f_i];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(tmp_l, tmp_k),
          _mm_set_pd(n_xi5, beta)), _mm_set_pd(AB_3, Bdot_3)));
        Bdot_3 = tmp_f[0];
        AB_3 = tmp_f[1];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Bdot_5,
          xcx_tmp), _mm_set_pd(normxidot, ddOmega_1)), _mm_set_pd(Cxi_2, alpha)));
        alpha = tmp_f[0];
        Cxi_2 = tmp_f[1];
        tmp_j = AB[f_i];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(etadot_[f_i],
          tmp_j), _mm_set1_pd(B_tmp)), _mm_set_pd(cx, BA_3)));
        e_j = 3 * e_i + 2;
        beta = dBA[e_j];
        n_xi5 = AB[i + 6];
        normxidot = eta_[e_j];
        ddOmega_1 = etadot_[i + 6];
        A6_tmp = 3 * e_i + i;
        BA_1[A6_tmp] = etadot_[e_j] * BA_4 + tmp_f[1];
        b_s_tmp = AB[e_j];
        B_0[A6_tmp] = (tmp_j * xixidot + Cxi_3 * dAB_1) + b_s_tmp * dddexp3__0;
        Cxi_3 = Cxi[i + 6];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Cxi_3, b_s_tmp),
          _mm_set_pd(normxidot, BA_4)), _mm_set_pd(Cxi_2, tmp_f[0])));
        BA_0[A6_tmp] = tmp_f[0];
        Cxi_1[A6_tmp] = tmp_f[1];
        tmp_j = Cxi[e_j];
        Cxi_2 = Bdot[e_j];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Cxi_2, tmp_j),
          _mm_set_pd(n_xi5, ddOmega_1)), _mm_set_pd(AB_3, alpha)));
        dAB_0[A6_tmp] = tmp_f[0];
        AB_2[A6_tmp] = tmp_f[1];
        AB_3 = ceil_xi[e_j];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(AB_3, tmp_j),
          _mm_set1_pd(n_xi5)), _mm_set_pd(dBA_1, AB_4)));
        AB_1[A6_tmp] = tmp_f[0];
        AB_0[A6_tmp] = tmp_f[1];
        dBA_1 = ceil_xi[i + 6];
        ceil_xi_0[A6_tmp] = dBA_1 * beta + beta_2;
        Cxi_0[A6_tmp] = ((Cxi_3 * beta + tmp) + (dBA_1 * normxidot + ceil_xi_2))
          + (AB_3 * ddOmega_1 + ddOmega_1_tmp);
        ddOmega_1_tmp = Bdot[i + 6];
        Bdot_0[A6_tmp] = (((AA[3 * e_i] * ceil_xi[i] + AA[f_i] * ceil_xi_tmp) +
                           AA[e_j] * dBA_1) + (ddOmega_1_tmp * beta + Bdot_3)) +
          ((Bdot_4 * ddBA_0 + dddexp3__tmp * ddBA_1) + AB_3 * ddBA_2);
        Bdot_1[A6_tmp] = (((Bdot_5 * 2.0 * xcx_tmp + 2.0 * Cxi[i] *
                            dddexp3__tmp_0) + Cxi_3 * 2.0 * tmp_j) + ((Bdot_4 *
          tmp_k + dddexp3__tmp * Bdot[i]) + AB_3 * ddOmega_1_tmp)) + ((tmp_l *
          ceil_xi_tmp + dddexp3__tmp_1 * ceil_xi[i]) + Cxi_2 * dBA_1);
      }

      beta = dBA[i];
      dBA_1 = dBA[i + 3];
      alpha = dBA[i + 6];
      normxidot = eta_[i];
      b_s_tmp = eta_[i + 3];
      theta_sqr = eta_[i + 6];
      cx = AA[i];
      xn2sx = AA[i + 3];
      b_dGamma_2 = AA[i + 6];
      Bdot_3 = Bdot_1[i + 3];
      Bdot_4 = Bdot_1[i];
      Bdot_5 = Bdot_1[i + 6];
      for (e_i = 0; e_i < 3; e_i++) {
        beta_2 = ceil_xi[3 * e_i];
        ddOmega_1 = beta * beta_2;
        n_xi5 = normxidot * beta_2;
        dddexp3__tmp = cx * beta_2;
        A6_tmp = 3 * e_i + 1;
        beta_2 = ceil_xi[A6_tmp];
        ddOmega_1 += dBA_1 * beta_2;
        n_xi5 += b_s_tmp * beta_2;
        dddexp3__tmp += xn2sx * beta_2;
        e_j = 3 * e_i + 2;
        beta_2 = ceil_xi[e_j];
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Cxi[e_j],
          Bdot[e_j]), _mm_set1_pd(alpha)), _mm_set_pd(Cxi[A6_tmp] * dBA_1 + Cxi
          [3 * e_i] * beta, Bdot[A6_tmp] * dBA_1 + Bdot[3 * e_i] * beta)));
        f_i = 3 * e_i + i;
        BA_2[f_i] = tmp_f[0];
        ceil_xi_1[f_i] = tmp_f[1];
        eta__0[f_i] = theta_sqr * beta_2 + n_xi5;
        dBA_0[f_i] = alpha * beta_2 + ddOmega_1;
        Bdot_2[f_i] = (((AB[3 * e_i] * Bdot_4 + AB[A6_tmp] * Bdot_3) + AB[e_j] *
                        Bdot_5) + ((xixidot * 2.0 * etadot_[A6_tmp] + 2.0 *
          dAB_1 * etadot_[3 * e_i]) + dddexp3__0 * 2.0 * etadot_[e_j])) +
          ((ddBA[3 * e_i] * ddGamma_1 + ddBA[A6_tmp] * B_tmp) + ddBA[e_j] * BA_4);
        dddexp3__tmp_0 = dAB[f_i];
        AA_0[f_i] = ((b_dGamma_2 * beta_2 + dddexp3__tmp) + dddexp3__tmp_0) +
          dddexp3__tmp_0;
        c_b[e_i + 6 * i] = B[3 * i + e_i];
      }
    }

    for (i = 0; i < 3; i++) {
      e_i = (i + 3) * 6;
      c_b[e_i] = ((((((((((((dBA[3 * i] + AB[3 * i]) * b_s + etaddot_[3 * i] *
                            0.5) + (eta_[3 * i] + etadot_[3 * i]) * n_xi6) +
                          (AA[3 * i] + ddBA[3 * i]) * Gamma_1) + (ceil_xi_0[3 *
        i] + AB_0[3 * i]) * xiTxidot) + (Cxi_0[3 * i] + AB_1[3 * i]) * n_xi2) +
                       ((dAB_0[3 * i] + Cxi_1[3 * i]) * 2.0 + (Bdot_0[3 * i] +
        AB_2[3 * i])) * Gamma_2) + BA_0[3 * i] * dGamma_1) + (B_0[3 * i] + BA_1
        [3 * i]) * dGamma_2) + Bdot_2[3 * i] * Gamma_3) + dBA_0[3 * i] * d6) +
                  (eta__0[3 * i] + ceil_xi_1[3 * i]) * eta1) + (AA_0[3 * i] +
        BA_2[3 * i]) * Gamma_4;
      c_b[6 * i + 3] = 0.0;
      c_b[e_i + 3] = B[3 * i];
      A6_tmp = 3 * i + 1;
      c_b[e_i + 1] = ((((((((((((dBA[A6_tmp] + AB[A6_tmp]) * b_s +
        etaddot_[A6_tmp] * 0.5) + (eta_[A6_tmp] + etadot_[A6_tmp]) * n_xi6) +
        (AA[A6_tmp] + ddBA[A6_tmp]) * Gamma_1) + (ceil_xi_0[A6_tmp] +
        AB_0[A6_tmp]) * xiTxidot) + (Cxi_0[A6_tmp] + AB_1[A6_tmp]) * n_xi2) +
                           ((dAB_0[A6_tmp] + Cxi_1[A6_tmp]) * 2.0 +
                            (Bdot_0[A6_tmp] + AB_2[A6_tmp])) * Gamma_2) +
                          BA_0[A6_tmp] * dGamma_1) + (B_0[A6_tmp] + BA_1[A6_tmp])
                         * dGamma_2) + Bdot_2[A6_tmp] * Gamma_3) + dBA_0[A6_tmp]
                       * d6) + (eta__0[A6_tmp] + ceil_xi_1[A6_tmp]) * eta1) +
        (AA_0[A6_tmp] + BA_2[A6_tmp]) * Gamma_4;
      c_b[6 * i + 4] = 0.0;
      c_b[e_i + 4] = B[A6_tmp];
      A6_tmp = 3 * i + 2;
      c_b[e_i + 2] = ((((((((((((dBA[A6_tmp] + AB[A6_tmp]) * b_s +
        etaddot_[A6_tmp] * 0.5) + (eta_[A6_tmp] + etadot_[A6_tmp]) * n_xi6) +
        (AA[A6_tmp] + ddBA[A6_tmp]) * Gamma_1) + (ceil_xi_0[A6_tmp] +
        AB_0[A6_tmp]) * xiTxidot) + (Cxi_0[A6_tmp] + AB_1[A6_tmp]) * n_xi2) +
                           ((dAB_0[A6_tmp] + Cxi_1[A6_tmp]) * 2.0 +
                            (Bdot_0[A6_tmp] + AB_2[A6_tmp])) * Gamma_2) +
                          BA_0[A6_tmp] * dGamma_1) + (B_0[A6_tmp] + BA_1[A6_tmp])
                         * dGamma_2) + Bdot_2[A6_tmp] * Gamma_3) + dBA_0[A6_tmp]
                       * d6) + (eta__0[A6_tmp] + ceil_xi_1[A6_tmp]) * eta1) +
        (AA_0[A6_tmp] + BA_2[A6_tmp]) * Gamma_4;
      c_b[6 * i + 5] = 0.0;
      c_b[e_i + 5] = B[A6_tmp];
    }
  }

  for (i = 0; i < 6; i++) {
    for (e_i = 0; e_i < 6; e_i++) {
      dddexp3__tmp = 0.0;
      n_xi2 = 0.0;
      for (A6_tmp = 0; A6_tmp < 6; A6_tmp++) {
        dAB_tmp = 6 * e_i + A6_tmp;
        dddexp3__tmp_0 = dlog6mat[6 * A6_tmp + i];
        dddexp3__tmp += -2.0 * dddexp3__tmp_0 * b[dAB_tmp];
        n_xi2 += c_b[dAB_tmp] * dddexp3__tmp_0;
      }

      f_i = 6 * e_i + i;
      dlog6mat_0[f_i] = n_xi2;
      tmp_1[f_i] = dddexp3__tmp;
    }

    for (e_i = 0; e_i < 6; e_i++) {
      dddexp3__tmp = 0.0;
      n_xi2 = 0.0;
      for (A6_tmp = 0; A6_tmp < 6; A6_tmp++) {
        dAB_tmp = 6 * A6_tmp + i;
        f_i = 6 * e_i + A6_tmp;
        _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
          (dlog6mat_0[dAB_tmp], tmp_1[dAB_tmp]), _mm_set_pd(dlog6mat[f_i],
          b_b[f_i])), _mm_set_pd(n_xi2, dddexp3__tmp)));
        dddexp3__tmp = tmp_f[0];
        n_xi2 = tmp_f[1];
      }

      f_i = 6 * e_i + i;
      dlog6mat_1[f_i] = n_xi2;
      tmp_2[f_i] = dddexp3__tmp;
    }
  }

  /* Outport: '<Root>/dddexp6inv' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function13'
   */
  for (i = 0; i <= 34; i += 2) {
    /* MATLAB Function: '<Root>/MATLAB Function13' */
    tmp_b = _mm_loadu_pd(&tmp_2[i]);
    tmp_c = _mm_loadu_pd(&dlog6mat_1[i]);
    _mm_storeu_pd(&LR_Y.dddexp6inv[i], _mm_sub_pd(tmp_b, tmp_c));
  }

  /* End of Outport: '<Root>/dddexp6inv' */

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Inport: '<Root>/log6_T'
   */
  cx = 0.0;
  for (i = 0; i < 16; i++) {
    A[i] = LR_U.log6_T[i];
    d6 = fabs(LR_U.log6_T[i]);
    if (rtIsNaN(d6) || (d6 > cx)) {
      cx = d6;
    }
  }

  if ((!rtIsInf(cx)) && (!rtIsNaN(cx))) {
    LR_svd(LR_U.log6_T, w);
    cx = w[0];
  }

  if (cx < 2.2204460492503131E-16) {
    memset(&A[0], 0, sizeof(real_T) << 4U);
    A[0] = 1.0;
    A[5] = 1.0;
    A[10] = 1.0;
    A[15] = 1.0;
  }

  recomputeDiags = true;
  for (e_j = 0; e_j < 16; e_j++) {
    se3mat[e_j].re = 0.0;
    se3mat[e_j].im = 0.0;
    if (recomputeDiags && ((!rtIsInf(A[e_j])) && (!rtIsNaN(A[e_j])))) {
    } else {
      recomputeDiags = false;
    }
  }

  if (!recomputeDiags) {
    for (i = 0; i < 16; i++) {
      se3mat[i].re = (rtNaN);
      se3mat[i].im = 0.0;
    }
  } else {
    e_j = 3;
    while (recomputeDiags && (e_j <= 4)) {
      f_i = e_j;
      while (recomputeDiags && (f_i <= 4)) {
        recomputeDiags = (A[(((e_j - 3) << 2) + f_i) - 1] == 0.0);
        f_i++;
      }

      e_j++;
    }

    if (recomputeDiags) {
      e_j = 1;
      exitg2 = false;
      while ((!exitg2) && (e_j - 1 < 3)) {
        i = ((e_j - 1) << 2) + e_j;
        ddOmega_1 = A[i];
        if (ddOmega_1 != 0.0) {
          if ((e_j != 3) && (A[((e_j << 2) + e_j) + 1] != 0.0)) {
            recomputeDiags = false;
            exitg2 = true;
          } else {
            e_i = (e_j << 2) + e_j;
            if (A[i - 1] != A[e_i]) {
              recomputeDiags = false;
              exitg2 = true;
            } else {
              n_xi2 = A[e_i - 1];
              if (rtIsNaN(ddOmega_1)) {
                dddexp3__tmp = (rtNaN);
              } else if (ddOmega_1 < 0.0) {
                dddexp3__tmp = -1.0;
              } else {
                dddexp3__tmp = (ddOmega_1 > 0.0);
              }

              if (rtIsNaN(n_xi2)) {
                dddexp3__tmp_0 = (rtNaN);
              } else if (n_xi2 < 0.0) {
                dddexp3__tmp_0 = -1.0;
              } else {
                dddexp3__tmp_0 = (n_xi2 > 0.0);
              }

              if (dddexp3__tmp * dddexp3__tmp_0 != -1.0) {
                recomputeDiags = false;
                exitg2 = true;
              } else {
                e_j++;
              }
            }
          }
        } else {
          e_j++;
        }
      }
    }

    if (recomputeDiags) {
      memcpy(&A4[0], &A[0], sizeof(real_T) << 4U);
    } else {
      memcpy(&A4[0], &A[0], sizeof(real_T) << 4U);
      LR_schur_o(A4, A2);
    }

    LR_ordeig(A4, d);
    isPrincipalLog = true;
    e_j = 0;
    exitg2 = false;
    while ((!exitg2) && (e_j < 4)) {
      if ((d[e_j].re <= 0.0) && (d[e_j].im == 0.0)) {
        isPrincipalLog = false;
        exitg2 = true;
      } else {
        e_j++;
      }
    }

    p = true;
    e_j = 0;
    exitg2 = false;
    while ((!exitg2) && (e_j < 4)) {
      f_i = 0;
      do {
        exitg1 = 0;
        if (f_i <= e_j) {
          if (!(A[(e_j << 2) + f_i] == A[(f_i << 2) + e_j])) {
            p = false;
            exitg1 = 1;
          } else {
            f_i++;
          }
        } else {
          e_j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    guard1 = false;
    guard2 = false;
    if (p) {
      guard2 = true;
    } else {
      e_j = 0;
      do {
        exitg3 = 0;
        if (e_j < 4) {
          e_i = 0;
          do {
            exitg1 = 0;
            if (e_i <= e_j - 1) {
              if (A4[(e_j << 2) + e_i] != 0.0) {
                guard1 = true;
                exitg1 = 1;
              } else {
                e_i++;
              }
            } else {
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg3 = 1;
          } else if ((e_j + 1 != 4) && (A4[((e_j << 2) + e_j) + 1] != 0.0)) {
            guard1 = true;
            exitg3 = 1;
          } else {
            e_j++;
          }
        } else {
          guard2 = true;
          exitg3 = 1;
        }
      } while (exitg3 == 0);
    }

    if (guard2) {
      if (recomputeDiags) {
        for (e_i = 0; e_i < 4; e_i++) {
          n_xi2 = d[e_i].re;
          eta1 = d[e_i].im;
          if (eta1 == 0.0) {
            if (n_xi2 < 0.0) {
              i = (e_i << 2) + e_i;
              se3mat[i].re = log(fabs(n_xi2));
              se3mat[i].im = 3.1415926535897931;
            } else {
              i = (e_i << 2) + e_i;
              se3mat[i].re = log(n_xi2);
              se3mat[i].im = 0.0;
            }
          } else if ((fabs(n_xi2) > 8.9884656743115785E+307) || (fabs(eta1) >
                      8.9884656743115785E+307)) {
            i = (e_i << 2) + e_i;
            se3mat[i].re = log(rt_hypotd_snf(n_xi2 / 2.0, eta1 / 2.0)) +
              0.69314718055994529;
            se3mat[i].im = rt_atan2d_snf(eta1, n_xi2);
          } else {
            i = (e_i << 2) + e_i;
            se3mat[i].re = log(rt_hypotd_snf(n_xi2, eta1));
            se3mat[i].im = rt_atan2d_snf(eta1, n_xi2);
          }
        }
      } else {
        for (e_j = 0; e_j < 4; e_j++) {
          n_xi2 = d[e_j].re;
          eta1 = d[e_j].im;
          if (eta1 == 0.0) {
            if (n_xi2 < 0.0) {
              ddOmega_1 = log(fabs(n_xi2));
              n_xi2 = 3.1415926535897931;
            } else {
              ddOmega_1 = log(n_xi2);
              n_xi2 = 0.0;
            }
          } else if ((fabs(n_xi2) > 8.9884656743115785E+307) || (fabs(eta1) >
                      8.9884656743115785E+307)) {
            ddOmega_1 = log(rt_hypotd_snf(n_xi2 / 2.0, eta1 / 2.0)) +
              0.69314718055994529;
            n_xi2 = rt_atan2d_snf(eta1, n_xi2);
          } else {
            ddOmega_1 = log(rt_hypotd_snf(n_xi2, eta1));
            n_xi2 = rt_atan2d_snf(eta1, n_xi2);
          }

          i = e_j << 2;
          tmp_b = _mm_set_pd(n_xi2, ddOmega_1);
          _mm_storeu_pd((real_T *)&se3mat[i], _mm_mul_pd(_mm_set1_pd(A2[i]),
            tmp_b));
          U[i].re = A2[e_j];
          U[i].im = 0.0;
          _mm_storeu_pd((real_T *)&se3mat[i + 1], _mm_mul_pd(_mm_set1_pd(A2[i +
            1]), tmp_b));
          U[i + 1].re = A2[e_j + 4];
          U[i + 1].im = 0.0;
          _mm_storeu_pd((real_T *)&se3mat[i + 2], _mm_mul_pd(_mm_set1_pd(A2[i +
            2]), tmp_b));
          U[i + 2].re = A2[e_j + 8];
          U[i + 2].im = 0.0;
          _mm_storeu_pd((real_T *)&se3mat[i + 3], _mm_mul_pd(_mm_set1_pd(A2[i +
            3]), tmp_b));
          U[i + 3].re = A2[e_j + 12];
          U[i + 3].im = 0.0;
        }

        for (i = 0; i < 4; i++) {
          A6_tmp = i << 2;
          n_xi2 = U[A6_tmp + 1].re;
          ddOmega_1 = U[A6_tmp + 1].im;
          eta1 = U[A6_tmp].re;
          n_xi6 = U[A6_tmp].im;
          d6 = U[A6_tmp + 2].re;
          b_s = U[A6_tmp + 2].im;
          dddexp3__tmp = U[A6_tmp + 3].re;
          dGamma_1 = U[A6_tmp + 3].im;
          for (e_i = 0; e_i < 4; e_i++) {
            tmp_b = _mm_set_pd(1.0, -1.0);
            _mm_storeu_pd((real_T *)&b_0[e_i + A6_tmp], _mm_add_pd(_mm_add_pd
              (_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd(ddOmega_1, n_xi2),
              _mm_set1_pd(se3mat[e_i + 4].re)), _mm_mul_pd(_mm_mul_pd(_mm_set_pd
              (n_xi2, ddOmega_1), _mm_set1_pd(se3mat[e_i + 4].im)), tmp_b)),
                          _mm_add_pd(_mm_mul_pd(_mm_set_pd(n_xi6, eta1),
              _mm_set1_pd(se3mat[e_i].re)), _mm_mul_pd(_mm_mul_pd(_mm_set_pd
              (eta1, n_xi6), _mm_set1_pd(se3mat[e_i].im)), tmp_b))), _mm_add_pd
               (_mm_mul_pd(_mm_set_pd(b_s, d6), _mm_set1_pd(se3mat[e_i + 8].re)),
                _mm_mul_pd(_mm_mul_pd(_mm_set_pd(d6, b_s), _mm_set1_pd
              (se3mat[e_i + 8].im)), tmp_b))), _mm_add_pd(_mm_mul_pd(_mm_set_pd
              (dGamma_1, dddexp3__tmp), _mm_set1_pd(se3mat[e_i + 12].re)),
              _mm_mul_pd(_mm_mul_pd(_mm_set_pd(dddexp3__tmp, dGamma_1),
              _mm_set1_pd(se3mat[e_i + 12].im)), tmp_b))));
          }
        }

        memcpy(&se3mat[0], &b_0[0], sizeof(creal_T) << 4U);
      }
    }

    if (guard1) {
      if (isPrincipalLog) {
        LR_computeLogOfSchurForm(A4, d, A, &f_i);
        if (recomputeDiags) {
          for (i = 0; i < 16; i++) {
            se3mat[i].re = A[i];
            se3mat[i].im = 0.0;
          }
        } else {
          for (i = 0; i < 4; i++) {
            n_xi2 = A2[i + 4];
            ddOmega_1 = A2[i];
            eta1 = A2[i + 8];
            n_xi6 = A2[i + 12];
            for (e_i = 0; e_i <= 2; e_i += 2) {
              A6_tmp = (e_i + 1) << 2;
              dAB_tmp = e_i << 2;
              _mm_storeu_pd(&tmp_f[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                (_mm_mul_pd(_mm_set_pd(A[A6_tmp + 1], A[dAB_tmp + 1]),
                            _mm_set1_pd(n_xi2)), _mm_mul_pd(_mm_set_pd(A[A6_tmp],
                A[dAB_tmp]), _mm_set1_pd(ddOmega_1))), _mm_mul_pd(_mm_set_pd
                (A[A6_tmp + 2], A[dAB_tmp + 2]), _mm_set1_pd(eta1))), _mm_mul_pd
                (_mm_set_pd(A[A6_tmp + 3], A[dAB_tmp + 3]), _mm_set1_pd(n_xi6))));
              A4[i + dAB_tmp] = tmp_f[0];
              A4[i + A6_tmp] = tmp_f[1];
            }

            n_xi2 = A4[i + 4];
            ddOmega_1 = A4[i];
            eta1 = A4[i + 8];
            n_xi6 = A4[i + 12];
            for (e_i = 0; e_i < 4; e_i++) {
              A6_tmp = (e_i << 2) + i;
              se3mat[A6_tmp].re = ((A2[e_i + 4] * n_xi2 + ddOmega_1 * A2[e_i]) +
                                   A2[e_i + 8] * eta1) + A2[e_i + 12] * n_xi6;
              se3mat[A6_tmp].im = 0.0;
            }
          }
        }
      } else {
        if (recomputeDiags) {
          memset(&A2[0], 0, sizeof(real_T) << 4U);
          A2[0] = 1.0;
          A2[5] = 1.0;
          A2[10] = 1.0;
          A2[15] = 1.0;
        }

        for (i = 0; i < 16; i++) {
          se3mat[i].re = A4[i];
          se3mat[i].im = 0.0;
          U[i].re = A2[i];
          U[i].im = 0.0;
        }

        for (f_i = 2; f_i >= 0; f_i--) {
          i = f_i << 2;
          A6_tmp = i + f_i;
          n_xi2 = A4[A6_tmp + 1];
          if (n_xi2 != 0.0) {
            b_s = A4[A6_tmp];
            dAB_tmp = (f_i + 1) << 2;
            e_i = dAB_tmp + f_i;
            d6 = A4[e_i];
            eta1 = n_xi2;
            n_xi6 = A4[e_i + 1];
            cx = n_xi6;
            LR_xdlanv2(&b_s, &d6, &eta1, &cx, &Gamma_1, &Gamma_2, &Gamma_3,
                       &Gamma_4, &dGamma_1, &dGamma_2);
            ddOmega_1 = Gamma_1 - n_xi6;
            d6 = rt_hypotd_snf(rt_hypotd_snf(ddOmega_1, Gamma_2), n_xi2);
            if (Gamma_2 == 0.0) {
              eta1 = ddOmega_1 / d6;
              n_xi6 = 0.0;
            } else if (ddOmega_1 == 0.0) {
              eta1 = 0.0;
              n_xi6 = Gamma_2 / d6;
            } else {
              eta1 = ddOmega_1 / d6;
              n_xi6 = Gamma_2 / d6;
            }

            ddOmega_1 = n_xi2 / d6;
            for (e_j = f_i + 1; e_j < 5; e_j++) {
              se3mat_re_tmp = ((e_j - 1) << 2) + f_i;
              n_xi2 = se3mat[se3mat_re_tmp].re;
              d6 = se3mat[se3mat_re_tmp].im;
              b_s = se3mat[se3mat_re_tmp + 1].re;
              dddexp3__tmp = se3mat[se3mat_re_tmp + 1].im;
              tmp_b = _mm_set1_pd(eta1);
              tmp_c = _mm_set_pd(dddexp3__tmp, b_s);
              tmp_d = _mm_set1_pd(n_xi6);
              tmp_e = _mm_set1_pd(ddOmega_1);
              tmp_3 = _mm_set_pd(d6, n_xi2);
              _mm_storeu_pd((real_T *)&se3mat[se3mat_re_tmp], _mm_add_pd
                            (_mm_add_pd(_mm_mul_pd(tmp_b, tmp_3), _mm_mul_pd
                (_mm_mul_pd(tmp_d, _mm_set_pd(n_xi2, d6)), _mm_set_pd(-1.0, 1.0))),
                             _mm_mul_pd(tmp_e, tmp_c)));
              _mm_storeu_pd((real_T *)&se3mat[se3mat_re_tmp + 1], _mm_sub_pd
                            (_mm_add_pd(_mm_mul_pd(tmp_b, tmp_c), _mm_mul_pd
                (_mm_mul_pd(tmp_d, _mm_set_pd(b_s, dddexp3__tmp)), _mm_set_pd
                 (1.0, -1.0))), _mm_mul_pd(tmp_e, tmp_3)));
            }

            for (e_i = 0; e_i <= f_i + 1; e_i++) {
              se3mat_re_tmp = i + e_i;
              n_xi2 = se3mat[se3mat_re_tmp].re;
              d6 = se3mat[se3mat_re_tmp].im;
              e_j = dAB_tmp + e_i;
              b_s = se3mat[e_j].re;
              dddexp3__tmp = se3mat[e_j].im;
              tmp_b = _mm_set1_pd(eta1);
              tmp_c = _mm_set_pd(dddexp3__tmp, b_s);
              tmp_d = _mm_set1_pd(n_xi6);
              tmp_e = _mm_set1_pd(ddOmega_1);
              tmp_3 = _mm_set_pd(d6, n_xi2);
              _mm_storeu_pd((real_T *)&se3mat[se3mat_re_tmp], _mm_add_pd
                            (_mm_add_pd(_mm_mul_pd(tmp_b, tmp_3), _mm_mul_pd
                (_mm_mul_pd(tmp_d, _mm_set_pd(n_xi2, d6)), _mm_set_pd(1.0, -1.0))),
                             _mm_mul_pd(tmp_e, tmp_c)));
              _mm_storeu_pd((real_T *)&se3mat[e_j], _mm_sub_pd(_mm_add_pd
                (_mm_mul_pd(tmp_b, tmp_c), _mm_mul_pd(_mm_mul_pd(tmp_d,
                _mm_set_pd(b_s, dddexp3__tmp)), _mm_set_pd(-1.0, 1.0))),
                _mm_mul_pd(tmp_e, tmp_3)));
            }

            n_xi2 = U[i].re;
            d6 = U[i].im;
            b_s = U[dAB_tmp].re;
            dddexp3__tmp = U[dAB_tmp].im;
            tmp_b = _mm_set1_pd(eta1);
            tmp_c = _mm_set_pd(dddexp3__tmp, b_s);
            tmp_d = _mm_set1_pd(n_xi6);
            tmp_e = _mm_set1_pd(ddOmega_1);
            tmp_3 = _mm_set_pd(d6, n_xi2);
            tmp_i = _mm_set_pd(1.0, -1.0);
            _mm_storeu_pd((real_T *)&U[i], _mm_add_pd(_mm_add_pd(_mm_mul_pd
              (tmp_b, tmp_3), _mm_mul_pd(_mm_mul_pd(tmp_d, _mm_set_pd(n_xi2, d6)),
              tmp_i)), _mm_mul_pd(tmp_e, tmp_c)));
            tmp_g = _mm_set_pd(-1.0, 1.0);
            _mm_storeu_pd((real_T *)&U[dAB_tmp], _mm_sub_pd(_mm_add_pd
              (_mm_mul_pd(tmp_b, tmp_c), _mm_mul_pd(_mm_mul_pd(tmp_d, _mm_set_pd
              (b_s, dddexp3__tmp)), tmp_g)), _mm_mul_pd(tmp_e, tmp_3)));
            n_xi2 = U[i + 1].re;
            d6 = U[i + 1].im;
            b_s = U[dAB_tmp + 1].re;
            dddexp3__tmp = U[dAB_tmp + 1].im;
            tmp_c = _mm_set_pd(dddexp3__tmp, b_s);
            tmp_3 = _mm_set_pd(d6, n_xi2);
            _mm_storeu_pd((real_T *)&U[i + 1], _mm_add_pd(_mm_add_pd(_mm_mul_pd
              (tmp_b, tmp_3), _mm_mul_pd(_mm_mul_pd(tmp_d, _mm_set_pd(n_xi2, d6)),
              tmp_i)), _mm_mul_pd(tmp_e, tmp_c)));
            _mm_storeu_pd((real_T *)&U[dAB_tmp + 1], _mm_sub_pd(_mm_add_pd
              (_mm_mul_pd(tmp_b, tmp_c), _mm_mul_pd(_mm_mul_pd(tmp_d, _mm_set_pd
              (b_s, dddexp3__tmp)), tmp_g)), _mm_mul_pd(tmp_e, tmp_3)));
            n_xi2 = U[i + 2].re;
            d6 = U[i + 2].im;
            b_s = U[dAB_tmp + 2].re;
            dddexp3__tmp = U[dAB_tmp + 2].im;
            tmp_c = _mm_set_pd(dddexp3__tmp, b_s);
            tmp_3 = _mm_set_pd(d6, n_xi2);
            _mm_storeu_pd((real_T *)&U[i + 2], _mm_add_pd(_mm_add_pd(_mm_mul_pd
              (tmp_b, tmp_3), _mm_mul_pd(_mm_mul_pd(tmp_d, _mm_set_pd(n_xi2, d6)),
              tmp_i)), _mm_mul_pd(tmp_e, tmp_c)));
            _mm_storeu_pd((real_T *)&U[dAB_tmp + 2], _mm_sub_pd(_mm_add_pd
              (_mm_mul_pd(tmp_b, tmp_c), _mm_mul_pd(_mm_mul_pd(tmp_d, _mm_set_pd
              (b_s, dddexp3__tmp)), tmp_g)), _mm_mul_pd(tmp_e, tmp_3)));
            n_xi2 = U[i + 3].re;
            d6 = U[i + 3].im;
            b_s = U[dAB_tmp + 3].re;
            dddexp3__tmp = U[dAB_tmp + 3].im;
            tmp_c = _mm_set_pd(dddexp3__tmp, b_s);
            tmp_3 = _mm_set_pd(d6, n_xi2);
            _mm_storeu_pd((real_T *)&U[i + 3], _mm_add_pd(_mm_add_pd(_mm_mul_pd
              (tmp_b, tmp_3), _mm_mul_pd(_mm_mul_pd(tmp_d, _mm_set_pd(n_xi2, d6)),
              tmp_i)), _mm_mul_pd(tmp_e, tmp_c)));
            _mm_storeu_pd((real_T *)&U[dAB_tmp + 3], _mm_sub_pd(_mm_add_pd
              (_mm_mul_pd(tmp_b, tmp_c), _mm_mul_pd(_mm_mul_pd(tmp_d, _mm_set_pd
              (b_s, dddexp3__tmp)), tmp_g)), _mm_mul_pd(tmp_e, tmp_3)));
            se3mat[A6_tmp + 1].re = 0.0;
            se3mat[A6_tmp + 1].im = 0.0;
          }
        }

        LR_computeLogOfSchurForm_m(se3mat, d, b_0, &f_i);
        for (i = 0; i < 4; i++) {
          n_xi2 = U[i + 4].re;
          ddOmega_1 = U[i + 4].im;
          eta1 = U[i].re;
          n_xi6 = U[i].im;
          d6 = U[i + 8].re;
          b_s = U[i + 8].im;
          dGamma_1 = U[i + 12].re;
          Gamma_2 = U[i + 12].im;
          for (e_i = 0; e_i < 4; e_i++) {
            A6_tmp = e_i << 2;
            tmp_b = _mm_set_pd(1.0, -1.0);
            tmp_b = _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
              (_mm_loadu_pd((const real_T *)&b_0[A6_tmp + 1]), _mm_set1_pd(n_xi2)),
              _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)
              &b_0[A6_tmp + 1]), _mm_loadu_pd((const real_T *)&b_0[A6_tmp + 1]),
              1), _mm_set1_pd(ddOmega_1)), tmp_b)), _mm_add_pd(_mm_mul_pd
              (_mm_loadu_pd((const real_T *)&b_0[A6_tmp]), _mm_set1_pd(eta1)),
              _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)
              &b_0[A6_tmp]), _mm_loadu_pd((const real_T *)&b_0[A6_tmp]), 1),
              _mm_set1_pd(n_xi6)), tmp_b))), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
              ((const real_T *)&b_0[A6_tmp + 2]), _mm_set1_pd(d6)), _mm_mul_pd
              (_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((const real_T *)
              &b_0[A6_tmp + 2]), _mm_loadu_pd((const real_T *)&b_0[A6_tmp + 2]),
              1), _mm_set1_pd(b_s)), tmp_b))), _mm_add_pd(_mm_mul_pd
              (_mm_loadu_pd((const real_T *)&b_0[A6_tmp + 3]), _mm_set1_pd
               (dGamma_1)), _mm_mul_pd(_mm_mul_pd(_mm_shuffle_pd(_mm_loadu_pd((
              const real_T *)&b_0[A6_tmp + 3]), _mm_loadu_pd((const real_T *)
              &b_0[A6_tmp + 3]), 1), _mm_set1_pd(Gamma_2)), tmp_b)));
            _mm_storeu_pd((real_T *)&U_0[i + A6_tmp], tmp_b);
          }

          n_xi2 = U_0[i + 4].re;
          ddOmega_1 = U_0[i + 4].im;
          eta1 = U_0[i].re;
          n_xi6 = U_0[i].im;
          d6 = U_0[i + 8].re;
          b_s = U_0[i + 8].im;
          dGamma_1 = U_0[i + 12].re;
          Gamma_2 = U_0[i + 12].im;
          for (e_i = 0; e_i < 4; e_i++) {
            dddexp3__tmp = U[e_i + 4].re;
            dddexp3__tmp_0 = -U[e_i + 4].im;
            ddOmega_1_tmp = U[e_i].re;
            dddexp3__tmp_1 = -U[e_i].im;
            tmp_b = _mm_set_pd(1.0, -1.0);
            Cxi_3 = U[e_i + 8].re;
            Bdot_4 = -U[e_i + 8].im;
            Bdot_5 = U[e_i + 12].re;
            xcx_tmp = -U[e_i + 12].im;
            _mm_storeu_pd((real_T *)&se3mat[i + (e_i << 2)], _mm_add_pd
                          (_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
              (_mm_set1_pd(n_xi2), _mm_set_pd(dddexp3__tmp_0, dddexp3__tmp)),
              _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(ddOmega_1), _mm_set_pd
              (dddexp3__tmp, dddexp3__tmp_0)), tmp_b)), _mm_add_pd(_mm_mul_pd
              (_mm_set1_pd(eta1), _mm_set_pd(dddexp3__tmp_1, ddOmega_1_tmp)),
              _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(n_xi6), _mm_set_pd(ddOmega_1_tmp,
              dddexp3__tmp_1)), tmp_b))), _mm_add_pd(_mm_mul_pd(_mm_set1_pd(d6),
              _mm_set_pd(Bdot_4, Cxi_3)), _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(b_s),
              _mm_set_pd(Cxi_3, Bdot_4)), tmp_b))), _mm_add_pd(_mm_mul_pd
              (_mm_set1_pd(dGamma_1), _mm_set_pd(xcx_tmp, Bdot_5)), _mm_mul_pd
              (_mm_mul_pd(_mm_set1_pd(Gamma_2), _mm_set_pd(Bdot_5, xcx_tmp)),
               tmp_b))));
          }
        }
      }
    }
  }

  /* Outport: '<Root>/lambda' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  LR_Y.lambda[0] = se3mat[12];
  LR_Y.lambda[1] = se3mat[13];
  LR_Y.lambda[2] = se3mat[14];
  LR_Y.lambda[3] = se3mat[6];
  LR_Y.lambda[4] = se3mat[8];
  LR_Y.lambda[5] = se3mat[1];

  /* Matfile logging */
  rt_UpdateTXYLogVars(LR_M->rtwLogInfo, (&LR_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.001s, 0.0s] */
    if ((rtmGetTFinal(LR_M)!=-1) &&
        !((rtmGetTFinal(LR_M)-LR_M->Timing.taskTime0) > LR_M->Timing.taskTime0 *
          (DBL_EPSILON))) {
      rtmSetErrorStatus(LR_M, "Simulation finished");
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
  if (!(++LR_M->Timing.clockTick0)) {
    ++LR_M->Timing.clockTickH0;
  }

  LR_M->Timing.taskTime0 = LR_M->Timing.clockTick0 * LR_M->Timing.stepSize0 +
    LR_M->Timing.clockTickH0 * LR_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void LR_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)LR_M, 0,
                sizeof(RT_MODEL_LR_T));
  rtmSetTFinal(LR_M, -1);
  LR_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    LR_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(LR_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(LR_M->rtwLogInfo, (NULL));
    rtliSetLogT(LR_M->rtwLogInfo, "tout");
    rtliSetLogX(LR_M->rtwLogInfo, "");
    rtliSetLogXFinal(LR_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(LR_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(LR_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(LR_M->rtwLogInfo, 0);
    rtliSetLogDecimation(LR_M->rtwLogInfo, 1);
    rtliSetLogY(LR_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(LR_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(LR_M->rtwLogInfo, (NULL));
  }

  /* external inputs */
  (void)memset(&LR_U, 0, sizeof(ExtU_LR_T));

  /* external outputs */
  (void)memset(&LR_Y, 0, sizeof(ExtY_LR_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(LR_M->rtwLogInfo, 0.0, rtmGetTFinal(LR_M),
    LR_M->Timing.stepSize0, (&rtmGetErrorStatus(LR_M)));
}

/* Model terminate function */
void LR_terminate(void)
{
  /* (no terminate code required) */
}
