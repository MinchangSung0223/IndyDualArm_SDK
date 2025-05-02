/*
 * TaskSpaceController.c
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

#include "TaskSpaceController.h"
#include "rtwtypes.h"
#include "TaskSpaceController_types.h"
#include <string.h>
#include <emmintrin.h>
#include <math.h>
#include "TaskSpaceController_private.h"
#include "rt_nonfinite.h"

/* Block signals (default storage) */
B_TaskSpaceController_T TaskSpaceController_B;

/* Block states (default storage) */
DW_TaskSpaceController_T TaskSpaceController_DW;

/* External inputs (root inport signals with default storage) */
ExtU_TaskSpaceController_T TaskSpaceController_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_TaskSpaceController_T TaskSpaceController_Y;

/* Real-time model */
static RT_MODEL_TaskSpaceController_T TaskSpaceController_M_;
RT_MODEL_TaskSpaceController_T *const TaskSpaceController_M =
  &TaskSpaceController_M_;

/* Forward declaration for local functions */
static real_T TaskSpaceController_norm(const real_T x[3]);
static void TaskSpaceController_xzgetrf(real_T A[36], int32_T ipiv[6], int32_T
  *info);
static real_T TaskSpaceController_xnrm2(int32_T n, const real_T x[175], int32_T
  ix0);
static real_T TaskSpaceController_xzlarfg(int32_T n, real_T *alpha1, real_T x
  [175], int32_T ix0);
static void TaskSpaceController_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C[175], int32_T ic0, real_T work[25]);
static void TaskSpaceController_qrf(real_T A[175], int32_T m, int32_T n, int32_T
  nfxd, real_T tau[7]);
static void TaskSpaceController_xgeqp3(real_T A[175], int32_T m, int32_T n,
  int32_T jpvt[25], real_T tau[7]);
static void TaskSpaceController_countsort(int32_T x[25], int32_T xLen, int32_T
  workspace[25], int32_T xMin, int32_T xMax);
static void TaskSpaceControlle_removeConstr(sAElXDmDj36R7Z42SImJxmG_TaskS_T *obj,
  int32_T idx_global);
static void TaskSpaceC_RemoveDependentIneq_(sAElXDmDj36R7Z42SImJxmG_TaskS_T
  *workingset, skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager,
  sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T *memspace, real_T tolfactor);
static void TaskSpaceController_computeQ_(skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *obj,
  int32_T nrows);
static int32_T TaskSpaceController_rank(const real_T qrmanager_QR[175], int32_T
  qrmanager_mrows, int32_T qrmanager_ncols);
static void TaskSpaceController_xgemv(int32_T m, const real_T A[168], const
  real_T x[175], real_T y[25]);
static void TaskSpaceController_xgemv_l(int32_T m, const real_T A[168], const
  real_T x[175], real_T y[25]);
static boolean_T TaskSpa_feasibleX0ForWorkingSet(real_T workspace[175], real_T
  xCurrent[7], sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager);
static void TaskSpaceController_xgemv_l1(int32_T m, const real_T A[168], const
  real_T x[7], real_T y[25]);
static real_T TaskSpac_maxConstraintViolation(sAElXDmDj36R7Z42SImJxmG_TaskS_T
  *obj, const real_T x[7]);
static void TaskSpa_modifyOverheadPhaseOne_(sAElXDmDj36R7Z42SImJxmG_TaskS_T *obj);
static void TaskSpaceControl_setProblemType(sAElXDmDj36R7Z42SImJxmG_TaskS_T *obj,
  int32_T PROBLEM_TYPE);
static void TaskSpaceController_xgemv_l1n(int32_T m, int32_T n, int32_T lda,
  const real_T x[7], real_T y[6]);
static void TaskSpaceCo_computeGrad_StoreHx(slzZ8M58FXlZqTD433BZJUH_TaskS_T *obj,
  const real_T f[6], const real_T x[7]);
static real_T TaskSpaceCo_computeFval_ReuseHx(const
  slzZ8M58FXlZqTD433BZJUH_TaskS_T *obj, real_T workspace[175], const real_T f[6],
  const real_T x[7]);
static void TaskSpaceController_factorQR(skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *obj,
  const real_T A[175], int32_T mrows, int32_T ncols);
static void TaskSpaceController_xrotg(real_T *a, real_T *b, real_T *c, real_T *s);
static void TaskSpaceCont_squareQ_appendCol(skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *obj,
  const real_T vec[175], int32_T iv0);
static void TaskSpaceContr_deleteColMoveEnd(skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *obj,
  int32_T idx);
static void TaskSpaceControlle_fullColLDL2_(s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T *obj,
  int32_T NColsRemain, real_T REG_PRIMAL);
static void TaskSpaceController_xgemv_l1nm(int32_T m, int32_T n, const real_T A
  [49], int32_T ia0, const real_T x[175], real_T y[7]);
static void TaskSpaceControl_compute_deltax(smNINkioqq1a7FyOE4CETSB_TaskS_T
  *solution, sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T *memspace, const
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager, s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T
  *cholmanager, const slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective);
static real_T TaskSpaceController_xnrm2_n(int32_T n, const real_T x[7]);
static void TaskSpaceController_xgemv_l1nmk(int32_T m, const real_T A[168],
  const real_T x[7], real_T y[175]);
static void Task_addBoundToActiveSetMatrix_(sAElXDmDj36R7Z42SImJxmG_TaskS_T *obj,
  int32_T TYPE, int32_T idx_local);
static void TaskSpaceControl_addAineqConstr(sAElXDmDj36R7Z42SImJxmG_TaskS_T *obj,
  int32_T idx_local);
static void TaskSpaceControl_compute_lambda(real_T workspace[175],
  smNINkioqq1a7FyOE4CETSB_TaskS_T *solution, const
  slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective, const
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager);
static void TaskSpaceController_phaseone(const real_T f[6],
  smNINkioqq1a7FyOE4CETSB_TaskS_T *solution, sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T
  *memspace, sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager, s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T
  *cholmanager, const sIOJhD9KwAkF5sEguPjYquC_TaskS_T *runTimeOptions,
  slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective);
static int32_T TaskSpaceCon_RemoveDependentEq_(sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T
  *memspace, const sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager);
static void TaskSpaceController_ratiotest(const real_T solution_xstar[7], const
  real_T solution_searchDir[7], real_T workspace[175], int32_T workingset_nVar,
  const real_T workingset_Aineq[168], const real_T workingset_bineq[24], const
  int32_T workingset_indexLB[7], const int32_T workingset_sizes[5], const
  int32_T workingset_isActiveIdx[6], const boolean_T workingset_isActiveConstr
  [25], const int32_T workingset_nWConstr[5], real_T *toldelta, real_T *alpha,
  boolean_T *newBlocking, int32_T *constrType, int32_T *constrIdx);
static void TaskSpaceCont_feasibleratiotest(const real_T solution_xstar[7],
  const real_T solution_searchDir[7], real_T workspace[175], int32_T
  workingset_nVar, const real_T workingset_Aineq[168], const real_T
  workingset_bineq[24], const int32_T workingset_indexLB[7], const int32_T
  workingset_sizes[5], const int32_T workingset_isActiveIdx[6], const boolean_T
  workingset_isActiveConstr[25], const int32_T workingset_nWConstr[5], boolean_T
  isPhaseOne, real_T *alpha, boolean_T *newBlocking, int32_T *constrType,
  int32_T *constrIdx);
static void TaskSpaceController_iterate(const real_T f[6],
  smNINkioqq1a7FyOE4CETSB_TaskS_T *solution, sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T
  *memspace, sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager, s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T
  *cholmanager, slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective, boolean_T
  runTimeOptions_RemainFeasible, real_T runTimeOptions_ConstrRelTolFact, real_T
  runTimeOptions_ProbRelTolFactor);
static void TaskSpaceC_computeFirstOrderOpt(smNINkioqq1a7FyOE4CETSB_TaskS_T
  *solution, const slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective, int32_T
  workingset_nVar, const real_T workingset_ATwset[175], int32_T
  workingset_nActiveConstr, real_T workspace[175]);
static void TaskSpaceController_phaseone_a(const real_T f[6],
  smNINkioqq1a7FyOE4CETSB_TaskS_T *solution, sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T
  *memspace, sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager, s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T
  *cholmanager, slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective, const
  sIOJhD9KwAkF5sEguPjYquC_TaskS_T *runTimeOptions);
static void TaskSpaceController_linearForm_(boolean_T obj_hasLinear, int32_T
  obj_nvar, real_T workspace[175], const real_T f[6], const real_T x[7]);
static real_T TaskSpaceController_computeFval(const
  slzZ8M58FXlZqTD433BZJUH_TaskS_T *obj, real_T workspace[175], const real_T f[6],
  const real_T x[7]);
static void TaskSpaceController_driver(const real_T f[6],
  smNINkioqq1a7FyOE4CETSB_TaskS_T *solution, sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T
  *memspace, sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T *cholmanager, sIOJhD9KwAkF5sEguPjYquC_TaskS_T
  runTimeOptions, skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager,
  slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective);
static void TaskSpaceControll_linearForm__b(boolean_T obj_hasLinear, int32_T
  obj_nvar, real_T workspace[7], const real_T f[6], const real_T x[7]);
static void TaskSpaceController_quadprog(const real_T f[6], const real_T bineq
  [24], const real_T x0[6], real_T x[6], real_T *fval, real_T *exitflag, char_T
  output_algorithm[10], real_T *output_firstorderopt, real_T
  *output_constrviolation, real_T *output_iterations,
  sVMzcbzaOHlwbaxNrLGVFM_TaskSp_T *lambda);
int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static real_T TaskSpaceController_norm(const real_T x[3])
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

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Right1' */
void TaskSpaceController_dlog6(const real_T lambda[6], real_T dlog[36])
{
  __m128d tmp_5;
  __m128d tmp_6;
  __m128d tmp_7;
  real_T D[9];
  real_T D_0[9];
  real_T b_I_0[9];
  real_T beta_0[9];
  real_T dlog_3[9];
  real_T tmp[9];
  real_T tmp_0[9];
  real_T tmp_8[2];
  real_T b_gamma_tmp;
  real_T b_s;
  real_T beta;
  real_T norm_xi_tmp_tmp;
  real_T s_tmp;
  real_T tmp_1;
  real_T tmp_2;
  real_T tmp_3;
  real_T tmp_4;
  int32_T D_tmp;
  int32_T b_k;
  int32_T i;
  int8_T b_I[9];
  norm_xi_tmp_tmp = TaskSpaceController_norm(&lambda[3]);
  s_tmp = sin(norm_xi_tmp_tmp / 2.0) / (norm_xi_tmp_tmp / 2.0);
  beta = s_tmp * s_tmp;
  s_tmp = cos(norm_xi_tmp_tmp / 2.0) * s_tmp / beta;
  if (norm_xi_tmp_tmp < 2.2204460492503131E-16) {
    memset(&dlog_3[0], 0, 9U * sizeof(real_T));
    dlog_3[0] = 1.0;
    dlog_3[4] = 1.0;
    dlog_3[8] = 1.0;
  } else {
    b_gamma_tmp = (1.0 - s_tmp) / (norm_xi_tmp_tmp * norm_xi_tmp_tmp);
    D[0] = 0.0;
    D[3] = -lambda[5];
    D[6] = lambda[4];
    D[1] = lambda[5];
    D[4] = 0.0;
    D[7] = -lambda[3];
    D[2] = -lambda[4];
    D[5] = lambda[3];
    D[8] = 0.0;
    for (i = 0; i < 9; i++) {
      b_I[i] = 0;
    }

    for (b_k = 0; b_k < 3; b_k++) {
      b_I[b_k + 3 * b_k] = 1;
    }

    for (i = 0; i < 3; i++) {
      for (b_k = 0; b_k <= 0; b_k += 2) {
        tmp_5 = _mm_loadu_pd(&D[b_k + 3]);
        tmp_6 = _mm_loadu_pd(&D[b_k]);
        tmp_7 = _mm_loadu_pd(&D[b_k + 6]);
        _mm_storeu_pd(&D_0[b_k + 3 * i], _mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(D[3 * i + 1]), tmp_5), _mm_mul_pd(_mm_set1_pd(D[3 * i]),
          tmp_6)), _mm_mul_pd(_mm_set1_pd(D[3 * i + 2]), tmp_7)));
      }

      for (b_k = 2; b_k < 3; b_k++) {
        D_0[b_k + 3 * i] = (D[3 * i + 1] * D[b_k + 3] + D[3 * i] * D[b_k]) + D[3
          * i + 2] * D[b_k + 6];
      }
    }

    b_I_0[0] = b_I[0];
    tmp_5 = _mm_set1_pd(0.5);
    _mm_storeu_pd(&b_I_0[1], _mm_sub_pd(_mm_set_pd(b_I[2], b_I[1]), _mm_mul_pd
      (tmp_5, _mm_set_pd(-lambda[4], lambda[5]))));
    b_I_0[3] = (real_T)b_I[3] - 0.5 * -lambda[5];
    b_I_0[4] = b_I[4];
    _mm_storeu_pd(&b_I_0[5], _mm_sub_pd(_mm_set_pd(b_I[6], b_I[5]), _mm_mul_pd
      (tmp_5, _mm_loadu_pd(&lambda[3]))));
    b_I_0[7] = (real_T)b_I[7] - 0.5 * -lambda[3];
    b_I_0[8] = b_I[8];
    for (i = 0; i <= 6; i += 2) {
      tmp_5 = _mm_loadu_pd(&D_0[i]);
      tmp_6 = _mm_loadu_pd(&b_I_0[i]);
      _mm_storeu_pd(&dlog_3[i], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(b_gamma_tmp),
        tmp_5), tmp_6));
    }

    for (i = 8; i < 9; i++) {
      dlog_3[i] = b_gamma_tmp * D_0[i] + b_I_0[i];
    }
  }

  b_s = norm_xi_tmp_tmp * norm_xi_tmp_tmp;
  b_gamma_tmp = (1.0 - s_tmp) / b_s;
  beta = ((1.0 / beta + s_tmp) - 2.0) * (1.0 / b_s) / b_s;
  beta = (beta * lambda[3] * lambda[0] + beta * lambda[4] * lambda[1]) + beta *
    lambda[5] * lambda[2];
  D_0[0] = 0.0;
  D_0[3] = -lambda[2];
  D_0[6] = lambda[1];
  D_0[1] = lambda[2];
  D_0[4] = 0.0;
  D_0[7] = -lambda[0];
  D_0[2] = -lambda[1];
  D_0[5] = lambda[0];
  D_0[8] = 0.0;
  b_I_0[0] = 0.0;
  b_I_0[3] = -lambda[5];
  b_I_0[6] = lambda[4];
  b_I_0[1] = lambda[5];
  b_I_0[4] = 0.0;
  b_I_0[7] = -lambda[3];
  b_I_0[2] = -lambda[4];
  b_I_0[5] = lambda[3];
  b_I_0[8] = 0.0;
  D[0] = 0.0;
  D[3] = -lambda[5];
  D[6] = lambda[4];
  D[1] = lambda[5];
  D[4] = 0.0;
  D[7] = -lambda[3];
  D[2] = -lambda[4];
  D[5] = lambda[3];
  D[8] = 0.0;
  beta_0[0] = 0.0;
  beta_0[3] = -lambda[2];
  beta_0[6] = lambda[1];
  beta_0[1] = lambda[2];
  beta_0[4] = 0.0;
  beta_0[7] = -lambda[0];
  beta_0[2] = -lambda[1];
  beta_0[5] = lambda[0];
  beta_0[8] = 0.0;
  for (i = 0; i < 3; i++) {
    b_k = 3 * i + 1;
    s_tmp = beta_0[b_k];
    b_s = beta_0[3 * i];
    D_tmp = 3 * i + 2;
    tmp_1 = beta_0[D_tmp];
    tmp_2 = b_I_0[b_k];
    tmp_3 = b_I_0[3 * i];
    tmp_4 = b_I_0[D_tmp];
    for (b_k = 0; b_k <= 0; b_k += 2) {
      tmp_5 = _mm_loadu_pd(&D[b_k + 3]);
      tmp_6 = _mm_loadu_pd(&D[b_k]);
      tmp_7 = _mm_loadu_pd(&D[b_k + 6]);
      D_tmp = 3 * i + b_k;
      _mm_storeu_pd(&tmp_0[D_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (s_tmp), tmp_5), _mm_mul_pd(_mm_set1_pd(b_s), tmp_6)), _mm_mul_pd
        (_mm_set1_pd(tmp_1), tmp_7)));
      tmp_5 = _mm_loadu_pd(&D_0[b_k + 3]);
      tmp_6 = _mm_loadu_pd(&D_0[b_k]);
      tmp_7 = _mm_loadu_pd(&D_0[b_k + 6]);
      _mm_storeu_pd(&tmp[D_tmp], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (tmp_2), tmp_5), _mm_mul_pd(_mm_set1_pd(tmp_3), tmp_6)), _mm_mul_pd
        (_mm_set1_pd(tmp_4), tmp_7)));
    }

    for (b_k = 2; b_k < 3; b_k++) {
      _mm_storeu_pd(&tmp_8[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd(tmp_2,
        s_tmp), _mm_set_pd(D_0[b_k + 3], D[b_k + 3])), _mm_mul_pd(_mm_set_pd
        (tmp_3, b_s), _mm_set_pd(D_0[b_k], D[b_k]))), _mm_mul_pd(_mm_set_pd
        (tmp_4, tmp_1), _mm_set_pd(D_0[b_k + 6], D[b_k + 6]))));
      D_tmp = 3 * i + b_k;
      tmp_0[D_tmp] = tmp_8[0];
      tmp[D_tmp] = tmp_8[1];
    }
  }

  D_0[0] = -0.0;
  tmp_5 = _mm_set1_pd(-0.5);
  tmp_6 = _mm_mul_pd(tmp_5, _mm_set_pd(lambda[1], -lambda[2]));
  _mm_storeu_pd(&tmp_8[0], tmp_6);
  D_0[3] = tmp_8[0];
  D_0[6] = tmp_8[1];
  D_0[1] = -0.5 * lambda[2];
  D_0[4] = -0.0;
  tmp_5 = _mm_mul_pd(tmp_5, _mm_set_pd(-lambda[1], -lambda[0]));
  _mm_storeu_pd(&tmp_8[0], tmp_5);
  D_0[7] = tmp_8[0];
  D_0[2] = tmp_8[1];
  D_0[5] = -0.5 * lambda[0];
  D_0[8] = -0.0;
  beta_0[0] = beta * 0.0;
  tmp_7 = _mm_set1_pd(beta);
  _mm_storeu_pd(&tmp_8[0], _mm_mul_pd(tmp_7, _mm_set_pd(lambda[4], -lambda[5])));
  beta_0[3] = tmp_8[0];
  beta_0[6] = tmp_8[1];
  beta_0[1] = beta * lambda[5];
  beta_0[4] = beta * 0.0;
  _mm_storeu_pd(&tmp_8[0], _mm_mul_pd(tmp_7, _mm_set_pd(-lambda[4], -lambda[3])));
  beta_0[7] = tmp_8[0];
  beta_0[2] = tmp_8[1];
  beta_0[5] = beta * lambda[3];
  beta_0[8] = beta * 0.0;
  b_I_0[0] = 0.0;
  b_I_0[3] = -lambda[5];
  b_I_0[6] = lambda[4];
  b_I_0[1] = lambda[5];
  b_I_0[4] = 0.0;
  b_I_0[7] = -lambda[3];
  b_I_0[2] = -lambda[4];
  b_I_0[5] = lambda[3];
  b_I_0[8] = 0.0;
  for (i = 0; i < 3; i++) {
    beta = beta_0[i + 3];
    s_tmp = beta_0[i];
    b_s = beta_0[i + 6];
    for (b_k = 0; b_k < 3; b_k++) {
      D_tmp = 3 * b_k + i;
      D[D_tmp] = ((b_I_0[3 * b_k + 1] * beta + b_I_0[3 * b_k] * s_tmp) + b_I_0[3
                  * b_k + 2] * b_s) + ((tmp[D_tmp] + tmp_0[D_tmp]) * b_gamma_tmp
        + D_0[D_tmp]);
    }
  }

  if (norm_xi_tmp_tmp < 2.2204460492503131E-16) {
    D[0] = -0.0;
    _mm_storeu_pd(&tmp_8[0], tmp_6);
    D[3] = tmp_8[0];
    D[6] = tmp_8[1];
    D[1] = -0.5 * lambda[2];
    D[4] = -0.0;
    _mm_storeu_pd(&tmp_8[0], tmp_5);
    D[7] = tmp_8[0];
    D[2] = tmp_8[1];
    D[5] = -0.5 * lambda[0];
    D[8] = -0.0;
  }

  for (i = 0; i < 3; i++) {
    norm_xi_tmp_tmp = dlog_3[3 * i];
    dlog[6 * i] = norm_xi_tmp_tmp;
    b_k = (i + 3) * 6;
    dlog[b_k] = D[3 * i];
    dlog[6 * i + 3] = 0.0;
    dlog[b_k + 3] = norm_xi_tmp_tmp;
    D_tmp = 3 * i + 1;
    norm_xi_tmp_tmp = dlog_3[D_tmp];
    dlog[6 * i + 1] = norm_xi_tmp_tmp;
    dlog[b_k + 1] = D[D_tmp];
    dlog[6 * i + 4] = 0.0;
    dlog[b_k + 4] = norm_xi_tmp_tmp;
    D_tmp = 3 * i + 2;
    norm_xi_tmp_tmp = dlog_3[D_tmp];
    dlog[6 * i + 2] = norm_xi_tmp_tmp;
    dlog[b_k + 2] = D[D_tmp];
    dlog[6 * i + 5] = 0.0;
    dlog[b_k + 5] = norm_xi_tmp_tmp;
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_xzgetrf(real_T A[36], int32_T ipiv[6], int32_T
  *info)
{
  int32_T iy;
  int32_T j;
  int32_T k;
  static const int32_T offsets[4] = { 0, 1, 2, 3 };

  for (j = 0; j <= 0; j += 4) {
    _mm_storeu_si128((__m128i *)&ipiv[j], _mm_add_epi32(_mm_add_epi32
      (_mm_set1_epi32(j), _mm_loadu_si128((const __m128i *)&offsets[0])),
      _mm_set1_epi32(1)));
  }

  for (j = 4; j < 6; j++) {
    ipiv[j] = j + 1;
  }

  *info = 0;
  for (j = 0; j < 5; j++) {
    real_T smax;
    int32_T b_ix;
    int32_T jj;
    int32_T vectorUB;
    jj = j * 7;
    iy = 5 - j;
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
        for (k = 0; k < 6; k++) {
          b_ix = k * 6 + j;
          smax = A[b_ix];
          A[b_ix] = A[iy];
          A[iy] = smax;
          iy += 6;
        }
      }

      iy = (jj - j) + 6;
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

    b_ix = jj + 8;
    vectorUB = 4 - j;
    for (k = 0; k <= vectorUB; k++) {
      smax = A[(k * 6 + jj) + 6];
      if (smax != 0.0) {
        int32_T d;
        d = (b_ix - j) + 4;
        for (iy = b_ix; iy <= d; iy++) {
          A[iy - 1] += A[((jj + iy) - b_ix) + 1] * -smax;
        }
      }

      b_ix += 6;
    }
  }

  if ((*info == 0) && (!(A[35] != 0.0))) {
    *info = 6;
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static real_T TaskSpaceController_xnrm2(int32_T n, const real_T x[175], int32_T
  ix0)
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

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static real_T TaskSpaceController_xzlarfg(int32_T n, real_T *alpha1, real_T x
  [175], int32_T ix0)
{
  __m128d tmp;
  real_T a;
  real_T tau;
  real_T xnorm;
  int32_T c;
  int32_T knt;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T vectorUB_tmp;
  tau = 0.0;
  if (n > 0) {
    xnorm = TaskSpaceController_xnrm2(n - 1, x, ix0);
    if (xnorm != 0.0) {
      xnorm = rt_hypotd_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (fabs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        do {
          knt++;
          scalarLB = (ix0 + n) - 2;
          vectorUB = ((((scalarLB - ix0) + 1) / 2) << 1) + ix0;
          vectorUB_tmp = vectorUB - 2;
          for (c = ix0; c <= vectorUB_tmp; c += 2) {
            tmp = _mm_loadu_pd(&x[c - 1]);
            _mm_storeu_pd(&x[c - 1], _mm_mul_pd(tmp, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (c = vectorUB; c <= scalarLB; c++) {
            x[c - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while ((fabs(xnorm) < 1.0020841800044864E-292) && (knt < 20));

        xnorm = rt_hypotd_snf(*alpha1, TaskSpaceController_xnrm2(n - 1, x, ix0));
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        a = 1.0 / (*alpha1 - xnorm);
        for (c = ix0; c <= vectorUB_tmp; c += 2) {
          tmp = _mm_loadu_pd(&x[c - 1]);
          _mm_storeu_pd(&x[c - 1], _mm_mul_pd(tmp, _mm_set1_pd(a)));
        }

        for (c = vectorUB; c <= scalarLB; c++) {
          x[c - 1] *= a;
        }

        for (c = 0; c < knt; c++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        a = 1.0 / (*alpha1 - xnorm);
        c = (ix0 + n) - 2;
        scalarLB = ((((c - ix0) + 1) / 2) << 1) + ix0;
        vectorUB = scalarLB - 2;
        for (knt = ix0; knt <= vectorUB; knt += 2) {
          tmp = _mm_loadu_pd(&x[knt - 1]);
          _mm_storeu_pd(&x[knt - 1], _mm_mul_pd(tmp, _mm_set1_pd(a)));
        }

        for (knt = scalarLB; knt <= c; knt++) {
          x[knt - 1] *= a;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C[175], int32_T ic0, real_T work[25])
{
  int32_T b_ia;
  int32_T coltop;
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

    lastc = n - 1;
    exitg2 = false;
    while ((!exitg2) && (lastc + 1 > 0)) {
      int32_T exitg1;
      coltop = lastc * 7 + ic0;
      b_ia = coltop;
      do {
        exitg1 = 0;
        if (b_ia <= (coltop + lastv) - 1) {
          if (C[b_ia - 1] != 0.0) {
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
    lastv = 0;
    lastc = -1;
  }

  if (lastv > 0) {
    real_T c;
    int32_T d;
    int32_T jy;
    if (lastc + 1 != 0) {
      if (lastc >= 0) {
        memset(&work[0], 0, (uint32_T)(lastc + 1) * sizeof(real_T));
      }

      jy = 7 * lastc + ic0;
      for (coltop = ic0; coltop <= jy; coltop += 7) {
        c = 0.0;
        d = (coltop + lastv) - 1;
        for (b_ia = coltop; b_ia <= d; b_ia++) {
          c += C[((iv0 + b_ia) - coltop) - 1] * C[b_ia - 1];
        }

        b_ia = div_nde_s32_floor(coltop - ic0, 7);
        work[b_ia] += c;
      }
    }

    if (!(-tau == 0.0)) {
      jy = ic0;
      for (coltop = 0; coltop <= lastc; coltop++) {
        c = work[coltop];
        if (c != 0.0) {
          c *= -tau;
          d = (lastv + jy) - 1;
          for (b_ia = jy; b_ia <= d; b_ia++) {
            C[b_ia - 1] += C[((iv0 + b_ia) - jy) - 1] * c;
          }
        }

        jy += 7;
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_qrf(real_T A[175], int32_T m, int32_T n, int32_T
  nfxd, real_T tau[7])
{
  real_T work[25];
  real_T b_atmp;
  real_T tau_0;
  int32_T b;
  int32_T i;
  int32_T ii;
  int32_T mmi;
  memset(&work[0], 0, 25U * sizeof(real_T));
  b = (uint8_T)nfxd;
  for (i = 0; i < b; i++) {
    ii = i * 7 + i;
    mmi = m - i;
    if (i + 1 < m) {
      b_atmp = A[ii];
      tau_0 = TaskSpaceController_xzlarfg(mmi, &b_atmp, A, ii + 2);
      tau[i] = tau_0;
      A[ii] = b_atmp;
    } else {
      tau_0 = 0.0;
      tau[i] = 0.0;
    }

    if (i + 1 < n) {
      b_atmp = A[ii];
      A[ii] = 1.0;
      TaskSpaceController_xzlarf(mmi, (n - i) - 1, ii + 1, tau_0, A, ii + 8,
        work);
      A[ii] = b_atmp;
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_xgeqp3(real_T A[175], int32_T m, int32_T n,
  int32_T jpvt[25], real_T tau[7])
{
  real_T vn1[25];
  real_T vn2[25];
  real_T work[25];
  real_T s;
  real_T temp;
  real_T vn1_0;
  int32_T b_ix;
  int32_T i;
  int32_T itemp;
  int32_T ix;
  int32_T iy;
  int32_T minmn_tmp;
  int32_T mmi;
  int32_T nfxd;
  int32_T pvt;
  static const int32_T offsets[4] = { 0, 1, 2, 3 };

  int32_T ix_tmp;
  int32_T temp_tmp;
  int32_T tmp;
  if (m <= n) {
    minmn_tmp = m;
  } else {
    minmn_tmp = n;
  }

  for (i = 0; i < 7; i++) {
    tau[i] = 0.0;
  }

  if (minmn_tmp < 1) {
    i = (n / 4) << 2;
    ix = i - 4;
    for (minmn_tmp = 0; minmn_tmp <= ix; minmn_tmp += 4) {
      _mm_storeu_si128((__m128i *)&jpvt[minmn_tmp], _mm_add_epi32(_mm_add_epi32
        (_mm_set1_epi32(minmn_tmp), _mm_loadu_si128((const __m128i *)&offsets[0])),
        _mm_set1_epi32(1)));
    }

    for (minmn_tmp = i; minmn_tmp < n; minmn_tmp++) {
      jpvt[minmn_tmp] = minmn_tmp + 1;
    }
  } else {
    nfxd = -1;
    for (i = 0; i < n; i++) {
      if (jpvt[i] != 0) {
        nfxd++;
        if (i + 1 != nfxd + 1) {
          ix = i * 7;
          iy = nfxd * 7;
          for (mmi = 0; mmi < m; mmi++) {
            temp_tmp = ix + mmi;
            temp = A[temp_tmp];
            tmp = iy + mmi;
            A[temp_tmp] = A[tmp];
            A[tmp] = temp;
          }

          jpvt[i] = jpvt[nfxd];
          jpvt[nfxd] = i + 1;
        } else {
          jpvt[i] = i + 1;
        }
      } else {
        jpvt[i] = i + 1;
      }
    }

    if (nfxd + 1 <= minmn_tmp) {
      nfxd++;
    } else {
      nfxd = minmn_tmp;
    }

    for (i = 0; i < 7; i++) {
      tau[i] = 0.0;
    }

    TaskSpaceController_qrf(A, m, n, nfxd, tau);
    if (nfxd < minmn_tmp) {
      memset(&work[0], 0, 25U * sizeof(real_T));
      memset(&vn1[0], 0, 25U * sizeof(real_T));
      memset(&vn2[0], 0, 25U * sizeof(real_T));
      for (i = nfxd + 1; i <= n; i++) {
        vn1_0 = TaskSpaceController_xnrm2(m - nfxd, A, ((i - 1) * 7 + nfxd) + 1);
        vn1[i - 1] = vn1_0;
        vn2[i - 1] = vn1_0;
      }

      for (i = nfxd + 1; i <= minmn_tmp; i++) {
        ix_tmp = (i - 1) * 7;
        ix = (ix_tmp + i) - 1;
        iy = n - i;
        mmi = m - i;
        if (iy + 1 < 1) {
          pvt = -2;
        } else {
          pvt = -1;
          if (iy + 1 > 1) {
            temp = fabs(vn1[i - 1]);
            for (itemp = 2; itemp <= iy + 1; itemp++) {
              s = fabs(vn1[(i + itemp) - 2]);
              if (s > temp) {
                pvt = itemp - 2;
                temp = s;
              }
            }
          }
        }

        pvt += i;
        if (pvt + 1 != i) {
          b_ix = pvt * 7;
          for (itemp = 0; itemp < m; itemp++) {
            temp_tmp = b_ix + itemp;
            temp = A[temp_tmp];
            tmp = ix_tmp + itemp;
            A[temp_tmp] = A[tmp];
            A[tmp] = temp;
          }

          itemp = jpvt[pvt];
          jpvt[pvt] = jpvt[i - 1];
          jpvt[i - 1] = itemp;
          vn1[pvt] = vn1[i - 1];
          vn2[pvt] = vn2[i - 1];
        }

        if (i < m) {
          temp = A[ix];
          vn1_0 = TaskSpaceController_xzlarfg(mmi + 1, &temp, A, ix + 2);
          tau[i - 1] = vn1_0;
          A[ix] = temp;
        } else {
          vn1_0 = 0.0;
          tau[i - 1] = 0.0;
        }

        if (i < n) {
          temp = A[ix];
          A[ix] = 1.0;
          TaskSpaceController_xzlarf(mmi + 1, iy, ix + 1, vn1_0, A, ix + 8, work);
          A[ix] = temp;
        }

        for (ix = i + 1; ix <= n; ix++) {
          iy = (ix - 1) * 7 + i;
          vn1_0 = vn1[ix - 1];
          if (vn1_0 != 0.0) {
            temp = fabs(A[iy - 1]) / vn1_0;
            temp = 1.0 - temp * temp;
            if (temp < 0.0) {
              temp = 0.0;
            }

            s = vn1_0 / vn2[ix - 1];
            s = s * s * temp;
            if (s <= 1.4901161193847656E-8) {
              if (i < m) {
                vn1_0 = TaskSpaceController_xnrm2(mmi, A, iy + 1);
                vn1[ix - 1] = vn1_0;
                vn2[ix - 1] = vn1_0;
              } else {
                vn1[ix - 1] = 0.0;
                vn2[ix - 1] = 0.0;
              }
            } else {
              vn1[ix - 1] = vn1_0 * sqrt(temp);
            }
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_countsort(int32_T x[25], int32_T xLen, int32_T
  workspace[25], int32_T xMin, int32_T xMax)
{
  int32_T b_tmp;
  int32_T idxFill;
  int32_T maxOffset;
  if ((xLen > 1) && (xMax > xMin)) {
    int32_T idxEnd;
    int32_T idxStart;
    b_tmp = xMax - xMin;
    if (b_tmp >= 0) {
      memset(&workspace[0], 0, (uint32_T)(b_tmp + 1) * sizeof(int32_T));
    }

    maxOffset = b_tmp - 1;
    for (b_tmp = 0; b_tmp < xLen; b_tmp++) {
      idxFill = x[b_tmp] - xMin;
      workspace[idxFill]++;
    }

    for (b_tmp = 2; b_tmp <= maxOffset + 2; b_tmp++) {
      workspace[b_tmp - 1] += workspace[b_tmp - 2];
    }

    idxStart = 1;
    idxEnd = workspace[0];
    for (b_tmp = 0; b_tmp <= maxOffset; b_tmp++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x[idxFill - 1] = b_tmp + xMin;
      }

      idxStart = workspace[b_tmp] + 1;
      idxEnd = workspace[b_tmp + 1];
    }

    for (maxOffset = idxStart; maxOffset <= idxEnd; maxOffset++) {
      x[maxOffset - 1] = xMax;
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceControlle_removeConstr(sAElXDmDj36R7Z42SImJxmG_TaskS_T *obj,
  int32_T idx_global)
{
  int32_T TYPE_tmp;
  int32_T idx;
  TYPE_tmp = obj->Wid[idx_global - 1] - 1;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE_tmp] + obj->Wlocalidx[idx_global -
                       1]) - 2] = false;
  if (idx_global < obj->nActiveConstr) {
    int32_T b;
    obj->Wid[idx_global - 1] = obj->Wid[obj->nActiveConstr - 1];
    obj->Wlocalidx[idx_global - 1] = obj->Wlocalidx[obj->nActiveConstr - 1];
    b = (uint8_T)obj->nVar;
    for (idx = 0; idx < b; idx++) {
      obj->ATwset[idx + 7 * (idx_global - 1)] = obj->ATwset[(obj->nActiveConstr
        - 1) * 7 + idx];
    }

    obj->bwset[idx_global - 1] = obj->bwset[obj->nActiveConstr - 1];
  }

  obj->nActiveConstr--;
  obj->nWConstr[TYPE_tmp]--;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceC_RemoveDependentIneq_(sAElXDmDj36R7Z42SImJxmG_TaskS_T
  *workingset, skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager,
  sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T *memspace, real_T tolfactor)
{
  real_T maxDiag;
  real_T tol;
  int32_T iy0_tmp;
  int32_T nActiveConstr;
  int32_T nDepIneq;
  int32_T nFixedConstr;
  int32_T nVar;
  nActiveConstr = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    if (workingset->nVar >= workingset->nActiveConstr) {
      nDepIneq = workingset->nVar;
    } else {
      nDepIneq = workingset->nActiveConstr;
    }

    tol = fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * (real_T)nDepIneq)
      * tolfactor;
    for (nDepIneq = 0; nDepIneq < nFixedConstr; nDepIneq++) {
      qrmanager->jpvt[nDepIneq] = 1;
    }

    if (nFixedConstr + 1 <= workingset->nActiveConstr) {
      memset(&qrmanager->jpvt[nFixedConstr], 0, (uint32_T)
             (workingset->nActiveConstr - nFixedConstr) * sizeof(int32_T));
    }

    for (nDepIneq = 0; nDepIneq < nActiveConstr; nDepIneq++) {
      iy0_tmp = 7 * nDepIneq;
      memcpy(&qrmanager->QR[iy0_tmp], &workingset->ATwset[iy0_tmp], (uint8_T)
             nVar * sizeof(real_T));
    }

    if (workingset->nVar * workingset->nActiveConstr == 0) {
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      if (workingset->nVar <= workingset->nActiveConstr) {
        qrmanager->minRowCol = workingset->nVar;
      } else {
        qrmanager->minRowCol = workingset->nActiveConstr;
      }

      TaskSpaceController_xgeqp3(qrmanager->QR, workingset->nVar,
        workingset->nActiveConstr, qrmanager->jpvt, qrmanager->tau);
    }

    nDepIneq = 0;
    for (nActiveConstr = workingset->nActiveConstr - 1; nActiveConstr + 1 > nVar;
         nActiveConstr--) {
      nDepIneq++;
      memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[nActiveConstr];
    }

    maxDiag = fabs(qrmanager->QR[0]);
    for (nVar = 0; nVar < nActiveConstr; nVar++) {
      maxDiag = fmax(maxDiag, fabs(qrmanager->QR[((nVar + 1) * 7 + nVar) + 1]));
    }

    if (nActiveConstr + 1 <= workingset->nVar) {
      nVar = 7 * nActiveConstr + nActiveConstr;
      while ((nActiveConstr + 1 > nFixedConstr) && (fabs(qrmanager->QR[nVar]) <
              tol * maxDiag)) {
        nDepIneq++;
        memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[nActiveConstr];
        nActiveConstr--;
        nVar -= 8;
      }
    }

    TaskSpaceController_countsort(memspace->workspace_int, nDepIneq,
      memspace->workspace_sort, nFixedConstr + 1, workingset->nActiveConstr);
    for (nFixedConstr = nDepIneq; nFixedConstr >= 1; nFixedConstr--) {
      TaskSpaceControlle_removeConstr(workingset, memspace->
        workspace_int[nFixedConstr - 1]);
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_computeQ_(skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *obj,
  int32_T nrows)
{
  real_T work[7];
  int32_T c_ia;
  int32_T coltop;
  int32_T i;
  int32_T iQR0;
  int32_T ia;
  int32_T idx;
  int32_T itau;
  int32_T lastc;
  lastc = obj->minRowCol;
  for (idx = 0; idx < lastc; idx++) {
    iQR0 = 7 * idx + idx;
    ia = obj->mrows - idx;
    if (ia - 2 >= 0) {
      memcpy(&obj->Q[iQR0 + 1], &obj->QR[iQR0 + 1], (uint32_T)(ia - 1) * sizeof
             (real_T));
    }
  }

  idx = obj->mrows;
  if (nrows >= 1) {
    for (itau = lastc; itau < nrows; itau++) {
      ia = itau * 7;
      memset(&obj->Q[ia], 0, (uint32_T)idx * sizeof(real_T));
      obj->Q[ia + itau] = 1.0;
    }

    itau = obj->minRowCol - 1;
    for (i = 0; i < 7; i++) {
      work[i] = 0.0;
    }

    for (i = obj->minRowCol; i >= 1; i--) {
      iQR0 = ((i - 1) * 7 + i) - 1;
      if (i < nrows) {
        obj->Q[iQR0] = 1.0;
        ia = idx - i;
        if (obj->tau[itau] != 0.0) {
          boolean_T exitg2;
          lastc = iQR0 + ia;
          while ((ia + 1 > 0) && (obj->Q[lastc] == 0.0)) {
            ia--;
            lastc--;
          }

          lastc = (nrows - i) - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc + 1 > 0)) {
            int32_T exitg1;
            coltop = (lastc * 7 + iQR0) + 8;
            c_ia = coltop;
            do {
              exitg1 = 0;
              if (c_ia <= coltop + ia) {
                if (obj->Q[c_ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  c_ia++;
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
          ia = -1;
          lastc = -1;
        }

        if (ia + 1 > 0) {
          real_T b_c;
          int32_T f;
          int32_T jy;
          if (lastc + 1 != 0) {
            if (lastc >= 0) {
              memset(&work[0], 0, (uint32_T)(lastc + 1) * sizeof(real_T));
            }

            jy = (7 * lastc + iQR0) + 8;
            for (coltop = iQR0 + 8; coltop <= jy; coltop += 7) {
              b_c = 0.0;
              f = coltop + ia;
              for (c_ia = coltop; c_ia <= f; c_ia++) {
                b_c += obj->Q[(iQR0 + c_ia) - coltop] * obj->Q[c_ia - 1];
              }

              c_ia = div_nde_s32_floor((coltop - iQR0) - 8, 7);
              work[c_ia] += b_c;
            }
          }

          if (!(-obj->tau[itau] == 0.0)) {
            jy = iQR0 + 8;
            for (coltop = 0; coltop <= lastc; coltop++) {
              b_c = work[coltop];
              if (b_c != 0.0) {
                b_c *= -obj->tau[itau];
                f = ia + jy;
                for (c_ia = jy; c_ia <= f; c_ia++) {
                  obj->Q[c_ia - 1] += obj->Q[(iQR0 + c_ia) - jy] * b_c;
                }
              }

              jy += 7;
            }
          }
        }
      }

      if (i < idx) {
        lastc = ((iQR0 + idx) - i) + 1;
        coltop = (((((lastc - iQR0) - 1) / 2) << 1) + iQR0) + 2;
        c_ia = coltop - 2;
        for (ia = iQR0 + 2; ia <= c_ia; ia += 2) {
          __m128d tmp;
          tmp = _mm_loadu_pd(&obj->Q[ia - 1]);
          _mm_storeu_pd(&obj->Q[ia - 1], _mm_mul_pd(tmp, _mm_set1_pd(-obj->
            tau[itau])));
        }

        for (ia = coltop; ia <= lastc; ia++) {
          obj->Q[ia - 1] *= -obj->tau[itau];
        }
      }

      obj->Q[iQR0] = 1.0 - obj->tau[itau];
      for (ia = 0; ia <= i - 2; ia++) {
        obj->Q[(iQR0 - ia) - 1] = 0.0;
      }

      itau--;
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static int32_T TaskSpaceController_rank(const real_T qrmanager_QR[175], int32_T
  qrmanager_mrows, int32_T qrmanager_ncols)
{
  int32_T minmn;
  int32_T r;
  r = 0;
  if (qrmanager_mrows < qrmanager_ncols) {
    minmn = 6;
  } else {
    minmn = qrmanager_ncols;
  }

  if (minmn > 0) {
    real_T tol;
    int32_T tmp;
    if (qrmanager_mrows >= qrmanager_ncols) {
      tmp = qrmanager_mrows;
    } else {
      tmp = qrmanager_ncols;
    }

    tol = fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * (real_T)tmp) *
      fabs(qrmanager_QR[0]);
    while ((r < minmn) && (!(fabs(qrmanager_QR[7 * r + r]) <= tol))) {
      r++;
    }
  }

  return r;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_xgemv(int32_T m, const real_T A[168], const
  real_T x[175], real_T y[25])
{
  int32_T b_iy;
  int32_T ia;
  for (b_iy = 0; b_iy <= 22; b_iy += 2) {
    __m128d tmp;
    tmp = _mm_loadu_pd(&y[b_iy]);
    _mm_storeu_pd(&y[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
  }

  for (b_iy = 0; b_iy <= 161; b_iy += 7) {
    real_T c;
    int32_T b;
    c = 0.0;
    b = b_iy + m;
    for (ia = b_iy + 1; ia <= b; ia++) {
      c += x[(ia - b_iy) - 1] * A[ia - 1];
    }

    ia = div_nde_s32_floor(b_iy, 7);
    y[ia] += c;
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_xgemv_l(int32_T m, const real_T A[168], const
  real_T x[175], real_T y[25])
{
  int32_T b_iy;
  int32_T ia;
  for (b_iy = 0; b_iy <= 22; b_iy += 2) {
    __m128d tmp;
    tmp = _mm_loadu_pd(&y[b_iy]);
    _mm_storeu_pd(&y[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
  }

  for (b_iy = 0; b_iy <= 161; b_iy += 7) {
    real_T c;
    int32_T b;
    c = 0.0;
    b = b_iy + m;
    for (ia = b_iy + 1; ia <= b; ia++) {
      c += x[(ia - b_iy) + 24] * A[ia - 1];
    }

    ia = div_nde_s32_floor(b_iy, 7);
    y[ia] += c;
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static boolean_T TaskSpa_feasibleX0ForWorkingSet(real_T workspace[175], real_T
  xCurrent[7], sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T B[175];
  real_T d_v;
  real_T temp;
  real_T workingset_maxConstrWorkspace;
  int32_T exitg1;
  int32_T h_k;
  int32_T iAcol;
  int32_T ia;
  int32_T ix;
  int32_T ix0;
  int32_T iy;
  int32_T mWConstr;
  int32_T nVar;
  int32_T rankQR;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T guard1;
  boolean_T nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (workingset->nActiveConstr != 0) {
    if (workingset->nActiveConstr >= workingset->nVar) {
      vectorUB = (uint8_T)workingset->nVar;
      for (rankQR = 0; rankQR < vectorUB; rankQR++) {
        ix = 7 * rankQR;
        for (h_k = 0; h_k < mWConstr; h_k++) {
          qrmanager->QR[h_k + ix] = workingset->ATwset[7 * h_k + rankQR];
        }
      }

      memset(&qrmanager->jpvt[0], 0, (uint8_T)workingset->nVar * sizeof(int32_T));
      if (workingset->nActiveConstr * workingset->nVar == 0) {
        qrmanager->mrows = workingset->nActiveConstr;
        qrmanager->ncols = workingset->nVar;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = true;
        qrmanager->mrows = workingset->nActiveConstr;
        qrmanager->ncols = workingset->nVar;
        if (workingset->nActiveConstr <= workingset->nVar) {
          qrmanager->minRowCol = workingset->nActiveConstr;
        } else {
          qrmanager->minRowCol = workingset->nVar;
        }

        TaskSpaceController_xgeqp3(qrmanager->QR, workingset->nActiveConstr,
          workingset->nVar, qrmanager->jpvt, qrmanager->tau);
      }

      TaskSpaceController_computeQ_(qrmanager, qrmanager->mrows);
      rankQR = TaskSpaceController_rank(qrmanager->QR, qrmanager->mrows,
        qrmanager->ncols);
      for (h_k = 0; h_k < mWConstr; h_k++) {
        workspace[h_k] = workingset->bwset[h_k];
        workspace[h_k + 25] = workingset->bwset[h_k];
      }

      iy = (workingset->nActiveConstr - 1) * 7 + 1;
      for (h_k = 1; h_k <= iy; h_k += 7) {
        temp = 0.0;
        ia = (h_k + nVar) - 1;
        for (ix0 = h_k; ix0 <= ia; ix0++) {
          temp += workingset->ATwset[ix0 - 1] * xCurrent[ix0 - h_k];
        }

        ia = div_nde_s32_floor(h_k - 1, 7);
        workspace[ia] -= temp;
      }

      memcpy(&B[0], &workspace[0], 175U * sizeof(real_T));
      for (h_k = 0; h_k <= 25; h_k += 25) {
        ix = h_k + nVar;
        for (ix0 = h_k + 1; ix0 <= ix; ix0++) {
          workspace[ix0 - 1] = 0.0;
        }
      }

      iy = -1;
      for (h_k = 0; h_k <= 25; h_k += 25) {
        iAcol = -1;
        ia = h_k + nVar;
        for (ix0 = h_k + 1; ix0 <= ia; ix0++) {
          temp = 0.0;
          for (ix = 0; ix < mWConstr; ix++) {
            temp += qrmanager->Q[(ix + iAcol) + 1] * B[(ix + iy) + 1];
          }

          workspace[ix0 - 1] += temp;
          iAcol += 7;
        }

        iy += 25;
      }

      for (mWConstr = 0; mWConstr < 2; mWConstr++) {
        ix = 25 * mWConstr - 1;
        for (h_k = rankQR; h_k >= 1; h_k--) {
          iy = (h_k - 1) * 7;
          ia = h_k + ix;
          temp = workspace[ia];
          if (temp != 0.0) {
            workspace[ia] = temp / qrmanager->QR[(h_k + iy) - 1];
            iAcol = (uint8_T)(h_k - 1);
            for (ix0 = 0; ix0 < iAcol; ix0++) {
              scalarLB = (ix0 + ix) + 1;
              workspace[scalarLB] -= qrmanager->QR[ix0 + iy] * workspace[ia];
            }
          }
        }
      }

      for (mWConstr = rankQR + 1; mWConstr <= nVar; mWConstr++) {
        workspace[mWConstr - 1] = 0.0;
        workspace[mWConstr + 24] = 0.0;
      }

      for (rankQR = 0; rankQR < vectorUB; rankQR++) {
        workspace[qrmanager->jpvt[rankQR] + 49] = workspace[rankQR];
      }

      for (rankQR = 0; rankQR < vectorUB; rankQR++) {
        workspace[rankQR] = workspace[rankQR + 50];
      }

      for (rankQR = 0; rankQR < vectorUB; rankQR++) {
        workspace[qrmanager->jpvt[rankQR] + 49] = workspace[rankQR + 25];
      }

      for (rankQR = 0; rankQR < vectorUB; rankQR++) {
        workspace[rankQR + 25] = workspace[rankQR + 50];
      }
    } else {
      if (workingset->nActiveConstr - 1 >= 0) {
        memset(&qrmanager->jpvt[0], 0, (uint32_T)workingset->nActiveConstr *
               sizeof(int32_T));
      }

      rankQR = workingset->nVar * workingset->nActiveConstr;
      guard1 = false;
      if (rankQR > 0) {
        for (rankQR = 0; rankQR < mWConstr; rankQR++) {
          ix = 7 * rankQR;
          memcpy(&qrmanager->QR[ix], &workingset->ATwset[ix], (uint8_T)nVar *
                 sizeof(real_T));
        }

        guard1 = true;
      } else if (rankQR == 0) {
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = workingset->nActiveConstr;
        qrmanager->minRowCol = 0;
      } else {
        guard1 = true;
      }

      if (guard1) {
        qrmanager->usedPivoting = true;
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = workingset->nActiveConstr;
        if (workingset->nVar <= workingset->nActiveConstr) {
          qrmanager->minRowCol = workingset->nVar;
        } else {
          qrmanager->minRowCol = workingset->nActiveConstr;
        }

        TaskSpaceController_xgeqp3(qrmanager->QR, workingset->nVar,
          workingset->nActiveConstr, qrmanager->jpvt, qrmanager->tau);
      }

      TaskSpaceController_computeQ_(qrmanager, qrmanager->minRowCol);
      rankQR = TaskSpaceController_rank(qrmanager->QR, qrmanager->mrows,
        qrmanager->ncols);
      for (h_k = 0; h_k < mWConstr; h_k++) {
        ix = (qrmanager->jpvt[h_k] - 1) * 7;
        temp = 0.0;
        iAcol = (uint8_T)nVar;
        for (ix0 = 0; ix0 < iAcol; ix0++) {
          temp += workingset->ATwset[ix + ix0] * xCurrent[ix0];
        }

        workingset_maxConstrWorkspace = workingset->bwset[qrmanager->jpvt[h_k] -
          1];
        workspace[h_k] = workingset_maxConstrWorkspace - temp;
        workspace[h_k + 25] = workingset_maxConstrWorkspace;
      }

      iy = (uint8_T)rankQR;
      for (mWConstr = 0; mWConstr < 2; mWConstr++) {
        ix = 25 * mWConstr;
        for (h_k = 0; h_k < iy; h_k++) {
          iAcol = 7 * h_k;
          ia = h_k + ix;
          temp = workspace[ia];
          for (ix0 = 0; ix0 < h_k; ix0++) {
            temp -= qrmanager->QR[ix0 + iAcol] * workspace[ix0 + ix];
          }

          workspace[ia] = temp / qrmanager->QR[h_k + iAcol];
        }
      }

      memcpy(&B[0], &workspace[0], 175U * sizeof(real_T));
      for (mWConstr = 0; mWConstr <= 25; mWConstr += 25) {
        ix0 = mWConstr + nVar;
        for (h_k = mWConstr + 1; h_k <= ix0; h_k++) {
          workspace[h_k - 1] = 0.0;
        }
      }

      ix = 1;
      for (mWConstr = 0; mWConstr <= 25; mWConstr += 25) {
        iy = -1;
        iAcol = ix + rankQR;
        for (h_k = ix; h_k < iAcol; h_k++) {
          ia = mWConstr + nVar;
          scalarLB = ((((ia - mWConstr) / 2) << 1) + mWConstr) + 1;
          vectorUB = scalarLB - 2;
          for (ix0 = mWConstr + 1; ix0 <= vectorUB; ix0 += 2) {
            tmp = _mm_loadu_pd(&qrmanager->Q[(iy + ix0) - mWConstr]);
            tmp_0 = _mm_loadu_pd(&workspace[ix0 - 1]);
            _mm_storeu_pd(&workspace[ix0 - 1], _mm_add_pd(_mm_mul_pd(tmp,
              _mm_set1_pd(B[h_k - 1])), tmp_0));
          }

          for (ix0 = scalarLB; ix0 <= ia; ix0++) {
            workspace[ix0 - 1] += qrmanager->Q[(iy + ix0) - mWConstr] * B[h_k -
              1];
          }

          iy += 7;
        }

        ix += 25;
      }
    }

    rankQR = 0;
    do {
      exitg1 = 0;
      if (rankQR <= (uint8_T)nVar - 1) {
        if (rtIsInf(workspace[rankQR]) || rtIsNaN(workspace[rankQR])) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          workingset_maxConstrWorkspace = workspace[rankQR + 25];
          if (rtIsInf(workingset_maxConstrWorkspace) || rtIsNaN
              (workingset_maxConstrWorkspace)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            rankQR++;
          }
        }
      } else {
        scalarLB = (nVar / 2) << 1;
        vectorUB = scalarLB - 2;
        for (rankQR = 0; rankQR <= vectorUB; rankQR += 2) {
          tmp = _mm_loadu_pd(&workspace[rankQR]);
          tmp_0 = _mm_loadu_pd(&xCurrent[rankQR]);
          _mm_storeu_pd(&workspace[rankQR], _mm_add_pd(tmp, tmp_0));
        }

        for (rankQR = scalarLB; rankQR < nVar; rankQR++) {
          workspace[rankQR] += xCurrent[rankQR];
        }

        if (workingset->probType == 2) {
          temp = 0.0;
          memcpy(&workingset->maxConstrWorkspace[0], &workingset->bineq[0], 24U *
                 sizeof(real_T));
          TaskSpaceController_xgemv(6, workingset->Aineq, workspace,
            workingset->maxConstrWorkspace);
          for (rankQR = 0; rankQR < 24; rankQR++) {
            workingset_maxConstrWorkspace = workingset->
              maxConstrWorkspace[rankQR] - workspace[rankQR + 6];
            workingset->maxConstrWorkspace[rankQR] =
              workingset_maxConstrWorkspace;
            temp = fmax(temp, workingset_maxConstrWorkspace);
          }
        } else {
          temp = 0.0;
          memcpy(&workingset->maxConstrWorkspace[0], &workingset->bineq[0], 24U *
                 sizeof(real_T));
          TaskSpaceController_xgemv(workingset->nVar, workingset->Aineq,
            workspace, workingset->maxConstrWorkspace);
          for (rankQR = 0; rankQR < 24; rankQR++) {
            temp = fmax(temp, workingset->maxConstrWorkspace[rankQR]);
          }
        }

        if (workingset->sizes[3] > 0) {
          mWConstr = (uint8_T)workingset->sizes[3];
          for (rankQR = 0; rankQR < mWConstr; rankQR++) {
            temp = fmax(temp, -workspace[workingset->indexLB[rankQR] - 1]);
          }
        }

        if (workingset->probType == 2) {
          d_v = 0.0;
          memcpy(&workingset->maxConstrWorkspace[0], &workingset->bineq[0], 24U *
                 sizeof(real_T));
          TaskSpaceController_xgemv_l(6, workingset->Aineq, workspace,
            workingset->maxConstrWorkspace);
          for (rankQR = 0; rankQR < 24; rankQR++) {
            workingset_maxConstrWorkspace = workingset->
              maxConstrWorkspace[rankQR] - workspace[rankQR + 31];
            workingset->maxConstrWorkspace[rankQR] =
              workingset_maxConstrWorkspace;
            d_v = fmax(d_v, workingset_maxConstrWorkspace);
          }
        } else {
          d_v = 0.0;
          memcpy(&workingset->maxConstrWorkspace[0], &workingset->bineq[0], 24U *
                 sizeof(real_T));
          TaskSpaceController_xgemv_l(workingset->nVar, workingset->Aineq,
            workspace, workingset->maxConstrWorkspace);
          for (rankQR = 0; rankQR < 24; rankQR++) {
            d_v = fmax(d_v, workingset->maxConstrWorkspace[rankQR]);
          }
        }

        if (workingset->sizes[3] > 0) {
          mWConstr = (uint8_T)workingset->sizes[3];
          for (rankQR = 0; rankQR < mWConstr; rankQR++) {
            d_v = fmax(d_v, -workspace[workingset->indexLB[rankQR] + 24]);
          }
        }

        if ((temp <= 2.2204460492503131E-16) || (temp < d_v)) {
          memcpy(&xCurrent[0], &workspace[0], (uint8_T)nVar * sizeof(real_T));
        } else {
          memcpy(&xCurrent[0], &workspace[25], (uint8_T)nVar * sizeof(real_T));
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return nonDegenerateWset;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_xgemv_l1(int32_T m, const real_T A[168], const
  real_T x[7], real_T y[25])
{
  int32_T b_iy;
  int32_T ia;
  for (b_iy = 0; b_iy <= 22; b_iy += 2) {
    __m128d tmp;
    tmp = _mm_loadu_pd(&y[b_iy]);
    _mm_storeu_pd(&y[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
  }

  for (b_iy = 0; b_iy <= 161; b_iy += 7) {
    real_T c;
    int32_T b;
    c = 0.0;
    b = b_iy + m;
    for (ia = b_iy + 1; ia <= b; ia++) {
      c += x[(ia - b_iy) - 1] * A[ia - 1];
    }

    ia = div_nde_s32_floor(b_iy, 7);
    y[ia] += c;
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static real_T TaskSpac_maxConstraintViolation(sAElXDmDj36R7Z42SImJxmG_TaskS_T
  *obj, const real_T x[7])
{
  real_T obj_maxConstrWorkspace;
  real_T v;
  int32_T b;
  int32_T k;
  if (obj->probType == 2) {
    v = 0.0;
    memcpy(&obj->maxConstrWorkspace[0], &obj->bineq[0], 24U * sizeof(real_T));
    TaskSpaceController_xgemv_l1(6, obj->Aineq, x, obj->maxConstrWorkspace);
    for (k = 0; k < 24; k++) {
      obj_maxConstrWorkspace = obj->maxConstrWorkspace[k] - x[6];
      obj->maxConstrWorkspace[k] = obj_maxConstrWorkspace;
      v = fmax(v, obj_maxConstrWorkspace);
    }
  } else {
    v = 0.0;
    memcpy(&obj->maxConstrWorkspace[0], &obj->bineq[0], 24U * sizeof(real_T));
    TaskSpaceController_xgemv_l1(obj->nVar, obj->Aineq, x,
      obj->maxConstrWorkspace);
    for (k = 0; k < 24; k++) {
      v = fmax(v, obj->maxConstrWorkspace[k]);
    }
  }

  if (obj->sizes[3] > 0) {
    b = (uint8_T)obj->sizes[3];
    for (k = 0; k < b; k++) {
      v = fmax(v, -x[obj->indexLB[k] - 1]);
    }
  }

  return v;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpa_modifyOverheadPhaseOne_(sAElXDmDj36R7Z42SImJxmG_TaskS_T *obj)
{
  int32_T b;
  int32_T idx;
  int32_T idxStartIneq;
  for (idx = 0; idx < 24; idx++) {
    obj->Aineq[7 * idx + 6] = -1.0;
  }

  obj->indexLB[obj->sizes[3] - 1] = 7;
  obj->lb[6] = 0.0;
  idxStartIneq = obj->isActiveIdx[2];
  b = obj->nActiveConstr;
  for (idx = idxStartIneq; idx <= b; idx++) {
    obj->ATwset[7 * (idx - 1) + 6] = -1.0;
  }

  if (obj->nWConstr[4] <= 0) {
    obj->isActiveConstr[obj->isActiveIdx[4] - 1] = false;
  }

  obj->isActiveConstr[obj->isActiveIdx[4] - 1] = false;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceControl_setProblemType(sAElXDmDj36R7Z42SImJxmG_TaskS_T *obj,
  int32_T PROBLEM_TYPE)
{
  int32_T colOffsetATw;
  int32_T colOffsetAineq;
  int32_T e_tmp;
  int32_T idx_col;
  int32_T idx_lb;
  switch (PROBLEM_TYPE) {
   case 3:
    obj->nVar = 6;
    obj->mConstr = 24;
    for (idx_col = 0; idx_col < 5; idx_col++) {
      obj->sizes[idx_col] = obj->sizesNormal[idx_col];
    }

    for (idx_col = 0; idx_col < 6; idx_col++) {
      obj->isActiveIdx[idx_col] = obj->isActiveIdxNormal[idx_col];
    }
    break;

   case 1:
    obj->nVar = 7;
    obj->mConstr = 25;
    for (idx_col = 0; idx_col < 5; idx_col++) {
      obj->sizes[idx_col] = obj->sizesPhaseOne[idx_col];
    }

    TaskSpa_modifyOverheadPhaseOne_(obj);
    for (idx_col = 0; idx_col < 6; idx_col++) {
      obj->isActiveIdx[idx_col] = obj->isActiveIdxPhaseOne[idx_col];
    }
    break;

   case 2:
    obj->nVar = 6;
    obj->mConstr = 24;
    for (idx_col = 0; idx_col < 5; idx_col++) {
      obj->sizes[idx_col] = obj->sizesRegularized[idx_col];
    }

    if (obj->probType != 4) {
      for (idx_col = 0; idx_col < 24; idx_col++) {
        colOffsetAineq = 7 * idx_col;
        for (idx_lb = 7; idx_lb <= idx_col + 6; idx_lb++) {
          obj->Aineq[(idx_lb + colOffsetAineq) - 1] = 0.0;
        }

        obj->Aineq[(idx_col + colOffsetAineq) + 6] = -1.0;
      }

      idx_lb = 6;
      for (idx_col = 0; idx_col < 24; idx_col++) {
        idx_lb++;
        obj->indexLB[idx_col] = idx_lb;
      }

      idx_lb = obj->isActiveIdx[4];
      colOffsetAineq = obj->isActiveIdxRegularized[4];
      if (idx_lb <= colOffsetAineq - 1) {
        memset(&obj->isActiveConstr[idx_lb + -1], 0, (uint32_T)(colOffsetAineq -
                idx_lb) * sizeof(boolean_T));
      }

      obj->lb[6] = 0.0;
      idx_lb = obj->isActiveIdx[2];
      colOffsetAineq = obj->nActiveConstr;
      for (idx_col = idx_lb; idx_col <= colOffsetAineq; idx_col++) {
        colOffsetATw = (idx_col - 1) * 7 - 1;
        if (obj->Wid[idx_col - 1] == 3) {
          e_tmp = obj->Wlocalidx[idx_col - 1];
          if (e_tmp + 5 >= 7) {
            memset(&obj->ATwset[colOffsetATw + 7], 0, (uint32_T)((e_tmp + 5) - 6)
                   * sizeof(real_T));
          }

          obj->ATwset[(e_tmp + colOffsetATw) + 6] = -1.0;
          if (e_tmp + 7 <= 6) {
            memset(&obj->ATwset[(e_tmp + 7) + colOffsetATw], 0, (uint32_T)
                   (((colOffsetATw - (e_tmp + 7)) - colOffsetATw) + 7) * sizeof
                   (real_T));
          }
        }
      }
    }

    for (idx_col = 0; idx_col < 6; idx_col++) {
      obj->isActiveIdx[idx_col] = obj->isActiveIdxRegularized[idx_col];
    }
    break;

   default:
    obj->nVar = 7;
    obj->mConstr = 25;
    for (idx_col = 0; idx_col < 5; idx_col++) {
      obj->sizes[idx_col] = obj->sizesRegPhaseOne[idx_col];
    }

    TaskSpa_modifyOverheadPhaseOne_(obj);
    for (idx_col = 0; idx_col < 6; idx_col++) {
      obj->isActiveIdx[idx_col] = obj->isActiveIdxRegPhaseOne[idx_col];
    }
    break;
  }

  obj->probType = PROBLEM_TYPE;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_xgemv_l1n(int32_T m, int32_T n, int32_T lda,
  const real_T x[7], real_T y[6])
{
  int32_T b_iy;
  int32_T ia;
  static const int8_T d[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  if ((m != 0) && (n != 0)) {
    int32_T b;
    int32_T ix;
    if (m - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)m * sizeof(real_T));
    }

    ix = 0;
    b = (n - 1) * lda + 1;
    for (b_iy = 1; lda < 0 ? b_iy >= b : b_iy <= b; b_iy += lda) {
      int32_T c;
      c = (b_iy + m) - 1;
      for (ia = b_iy; ia <= c; ia++) {
        int32_T tmp;
        tmp = ia - b_iy;
        y[tmp] += (real_T)d[ia - 1] * x[ix];
      }

      ix++;
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceCo_computeGrad_StoreHx(slzZ8M58FXlZqTD433BZJUH_TaskS_T *obj,
  const real_T f[6], const real_T x[7])
{
  __m128d tmp;
  int32_T b_ixlast;
  int32_T idx;
  int32_T scalarLB;
  int32_T vectorUB;
  switch (obj->objtype) {
   case 5:
    if (obj->nvar - 2 >= 0) {
      memset(&obj->grad[0], 0, (uint32_T)(obj->nvar - 1) * sizeof(real_T));
    }

    obj->grad[obj->nvar - 1] = obj->gammaScalar;
    break;

   case 3:
    TaskSpaceController_xgemv_l1n(obj->nvar, obj->nvar, obj->nvar, x, obj->Hx);
    if (obj->nvar - 1 >= 0) {
      memcpy(&obj->grad[0], &obj->Hx[0], (uint32_T)obj->nvar * sizeof(real_T));
    }

    if (obj->hasLinear && (obj->nvar >= 1)) {
      b_ixlast = obj->nvar;
      scalarLB = (obj->nvar / 2) << 1;
      vectorUB = scalarLB - 2;
      for (idx = 0; idx <= vectorUB; idx += 2) {
        tmp = _mm_loadu_pd(&obj->grad[idx]);
        _mm_storeu_pd(&obj->grad[idx], _mm_add_pd(tmp, _mm_loadu_pd(&f[idx])));
      }

      for (idx = scalarLB; idx < b_ixlast; idx++) {
        obj->grad[idx] += f[idx];
      }
    }
    break;

   default:
    TaskSpaceController_xgemv_l1n(obj->nvar, obj->nvar, obj->nvar, x, obj->Hx);
    b_ixlast = obj->nvar + 1;
    scalarLB = ((((6 - obj->nvar) / 2) << 1) + obj->nvar) + 1;
    vectorUB = scalarLB - 2;
    for (idx = b_ixlast; idx <= vectorUB; idx += 2) {
      _mm_storeu_pd(&obj->Hx[idx - 1], _mm_mul_pd(_mm_loadu_pd(&x[idx - 1]),
        _mm_set1_pd(0.0)));
    }

    for (idx = scalarLB; idx < 7; idx++) {
      obj->Hx[idx - 1] = x[idx - 1] * 0.0;
    }

    for (idx = 0; idx < 6; idx++) {
      obj->grad[idx] = obj->Hx[idx];
    }

    if (obj->hasLinear && (obj->nvar >= 1)) {
      b_ixlast = obj->nvar;
      scalarLB = (obj->nvar / 2) << 1;
      vectorUB = scalarLB - 2;
      for (idx = 0; idx <= vectorUB; idx += 2) {
        tmp = _mm_loadu_pd(&obj->grad[idx]);
        _mm_storeu_pd(&obj->grad[idx], _mm_add_pd(tmp, _mm_loadu_pd(&f[idx])));
      }

      for (idx = scalarLB; idx < b_ixlast; idx++) {
        obj->grad[idx] += f[idx];
      }
    }
    break;
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static real_T TaskSpaceCo_computeFval_ReuseHx(const
  slzZ8M58FXlZqTD433BZJUH_TaskS_T *obj, real_T workspace[175], const real_T f[6],
  const real_T x[7])
{
  real_T val;
  int32_T k;
  switch (obj->objtype) {
   case 5:
    val = x[obj->nvar - 1] * obj->gammaScalar;
    break;

   case 3:
    {
      if (obj->hasLinear) {
        int32_T ixlast;
        int32_T scalarLB;
        int32_T vectorUB;
        ixlast = obj->nvar;
        scalarLB = (obj->nvar / 2) << 1;
        vectorUB = scalarLB - 2;
        for (k = 0; k <= vectorUB; k += 2) {
          __m128d tmp;
          tmp = _mm_loadu_pd(&obj->Hx[k]);
          _mm_storeu_pd(&workspace[k], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.5),
            tmp), _mm_loadu_pd(&f[k])));
        }

        for (k = scalarLB; k < ixlast; k++) {
          workspace[k] = 0.5 * obj->Hx[k] + f[k];
        }

        val = 0.0;
        if (obj->nvar >= 1) {
          for (k = 0; k < ixlast; k++) {
            val += x[k] * workspace[k];
          }
        }
      } else {
        val = 0.0;
        if (obj->nvar >= 1) {
          int32_T ixlast;
          ixlast = obj->nvar;
          for (k = 0; k < ixlast; k++) {
            val += x[k] * obj->Hx[k];
          }
        }

        val *= 0.5;
      }
    }
    break;

   default:
    {
      if (obj->hasLinear) {
        int32_T ixlast;
        if (obj->nvar - 1 >= 0) {
          memcpy(&workspace[0], &f[0], (uint32_T)obj->nvar * sizeof(real_T));
        }

        ixlast = 6 - obj->nvar;
        for (k = 0; k < ixlast; k++) {
          workspace[obj->nvar + k] = 0.0;
        }

        val = 0.0;
        for (k = 0; k < 6; k++) {
          real_T workspace_0;
          workspace_0 = 0.5 * obj->Hx[k] + workspace[k];
          workspace[k] = workspace_0;
          val += x[k] * workspace_0;
        }
      } else {
        int32_T ixlast;
        val = 0.0;
        for (k = 0; k < 6; k++) {
          val += x[k] * obj->Hx[k];
        }

        val *= 0.5;
        ixlast = obj->nvar + 1;
        for (k = ixlast; k < 7; k++) {
          val += x[k - 1] * 0.0;
        }
      }
    }
    break;
  }

  return val;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_factorQR(skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *obj,
  const real_T A[175], int32_T mrows, int32_T ncols)
{
  int32_T i;
  int32_T idx;
  static const int32_T offsets[4] = { 0, 1, 2, 3 };

  int32_T ix0_tmp;
  boolean_T guard1;
  idx = mrows * ncols;
  guard1 = false;
  if (idx > 0) {
    for (idx = 0; idx < ncols; idx++) {
      ix0_tmp = 7 * idx;
      memcpy(&obj->QR[ix0_tmp], &A[ix0_tmp], (uint8_T)mrows * sizeof(real_T));
    }

    guard1 = true;
  } else if (idx == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    i = (ncols / 4) << 2;
    ix0_tmp = i - 4;
    for (idx = 0; idx <= ix0_tmp; idx += 4) {
      _mm_storeu_si128((__m128i *)&obj->jpvt[idx], _mm_add_epi32(_mm_add_epi32
        (_mm_set1_epi32(idx), _mm_loadu_si128((const __m128i *)&offsets[0])),
        _mm_set1_epi32(1)));
    }

    for (idx = i; idx < ncols; idx++) {
      obj->jpvt[idx] = idx + 1;
    }

    if (mrows <= ncols) {
      idx = mrows;
    } else {
      idx = ncols;
    }

    obj->minRowCol = idx;
    for (i = 0; i < 7; i++) {
      obj->tau[i] = 0.0;
    }

    if (idx >= 1) {
      for (i = 0; i < 7; i++) {
        obj->tau[i] = 0.0;
      }

      TaskSpaceController_qrf(obj->QR, mrows, ncols, idx, obj->tau);
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_xrotg(real_T *a, real_T *b, real_T *c, real_T *s)
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

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceCont_squareQ_appendCol(skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *obj,
  const real_T vec[175], int32_T iv0)
{
  real_T b_c;
  real_T s;
  real_T temp;
  real_T temp_tmp;
  int32_T b_iy;
  int32_T d;
  int32_T e;
  int32_T idxRotGCol;
  int32_T iyend;
  if (obj->mrows <= obj->ncols + 1) {
    obj->minRowCol = obj->mrows;
  } else {
    obj->minRowCol = obj->ncols + 1;
  }

  b_iy = 7 * obj->ncols;
  if (obj->mrows != 0) {
    iyend = b_iy + obj->mrows;
    if (b_iy + 1 <= iyend) {
      memset(&obj->QR[b_iy], 0, (uint32_T)(iyend - b_iy) * sizeof(real_T));
    }

    d = (obj->mrows - 1) * 7 + 1;
    for (idxRotGCol = 1; idxRotGCol <= d; idxRotGCol += 7) {
      b_c = 0.0;
      e = (idxRotGCol + obj->mrows) - 1;
      for (iyend = idxRotGCol; iyend <= e; iyend++) {
        b_c += vec[((iv0 + iyend) - idxRotGCol) - 1] * obj->Q[iyend - 1];
      }

      iyend = div_nde_s32_floor(idxRotGCol - 1, 7) + b_iy;
      obj->QR[iyend] += b_c;
    }
  }

  obj->ncols++;
  obj->jpvt[obj->ncols - 1] = obj->ncols;
  for (b_iy = obj->mrows - 2; b_iy + 2 > obj->ncols; b_iy--) {
    e = (obj->ncols - 1) * 7 + b_iy;
    temp = obj->QR[e + 1];
    TaskSpaceController_xrotg(&obj->QR[e], &temp, &b_c, &s);
    obj->QR[e + 1] = temp;
    iyend = 7 * b_iy;
    d = obj->mrows;
    if (obj->mrows >= 1) {
      for (idxRotGCol = 0; idxRotGCol < d; idxRotGCol++) {
        e = iyend + idxRotGCol;
        temp_tmp = obj->Q[e + 7];
        temp = temp_tmp * s + obj->Q[e] * b_c;
        obj->Q[e + 7] = temp_tmp * b_c - obj->Q[e] * s;
        obj->Q[e] = temp;
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceContr_deleteColMoveEnd(skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *obj,
  int32_T idx)
{
  real_T b_s;
  real_T b_temp;
  real_T b_temp_tmp_0;
  real_T c_c;
  int32_T QRk0;
  int32_T b_ix;
  int32_T b_n;
  int32_T b_temp_tmp;
  int32_T i;
  int32_T idxRotGCol;
  int32_T idxRotGCol_tmp;
  int32_T k;
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt[i - 1] != idx)) {
      i++;
    }

    idx = i;
  }

  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    obj->jpvt[idx - 1] = obj->jpvt[obj->ncols - 1];
    idxRotGCol = obj->minRowCol;
    for (i = 0; i < idxRotGCol; i++) {
      obj->QR[i + 7 * (idx - 1)] = obj->QR[(obj->ncols - 1) * 7 + i];
    }

    obj->ncols--;
    if (obj->mrows <= obj->ncols) {
      obj->minRowCol = obj->mrows;
    } else {
      obj->minRowCol = obj->ncols;
    }

    if (idx < obj->mrows) {
      if (obj->mrows - 1 <= obj->ncols) {
        i = obj->mrows - 1;
      } else {
        i = obj->ncols;
      }

      k = i;
      idxRotGCol = (idx - 1) * 7;
      while (k >= idx) {
        b_temp_tmp = k + idxRotGCol;
        b_temp = obj->QR[b_temp_tmp];
        TaskSpaceController_xrotg(&obj->QR[b_temp_tmp - 1], &b_temp, &c_c, &b_s);
        obj->QR[b_temp_tmp] = b_temp;
        idxRotGCol_tmp = (k - 1) * 7;
        obj->QR[k + idxRotGCol_tmp] = 0.0;
        QRk0 = 7 * idx + k;
        b_ix = obj->ncols - idx;
        if (b_ix >= 1) {
          for (b_n = 0; b_n < b_ix; b_n++) {
            b_temp_tmp = b_n * 7 + QRk0;
            b_temp_tmp_0 = obj->QR[b_temp_tmp - 1];
            b_temp = b_temp_tmp_0 * c_c + obj->QR[b_temp_tmp] * b_s;
            obj->QR[b_temp_tmp] = obj->QR[b_temp_tmp] * c_c - b_temp_tmp_0 * b_s;
            obj->QR[b_temp_tmp - 1] = b_temp;
          }
        }

        b_ix = obj->mrows;
        if (obj->mrows >= 1) {
          for (b_n = 0; b_n < b_ix; b_n++) {
            b_temp_tmp = idxRotGCol_tmp + b_n;
            b_temp_tmp_0 = obj->Q[b_temp_tmp + 7];
            b_temp = b_temp_tmp_0 * b_s + obj->Q[b_temp_tmp] * c_c;
            obj->Q[b_temp_tmp + 7] = b_temp_tmp_0 * c_c - obj->Q[b_temp_tmp] *
              b_s;
            obj->Q[b_temp_tmp] = b_temp;
          }
        }

        k--;
      }

      for (k = idx + 1; k <= i; k++) {
        idxRotGCol_tmp = (k - 1) * 7;
        b_temp_tmp = k + idxRotGCol_tmp;
        b_temp = obj->QR[b_temp_tmp];
        TaskSpaceController_xrotg(&obj->QR[b_temp_tmp - 1], &b_temp, &c_c, &b_s);
        obj->QR[b_temp_tmp] = b_temp;
        QRk0 = k << 3;
        b_n = obj->ncols - k;
        if (b_n >= 1) {
          for (idxRotGCol = 0; idxRotGCol < b_n; idxRotGCol++) {
            b_temp_tmp = idxRotGCol * 7 + QRk0;
            b_temp = obj->QR[b_temp_tmp - 1] * c_c + obj->QR[b_temp_tmp] * b_s;
            obj->QR[b_temp_tmp] = obj->QR[b_temp_tmp] * c_c - obj->QR[b_temp_tmp
              - 1] * b_s;
            obj->QR[b_temp_tmp - 1] = b_temp;
          }
        }

        b_n = obj->mrows;
        if (obj->mrows >= 1) {
          for (idxRotGCol = 0; idxRotGCol < b_n; idxRotGCol++) {
            b_temp_tmp = idxRotGCol_tmp + idxRotGCol;
            b_temp = obj->Q[b_temp_tmp + 7] * b_s + obj->Q[b_temp_tmp] * c_c;
            obj->Q[b_temp_tmp + 7] = obj->Q[b_temp_tmp + 7] * c_c - obj->
              Q[b_temp_tmp] * b_s;
            obj->Q[b_temp_tmp] = b_temp;
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceControlle_fullColLDL2_(s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T *obj,
  int32_T NColsRemain, real_T REG_PRIMAL)
{
  int32_T b_k;
  int32_T ijA;
  int32_T lastDiag;
  int32_T subMatrixDim;
  for (lastDiag = 0; lastDiag < NColsRemain; lastDiag++) {
    __m128d tmp;
    real_T obj_FMat;
    int32_T LD_diagOffset;
    int32_T scalarLB;
    int32_T vectorUB;
    LD_diagOffset = lastDiag << 3;
    obj_FMat = obj->FMat[LD_diagOffset];
    if (fabs(obj_FMat) <= obj->regTol_) {
      obj_FMat += REG_PRIMAL;
      obj->FMat[LD_diagOffset] = obj_FMat;
    }

    obj_FMat = -1.0 / obj_FMat;
    subMatrixDim = (NColsRemain - lastDiag) - 2;
    for (b_k = 0; b_k <= subMatrixDim; b_k++) {
      obj->workspace_[b_k] = obj->FMat[(LD_diagOffset + b_k) + 1];
    }

    if (!(obj_FMat == 0.0)) {
      int32_T jA;
      jA = LD_diagOffset + 9;
      for (b_k = 0; b_k <= subMatrixDim; b_k++) {
        real_T temp;
        temp = obj->workspace_[b_k];
        if (temp != 0.0) {
          int32_T b;
          temp *= obj_FMat;
          b = subMatrixDim + jA;
          scalarLB = ((((b - jA) + 1) / 2) << 1) + jA;
          vectorUB = scalarLB - 2;
          for (ijA = jA; ijA <= vectorUB; ijA += 2) {
            __m128d tmp_0;
            tmp = _mm_loadu_pd(&obj->workspace_[ijA - jA]);
            tmp_0 = _mm_loadu_pd(&obj->FMat[ijA - 1]);
            _mm_storeu_pd(&obj->FMat[ijA - 1], _mm_add_pd(_mm_mul_pd(tmp,
              _mm_set1_pd(temp)), tmp_0));
          }

          for (ijA = scalarLB; ijA <= b; ijA++) {
            obj->FMat[ijA - 1] += obj->workspace_[ijA - jA] * temp;
          }
        }

        jA += 7;
      }
    }

    obj_FMat = 1.0 / obj->FMat[LD_diagOffset];
    b_k = LD_diagOffset + subMatrixDim;
    scalarLB = (((((b_k - LD_diagOffset) + 1) / 2) << 1) + LD_diagOffset) + 2;
    vectorUB = scalarLB - 2;
    for (subMatrixDim = LD_diagOffset + 2; subMatrixDim <= vectorUB;
         subMatrixDim += 2) {
      tmp = _mm_loadu_pd(&obj->FMat[subMatrixDim - 1]);
      _mm_storeu_pd(&obj->FMat[subMatrixDim - 1], _mm_mul_pd(tmp, _mm_set1_pd
        (obj_FMat)));
    }

    for (subMatrixDim = scalarLB; subMatrixDim <= b_k + 2; subMatrixDim++) {
      obj->FMat[subMatrixDim - 1] *= obj_FMat;
    }
  }

  lastDiag = (NColsRemain - 1) << 3;
  if (fabs(obj->FMat[lastDiag]) <= obj->regTol_) {
    obj->FMat[lastDiag] += REG_PRIMAL;
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_xgemv_l1nm(int32_T m, int32_T n, const real_T A
  [49], int32_T ia0, const real_T x[175], real_T y[7])
{
  int32_T b_iy;
  int32_T ia;
  if (m != 0) {
    int32_T b;
    int32_T ix;
    if (m - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)m * sizeof(real_T));
    }

    ix = 0;
    b = (n - 1) * 7 + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 7) {
      int32_T c;
      c = (b_iy + m) - 1;
      for (ia = b_iy; ia <= c; ia++) {
        int32_T tmp;
        tmp = ia - b_iy;
        y[tmp] += A[ia - 1] * x[ix];
      }

      ix++;
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceControl_compute_deltax(smNINkioqq1a7FyOE4CETSB_TaskS_T
  *solution, sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T *memspace, const
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager, s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T
  *cholmanager, const slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective)
{
  __m128d tmp;
  real_T s;
  real_T smax;
  real_T temp;
  int32_T ar;
  int32_T b_jjA;
  int32_T br;
  int32_T h;
  int32_T i;
  int32_T ix;
  int32_T lastColC;
  int32_T mNull_tmp;
  int32_T nVar;
  int32_T nVars;
  int32_T nullStart;
  int32_T nullStartIdx;
  int32_T nullStartIdx_tmp;
  static const int8_T p[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  int32_T exitg1;
  nVar = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    if (qrmanager->mrows - 1 >= 0) {
      memset(&solution->searchDir[0], 0, (uint32_T)((qrmanager->mrows - 1) + 1) *
             sizeof(real_T));
    }
  } else {
    nVars = (qrmanager->mrows / 2) << 1;
    ix = nVars - 2;
    for (nullStartIdx = 0; nullStartIdx <= ix; nullStartIdx += 2) {
      tmp = _mm_loadu_pd(&objective->grad[nullStartIdx]);
      _mm_storeu_pd(&solution->searchDir[nullStartIdx], _mm_mul_pd(tmp,
        _mm_set1_pd(-1.0)));
    }

    for (nullStartIdx = nVars; nullStartIdx <= nVar; nullStartIdx++) {
      solution->searchDir[nullStartIdx] = -objective->grad[nullStartIdx];
    }

    if (qrmanager->ncols <= 0) {
      if (objective->objtype == 3) {
        temp = 1.4901161193847656E-8 * cholmanager->scaleFactor * (real_T)
          qrmanager->mrows;
        cholmanager->ndims = qrmanager->mrows;
        for (b_jjA = 0; b_jjA <= nVar; b_jjA++) {
          nVars = (nVar + 1) * b_jjA;
          lastColC = 7 * b_jjA;
          for (nullStartIdx = 0; nullStartIdx <= nVar; nullStartIdx++) {
            cholmanager->FMat[lastColC + nullStartIdx] = p[nullStartIdx + nVars];
          }
        }

        if (qrmanager->mrows < 1) {
          nullStartIdx = -1;
        } else {
          nullStartIdx = 0;
          if (qrmanager->mrows > 1) {
            smax = fabs(cholmanager->FMat[0]);
            for (b_jjA = 2; b_jjA <= nVar + 1; b_jjA++) {
              s = fabs(cholmanager->FMat[(b_jjA - 1) << 3]);
              if (s > smax) {
                nullStartIdx = b_jjA - 1;
                smax = s;
              }
            }
          }
        }

        cholmanager->regTol_ = fmax(fabs(cholmanager->FMat[7 * nullStartIdx +
          nullStartIdx]) * 2.2204460492503131E-16, fabs(temp));
        TaskSpaceControlle_fullColLDL2_(cholmanager, qrmanager->mrows, temp);
        if (cholmanager->ConvexCheck) {
          b_jjA = 0;
          do {
            exitg1 = 0;
            if (b_jjA <= nVar) {
              if (cholmanager->FMat[7 * b_jjA + b_jjA] <= 0.0) {
                cholmanager->info = -b_jjA - 1;
                exitg1 = 1;
              } else {
                b_jjA++;
              }
            } else {
              cholmanager->ConvexCheck = false;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          nVar = cholmanager->ndims;
          if (cholmanager->ndims != 0) {
            for (b_jjA = 0; b_jjA < nVar; b_jjA++) {
              lastColC = b_jjA * 7 + b_jjA;
              nullStart = (nVar - b_jjA) - 2;
              for (nullStartIdx = 0; nullStartIdx <= nullStart; nullStartIdx++)
              {
                ix = (nullStartIdx + b_jjA) + 1;
                solution->searchDir[ix] -= cholmanager->FMat[(nullStartIdx +
                  lastColC) + 1] * solution->searchDir[b_jjA];
              }
            }
          }

          for (b_jjA = 0; b_jjA < nVar; b_jjA++) {
            solution->searchDir[b_jjA] /= cholmanager->FMat[7 * b_jjA + b_jjA];
          }

          if (cholmanager->ndims != 0) {
            for (b_jjA = nVar; b_jjA >= 1; b_jjA--) {
              nVars = (b_jjA - 1) * 7;
              temp = solution->searchDir[b_jjA - 1];
              for (nullStartIdx = nVar; nullStartIdx >= b_jjA + 1; nullStartIdx
                   --) {
                temp -= cholmanager->FMat[(nVars + nullStartIdx) - 1] *
                  solution->searchDir[nullStartIdx - 1];
              }

              solution->searchDir[b_jjA - 1] = temp;
            }
          }
        }
      }
    } else {
      nullStartIdx_tmp = 7 * qrmanager->ncols;
      nullStartIdx = nullStartIdx_tmp + 1;
      if (objective->objtype == 5) {
        for (nVars = 0; nVars < mNull_tmp; nVars++) {
          memspace->workspace_float[nVars] = -qrmanager->Q[(qrmanager->ncols +
            nVars) * 7 + nVar];
        }

        TaskSpaceController_xgemv_l1nm(qrmanager->mrows, mNull_tmp, qrmanager->Q,
          nullStartIdx_tmp + 1, memspace->workspace_float, solution->searchDir);
      } else {
        if (objective->objtype == 3) {
          nVars = qrmanager->mrows;
          if ((qrmanager->mrows != 0) && (mNull_tmp != 0)) {
            br = nullStartIdx_tmp;
            lastColC = (mNull_tmp - 1) * 25;
            for (b_jjA = 0; b_jjA <= lastColC; b_jjA += 25) {
              nullStart = b_jjA + nVars;
              for (ix = b_jjA + 1; ix <= nullStart; ix++) {
                memspace->workspace_float[ix - 1] = 0.0;
              }
            }

            for (b_jjA = 0; b_jjA <= lastColC; b_jjA += 25) {
              ar = -1;
              h = br + nVars;
              for (ix = br + 1; ix <= h; ix++) {
                i = b_jjA + nVars;
                for (nullStart = b_jjA + 1; nullStart <= i; nullStart++) {
                  memspace->workspace_float[nullStart - 1] += (real_T)p[(ar +
                    nullStart) - b_jjA] * qrmanager->Q[ix - 1];
                }

                ar += nVars;
              }

              br += 7;
            }
          }

          if (mNull_tmp != 0) {
            br = (mNull_tmp - 1) * 7;
            for (b_jjA = 0; b_jjA <= br; b_jjA += 7) {
              nullStart = b_jjA + mNull_tmp;
              for (ix = b_jjA + 1; ix <= nullStart; ix++) {
                cholmanager->FMat[ix - 1] = 0.0;
              }
            }

            lastColC = -1;
            for (b_jjA = 0; b_jjA <= br; b_jjA += 7) {
              ar = nullStartIdx_tmp;
              h = b_jjA + mNull_tmp;
              for (ix = b_jjA + 1; ix <= h; ix++) {
                temp = 0.0;
                for (nullStart = 0; nullStart < nVars; nullStart++) {
                  temp += memspace->workspace_float[(nullStart + lastColC) + 1] *
                    qrmanager->Q[nullStart + ar];
                }

                cholmanager->FMat[ix - 1] += temp;
                ar += 7;
              }

              lastColC += 25;
            }
          }
        }

        temp = 1.4901161193847656E-8 * cholmanager->scaleFactor * (real_T)
          mNull_tmp;
        cholmanager->ndims = mNull_tmp;
        lastColC = 0;
        if (mNull_tmp > 1) {
          smax = fabs(cholmanager->FMat[0]);
          for (nVars = 2; nVars <= mNull_tmp; nVars++) {
            s = fabs(cholmanager->FMat[(nVars - 1) << 3]);
            if (s > smax) {
              lastColC = nVars - 1;
              smax = s;
            }
          }
        }

        cholmanager->regTol_ = fmax(fabs(cholmanager->FMat[7 * lastColC +
          lastColC]) * 2.2204460492503131E-16, fabs(temp));
        TaskSpaceControlle_fullColLDL2_(cholmanager, mNull_tmp, temp);
        if (cholmanager->ConvexCheck) {
          nVars = 0;
          do {
            exitg1 = 0;
            if (nVars <= mNull_tmp - 1) {
              if (cholmanager->FMat[7 * nVars + nVars] <= 0.0) {
                cholmanager->info = -nVars - 1;
                exitg1 = 1;
              } else {
                nVars++;
              }
            } else {
              cholmanager->ConvexCheck = false;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (qrmanager->mrows != 0) {
            memset(&memspace->workspace_float[0], 0, (uint32_T)mNull_tmp *
                   sizeof(real_T));
            b_jjA = ((mNull_tmp - 1) * 7 + nullStartIdx_tmp) + 1;
            for (nVars = nullStartIdx; nVars <= b_jjA; nVars += 7) {
              temp = 0.0;
              nullStart = nVars + nVar;
              for (lastColC = nVars; lastColC <= nullStart; lastColC++) {
                temp += qrmanager->Q[lastColC - 1] * objective->grad[lastColC -
                  nVars];
              }

              ix = div_nde_s32_floor((nVars - nullStartIdx_tmp) - 1, 7);
              memspace->workspace_float[ix] -= temp;
            }
          }

          lastColC = cholmanager->ndims;
          if (cholmanager->ndims != 0) {
            for (nVar = 0; nVar < lastColC; nVar++) {
              b_jjA = nVar * 7 + nVar;
              ix = (lastColC - nVar) - 2;
              for (nVars = 0; nVars <= ix; nVars++) {
                br = (nVars + nVar) + 1;
                memspace->workspace_float[br] -= cholmanager->FMat[(nVars +
                  b_jjA) + 1] * memspace->workspace_float[nVar];
              }
            }
          }

          nVars = cholmanager->ndims;
          for (nVar = 0; nVar < nVars; nVar++) {
            memspace->workspace_float[nVar] /= cholmanager->FMat[7 * nVar + nVar];
          }

          lastColC = cholmanager->ndims;
          if (cholmanager->ndims != 0) {
            for (nVar = lastColC; nVar >= 1; nVar--) {
              nullStart = (nVar - 1) * 7;
              temp = memspace->workspace_float[nVar - 1];
              for (nVars = lastColC; nVars >= nVar + 1; nVars--) {
                temp -= cholmanager->FMat[(nullStart + nVars) - 1] *
                  memspace->workspace_float[nVars - 1];
              }

              memspace->workspace_float[nVar - 1] = temp;
            }
          }

          TaskSpaceController_xgemv_l1nm(qrmanager->mrows, mNull_tmp,
            qrmanager->Q, nullStartIdx_tmp + 1, memspace->workspace_float,
            solution->searchDir);
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static real_T TaskSpaceController_xnrm2_n(int32_T n, const real_T x[7])
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[0]);
    } else {
      real_T scale;
      scale = 3.3121686421112381E-170;
      for (k = 0; k < n; k++) {
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

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_xgemv_l1nmk(int32_T m, const real_T A[168],
  const real_T x[7], real_T y[175])
{
  int32_T b_iy;
  int32_T ia;
  for (b_iy = 0; b_iy <= 22; b_iy += 2) {
    __m128d tmp;
    tmp = _mm_loadu_pd(&y[b_iy]);
    _mm_storeu_pd(&y[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
  }

  for (b_iy = 0; b_iy <= 161; b_iy += 7) {
    real_T c;
    int32_T b;
    c = 0.0;
    b = b_iy + m;
    for (ia = b_iy + 1; ia <= b; ia++) {
      c += x[(ia - b_iy) - 1] * A[ia - 1];
    }

    ia = div_nde_s32_floor(b_iy, 7);
    y[ia] += c;
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void Task_addBoundToActiveSetMatrix_(sAElXDmDj36R7Z42SImJxmG_TaskS_T *obj,
  int32_T TYPE, int32_T idx_local)
{
  int32_T colOffset;
  int32_T idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid[obj->nActiveConstr - 1] = TYPE;
  obj->Wlocalidx[obj->nActiveConstr - 1] = idx_local;
  colOffset = (obj->nActiveConstr - 1) * 7 - 1;
  if (TYPE == 5) {
    /* Check node always fails. would cause program termination and was eliminated */
  } else {
    idx_bnd_local = obj->indexLB[idx_local - 1];
    obj->bwset[obj->nActiveConstr - 1] = 0.0;
  }

  if (idx_bnd_local - 2 >= 0) {
    memset(&obj->ATwset[colOffset + 1], 0, (uint32_T)(idx_bnd_local - 1) *
           sizeof(real_T));
  }

  obj->ATwset[idx_bnd_local + colOffset] = (real_T)(TYPE == 5) * 2.0 - 1.0;
  if (idx_bnd_local + 1 <= obj->nVar) {
    memset(&obj->ATwset[(idx_bnd_local + colOffset) + 1], 0, (uint32_T)
           (((obj->nVar + colOffset) - idx_bnd_local) - colOffset) * sizeof
           (real_T));
  }

  switch (obj->probType) {
   case 3:
   case 2:
    break;

   default:
    obj->ATwset[obj->nVar + colOffset] = -1.0;
    break;
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceControl_addAineqConstr(sAElXDmDj36R7Z42SImJxmG_TaskS_T *obj,
  int32_T idx_local)
{
  int32_T b;
  int32_T iAineq0;
  int32_T iAw0;
  int32_T idx;
  obj->nWConstr[2]++;
  obj->isActiveConstr[(obj->isActiveIdx[2] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid[obj->nActiveConstr - 1] = 3;
  obj->Wlocalidx[obj->nActiveConstr - 1] = idx_local;
  iAineq0 = (idx_local - 1) * 7;
  iAw0 = (obj->nActiveConstr - 1) * 7;
  b = obj->nVar;
  for (idx = 0; idx < b; idx++) {
    obj->ATwset[iAw0 + idx] = obj->Aineq[iAineq0 + idx];
  }

  obj->bwset[obj->nActiveConstr - 1] = obj->bineq[idx_local - 1];
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceControl_compute_lambda(real_T workspace[175],
  smNINkioqq1a7FyOE4CETSB_TaskS_T *solution, const
  slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective, const
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager)
{
  int32_T b_idx;
  int32_T idxQR;
  int32_T nActiveConstr_tmp_tmp;
  nActiveConstr_tmp_tmp = qrmanager->ncols;
  if (qrmanager->ncols > 0) {
    real_T c;
    boolean_T guard1;
    guard1 = false;
    if (objective->objtype != 4) {
      boolean_T nonDegenerate;
      if (qrmanager->mrows >= qrmanager->ncols) {
        idxQR = qrmanager->mrows;
      } else {
        idxQR = qrmanager->ncols;
      }

      c = fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * (real_T)idxQR);
      nonDegenerate = ((qrmanager->mrows > 0) && (qrmanager->ncols > 0));
      if (nonDegenerate) {
        boolean_T guard2;
        b_idx = qrmanager->ncols;
        guard2 = false;
        if (qrmanager->mrows < qrmanager->ncols) {
          idxQR = (qrmanager->ncols - 1) * 7 + qrmanager->mrows;
          while ((b_idx > qrmanager->mrows) && (fabs(qrmanager->QR[idxQR - 1]) >=
                  c)) {
            b_idx--;
            idxQR -= 7;
          }

          nonDegenerate = (b_idx == qrmanager->mrows);
          if (!nonDegenerate) {
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          idxQR = (b_idx - 1) * 7 + b_idx;
          while ((b_idx >= 1) && (fabs(qrmanager->QR[idxQR - 1]) >= c)) {
            b_idx--;
            idxQR -= 8;
          }

          nonDegenerate = (b_idx == 0);
        }
      }

      if (!nonDegenerate) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      int32_T ix;
      int32_T jjA;
      if (qrmanager->mrows != 0) {
        memset(&workspace[0], 0, (uint32_T)qrmanager->ncols * sizeof(real_T));
        jjA = (qrmanager->ncols - 1) * 7 + 1;
        for (b_idx = 1; b_idx <= jjA; b_idx += 7) {
          c = 0.0;
          ix = (b_idx + qrmanager->mrows) - 1;
          for (idxQR = b_idx; idxQR <= ix; idxQR++) {
            c += qrmanager->Q[idxQR - 1] * objective->grad[idxQR - b_idx];
          }

          idxQR = div_nde_s32_floor(b_idx - 1, 7);
          workspace[idxQR] += c;
        }
      }

      for (b_idx = nActiveConstr_tmp_tmp; b_idx >= 1; b_idx--) {
        jjA = ((b_idx - 1) * 7 + b_idx) - 2;
        workspace[b_idx - 1] /= qrmanager->QR[jjA + 1];
        for (idxQR = 0; idxQR <= b_idx - 2; idxQR++) {
          ix = (b_idx - idxQR) - 2;
          workspace[ix] -= workspace[b_idx - 1] * qrmanager->QR[jjA - idxQR];
        }
      }

      idxQR = (qrmanager->ncols / 2) << 1;
      jjA = idxQR - 2;
      for (b_idx = 0; b_idx <= jjA; b_idx += 2) {
        __m128d tmp;
        tmp = _mm_loadu_pd(&workspace[b_idx]);
        _mm_storeu_pd(&solution->lambda[b_idx], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
      }

      for (b_idx = idxQR; b_idx < nActiveConstr_tmp_tmp; b_idx++) {
        solution->lambda[b_idx] = -workspace[b_idx];
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_phaseone(const real_T f[6],
  smNINkioqq1a7FyOE4CETSB_TaskS_T *solution, sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T
  *memspace, sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager, s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T
  *cholmanager, const sIOJhD9KwAkF5sEguPjYquC_TaskS_T *runTimeOptions,
  slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T b_c;
  real_T denomTol;
  real_T normDelta;
  real_T options_ObjectiveLimit_tmp;
  real_T ratio;
  int32_T b_nVar;
  int32_T c;
  int32_T exitg1;
  int32_T idxEndIneq;
  int32_T idxMinLambda;
  int32_T idxStartIneq;
  int32_T idxStartIneq_tmp;
  int32_T idx_local;
  int32_T j;
  int32_T nVar;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T nonDegenerateWset;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  solution->xstar[6] = solution->maxConstr + 1.0;
  TaskSpaceControl_setProblemType(workingset, 1);
  idxEndIneq = workingset->nWConstr[0] + workingset->nWConstr[1];
  idxStartIneq_tmp = idxEndIneq + 1;
  idxMinLambda = workingset->nActiveConstr;
  for (nVar = idxStartIneq_tmp; nVar <= idxMinLambda; nVar++) {
    workingset->isActiveConstr[(workingset->isActiveIdx[workingset->Wid[nVar - 1]
      - 1] + workingset->Wlocalidx[nVar - 1]) - 2] = false;
  }

  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = idxEndIneq;
  for (idxEndIneq = 0; idxEndIneq < 7; idxEndIneq++) {
    objective->grad[idxEndIneq] = 0.0;
  }

  for (idxEndIneq = 0; idxEndIneq < 6; idxEndIneq++) {
    objective->Hx[idxEndIneq] = 0.0;
  }

  objective->maxVar = 7;
  objective->beta = 0.0;
  objective->rho = 0.0;
  objective->prev_objtype = 3;
  objective->prev_nvar = 6;
  objective->prev_hasLinear = true;
  objective->objtype = 5;
  objective->nvar = 7;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  options_ObjectiveLimit_tmp = 1.0E-8 * runTimeOptions->ConstrRelTolFactor;
  subProblemChanged = true;
  updateFval = true;
  idxEndIneq = 0;
  nVar = workingset->nVar;
  idxStartIneq = 0;
  TaskSpaceCo_computeGrad_StoreHx(objective, f, solution->xstar);
  solution->fstar = TaskSpaceCo_computeFval_ReuseHx(objective,
    memspace->workspace_float, f, solution->xstar);
  solution->state = -5;
  memset(&solution->lambda[0], 0, 25U * sizeof(real_T));
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard1 = false;
      if (subProblemChanged) {
        switch (idxEndIneq) {
         case 1:
          TaskSpaceCont_squareQ_appendCol(qrmanager, workingset->ATwset, 7 *
            (workingset->nActiveConstr - 1) + 1);
          break;

         case -1:
          TaskSpaceContr_deleteColMoveEnd(qrmanager, idxStartIneq);
          break;

         default:
          TaskSpaceController_factorQR(qrmanager, workingset->ATwset, nVar,
            workingset->nActiveConstr);
          TaskSpaceController_computeQ_(qrmanager, qrmanager->mrows);
          break;
        }

        TaskSpaceControl_compute_deltax(solution, memspace, qrmanager,
          cholmanager, objective);
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          normDelta = TaskSpaceController_xnrm2_n(nVar, solution->searchDir);
          guard1 = true;
        }
      } else {
        if (nVar - 1 >= 0) {
          memset(&solution->searchDir[0], 0, (uint32_T)nVar * sizeof(real_T));
        }

        normDelta = 0.0;
        guard1 = true;
      }

      if (guard1) {
        if ((!subProblemChanged) || (normDelta < 1.4901161193847657E-10) ||
            (workingset->nActiveConstr >= nVar)) {
          TaskSpaceControl_compute_lambda(memspace->workspace_float, solution,
            objective, qrmanager);
          if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
            idxMinLambda = 0;
            normDelta = 0.0 * runTimeOptions->ProbRelTolFactor * 0.0;
            b_nVar = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
            c = workingset->nActiveConstr;
            for (idxStartIneq_tmp = b_nVar; idxStartIneq_tmp <= c;
                 idxStartIneq_tmp++) {
              denomTol = solution->lambda[idxStartIneq_tmp - 1];
              if (denomTol < normDelta) {
                normDelta = denomTol;
                idxMinLambda = idxStartIneq_tmp;
              }
            }

            if (idxMinLambda == 0) {
              solution->state = 1;
            } else {
              idxEndIneq = -1;
              idxStartIneq = idxMinLambda;
              subProblemChanged = true;
              TaskSpaceControlle_removeConstr(workingset, idxMinLambda);
              if (idxMinLambda < workingset->nActiveConstr + 1) {
                solution->lambda[idxMinLambda - 1] = solution->lambda
                  [workingset->nActiveConstr];
              }

              solution->lambda[workingset->nActiveConstr] = 0.0;
            }
          } else {
            b_nVar = workingset->nActiveConstr;
            idxEndIneq = 0;
            idxStartIneq = workingset->nActiveConstr;
            subProblemChanged = true;
            TaskSpaceControlle_removeConstr(workingset,
              workingset->nActiveConstr);
            solution->lambda[b_nVar - 1] = 0.0;
          }

          updateFval = false;
        } else {
          b_nVar = workingset->nVar - 1;
          normDelta = 1.0E+30;
          updateFval = false;
          c = 0;
          idx_local = 0;
          denomTol = 2.2204460492503131E-13 * TaskSpaceController_xnrm2_n
            (workingset->nVar, solution->searchDir);
          if (workingset->nWConstr[2] < 24) {
            memcpy(&memspace->workspace_float[0], &workingset->bineq[0], 24U *
                   sizeof(real_T));
            TaskSpaceController_xgemv_l1nmk(workingset->nVar, workingset->Aineq,
              solution->xstar, memspace->workspace_float);
            memset(&memspace->workspace_float[25], 0, 24U * sizeof(real_T));
            for (idxStartIneq_tmp = 0; idxStartIneq_tmp <= 161; idxStartIneq_tmp
                 += 7) {
              b_c = 0.0;
              j = (idxStartIneq_tmp + b_nVar) + 1;
              for (idxMinLambda = idxStartIneq_tmp + 1; idxMinLambda <= j;
                   idxMinLambda++) {
                b_c += solution->searchDir[(idxMinLambda - idxStartIneq_tmp) - 1]
                  * workingset->Aineq[idxMinLambda - 1];
              }

              idxMinLambda = div_nde_s32_floor(idxStartIneq_tmp, 7) + 25;
              memspace->workspace_float[idxMinLambda] += b_c;
            }

            for (idxStartIneq_tmp = 0; idxStartIneq_tmp < 24; idxStartIneq_tmp++)
            {
              b_c = memspace->workspace_float[idxStartIneq_tmp + 25];
              if ((b_c > denomTol) && (!workingset->isActiveConstr
                   [(workingset->isActiveIdx[2] + idxStartIneq_tmp) - 1])) {
                ratio = memspace->workspace_float[idxStartIneq_tmp];
                b_c = fmin(fabs(ratio), 1.0E-8 - ratio) / b_c;
                if (b_c < normDelta) {
                  normDelta = b_c;
                  c = 3;
                  idx_local = idxStartIneq_tmp + 1;
                  updateFval = true;
                }
              }
            }
          }

          if (workingset->nWConstr[3] < workingset->sizes[3]) {
            idxMinLambda = workingset->sizes[3];
            for (idxStartIneq_tmp = 0; idxStartIneq_tmp <= idxMinLambda - 2;
                 idxStartIneq_tmp++) {
              b_c = -solution->searchDir[workingset->indexLB[idxStartIneq_tmp] -
                1] - solution->searchDir[b_nVar];
              if ((b_c > denomTol) && (!workingset->isActiveConstr
                   [(workingset->isActiveIdx[3] + idxStartIneq_tmp) - 1])) {
                ratio = -solution->xstar[workingset->indexLB[idxStartIneq_tmp] -
                  1] - solution->xstar[b_nVar];
                b_c = fmin(fabs(ratio), 1.0E-8 - ratio) / b_c;
                if (b_c < normDelta) {
                  normDelta = b_c;
                  c = 4;
                  idx_local = idxStartIneq_tmp + 1;
                  updateFval = true;
                }
              }
            }

            b_nVar = workingset->indexLB[workingset->sizes[3] - 1] - 1;
            b_c = -solution->searchDir[b_nVar];
            if ((b_c > denomTol) && (!workingset->isActiveConstr
                 [(workingset->isActiveIdx[3] + workingset->sizes[3]) - 2])) {
              denomTol = -solution->xstar[b_nVar];
              b_c = fmin(fabs(denomTol), 1.0E-8 - denomTol) / b_c;
              if (b_c < normDelta) {
                normDelta = b_c;
                c = 4;
                idx_local = workingset->sizes[3];
                updateFval = true;
              }
            }
          }

          if (updateFval) {
            switch (c) {
             case 3:
              TaskSpaceControl_addAineqConstr(workingset, idx_local);
              break;

             case 4:
              Task_addBoundToActiveSetMatrix_(workingset, 4, idx_local);
              break;

             default:
              Task_addBoundToActiveSetMatrix_(workingset, 5, idx_local);
              break;
            }

            idxEndIneq = 1;
          } else {
            if (objective->objtype == 5) {
              if (TaskSpaceController_xnrm2_n(objective->nvar,
                   solution->searchDir) > 100.0 * (real_T)objective->nvar *
                  1.4901161193847656E-8) {
                solution->state = 3;
              } else {
                solution->state = 4;
              }
            }

            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if (!(normDelta == 0.0)) {
            idxMinLambda = (nVar / 2) << 1;
            b_nVar = idxMinLambda - 2;
            for (idxStartIneq_tmp = 0; idxStartIneq_tmp <= b_nVar;
                 idxStartIneq_tmp += 2) {
              tmp = _mm_loadu_pd(&solution->searchDir[idxStartIneq_tmp]);
              tmp_0 = _mm_loadu_pd(&solution->xstar[idxStartIneq_tmp]);
              _mm_storeu_pd(&solution->xstar[idxStartIneq_tmp], _mm_add_pd
                            (_mm_mul_pd(_mm_set1_pd(normDelta), tmp), tmp_0));
            }

            for (idxStartIneq_tmp = idxMinLambda; idxStartIneq_tmp < nVar;
                 idxStartIneq_tmp++) {
              solution->xstar[idxStartIneq_tmp] += normDelta *
                solution->searchDir[idxStartIneq_tmp];
            }
          }

          TaskSpaceCo_computeGrad_StoreHx(objective, f, solution->xstar);
          updateFval = true;
        }

        solution->iterations++;
        if (solution->iterations >= 300) {
          solution->state = 0;
        }

        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr = TaskSpac_maxConstraintViolation(workingset,
            solution->xstar);
          if (solution->maxConstr - solution->xstar[6] >
              options_ObjectiveLimit_tmp) {
            for (idxEndIneq = 0; idxEndIneq < 7; idxEndIneq++) {
              solution->searchDir[idxEndIneq] = solution->xstar[idxEndIneq];
            }

            nonDegenerateWset = TaskSpa_feasibleX0ForWorkingSet
              (memspace->workspace_float, solution->searchDir, workingset,
               qrmanager);
            if ((!nonDegenerateWset) && (solution->state != 0)) {
              solution->state = -2;
            }

            idxEndIneq = 0;
            normDelta = TaskSpac_maxConstraintViolation(workingset,
              solution->searchDir);
            if (normDelta < solution->maxConstr) {
              for (idxStartIneq_tmp = 0; idxStartIneq_tmp < 7; idxStartIneq_tmp
                   ++) {
                solution->xstar[idxStartIneq_tmp] = solution->
                  searchDir[idxStartIneq_tmp];
              }
            }
          }
        }

        if (updateFval) {
          solution->fstar = TaskSpaceCo_computeFval_ReuseHx(objective,
            memspace->workspace_float, f, solution->xstar);
          if ((solution->fstar < options_ObjectiveLimit_tmp) && (solution->state
               != 0)) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = TaskSpaceCo_computeFval_ReuseHx(objective,
          memspace->workspace_float, f, solution->xstar);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (workingset->isActiveConstr[(workingset->isActiveIdx[3] + workingset->
       sizes[3]) - 2]) {
    nVar = 1;
    exitg2 = false;
    while ((!exitg2) && (nVar <= workingset->nActiveConstr)) {
      if ((workingset->Wid[nVar - 1] == 4) && (workingset->Wlocalidx[nVar - 1] ==
           workingset->sizes[3])) {
        TaskSpaceControlle_removeConstr(workingset, nVar);
        exitg2 = true;
      } else {
        nVar++;
      }
    }
  }

  for (nVar = workingset->nActiveConstr; nVar > 6; nVar--) {
    TaskSpaceControlle_removeConstr(workingset, nVar);
  }

  solution->maxConstr = solution->xstar[6];
  TaskSpaceControl_setProblemType(workingset, 3);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static int32_T TaskSpaceCon_RemoveDependentEq_(sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T
  *memspace, const sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager)
{
  real_T qtb;
  real_T tol;
  int32_T b;
  int32_T b_tmp;
  int32_T idxDiag;
  int32_T idx_row;
  int32_T ix;
  int32_T mTotalWorkingEq_tmp;
  int32_T mWorkingFixed;
  int32_T nDepInd;
  int32_T tmp;
  boolean_T exitg1;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp > 0) {
    b_tmp = (uint8_T)workingset->nVar;
    for (idx_row = 0; idx_row < mTotalWorkingEq_tmp; idx_row++) {
      for (idxDiag = 0; idxDiag < b_tmp; idxDiag++) {
        qrmanager->QR[idx_row + 7 * idxDiag] = workingset->ATwset[7 * idx_row +
          idxDiag];
      }
    }

    idx_row = mTotalWorkingEq_tmp - workingset->nVar;
    if (idx_row > 0) {
      nDepInd = idx_row;
    }

    memset(&qrmanager->jpvt[0], 0, (uint8_T)workingset->nVar * sizeof(int32_T));
    tmp = mTotalWorkingEq_tmp * workingset->nVar;
    if (tmp == 0) {
      qrmanager->mrows = mTotalWorkingEq_tmp;
      qrmanager->ncols = workingset->nVar;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = mTotalWorkingEq_tmp;
      qrmanager->ncols = workingset->nVar;
      if (mTotalWorkingEq_tmp <= workingset->nVar) {
        qrmanager->minRowCol = mTotalWorkingEq_tmp;
      } else {
        qrmanager->minRowCol = workingset->nVar;
      }

      TaskSpaceController_xgeqp3(qrmanager->QR, mTotalWorkingEq_tmp,
        workingset->nVar, qrmanager->jpvt, qrmanager->tau);
    }

    if (mTotalWorkingEq_tmp >= workingset->nVar) {
      idx_row = mTotalWorkingEq_tmp;
    } else {
      idx_row = workingset->nVar;
    }

    tol = fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * (real_T)idx_row);
    if (workingset->nVar <= mTotalWorkingEq_tmp) {
      idx_row = workingset->nVar;
    } else {
      idx_row = mTotalWorkingEq_tmp;
    }

    idxDiag = (idx_row - 1) * 7 + idx_row;
    while ((idxDiag > 0) && (fabs(qrmanager->QR[idxDiag - 1]) < tol * fabs
            (qrmanager->QR[0]))) {
      idxDiag -= 8;
      nDepInd++;
    }

    if (nDepInd > 0) {
      TaskSpaceController_computeQ_(qrmanager, qrmanager->mrows);
      b = 0;
      exitg1 = false;
      while ((!exitg1) && (b <= nDepInd - 1)) {
        ix = ((mTotalWorkingEq_tmp - b) - 1) * 7;
        qtb = 0.0;
        for (idxDiag = 0; idxDiag < mTotalWorkingEq_tmp; idxDiag++) {
          qtb += qrmanager->Q[ix + idxDiag] * workingset->bwset[idxDiag];
        }

        if (fabs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          b++;
        }
      }
    }

    if (nDepInd > 0) {
      for (idxDiag = 0; idxDiag < mTotalWorkingEq_tmp; idxDiag++) {
        ix = 7 * idxDiag;
        memcpy(&qrmanager->QR[ix], &workingset->ATwset[ix], (uint32_T)b_tmp *
               sizeof(real_T));
      }

      for (idxDiag = 0; idxDiag < mWorkingFixed; idxDiag++) {
        qrmanager->jpvt[idxDiag] = 1;
      }

      idxDiag = workingset->nWConstr[0] + 1;
      if (idxDiag <= mTotalWorkingEq_tmp) {
        memset(&qrmanager->jpvt[idxDiag + -1], 0, (uint32_T)
               ((mTotalWorkingEq_tmp - idxDiag) + 1) * sizeof(int32_T));
      }

      if (tmp == 0) {
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = true;
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp;
        qrmanager->minRowCol = idx_row;
        TaskSpaceController_xgeqp3(qrmanager->QR, workingset->nVar,
          mTotalWorkingEq_tmp, qrmanager->jpvt, qrmanager->tau);
      }

      for (mWorkingFixed = 0; mWorkingFixed < nDepInd; mWorkingFixed++) {
        memspace->workspace_int[mWorkingFixed] = qrmanager->jpvt
          [(mTotalWorkingEq_tmp - nDepInd) + mWorkingFixed];
      }

      TaskSpaceController_countsort(memspace->workspace_int, nDepInd,
        memspace->workspace_sort, 1, mTotalWorkingEq_tmp);
      if (mTotalWorkingEq_tmp != 0) {
        for (mWorkingFixed = nDepInd; mWorkingFixed >= 1; mWorkingFixed--) {
          idx_row = memspace->workspace_int[mWorkingFixed - 1];
          if ((idx_row <= mTotalWorkingEq_tmp) && (!((workingset->nActiveConstr ==
                 mTotalWorkingEq_tmp) || (idx_row == mTotalWorkingEq_tmp)))) {
            /* Check node always fails. would cause program termination and was eliminated */
          } else {
            /* Check node always fails. would cause program termination and was eliminated */
          }
        }
      }
    }
  }

  return nDepInd;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_ratiotest(const real_T solution_xstar[7], const
  real_T solution_searchDir[7], real_T workspace[175], int32_T workingset_nVar,
  const real_T workingset_Aineq[168], const real_T workingset_bineq[24], const
  int32_T workingset_indexLB[7], const int32_T workingset_sizes[5], const
  int32_T workingset_isActiveIdx[6], const boolean_T workingset_isActiveConstr
  [25], const int32_T workingset_nWConstr[5], real_T *toldelta, real_T *alpha,
  boolean_T *newBlocking, int32_T *constrType, int32_T *constrIdx)
{
  real_T tmp[2];
  real_T alphaTemp;
  real_T c;
  real_T denomTol;
  real_T p_max;
  real_T phaseOneCorrectionP;
  real_T pk_corrected;
  real_T workspace_0;
  int32_T d;
  int32_T ia;
  int32_T k;
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  p_max = 0.0;
  denomTol = 2.2204460492503131E-13 * TaskSpaceController_xnrm2_n
    (workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[2] < 24) {
    memcpy(&workspace[0], &workingset_bineq[0], 24U * sizeof(real_T));
    TaskSpaceController_xgemv_l1nmk(workingset_nVar, workingset_Aineq,
      solution_xstar, workspace);
    memset(&workspace[25], 0, 24U * sizeof(real_T));
    for (k = 0; k <= 161; k += 7) {
      c = 0.0;
      d = k + workingset_nVar;
      for (ia = k + 1; ia <= d; ia++) {
        c += solution_searchDir[(ia - k) - 1] * workingset_Aineq[ia - 1];
      }

      ia = div_nde_s32_floor(k, 7) + 25;
      workspace[ia] += c;
    }

    for (k = 0; k < 24; k++) {
      workspace_0 = workspace[k + 25];
      if ((workspace_0 > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[2] + k) - 1])) {
        c = workspace[k];
        alphaTemp = fmin(fabs(c - *toldelta), (1.0E-8 - c) + *toldelta) /
          workspace_0;
        if ((alphaTemp <= *alpha) && (fabs(workspace_0) > p_max)) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = k + 1;
          *newBlocking = true;
        }

        alphaTemp = fmin(fabs(c), 1.0E-8 - c) / workspace_0;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = k + 1;
          *newBlocking = true;
          p_max = fabs(workspace_0);
        }
      }
    }
  }

  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_set_pd
      (solution_searchDir[workingset_nVar - 1], solution_xstar[workingset_nVar -
       1]), _mm_set1_pd(0.0)));
    c = tmp[0];
    phaseOneCorrectionP = tmp[1];
    ia = workingset_sizes[3];
    for (k = 0; k <= ia - 2; k++) {
      d = workingset_indexLB[k];
      pk_corrected = -solution_searchDir[d - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[3] + k) - 1])) {
        workspace_0 = -solution_xstar[d - 1];
        alphaTemp = (workspace_0 - *toldelta) - c;
        alphaTemp = fmin(fabs(alphaTemp), 1.0E-8 - alphaTemp) / pk_corrected;
        if ((alphaTemp <= *alpha) && (fabs(pk_corrected) > p_max)) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = k + 1;
          *newBlocking = true;
        }

        alphaTemp = workspace_0 - c;
        alphaTemp = fmin(fabs(alphaTemp), 1.0E-8 - alphaTemp) / pk_corrected;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = k + 1;
          *newBlocking = true;
          p_max = fabs(pk_corrected);
        }
      }
    }

    ia = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    c = solution_searchDir[ia];
    if ((-c > denomTol) && (!workingset_isActiveConstr[(workingset_isActiveIdx[3]
          + workingset_sizes[3]) - 2])) {
      workspace_0 = -solution_xstar[ia];
      alphaTemp = workspace_0 - *toldelta;
      alphaTemp = fmin(fabs(alphaTemp), 1.0E-8 - alphaTemp) / -c;
      if ((alphaTemp <= *alpha) && (fabs(c) > p_max)) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }

      alphaTemp = fmin(fabs(workspace_0), 1.0E-8 - workspace_0) / -c;
      if (alphaTemp < *alpha) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
        p_max = fabs(c);
      }
    }
  }

  *toldelta += 6.608625846508183E-7;
  if (p_max > 0.0) {
    *alpha = fmax(*alpha, 6.608625846508183E-7 / p_max);
  }

  *newBlocking = (((!*newBlocking) || (!(*alpha > 1.0))) && (*newBlocking));
  *alpha = fmin(*alpha, 1.0);
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceCont_feasibleratiotest(const real_T solution_xstar[7],
  const real_T solution_searchDir[7], real_T workspace[175], int32_T
  workingset_nVar, const real_T workingset_Aineq[168], const real_T
  workingset_bineq[24], const int32_T workingset_indexLB[7], const int32_T
  workingset_sizes[5], const int32_T workingset_isActiveIdx[6], const boolean_T
  workingset_isActiveConstr[25], const int32_T workingset_nWConstr[5], boolean_T
  isPhaseOne, real_T *alpha, boolean_T *newBlocking, int32_T *constrType,
  int32_T *constrIdx)
{
  real_T tmp[2];
  real_T alphaTemp;
  real_T c;
  real_T denomTol;
  real_T phaseOneCorrectionP;
  real_T ratio;
  int32_T d;
  int32_T ia;
  int32_T k;
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 * TaskSpaceController_xnrm2_n
    (workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[2] < 24) {
    memcpy(&workspace[0], &workingset_bineq[0], 24U * sizeof(real_T));
    TaskSpaceController_xgemv_l1nmk(workingset_nVar, workingset_Aineq,
      solution_xstar, workspace);
    memset(&workspace[25], 0, 24U * sizeof(real_T));
    for (k = 0; k <= 161; k += 7) {
      c = 0.0;
      d = k + workingset_nVar;
      for (ia = k + 1; ia <= d; ia++) {
        c += solution_searchDir[(ia - k) - 1] * workingset_Aineq[ia - 1];
      }

      ia = div_nde_s32_floor(k, 7) + 25;
      workspace[ia] += c;
    }

    for (k = 0; k < 24; k++) {
      c = workspace[k + 25];
      if ((c > denomTol) && (!workingset_isActiveConstr[(workingset_isActiveIdx
            [2] + k) - 1])) {
        alphaTemp = workspace[k];
        alphaTemp = fmin(fabs(alphaTemp), 1.0E-8 - alphaTemp) / c;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = k + 1;
          *newBlocking = true;
        }
      }
    }
  }

  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_set_pd
      (solution_searchDir[workingset_nVar - 1], solution_xstar[workingset_nVar -
       1]), _mm_set1_pd(isPhaseOne)));
    c = tmp[0];
    phaseOneCorrectionP = tmp[1];
    ia = workingset_sizes[3];
    for (k = 0; k <= ia - 2; k++) {
      d = workingset_indexLB[k];
      alphaTemp = -solution_searchDir[d - 1] - phaseOneCorrectionP;
      if ((alphaTemp > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[3] + k) - 1])) {
        ratio = -solution_xstar[d - 1] - c;
        alphaTemp = fmin(fabs(ratio), 1.0E-8 - ratio) / alphaTemp;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = k + 1;
          *newBlocking = true;
        }
      }
    }

    ia = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    c = -solution_searchDir[ia];
    if ((c > denomTol) && (!workingset_isActiveConstr[(workingset_isActiveIdx[3]
          + workingset_sizes[3]) - 2])) {
      denomTol = -solution_xstar[ia];
      alphaTemp = fmin(fabs(denomTol), 1.0E-8 - denomTol) / c;
      if (alphaTemp < *alpha) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }

  if (!isPhaseOne) {
    *newBlocking = (((!*newBlocking) || (!(*alpha > 1.0))) && (*newBlocking));
    *alpha = fmin(*alpha, 1.0);
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_iterate(const real_T f[6],
  smNINkioqq1a7FyOE4CETSB_TaskS_T *solution, sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T
  *memspace, sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager, s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T
  *cholmanager, slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective, boolean_T
  runTimeOptions_RemainFeasible, real_T runTimeOptions_ConstrRelTolFact, real_T
  runTimeOptions_ProbRelTolFactor)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T normDelta;
  real_T solution_lambda;
  real_T tolDelta;
  int32_T TYPE;
  int32_T activeConstrChangedType;
  int32_T activeSetChangeID;
  int32_T c;
  int32_T exitg1;
  int32_T globalActiveConstrIdx;
  int32_T localActiveConstrIdx;
  int32_T nVar;
  int32_T vectorUB;
  boolean_T guard1;
  boolean_T newBlocking;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  tolDelta = 6.7434957617430445E-7;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  TaskSpaceCo_computeGrad_StoreHx(objective, f, solution->xstar);
  solution->fstar = TaskSpaceCo_computeFval_ReuseHx(objective,
    memspace->workspace_float, f, solution->xstar);
  if (solution->iterations < 300) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }

  memset(&solution->lambda[0], 0, 25U * sizeof(real_T));
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          TaskSpaceCont_squareQ_appendCol(qrmanager, workingset->ATwset, 7 *
            (workingset->nActiveConstr - 1) + 1);
          break;

         case -1:
          TaskSpaceContr_deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          TaskSpaceController_factorQR(qrmanager, workingset->ATwset, nVar,
            workingset->nActiveConstr);
          TaskSpaceController_computeQ_(qrmanager, qrmanager->mrows);
          break;
        }

        TaskSpaceControl_compute_deltax(solution, memspace, qrmanager,
          cholmanager, objective);
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          normDelta = TaskSpaceController_xnrm2_n(nVar, solution->searchDir);
          guard1 = true;
        }
      } else {
        if (nVar - 1 >= 0) {
          memset(&solution->searchDir[0], 0, (uint32_T)nVar * sizeof(real_T));
        }

        normDelta = 0.0;
        guard1 = true;
      }

      if (guard1) {
        if ((!subProblemChanged) || (normDelta < 1.0E-8) ||
            (workingset->nActiveConstr >= nVar)) {
          TaskSpaceControl_compute_lambda(memspace->workspace_float, solution,
            objective, qrmanager);
          if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
            localActiveConstrIdx = 0;
            normDelta = 0.0 * runTimeOptions_ProbRelTolFactor * (real_T)(TYPE !=
              5);
            vectorUB = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
            c = workingset->nActiveConstr;
            for (activeConstrChangedType = vectorUB; activeConstrChangedType <=
                 c; activeConstrChangedType++) {
              solution_lambda = solution->lambda[activeConstrChangedType - 1];
              if (solution_lambda < normDelta) {
                normDelta = solution_lambda;
                localActiveConstrIdx = activeConstrChangedType;
              }
            }

            if (localActiveConstrIdx == 0) {
              solution->state = 1;
            } else {
              activeSetChangeID = -1;
              globalActiveConstrIdx = localActiveConstrIdx;
              subProblemChanged = true;
              TaskSpaceControlle_removeConstr(workingset, localActiveConstrIdx);
              if (localActiveConstrIdx < workingset->nActiveConstr + 1) {
                solution->lambda[localActiveConstrIdx - 1] = solution->
                  lambda[workingset->nActiveConstr];
              }

              solution->lambda[workingset->nActiveConstr] = 0.0;
            }
          } else {
            localActiveConstrIdx = workingset->nActiveConstr;
            activeSetChangeID = 0;
            globalActiveConstrIdx = workingset->nActiveConstr;
            subProblemChanged = true;
            TaskSpaceControlle_removeConstr(workingset,
              workingset->nActiveConstr);
            solution->lambda[localActiveConstrIdx - 1] = 0.0;
          }

          updateFval = false;
        } else {
          updateFval = (TYPE == 5);
          if (updateFval || runTimeOptions_RemainFeasible) {
            TaskSpaceCont_feasibleratiotest(solution->xstar, solution->searchDir,
              memspace->workspace_float, workingset->nVar, workingset->Aineq,
              workingset->bineq, workingset->indexLB, workingset->sizes,
              workingset->isActiveIdx, workingset->isActiveConstr,
              workingset->nWConstr, updateFval, &normDelta, &newBlocking,
              &activeConstrChangedType, &localActiveConstrIdx);
          } else {
            TaskSpaceController_ratiotest(solution->xstar, solution->searchDir,
              memspace->workspace_float, workingset->nVar, workingset->Aineq,
              workingset->bineq, workingset->indexLB, workingset->sizes,
              workingset->isActiveIdx, workingset->isActiveConstr,
              workingset->nWConstr, &tolDelta, &normDelta, &newBlocking,
              &activeConstrChangedType, &localActiveConstrIdx);
          }

          if (newBlocking) {
            switch (activeConstrChangedType) {
             case 3:
              TaskSpaceControl_addAineqConstr(workingset, localActiveConstrIdx);
              break;

             case 4:
              Task_addBoundToActiveSetMatrix_(workingset, 4,
                localActiveConstrIdx);
              break;

             default:
              Task_addBoundToActiveSetMatrix_(workingset, 5,
                localActiveConstrIdx);
              break;
            }

            activeSetChangeID = 1;
          } else {
            if (objective->objtype == 5) {
              if (TaskSpaceController_xnrm2_n(objective->nvar,
                   solution->searchDir) > 100.0 * (real_T)objective->nvar *
                  1.4901161193847656E-8) {
                solution->state = 3;
              } else {
                solution->state = 4;
              }
            }

            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if (!(normDelta == 0.0)) {
            localActiveConstrIdx = (nVar / 2) << 1;
            vectorUB = localActiveConstrIdx - 2;
            for (activeConstrChangedType = 0; activeConstrChangedType <=
                 vectorUB; activeConstrChangedType += 2) {
              tmp = _mm_loadu_pd(&solution->searchDir[activeConstrChangedType]);
              tmp_0 = _mm_loadu_pd(&solution->xstar[activeConstrChangedType]);
              _mm_storeu_pd(&solution->xstar[activeConstrChangedType],
                            _mm_add_pd(_mm_mul_pd(_mm_set1_pd(normDelta), tmp),
                tmp_0));
            }

            for (activeConstrChangedType = localActiveConstrIdx;
                 activeConstrChangedType < nVar; activeConstrChangedType++) {
              solution->xstar[activeConstrChangedType] += normDelta *
                solution->searchDir[activeConstrChangedType];
            }
          }

          TaskSpaceCo_computeGrad_StoreHx(objective, f, solution->xstar);
          updateFval = true;
        }

        solution->iterations++;
        activeConstrChangedType = objective->nvar;
        if ((solution->iterations >= 300) && ((solution->state != 1) ||
             (objective->objtype == 5))) {
          solution->state = 0;
        }

        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr = TaskSpac_maxConstraintViolation(workingset,
            solution->xstar);
          normDelta = solution->maxConstr;
          if (objective->objtype == 5) {
            normDelta = solution->maxConstr - solution->xstar[objective->nvar -
              1];
          }

          if (normDelta > 1.0E-8 * runTimeOptions_ConstrRelTolFact) {
            if (objective->nvar - 1 >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0], (uint32_T)
                     objective->nvar * sizeof(real_T));
            }

            newBlocking = TaskSpa_feasibleX0ForWorkingSet
              (memspace->workspace_float, solution->searchDir, workingset,
               qrmanager);
            if ((!newBlocking) && (solution->state != 0)) {
              solution->state = -2;
            }

            activeSetChangeID = 0;
            normDelta = TaskSpac_maxConstraintViolation(workingset,
              solution->searchDir);
            if (normDelta < solution->maxConstr) {
              localActiveConstrIdx = (uint8_T)objective->nvar;
              if ((uint8_T)objective->nvar - 1 >= 0) {
                memcpy(&solution->xstar[0], &solution->searchDir[0], (uint8_T)
                       objective->nvar * sizeof(real_T));
              }

              solution->maxConstr = normDelta;
            }
          }
        }

        if (updateFval) {
          solution->fstar = TaskSpaceCo_computeFval_ReuseHx(objective,
            memspace->workspace_float, f, solution->xstar);
          if ((solution->fstar < -1.0E+20) && ((solution->state != 0) ||
               (objective->objtype != 5))) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = TaskSpaceCo_computeFval_ReuseHx(objective,
          memspace->workspace_float, f, solution->xstar);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceC_computeFirstOrderOpt(smNINkioqq1a7FyOE4CETSB_TaskS_T
  *solution, const slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective, int32_T
  workingset_nVar, const real_T workingset_ATwset[175], int32_T
  workingset_nActiveConstr, real_T workspace[175])
{
  real_T infNorm;
  int32_T b;
  int32_T k;
  boolean_T exitg1;
  memcpy(&workspace[0], &objective->grad[0], (uint8_T)workingset_nVar * sizeof
         (real_T));
  if (workingset_nActiveConstr != 0) {
    int32_T c;
    int32_T ix;
    ix = 0;
    c = (workingset_nActiveConstr - 1) * 7 + 1;
    for (k = 1; k <= c; k += 7) {
      int32_T d;
      d = (k + workingset_nVar) - 1;
      for (b = k; b <= d; b++) {
        int32_T tmp;
        tmp = b - k;
        workspace[tmp] += workingset_ATwset[b - 1] * solution->lambda[ix];
      }

      ix++;
    }
  }

  infNorm = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= (uint8_T)workingset_nVar - 1)) {
    real_T abs_workspace_i;
    abs_workspace_i = fabs(workspace[k]);
    if (rtIsNaN(abs_workspace_i)) {
      infNorm = (rtNaN);
      exitg1 = true;
    } else {
      infNorm = fmax(infNorm, abs_workspace_i);
      k++;
    }
  }

  solution->firstorderopt = infNorm;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_phaseone_a(const real_T f[6],
  smNINkioqq1a7FyOE4CETSB_TaskS_T *solution, sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T
  *memspace, sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager, s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T
  *cholmanager, slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective, const
  sIOJhD9KwAkF5sEguPjYquC_TaskS_T *runTimeOptions)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T b_options_ObjectiveLimit_tmp;
  real_T normDelta;
  real_T solution_lambda;
  int32_T PROBTYPE_ORIG;
  int32_T b_nVar;
  int32_T e;
  int32_T exitg1;
  int32_T idxEndIneq;
  int32_T idxMinLambda;
  int32_T idxStartIneq;
  int32_T idxStartIneq_tmp;
  int32_T nVar_tmp;
  int32_T vectorUB;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T nonDegenerateWset;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  PROBTYPE_ORIG = workingset->probType;
  nVar_tmp = workingset->nVar;
  solution->xstar[6] = solution->maxConstr + 1.0;
  if (workingset->probType == 3) {
    idxEndIneq = 1;
  } else {
    idxEndIneq = 4;
  }

  TaskSpaceControl_setProblemType(workingset, idxEndIneq);
  idxEndIneq = workingset->nWConstr[0] + workingset->nWConstr[1];
  idxStartIneq_tmp = idxEndIneq + 1;
  idxStartIneq = workingset->nActiveConstr;
  for (b_nVar = idxStartIneq_tmp; b_nVar <= idxStartIneq; b_nVar++) {
    workingset->isActiveConstr[(workingset->isActiveIdx[workingset->Wid[b_nVar -
      1] - 1] + workingset->Wlocalidx[b_nVar - 1]) - 2] = false;
  }

  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = idxEndIneq;
  objective->prev_objtype = objective->objtype;
  objective->prev_nvar = objective->nvar;
  objective->prev_hasLinear = objective->hasLinear;
  objective->objtype = 5;
  objective->nvar = 7;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  b_options_ObjectiveLimit_tmp = 1.0E-8 * runTimeOptions->ConstrRelTolFactor;
  subProblemChanged = true;
  updateFval = true;
  idxEndIneq = 0;
  idxStartIneq_tmp = workingset->nVar;
  idxStartIneq = 0;
  TaskSpaceCo_computeGrad_StoreHx(objective, f, solution->xstar);
  solution->fstar = TaskSpaceCo_computeFval_ReuseHx(objective,
    memspace->workspace_float, f, solution->xstar);
  solution->state = -5;
  memset(&solution->lambda[0], 0, 25U * sizeof(real_T));
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard1 = false;
      if (subProblemChanged) {
        switch (idxEndIneq) {
         case 1:
          TaskSpaceCont_squareQ_appendCol(qrmanager, workingset->ATwset, 7 *
            (workingset->nActiveConstr - 1) + 1);
          break;

         case -1:
          TaskSpaceContr_deleteColMoveEnd(qrmanager, idxStartIneq);
          break;

         default:
          TaskSpaceController_factorQR(qrmanager, workingset->ATwset,
            idxStartIneq_tmp, workingset->nActiveConstr);
          TaskSpaceController_computeQ_(qrmanager, qrmanager->mrows);
          break;
        }

        TaskSpaceControl_compute_deltax(solution, memspace, qrmanager,
          cholmanager, objective);
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          normDelta = TaskSpaceController_xnrm2_n(idxStartIneq_tmp,
            solution->searchDir);
          guard1 = true;
        }
      } else {
        if (idxStartIneq_tmp - 1 >= 0) {
          memset(&solution->searchDir[0], 0, (uint32_T)idxStartIneq_tmp * sizeof
                 (real_T));
        }

        normDelta = 0.0;
        guard1 = true;
      }

      if (guard1) {
        if ((!subProblemChanged) || (normDelta < 1.4901161193847657E-10) ||
            (workingset->nActiveConstr >= idxStartIneq_tmp)) {
          TaskSpaceControl_compute_lambda(memspace->workspace_float, solution,
            objective, qrmanager);
          if ((solution->state != -7) || (workingset->nActiveConstr >
               idxStartIneq_tmp)) {
            idxMinLambda = 0;
            normDelta = 0.0 * runTimeOptions->ProbRelTolFactor * 0.0;
            vectorUB = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
            e = workingset->nActiveConstr;
            for (b_nVar = vectorUB; b_nVar <= e; b_nVar++) {
              solution_lambda = solution->lambda[b_nVar - 1];
              if (solution_lambda < normDelta) {
                normDelta = solution_lambda;
                idxMinLambda = b_nVar;
              }
            }

            if (idxMinLambda == 0) {
              solution->state = 1;
            } else {
              idxEndIneq = -1;
              idxStartIneq = idxMinLambda;
              subProblemChanged = true;
              TaskSpaceControlle_removeConstr(workingset, idxMinLambda);
              if (idxMinLambda < workingset->nActiveConstr + 1) {
                solution->lambda[idxMinLambda - 1] = solution->lambda
                  [workingset->nActiveConstr];
              }

              solution->lambda[workingset->nActiveConstr] = 0.0;
            }
          } else {
            b_nVar = workingset->nActiveConstr;
            idxEndIneq = 0;
            idxStartIneq = workingset->nActiveConstr;
            subProblemChanged = true;
            TaskSpaceControlle_removeConstr(workingset,
              workingset->nActiveConstr);
            solution->lambda[b_nVar - 1] = 0.0;
          }

          updateFval = false;
        } else {
          TaskSpaceCont_feasibleratiotest(solution->xstar, solution->searchDir,
            memspace->workspace_float, workingset->nVar, workingset->Aineq,
            workingset->bineq, workingset->indexLB, workingset->sizes,
            workingset->isActiveIdx, workingset->isActiveConstr,
            workingset->nWConstr, true, &normDelta, &updateFval, &b_nVar,
            &idxMinLambda);
          if (updateFval) {
            switch (b_nVar) {
             case 3:
              TaskSpaceControl_addAineqConstr(workingset, idxMinLambda);
              break;

             case 4:
              Task_addBoundToActiveSetMatrix_(workingset, 4, idxMinLambda);
              break;

             default:
              Task_addBoundToActiveSetMatrix_(workingset, 5, idxMinLambda);
              break;
            }

            idxEndIneq = 1;
          } else {
            if (objective->objtype == 5) {
              if (TaskSpaceController_xnrm2_n(objective->nvar,
                   solution->searchDir) > 100.0 * (real_T)objective->nvar *
                  1.4901161193847656E-8) {
                solution->state = 3;
              } else {
                solution->state = 4;
              }
            }

            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if (!(normDelta == 0.0)) {
            idxMinLambda = (idxStartIneq_tmp / 2) << 1;
            vectorUB = idxMinLambda - 2;
            for (b_nVar = 0; b_nVar <= vectorUB; b_nVar += 2) {
              tmp = _mm_loadu_pd(&solution->searchDir[b_nVar]);
              tmp_0 = _mm_loadu_pd(&solution->xstar[b_nVar]);
              _mm_storeu_pd(&solution->xstar[b_nVar], _mm_add_pd(_mm_mul_pd
                (_mm_set1_pd(normDelta), tmp), tmp_0));
            }

            for (b_nVar = idxMinLambda; b_nVar < idxStartIneq_tmp; b_nVar++) {
              solution->xstar[b_nVar] += normDelta * solution->searchDir[b_nVar];
            }
          }

          TaskSpaceCo_computeGrad_StoreHx(objective, f, solution->xstar);
          updateFval = true;
        }

        solution->iterations++;
        idxMinLambda = objective->nvar;
        if ((solution->iterations >= 300) && ((solution->state != 1) ||
             (objective->objtype == 5))) {
          solution->state = 0;
        }

        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr = TaskSpac_maxConstraintViolation(workingset,
            solution->xstar);
          normDelta = solution->maxConstr;
          if (objective->objtype == 5) {
            normDelta = solution->maxConstr - solution->xstar[objective->nvar -
              1];
          }

          if (normDelta > b_options_ObjectiveLimit_tmp) {
            if (objective->nvar - 1 >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0], (uint32_T)
                     objective->nvar * sizeof(real_T));
            }

            nonDegenerateWset = TaskSpa_feasibleX0ForWorkingSet
              (memspace->workspace_float, solution->searchDir, workingset,
               qrmanager);
            if ((!nonDegenerateWset) && (solution->state != 0)) {
              solution->state = -2;
            }

            idxEndIneq = 0;
            normDelta = TaskSpac_maxConstraintViolation(workingset,
              solution->searchDir);
            if ((normDelta < solution->maxConstr) && (objective->nvar - 1 >= 0))
            {
              memcpy(&solution->xstar[0], &solution->searchDir[0], (uint32_T)
                     objective->nvar * sizeof(real_T));
            }
          }
        }

        if (updateFval) {
          solution->fstar = TaskSpaceCo_computeFval_ReuseHx(objective,
            memspace->workspace_float, f, solution->xstar);
          if ((solution->fstar < b_options_ObjectiveLimit_tmp) &&
              ((solution->state != 0) || (objective->objtype != 5))) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = TaskSpaceCo_computeFval_ReuseHx(objective,
          memspace->workspace_float, f, solution->xstar);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (workingset->isActiveConstr[(workingset->isActiveIdx[3] + workingset->
       sizes[3]) - 2]) {
    b_nVar = 1;
    exitg2 = false;
    while ((!exitg2) && (b_nVar <= workingset->nActiveConstr)) {
      if ((workingset->Wid[b_nVar - 1] == 4) && (workingset->Wlocalidx[b_nVar -
           1] == workingset->sizes[3])) {
        TaskSpaceControlle_removeConstr(workingset, b_nVar);
        exitg2 = true;
      } else {
        b_nVar++;
      }
    }
  }

  b_nVar = workingset->nActiveConstr;
  while ((b_nVar > 0) && (b_nVar > nVar_tmp)) {
    TaskSpaceControlle_removeConstr(workingset, b_nVar);
    b_nVar--;
  }

  solution->maxConstr = solution->xstar[6];
  TaskSpaceControl_setProblemType(workingset, PROBTYPE_ORIG);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_linearForm_(boolean_T obj_hasLinear, int32_T
  obj_nvar, real_T workspace[175], const real_T f[6], const real_T x[7])
{
  int32_T b;
  int32_T beta1;
  static const int8_T h[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  beta1 = 0;
  if (obj_hasLinear) {
    if ((uint8_T)obj_nvar - 1 >= 0) {
      memcpy(&workspace[0], &f[0], (uint8_T)obj_nvar * sizeof(real_T));
    }

    beta1 = 1;
  }

  if (obj_nvar != 0) {
    int32_T e;
    int32_T ix;
    if (beta1 != 1) {
      memset(&workspace[0], 0, (uint8_T)obj_nvar * sizeof(real_T));
    }

    ix = 0;
    e = (obj_nvar - 1) * obj_nvar + 1;
    for (beta1 = 1; obj_nvar < 0 ? beta1 >= e : beta1 <= e; beta1 += obj_nvar) {
      real_T c;
      int32_T g;
      c = 0.5 * x[ix];
      g = (beta1 + obj_nvar) - 1;
      for (b = beta1; b <= g; b++) {
        int32_T tmp;
        tmp = b - beta1;
        workspace[tmp] += (real_T)h[b - 1] * c;
      }

      ix++;
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static real_T TaskSpaceController_computeFval(const
  slzZ8M58FXlZqTD433BZJUH_TaskS_T *obj, real_T workspace[175], const real_T f[6],
  const real_T x[7])
{
  real_T val;
  int32_T idx;
  int32_T ixlast;
  int32_T scalarLB;
  int32_T vectorUB;
  switch (obj->objtype) {
   case 5:
    val = x[obj->nvar - 1] * obj->gammaScalar;
    break;

   case 3:
    TaskSpaceController_linearForm_(obj->hasLinear, obj->nvar, workspace, f, x);
    val = 0.0;
    if (obj->nvar >= 1) {
      ixlast = obj->nvar;
      for (idx = 0; idx < ixlast; idx++) {
        val += x[idx] * workspace[idx];
      }
    }
    break;

   default:
    TaskSpaceController_linearForm_(obj->hasLinear, obj->nvar, workspace, f, x);
    ixlast = obj->nvar + 1;
    scalarLB = ((((6 - obj->nvar) / 2) << 1) + obj->nvar) + 1;
    vectorUB = scalarLB - 2;
    for (idx = ixlast; idx <= vectorUB; idx += 2) {
      _mm_storeu_pd(&workspace[idx - 1], _mm_mul_pd(_mm_loadu_pd(&x[idx - 1]),
        _mm_set1_pd(0.0)));
    }

    for (idx = scalarLB; idx < 7; idx++) {
      workspace[idx - 1] = x[idx - 1] * 0.0;
    }

    val = 0.0;
    for (idx = 0; idx < 6; idx++) {
      val += x[idx] * workspace[idx];
    }
    break;
  }

  return val;
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_driver(const real_T f[6],
  smNINkioqq1a7FyOE4CETSB_TaskS_T *solution, sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T
  *memspace, sAElXDmDj36R7Z42SImJxmG_TaskS_T *workingset,
  s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T *cholmanager, sIOJhD9KwAkF5sEguPjYquC_TaskS_T
  runTimeOptions, skbvZoOR3lvVJ6HLoaviTXC_TaskS_T *qrmanager,
  slzZ8M58FXlZqTD433BZJUH_TaskS_T *objective)
{
  real_T constrViolation;
  real_T tmp;
  int32_T i;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T idxStartIneq_tmp;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T okWorkingSet;
  for (i = 0; i < 7; i++) {
    objective->grad[i] = 0.0;
  }

  for (i = 0; i < 6; i++) {
    objective->Hx[i] = 0.0;
  }

  objective->hasLinear = true;
  objective->nvar = 6;
  objective->maxVar = 7;
  objective->beta = 0.0;
  objective->rho = 0.0;
  objective->objtype = 3;
  objective->prev_objtype = 3;
  objective->prev_nvar = 0;
  objective->prev_hasLinear = false;
  objective->gammaScalar = 0.0;
  solution->iterations = 0;
  runTimeOptions.RemainFeasible = true;
  solution->state = 82;
  qrmanager->ldq = 7;
  memset(&qrmanager->QR[0], 0, 175U * sizeof(real_T));
  memset(&qrmanager->Q[0], 0, 49U * sizeof(real_T));
  memset(&qrmanager->jpvt[0], 0, 25U * sizeof(int32_T));
  qrmanager->mrows = 0;
  qrmanager->ncols = 0;
  for (i = 0; i < 7; i++) {
    qrmanager->tau[i] = 0.0;
  }

  qrmanager->minRowCol = 0;
  qrmanager->usedPivoting = false;
  TaskSpaceC_RemoveDependentIneq_(workingset, qrmanager, memspace, 1.0);
  okWorkingSet = TaskSpa_feasibleX0ForWorkingSet(memspace->workspace_float,
    solution->xstar, workingset, qrmanager);
  if (!okWorkingSet) {
    TaskSpaceC_RemoveDependentIneq_(workingset, qrmanager, memspace, 10.0);
    okWorkingSet = TaskSpa_feasibleX0ForWorkingSet(memspace->workspace_float,
      solution->xstar, workingset, qrmanager);
    if (!okWorkingSet) {
      solution->state = -7;
    }
  }

  if (solution->state >= 0) {
    solution->iterations = 0;
    solution->maxConstr = TaskSpac_maxConstraintViolation(workingset,
      solution->xstar);
    tmp = 1.0E-8 * runTimeOptions.ConstrRelTolFactor;
    guard1 = false;
    if (solution->maxConstr > tmp) {
      TaskSpaceController_phaseone(f, solution, memspace, workingset, qrmanager,
        cholmanager, &runTimeOptions, objective);
      if (solution->state == 0) {
      } else {
        solution->maxConstr = TaskSpac_maxConstraintViolation(workingset,
          solution->xstar);
        if (solution->maxConstr > tmp) {
          memset(&solution->lambda[0], 0, 25U * sizeof(real_T));
          solution->fstar = TaskSpaceController_computeFval(objective,
            memspace->workspace_float, f, solution->xstar);
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            for (i = 0; i < 6; i++) {
              solution->searchDir[i] = solution->xstar[i];
            }

            solution->state = 82;
            i = TaskSpaceCon_RemoveDependentEq_(memspace, workingset, qrmanager);
            if ((i != -1) && (workingset->nActiveConstr <= 7)) {
              TaskSpaceC_RemoveDependentIneq_(workingset, qrmanager, memspace,
                1.0);
              okWorkingSet = TaskSpa_feasibleX0ForWorkingSet
                (memspace->workspace_float, solution->xstar, workingset,
                 qrmanager);
              guard2 = false;
              if (!okWorkingSet) {
                TaskSpaceC_RemoveDependentIneq_(workingset, qrmanager, memspace,
                  10.0);
                okWorkingSet = TaskSpa_feasibleX0ForWorkingSet
                  (memspace->workspace_float, solution->xstar, workingset,
                   qrmanager);
                if (!okWorkingSet) {
                  solution->state = -7;
                } else {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }

              if (guard2) {
                if (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                    workingset->nVar) {
                  constrViolation = TaskSpac_maxConstraintViolation(workingset,
                    solution->xstar);
                  if (constrViolation > 1.0E-8) {
                    solution->state = -2;
                  }
                }
              }
            } else {
              solution->state = -3;
              idxStartIneq_tmp = workingset->nWConstr[0] + workingset->nWConstr
                [1];
              idxStartIneq = idxStartIneq_tmp + 1;
              idxEndIneq = workingset->nActiveConstr;
              for (i = idxStartIneq; i <= idxEndIneq; i++) {
                workingset->isActiveConstr[(workingset->isActiveIdx
                  [workingset->Wid[i - 1] - 1] + workingset->Wlocalidx[i - 1]) -
                  2] = false;
              }

              workingset->nWConstr[2] = 0;
              workingset->nWConstr[3] = 0;
              workingset->nWConstr[4] = 0;
              workingset->nActiveConstr = idxStartIneq_tmp;
            }

            constrViolation = TaskSpac_maxConstraintViolation(workingset,
              solution->xstar);
            if (constrViolation >= solution->maxConstr) {
              solution->maxConstr = constrViolation;
              for (i = 0; i < 6; i++) {
                solution->xstar[i] = solution->searchDir[i];
              }
            }
          }

          guard1 = true;
        }
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      TaskSpaceController_iterate(f, solution, memspace, workingset, qrmanager,
        cholmanager, objective, true, runTimeOptions.ConstrRelTolFactor,
        runTimeOptions.ProbRelTolFactor);
      if (solution->state != -6) {
        solution->maxConstr = TaskSpac_maxConstraintViolation(workingset,
          solution->xstar);
        TaskSpaceC_computeFirstOrderOpt(solution, objective, workingset->nVar,
          workingset->ATwset, workingset->nActiveConstr,
          memspace->workspace_float);
        runTimeOptions.RemainFeasible = false;
        while ((solution->iterations < 300) && ((solution->state == -7) ||
                ((solution->state == 1) && ((solution->maxConstr > tmp) ||
                  (solution->firstorderopt > 1.0E-8 *
                   runTimeOptions.ProbRelTolFactor))))) {
          TaskSpa_feasibleX0ForWorkingSet(memspace->workspace_float,
            solution->xstar, workingset, qrmanager);
          solution->state = 82;
          i = TaskSpaceCon_RemoveDependentEq_(memspace, workingset, qrmanager);
          if ((i != -1) && (workingset->nActiveConstr <= 7)) {
            TaskSpaceC_RemoveDependentIneq_(workingset, qrmanager, memspace, 1.0);
            okWorkingSet = TaskSpa_feasibleX0ForWorkingSet
              (memspace->workspace_float, solution->xstar, workingset, qrmanager);
            guard2 = false;
            if (!okWorkingSet) {
              TaskSpaceC_RemoveDependentIneq_(workingset, qrmanager, memspace,
                10.0);
              okWorkingSet = TaskSpa_feasibleX0ForWorkingSet
                (memspace->workspace_float, solution->xstar, workingset,
                 qrmanager);
              if (!okWorkingSet) {
                solution->state = -7;
              } else {
                guard2 = true;
              }
            } else {
              guard2 = true;
            }

            if (guard2) {
              if (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                  workingset->nVar) {
                constrViolation = TaskSpac_maxConstraintViolation(workingset,
                  solution->xstar);
                if (constrViolation > 1.0E-8) {
                  solution->state = -2;
                }
              }
            }
          } else {
            solution->state = -3;
            idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) +
              1;
            idxEndIneq = workingset->nActiveConstr;
            for (i = idxStartIneq; i <= idxEndIneq; i++) {
              workingset->isActiveConstr[(workingset->isActiveIdx
                [workingset->Wid[i - 1] - 1] + workingset->Wlocalidx[i - 1]) - 2]
                = false;
            }

            workingset->nWConstr[2] = 0;
            workingset->nWConstr[3] = 0;
            workingset->nWConstr[4] = 0;
            workingset->nActiveConstr = workingset->nWConstr[0] +
              workingset->nWConstr[1];
          }

          TaskSpaceController_phaseone_a(f, solution, memspace, workingset,
            qrmanager, cholmanager, objective, &runTimeOptions);
          TaskSpaceController_iterate(f, solution, memspace, workingset,
            qrmanager, cholmanager, objective, false,
            runTimeOptions.ConstrRelTolFactor, runTimeOptions.ProbRelTolFactor);
          solution->maxConstr = TaskSpac_maxConstraintViolation(workingset,
            solution->xstar);
          TaskSpaceC_computeFirstOrderOpt(solution, objective, workingset->nVar,
            workingset->ATwset, workingset->nActiveConstr,
            memspace->workspace_float);
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceControll_linearForm__b(boolean_T obj_hasLinear, int32_T
  obj_nvar, real_T workspace[7], const real_T f[6], const real_T x[7])
{
  int32_T b;
  int32_T beta1;
  static const int8_T g[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  beta1 = 0;
  if (obj_hasLinear) {
    if ((uint8_T)obj_nvar - 1 >= 0) {
      memcpy(&workspace[0], &f[0], (uint8_T)obj_nvar * sizeof(real_T));
    }

    beta1 = 1;
  }

  if (obj_nvar != 0) {
    int32_T d;
    int32_T ix;
    if (beta1 != 1) {
      memset(&workspace[0], 0, (uint8_T)obj_nvar * sizeof(real_T));
    }

    ix = 0;
    d = (obj_nvar - 1) * obj_nvar + 1;
    for (beta1 = 1; obj_nvar < 0 ? beta1 >= d : beta1 <= d; beta1 += obj_nvar) {
      int32_T e;
      e = (beta1 + obj_nvar) - 1;
      for (b = beta1; b <= e; b++) {
        int32_T tmp;
        tmp = b - beta1;
        workspace[tmp] += (real_T)g[b - 1] * x[ix];
      }

      ix++;
    }
  }
}

/* Function for MATLAB Function: '<S1>/SPDControllerWithQP_Left1' */
static void TaskSpaceController_quadprog(const real_T f[6], const real_T bineq
  [24], const real_T x0[6], real_T x[6], real_T *fval, real_T *exitflag, char_T
  output_algorithm[10], real_T *output_firstorderopt, real_T
  *output_constrviolation, real_T *output_iterations,
  sVMzcbzaOHlwbaxNrLGVFM_TaskSp_T *lambda)
{
  __m128d tmp;
  sAElXDmDj36R7Z42SImJxmG_TaskS_T WorkingSet;
  sIOJhD9KwAkF5sEguPjYquC_TaskS_T expl_temp;
  sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T memspace;
  skbvZoOR3lvVJ6HLoaviTXC_TaskS_T QRManager;
  slzZ8M58FXlZqTD433BZJUH_TaskS_T QPObjective;
  smNINkioqq1a7FyOE4CETSB_TaskS_T solution;
  real_T b_colSum;
  real_T colSum;
  real_T f_infnrm;
  real_T tol;
  int32_T i;
  int32_T idxFillStart;
  int32_T idxOffset;
  int32_T vectorUB;
  static const int8_T WorkingSet_tmp[5] = { 0, 0, 24, 0, 0 };

  static const int8_T m[5] = { 0, 0, 24, 1, 0 };

  static const int8_T q[6] = { 1, 0, 0, 24, 0, 0 };

  static const int8_T o[5] = { 0, 0, 24, 24, 0 };

  static const int8_T p[5] = { 0, 0, 24, 25, 0 };

  static const int8_T r[6] = { 1, 0, 0, 24, 1, 0 };

  static const int8_T s[6] = { 1, 0, 0, 24, 24, 0 };

  static const int8_T t[6] = { 1, 0, 0, 24, 25, 0 };

  static const int8_T u[144] = { 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, -1 };

  static const int8_T H[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const char_T v[10] = { 'a', 'c', 't', 'i', 'v', 'e', '-', 's', 'e', 't'
  };

  solution.fstar = 0.0;
  solution.firstorderopt = 0.0;
  memset(&solution.lambda[0], 0, 25U * sizeof(real_T));
  solution.state = 0;
  solution.maxConstr = 0.0;
  solution.iterations = 0;
  for (i = 0; i < 7; i++) {
    solution.searchDir[i] = 0.0;
  }

  for (idxFillStart = 0; idxFillStart < 6; idxFillStart++) {
    solution.xstar[idxFillStart] = x0[idxFillStart];
  }

  TaskSpaceController_B.CholRegManager.ndims = 0;
  TaskSpaceController_B.CholRegManager.info = 0;
  TaskSpaceController_B.CholRegManager.ConvexCheck = true;
  TaskSpaceController_B.CholRegManager.regTol_ = 0.0;
  WorkingSet.nVarOrig = 6;
  WorkingSet.nVarMax = 7;
  WorkingSet.ldA = 7;
  memset(&WorkingSet.Aineq[0], 0, 168U * sizeof(real_T));
  memset(&WorkingSet.bineq[0], 0, 24U * sizeof(real_T));
  for (i = 0; i < 7; i++) {
    WorkingSet.lb[i] = 0.0;
    WorkingSet.ub[i] = 0.0;
    WorkingSet.indexLB[i] = 0;
    WorkingSet.indexUB[i] = 0;
    WorkingSet.indexFixed[i] = 0;
  }

  WorkingSet.mEqRemoved = 0;
  memset(&WorkingSet.ATwset[0], 0, 175U * sizeof(real_T));
  memset(&WorkingSet.bwset[0], 0, 25U * sizeof(real_T));
  memset(&WorkingSet.maxConstrWorkspace[0], 0, 25U * sizeof(real_T));
  memset(&WorkingSet.Wid[0], 0, 25U * sizeof(int32_T));
  memset(&WorkingSet.Wlocalidx[0], 0, 25U * sizeof(int32_T));
  for (i = 0; i < 25; i++) {
    WorkingSet.isActiveConstr[i] = false;
  }

  WorkingSet.mConstrOrig = 24;
  WorkingSet.mConstrMax = 25;
  for (i = 0; i < 5; i++) {
    WorkingSet.sizesNormal[i] = WorkingSet_tmp[i];
    WorkingSet.sizesPhaseOne[i] = m[i];
    WorkingSet.sizesRegularized[i] = o[i];
    WorkingSet.sizesRegPhaseOne[i] = p[i];
  }

  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxRegPhaseOne[i] = q[i];
  }

  for (idxFillStart = 0; idxFillStart < 5; idxFillStart++) {
    WorkingSet.isActiveIdxRegPhaseOne[idxFillStart + 1] +=
      WorkingSet.isActiveIdxRegPhaseOne[idxFillStart];
  }

  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxNormal[i] = WorkingSet.isActiveIdxRegPhaseOne[i];
    WorkingSet.isActiveIdxRegPhaseOne[i] = r[i];
  }

  for (idxFillStart = 0; idxFillStart < 5; idxFillStart++) {
    WorkingSet.isActiveIdxRegPhaseOne[idxFillStart + 1] +=
      WorkingSet.isActiveIdxRegPhaseOne[idxFillStart];
  }

  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxPhaseOne[i] = WorkingSet.isActiveIdxRegPhaseOne[i];
    WorkingSet.isActiveIdxRegPhaseOne[i] = s[i];
  }

  for (idxFillStart = 0; idxFillStart < 5; idxFillStart++) {
    WorkingSet.isActiveIdxRegPhaseOne[idxFillStart + 1] +=
      WorkingSet.isActiveIdxRegPhaseOne[idxFillStart];
  }

  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxRegularized[i] = WorkingSet.isActiveIdxRegPhaseOne[i];
    WorkingSet.isActiveIdxRegPhaseOne[i] = t[i];
  }

  for (idxFillStart = 0; idxFillStart < 5; idxFillStart++) {
    WorkingSet.isActiveIdxRegPhaseOne[idxFillStart + 1] +=
      WorkingSet.isActiveIdxRegPhaseOne[idxFillStart];
  }

  for (i = 0; i < 6; i++) {
    for (idxFillStart = 0; idxFillStart < 24; idxFillStart++) {
      WorkingSet.Aineq[i + 7 * idxFillStart] = u[24 * i + idxFillStart];
    }
  }

  memcpy(&WorkingSet.bineq[0], &bineq[0], 24U * sizeof(real_T));
  WorkingSet.nVar = 6;
  WorkingSet.mConstr = 24;
  for (idxOffset = 0; idxOffset < 5; idxOffset++) {
    WorkingSet.sizes[idxOffset] = WorkingSet.sizesNormal[idxOffset];
  }

  for (idxOffset = 0; idxOffset < 6; idxOffset++) {
    WorkingSet.isActiveIdx[idxOffset] = WorkingSet.isActiveIdxNormal[idxOffset];
  }

  WorkingSet.probType = 3;
  idxFillStart = WorkingSet.isActiveIdx[2];
  for (i = idxFillStart; i < 26; i++) {
    WorkingSet.isActiveConstr[i - 1] = false;
  }

  WorkingSet.nWConstr[0] = 0;
  WorkingSet.nWConstr[1] = 0;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = 0;
  WorkingSet.SLACK0 = 0.0;
  tol = 1.0;
  for (idxFillStart = 0; idxFillStart < 24; idxFillStart++) {
    colSum = 0.0;
    i = 7 * idxFillStart;
    for (idxOffset = 0; idxOffset < 6; idxOffset++) {
      colSum += fabs(WorkingSet.Aineq[idxOffset + i]);
    }

    tol = fmax(tol, colSum);
  }

  colSum = 0.0;
  f_infnrm = 0.0;
  for (idxFillStart = 0; idxFillStart < 6; idxFillStart++) {
    b_colSum = 0.0;
    for (i = 0; i < 6; i++) {
      b_colSum += (real_T)H[6 * idxFillStart + i];
    }

    colSum = fmax(colSum, b_colSum);
    f_infnrm = fmax(f_infnrm, fabs(f[idxFillStart]));
  }

  colSum = fmax(fmax(tol, f_infnrm), colSum);
  TaskSpaceController_B.CholRegManager.scaleFactor = colSum;
  expl_temp.ProbRelTolFactor = colSum;
  expl_temp.ConstrRelTolFactor = tol;
  expl_temp.MaxIterations = 300;
  expl_temp.RemainFeasible = false;
  TaskSpaceController_driver(f, &solution, &memspace, &WorkingSet,
    &TaskSpaceController_B.CholRegManager, expl_temp, &QRManager, &QPObjective);
  for (idxFillStart = 0; idxFillStart < 6; idxFillStart++) {
    x[idxFillStart] = solution.xstar[idxFillStart];
  }

  if (solution.state > 0) {
    *fval = solution.fstar;
  } else {
    *fval = TaskSpaceController_computeFval(&QPObjective,
      memspace.workspace_float, f, solution.xstar);
  }

  switch (solution.state) {
   case 2:
    solution.state = -3;
    break;

   case -3:
    solution.state = -2;
    break;

   case 4:
    solution.state = -2;
    break;
  }

  *exitflag = solution.state;
  if (solution.state == -2) {
    solution.firstorderopt = (rtInf);
  } else if (solution.state <= 0) {
    solution.maxConstr = TaskSpac_maxConstraintViolation(&WorkingSet,
      solution.xstar);
    if (solution.maxConstr <= 1.0E-8 * tol) {
      switch (QPObjective.objtype) {
       case 5:
        if (QPObjective.nvar - 2 >= 0) {
          memset(&QPObjective.grad[0], 0, (uint32_T)(QPObjective.nvar - 1) *
                 sizeof(real_T));
        }

        QPObjective.grad[QPObjective.nvar - 1] = QPObjective.gammaScalar;
        break;

       case 3:
        TaskSpaceControll_linearForm__b(QPObjective.hasLinear, QPObjective.nvar,
          QPObjective.grad, f, solution.xstar);
        break;

       default:
        TaskSpaceControll_linearForm__b(QPObjective.hasLinear, QPObjective.nvar,
          QPObjective.grad, f, solution.xstar);
        idxFillStart = QPObjective.nvar + 1;
        idxOffset = ((((6 - QPObjective.nvar) / 2) << 1) + QPObjective.nvar) + 1;
        vectorUB = idxOffset - 2;
        for (i = idxFillStart; i <= vectorUB; i += 2) {
          tmp = _mm_loadu_pd(&solution.xstar[i - 1]);
          _mm_storeu_pd(&QPObjective.grad[i - 1], _mm_mul_pd(tmp, _mm_set1_pd
            (0.0)));
        }

        for (i = idxOffset; i < 7; i++) {
          QPObjective.grad[i - 1] = solution.xstar[i - 1] * 0.0;
        }
        break;
      }

      TaskSpaceC_computeFirstOrderOpt(&solution, &QPObjective, WorkingSet.nVar,
        WorkingSet.ATwset, WorkingSet.nActiveConstr, memspace.workspace_float);
    } else {
      solution.firstorderopt = (rtInf);
    }
  }

  for (idxOffset = 0; idxOffset < 10; idxOffset++) {
    output_algorithm[idxOffset] = v[idxOffset];
  }

  *output_firstorderopt = solution.firstorderopt;
  *output_constrviolation = fmax(0.0, solution.maxConstr);
  *output_iterations = solution.iterations;
  memset(&lambda->ineqlin[0], 0, 24U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    lambda->lower[i] = 0.0;
    lambda->upper[i] = 0.0;
  }

  if (WorkingSet.nActiveConstr > 0) {
    idxFillStart = (uint8_T)(WorkingSet.sizes[3] + 24);
    for (i = 0; i < idxFillStart; i++) {
      memspace.workspace_float[i] = solution.lambda[i];
      solution.lambda[i] = 0.0;
    }

    idxFillStart = 0;
    i = 0;
    while ((i + 1 <= WorkingSet.nActiveConstr) && (WorkingSet.Wid[i] <= 2)) {
      if (WorkingSet.Wid[i] == 1) {
        idxOffset = 1;
      } else {
        idxOffset = WorkingSet.isActiveIdx[1];
      }

      solution.lambda[(idxOffset + WorkingSet.Wlocalidx[i]) - 2] =
        memspace.workspace_float[idxFillStart];
      idxFillStart++;
      i++;
    }

    while (i + 1 <= WorkingSet.nActiveConstr) {
      switch (WorkingSet.Wid[i]) {
       case 3:
        idxOffset = WorkingSet.isActiveIdx[2];
        break;

       case 4:
        idxOffset = WorkingSet.isActiveIdx[3];
        break;

       default:
        idxOffset = WorkingSet.isActiveIdx[4];
        break;
      }

      solution.lambda[(idxOffset + WorkingSet.Wlocalidx[i]) - 2] =
        memspace.workspace_float[idxFillStart];
      idxFillStart++;
      i++;
    }

    memset(&lambda->ineqlin[0], 0, 24U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      lambda->lower[i] = 0.0;
      lambda->upper[i] = 0.0;
    }

    idxFillStart = WorkingSet.isActiveIdx[1];
    for (i = 1; i < idxFillStart; i++) {
      /* Check node always fails. would cause program termination and was eliminated */
    }

    idxFillStart = WorkingSet.isActiveIdx[1];
    i = WorkingSet.isActiveIdx[2];
    for (idxOffset = idxFillStart; idxOffset < i; idxOffset++) {
      /* Check node always fails. would cause program termination and was eliminated */
    }

    idxFillStart = WorkingSet.isActiveIdx[2];
    i = WorkingSet.isActiveIdx[3];
    if (idxFillStart <= i - 1) {
      memcpy(&lambda->ineqlin[0], &solution.lambda[idxFillStart + -1], (uint32_T)
             (i - idxFillStart) * sizeof(real_T));
    }

    idxFillStart = WorkingSet.isActiveIdx[3];
    i = WorkingSet.isActiveIdx[4];
    for (idxOffset = idxFillStart; idxOffset < i; idxOffset++) {
      lambda->lower[WorkingSet.indexLB[idxOffset - idxFillStart] - 1] =
        solution.lambda[idxOffset - 1];
    }

    idxFillStart = WorkingSet.isActiveIdx[4];
    i = WorkingSet.isActiveIdx[5];
    for (idxOffset = idxFillStart; idxOffset < i; idxOffset++) {
      /* Check node always fails. would cause program termination and was eliminated */
    }
  }
}

/* Model step function */
void TaskSpaceController_step(void)
{
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  __m128d tmp_5;
  sVMzcbzaOHlwbaxNrLGVFM_TaskSp_T b_lambda;
  real_T a[144];
  real_T Jlambda[36];
  real_T JlambdaT[36];
  real_T T[36];
  real_T TaskKp[36];
  real_T TaskKp_0[36];
  real_T TaskKv[36];
  real_T a_tmp[36];
  real_T a_tmp_0[36];
  real_T tmp[24];
  real_T Ttilde[16];
  real_T omgmat_1[16];
  real_T e[12];
  real_T edot[12];
  real_T edot_0[12];
  real_T rtb_eint[12];
  real_T rtb_qddot_b[12];
  real_T rtb_qddot_b_0[12];
  real_T omgmat[9];
  real_T omgmat_0[9];
  real_T T_0[6];
  real_T a_tmp_1[6];
  real_T a_tmp_2[6];
  real_T lambda[6];
  real_T lambda_next[6];
  real_T q[6];
  real_T q_0[6];
  real_T qdot[6];
  real_T qdot_0[6];
  real_T rtb_qddot_m[6];
  real_T se3mat[3];
  real_T tmp_4[2];
  real_T T_l;
  real_T acosinput;
  real_T d_a;
  real_T omgtheta_idx_0;
  real_T omgtheta_idx_1;
  int32_T ipiv[6];
  int32_T i;
  int32_T i_0;
  int32_T kAcol;
  int32_T nz;
  char_T expl_temp[10];
  int8_T lambda_tmp[36];
  int8_T b_I[9];
  boolean_T b_x[6];
  boolean_T b_p;
  boolean_T isodd;
  static const int8_T b_b[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const int8_T j_a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  boolean_T exitg1;

  /* MATLAB Function: '<S1>/SPDControllerWithQP_Left1' incorporates:
   *  Inport: '<Root>/Jb_l'
   *  Inport: '<Root>/Jbdot_l'
   *  Inport: '<Root>/M'
   *  Inport: '<Root>/T_des_l'
   *  Inport: '<Root>/T_l'
   *  Inport: '<Root>/TaskKp'
   *  Inport: '<Root>/TaskKv'
   *  Inport: '<Root>/V_des_l'
   *  Inport: '<Root>/V_l'
   *  Inport: '<Root>/Vdot_des_l'
   *  Inport: '<Root>/a'
   *  Inport: '<Root>/b0'
   *  Inport: '<Root>/c'
   *  Inport: '<Root>/dt'
   *  Inport: '<Root>/q'
   *  Inport: '<Root>/q_max'
   *  Inport: '<Root>/q_min'
   *  Inport: '<Root>/qdot'
   *  Inport: '<Root>/qdot_max'
   *  Inport: '<Root>/qdot_min'
   */
  for (i = 0; i < 6; i++) {
    qdot[i] = TaskSpaceController_U.qdot[i];
    q[i] = TaskSpaceController_U.q[i];
  }

  memset(&TaskKp[0], 0, 36U * sizeof(real_T));
  for (nz = 0; nz < 6; nz++) {
    TaskKp[nz + 6 * nz] = TaskSpaceController_U.TaskKp[nz];
  }

  memset(&TaskKv[0], 0, 36U * sizeof(real_T));
  for (nz = 0; nz < 6; nz++) {
    TaskKv[nz + 6 * nz] = TaskSpaceController_U.TaskKv[nz];
  }

  for (nz = 0; nz < 3; nz++) {
    omgmat[3 * nz] = TaskSpaceController_U.T_l[nz];
    omgmat[3 * nz + 1] = TaskSpaceController_U.T_l[nz + 4];
    omgmat[3 * nz + 2] = TaskSpaceController_U.T_l[nz + 8];
  }

  for (nz = 0; nz <= 6; nz += 2) {
    tmp_3 = _mm_loadu_pd(&omgmat[nz]);
    _mm_storeu_pd(&omgmat_0[nz], _mm_mul_pd(tmp_3, _mm_set1_pd(-1.0)));
  }

  for (nz = 8; nz < 9; nz++) {
    omgmat_0[nz] = -omgmat[nz];
  }

  acosinput = TaskSpaceController_U.T_l[13];
  T_l = TaskSpaceController_U.T_l[12];
  omgtheta_idx_0 = TaskSpaceController_U.T_l[14];
  for (nz = 0; nz < 3; nz++) {
    kAcol = nz << 2;
    omgmat_1[kAcol] = omgmat[3 * nz];
    omgmat_1[kAcol + 1] = omgmat[3 * nz + 1];
    omgmat_1[kAcol + 2] = omgmat[3 * nz + 2];
    omgmat_1[nz + 12] = (omgmat_0[nz + 3] * acosinput + omgmat_0[nz] * T_l) +
      omgmat_0[nz + 6] * omgtheta_idx_0;
  }

  omgmat_1[3] = 0.0;
  omgmat_1[7] = 0.0;
  omgmat_1[11] = 0.0;
  omgmat_1[15] = 1.0;
  for (nz = 0; nz < 4; nz++) {
    i = nz << 2;
    acosinput = TaskSpaceController_U.T_des_l[i + 1];
    T_l = TaskSpaceController_U.T_des_l[i];
    omgtheta_idx_0 = TaskSpaceController_U.T_des_l[i + 2];
    omgtheta_idx_1 = TaskSpaceController_U.T_des_l[i + 3];
    for (kAcol = 0; kAcol <= 2; kAcol += 2) {
      tmp_3 = _mm_loadu_pd(&omgmat_1[kAcol + 4]);
      tmp_0 = _mm_loadu_pd(&omgmat_1[kAcol]);
      tmp_1 = _mm_loadu_pd(&omgmat_1[kAcol + 8]);
      tmp_2 = _mm_loadu_pd(&omgmat_1[kAcol + 12]);
      _mm_storeu_pd(&Ttilde[kAcol + i], _mm_add_pd(_mm_add_pd(_mm_add_pd
        (_mm_mul_pd(_mm_set1_pd(acosinput), tmp_3), _mm_mul_pd(_mm_set1_pd(T_l),
        tmp_0)), _mm_mul_pd(_mm_set1_pd(omgtheta_idx_0), tmp_1)), _mm_mul_pd
        (_mm_set1_pd(omgtheta_idx_1), tmp_2)));
    }
  }

  T_l = (((Ttilde[0] + Ttilde[5]) + Ttilde[10]) - 1.0) / 2.0;
  if (T_l >= 1.0) {
    memset(&omgmat[0], 0, 9U * sizeof(real_T));
  } else if (T_l <= -1.0) {
    if (!(fabs(Ttilde[10] + 1.0) < 2.2204460492503131E-16)) {
      acosinput = 1.0 / sqrt((Ttilde[10] + 1.0) * 2.0);
      tmp_3 = _mm_mul_pd(_mm_set1_pd(acosinput), _mm_loadu_pd(&Ttilde[8]));
      _mm_storeu_pd(&tmp_4[0], tmp_3);
      omgtheta_idx_0 = tmp_4[0];
      omgtheta_idx_1 = tmp_4[1];
      acosinput *= Ttilde[10] + 1.0;
    } else if (!(fabs(Ttilde[5] + 1.0) < 2.2204460492503131E-16)) {
      acosinput = 1.0 / sqrt((Ttilde[5] + 1.0) * 2.0);
      omgtheta_idx_0 = acosinput * Ttilde[4];
      omgtheta_idx_1 = (Ttilde[5] + 1.0) * acosinput;
      acosinput *= Ttilde[6];
    } else {
      acosinput = 1.0 / sqrt((Ttilde[0] + 1.0) * 2.0);
      omgtheta_idx_0 = (Ttilde[0] + 1.0) * acosinput;
      tmp_3 = _mm_mul_pd(_mm_set1_pd(acosinput), _mm_loadu_pd(&Ttilde[1]));
      _mm_storeu_pd(&tmp_4[0], tmp_3);
      omgtheta_idx_1 = tmp_4[0];
      acosinput = tmp_4[1];
    }

    omgtheta_idx_0 *= 3.1415926535897931;
    omgtheta_idx_1 *= 3.1415926535897931;
    acosinput *= 3.1415926535897931;
    omgmat[0] = 0.0;
    omgmat[3] = -acosinput;
    omgmat[6] = omgtheta_idx_1;
    omgmat[1] = acosinput;
    omgmat[4] = 0.0;
    omgmat[7] = -omgtheta_idx_0;
    omgmat[2] = -omgtheta_idx_1;
    omgmat[5] = omgtheta_idx_0;
    omgmat[8] = 0.0;
  } else {
    acosinput = acos(T_l);
    acosinput *= 1.0 / (2.0 * sin(acosinput));
    for (nz = 0; nz < 3; nz++) {
      kAcol = nz << 2;
      tmp_3 = _mm_mul_pd(_mm_sub_pd(_mm_loadu_pd(&Ttilde[kAcol]), _mm_set_pd
        (Ttilde[nz + 4], Ttilde[nz])), _mm_set1_pd(acosinput));
      _mm_storeu_pd(&omgmat[3 * nz], tmp_3);
      omgmat[3 * nz + 2] = (Ttilde[kAcol + 2] - Ttilde[nz + 8]) * acosinput;
    }
  }

  isodd = false;
  b_p = true;
  nz = 0;
  exitg1 = false;
  while ((!exitg1) && (nz < 9)) {
    if (!(omgmat[nz] == 0.0)) {
      b_p = false;
      exitg1 = true;
    } else {
      nz++;
    }
  }

  if (b_p) {
    isodd = true;
  }

  if (isodd) {
    for (nz = 0; nz < 3; nz++) {
      kAcol = nz << 2;
      omgmat_1[kAcol] = 0.0;
      omgmat_1[kAcol + 1] = 0.0;
      omgmat_1[kAcol + 2] = 0.0;
      omgmat_1[nz + 12] = Ttilde[nz + 12];
    }
  } else {
    acosinput = acos(T_l);
    omgtheta_idx_0 = 1.0 / acosinput - 1.0 / tan(acosinput / 2.0) / 2.0;
    for (nz = 0; nz < 9; nz++) {
      b_I[nz] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (nz = 0; nz < 3; nz++) {
      T_l = 0.0;
      for (kAcol = 0; kAcol < 3; kAcol++) {
        i = 3 * kAcol + nz;
        T_l += (((omgmat[nz + 3] * omgtheta_idx_0 * omgmat[3 * kAcol + 1] +
                  omgtheta_idx_0 * omgmat[nz] * omgmat[3 * kAcol]) + omgmat[nz +
                 6] * omgtheta_idx_0 * omgmat[3 * kAcol + 2]) / acosinput +
                ((real_T)b_I[i] - omgmat[i] / 2.0)) * Ttilde[kAcol + 12];
        omgmat_1[kAcol + (nz << 2)] = omgmat[3 * nz + kAcol];
      }

      omgmat_1[nz + 12] = T_l;
    }
  }

  lambda[0] = omgmat_1[12];
  lambda[1] = omgmat_1[13];
  lambda[2] = omgmat_1[14];
  lambda[3] = omgmat_1[6];
  lambda[4] = omgmat_1[8];
  lambda[5] = omgmat_1[1];
  TaskSpaceController_dlog6(lambda, a_tmp);
  for (nz = 0; nz <= 34; nz += 2) {
    tmp_3 = _mm_loadu_pd(&a_tmp[nz]);
    _mm_storeu_pd(&a_tmp_0[nz], _mm_mul_pd(tmp_3, _mm_set1_pd(-1.0)));
  }

  for (nz = 0; nz < 6; nz++) {
    for (kAcol = 0; kAcol < 6; kAcol++) {
      acosinput = 0.0;
      for (i = 0; i < 6; i++) {
        acosinput += a_tmp_0[6 * i + kAcol] * TaskSpaceController_U.Jb_l[6 * nz
          + i];
      }

      Jlambda[kAcol + 6 * nz] = acosinput;
    }
  }

  for (nz = 0; nz < 3; nz++) {
    omgmat[3 * nz] = Ttilde[nz];
    omgmat[3 * nz + 1] = Ttilde[nz + 4];
    omgmat[3 * nz + 2] = Ttilde[nz + 8];
  }

  for (nz = 0; nz <= 6; nz += 2) {
    tmp_3 = _mm_loadu_pd(&omgmat[nz]);
    _mm_storeu_pd(&omgmat_0[nz], _mm_mul_pd(tmp_3, _mm_set1_pd(-1.0)));
  }

  for (nz = 8; nz < 9; nz++) {
    omgmat_0[nz] = -omgmat[nz];
  }

  acosinput = Ttilde[13];
  T_l = Ttilde[12];
  omgtheta_idx_0 = Ttilde[14];
  for (nz = 0; nz < 3; nz++) {
    kAcol = nz << 2;
    Ttilde[kAcol] = omgmat[3 * nz];
    Ttilde[kAcol + 1] = omgmat[3 * nz + 1];
    Ttilde[kAcol + 2] = omgmat[3 * nz + 2];
    Ttilde[nz + 12] = (omgmat_0[nz + 3] * acosinput + omgmat_0[nz] * T_l) +
      omgmat_0[nz + 6] * omgtheta_idx_0;
  }

  Ttilde[3] = 0.0;
  Ttilde[7] = 0.0;
  Ttilde[11] = 0.0;
  Ttilde[15] = 1.0;
  omgmat[0] = 0.0;
  omgmat[3] = -Ttilde[14];
  omgmat[6] = Ttilde[13];
  omgmat[1] = Ttilde[14];
  omgmat[4] = 0.0;
  omgmat[7] = -Ttilde[12];
  omgmat[2] = -Ttilde[13];
  omgmat[5] = Ttilde[12];
  omgmat[8] = 0.0;
  for (nz = 0; nz < 3; nz++) {
    T_l = omgmat[nz + 3];
    omgtheta_idx_0 = omgmat[nz];
    omgtheta_idx_1 = omgmat[nz + 6];
    for (kAcol = 0; kAcol < 3; kAcol++) {
      i = kAcol << 2;
      omgmat_0[nz + 3 * kAcol] = (Ttilde[i + 1] * T_l + Ttilde[i] *
        omgtheta_idx_0) + Ttilde[i + 2] * omgtheta_idx_1;
      T[kAcol + 6 * nz] = Ttilde[(nz << 2) + kAcol];
    }
  }

  for (nz = 0; nz < 3; nz++) {
    kAcol = (nz + 3) * 6;
    T[kAcol] = omgmat_0[3 * nz];
    T[6 * nz + 3] = 0.0;
    i = nz << 2;
    T[kAcol + 3] = Ttilde[i];
    T[kAcol + 1] = omgmat_0[3 * nz + 1];
    T[6 * nz + 4] = 0.0;
    T[kAcol + 4] = Ttilde[i + 1];
    T[kAcol + 2] = omgmat_0[3 * nz + 2];
    T[6 * nz + 5] = 0.0;
    T[kAcol + 5] = Ttilde[i + 2];
  }

  for (nz = 0; nz < 6; nz++) {
    T_l = 0.0;
    for (kAcol = 0; kAcol < 6; kAcol++) {
      T_l += T[6 * kAcol + nz] * TaskSpaceController_U.V_des_l[kAcol];
    }

    T_0[nz] = T_l - TaskSpaceController_U.V_l[nz];
  }

  for (nz = 0; nz < 6; nz++) {
    omgtheta_idx_0 = 0.0;
    for (kAcol = 0; kAcol < 6; kAcol++) {
      omgtheta_idx_0 += a_tmp[6 * kAcol + nz] * T_0[kAcol];
    }

    lambda_next[nz] = omgtheta_idx_0 * TaskSpaceController_U.dt + lambda[nz];
  }

  memcpy(&a_tmp[0], &TaskSpaceController_U.Jb_l[0], 36U * sizeof(real_T));
  TaskSpaceController_xzgetrf(a_tmp, ipiv, &nz);
  acosinput = a_tmp[0];
  isodd = false;
  for (nz = 0; nz < 5; nz++) {
    acosinput *= a_tmp[((nz + 1) * 6 + nz) + 1];
    if (ipiv[nz] > nz + 1) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    acosinput = -acosinput;
  }

  memcpy(&a_tmp[0], &TaskSpaceController_U.Jb_l[0], 36U * sizeof(real_T));
  TaskSpaceController_xzgetrf(a_tmp, ipiv, &nz);
  omgtheta_idx_0 = a_tmp[0];
  isodd = false;
  for (nz = 0; nz < 5; nz++) {
    omgtheta_idx_0 *= a_tmp[((nz + 1) * 6 + nz) + 1];
    if (ipiv[nz] > nz + 1) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    omgtheta_idx_0 = -omgtheta_idx_0;
  }

  acosinput = exp(-acosinput * omgtheta_idx_0 / TaskSpaceController_U.a[0] /
                  TaskSpaceController_U.a[0]) * TaskSpaceController_U.b0[0];
  TaskSpaceController_dlog6(lambda_next, a_tmp);
  for (i = 0; i < 6; i++) {
    for (nz = 0; nz < 6; nz++) {
      JlambdaT[nz + 6 * i] = Jlambda[6 * nz + i];
    }

    lambda[i] = -lambda_next[i];
  }

  omgmat_1[0] = 0.0;
  omgmat_1[4] = -lambda[5];
  omgmat_1[8] = lambda[4];
  omgmat_1[1] = lambda[5];
  omgmat_1[5] = 0.0;
  omgmat_1[9] = -lambda[3];
  omgmat_1[2] = -lambda[4];
  omgmat_1[6] = lambda[3];
  omgmat_1[10] = 0.0;
  omgmat_1[12] = lambda[0];
  omgmat_1[13] = lambda[1];
  omgmat_1[14] = lambda[2];
  omgmat_1[3] = 0.0;
  omgmat_1[7] = 0.0;
  omgmat_1[11] = 0.0;
  omgmat_1[15] = 0.0;
  se3mat[0] = lambda[3];
  se3mat[1] = lambda[4];
  se3mat[2] = lambda[5];
  omgtheta_idx_0 = TaskSpaceController_norm(se3mat);
  if (fabs(omgtheta_idx_0) < 2.2204460492503131E-16) {
    for (nz = 0; nz < 9; nz++) {
      b_I[nz] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (nz = 0; nz < 3; nz++) {
      kAcol = nz << 2;
      Ttilde[kAcol] = b_I[3 * nz];
      Ttilde[kAcol + 1] = b_I[3 * nz + 1];
      Ttilde[kAcol + 2] = b_I[3 * nz + 2];
      Ttilde[nz + 12] = omgmat_1[nz + 12];
    }

    Ttilde[3] = 0.0;
    Ttilde[7] = 0.0;
    Ttilde[11] = 0.0;
    Ttilde[15] = 1.0;
  } else {
    for (nz = 0; nz < 3; nz++) {
      kAcol = nz << 2;
      tmp_3 = _mm_div_pd(_mm_loadu_pd(&omgmat_1[kAcol]), _mm_set1_pd
                         (omgtheta_idx_0));
      _mm_storeu_pd(&omgmat[3 * nz], tmp_3);
      omgmat[3 * nz + 2] = omgmat_1[kAcol + 2] / omgtheta_idx_0;
    }

    omgtheta_idx_1 = cos(omgtheta_idx_0);
    T_l = sin(omgtheta_idx_0);
    d_a = omgtheta_idx_0 - T_l;
    for (nz = 0; nz < 9; nz++) {
      b_I[nz] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (nz = 0; nz < 3; nz++) {
      for (kAcol = 0; kAcol < 3; kAcol++) {
        i = 3 * kAcol + nz;
        omgmat_0[i] = (((1.0 - omgtheta_idx_1) * omgmat[nz] * omgmat[3 * kAcol]
                        + (1.0 - omgtheta_idx_1) * omgmat[nz + 3] * omgmat[3 *
                        kAcol + 1]) + (1.0 - omgtheta_idx_1) * omgmat[nz + 6] *
                       omgmat[3 * kAcol + 2]) + (omgmat[i] * T_l + (real_T)b_I[i]);
      }
    }

    for (nz = 0; nz < 3; nz++) {
      T_l = 0.0;
      for (kAcol = 0; kAcol < 3; kAcol++) {
        i = 3 * kAcol + nz;
        T_l += (((omgmat[nz + 3] * d_a * omgmat[3 * kAcol + 1] + d_a * omgmat[nz]
                  * omgmat[3 * kAcol]) + omgmat[nz + 6] * d_a * omgmat[3 * kAcol
                 + 2]) + ((1.0 - omgtheta_idx_1) * omgmat[i] + (real_T)j_a[i] *
                          omgtheta_idx_0)) * omgmat_1[kAcol + 12];
        Ttilde[kAcol + (nz << 2)] = omgmat_0[3 * nz + kAcol];
      }

      Ttilde[nz + 12] = T_l / omgtheta_idx_0;
    }

    Ttilde[3] = 0.0;
    Ttilde[7] = 0.0;
    Ttilde[11] = 0.0;
    Ttilde[15] = 1.0;
  }

  for (nz = 0; nz < 36; nz++) {
    /* MATLAB Function: '<S1>/SPDControllerWithQP_Right1' */
    lambda_tmp[nz] = b_b[nz];
    TaskKp_0[nz] = -TaskKp[nz];
  }

  omgmat[0] = 0.0;
  omgmat[3] = -Ttilde[14];
  omgmat[6] = Ttilde[13];
  omgmat[1] = Ttilde[14];
  omgmat[4] = 0.0;
  omgmat[7] = -Ttilde[12];
  omgmat[2] = -Ttilde[13];
  omgmat[5] = Ttilde[12];
  omgmat[8] = 0.0;
  for (nz = 0; nz < 3; nz++) {
    T_l = omgmat[nz + 3];
    omgtheta_idx_0 = omgmat[nz];
    omgtheta_idx_1 = omgmat[nz + 6];
    for (kAcol = 0; kAcol < 3; kAcol++) {
      i = kAcol << 2;
      omgmat_0[nz + 3 * kAcol] = (Ttilde[i + 1] * T_l + Ttilde[i] *
        omgtheta_idx_0) + Ttilde[i + 2] * omgtheta_idx_1;
      T[kAcol + 6 * nz] = Ttilde[(nz << 2) + kAcol];
    }
  }

  for (nz = 0; nz < 3; nz++) {
    kAcol = (nz + 3) * 6;
    T[kAcol] = omgmat_0[3 * nz];
    T[6 * nz + 3] = 0.0;
    i = nz << 2;
    T[kAcol + 3] = Ttilde[i];
    T[kAcol + 1] = omgmat_0[3 * nz + 1];
    T[6 * nz + 4] = 0.0;
    T[kAcol + 4] = Ttilde[i + 1];
    T[kAcol + 2] = omgmat_0[3 * nz + 2];
    T[6 * nz + 5] = 0.0;
    T[kAcol + 5] = Ttilde[i + 2];
  }

  for (nz = 0; nz < 6; nz++) {
    for (kAcol = 0; kAcol < 6; kAcol++) {
      omgtheta_idx_0 = 0.0;
      for (i = 0; i < 6; i++) {
        omgtheta_idx_0 += a_tmp[6 * i + nz] * T[6 * kAcol + i];
      }

      a_tmp_0[nz + 6 * kAcol] = omgtheta_idx_0;
    }

    T_0[nz] = TaskSpaceController_U.dt * TaskSpaceController_U.Vdot_des_l[nz] +
      TaskSpaceController_U.V_des_l[nz];
  }

  for (nz = 0; nz < 6; nz++) {
    a_tmp_1[nz] = 0.0;
    a_tmp_2[nz] = 0.0;
    T_l = 0.0;
    for (kAcol = 0; kAcol < 6; kAcol++) {
      i = 6 * kAcol + nz;
      _mm_storeu_pd(&tmp_4[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(a_tmp[i],
        a_tmp_0[i]), _mm_set_pd(TaskSpaceController_U.V_l[kAcol], T_0[kAcol])),
        _mm_set_pd(a_tmp_2[nz], a_tmp_1[nz])));
      a_tmp_1[nz] = tmp_4[0];
      a_tmp_2[nz] = tmp_4[1];
      omgtheta_idx_0 = 0.0;
      for (i = 0; i < 6; i++) {
        omgtheta_idx_0 += a_tmp[6 * i + nz] * TaskSpaceController_U.dt *
          TaskSpaceController_U.Jbdot_l[6 * kAcol + i];
      }

      T_l += omgtheta_idx_0 * TaskSpaceController_U.qdot[kAcol];
    }

    lambda[nz] = (a_tmp_1[nz] - a_tmp_2[nz]) - T_l;
  }

  for (nz = 0; nz < 6; nz++) {
    T_l = 0.0;
    omgtheta_idx_0 = 0.0;
    for (kAcol = 0; kAcol < 6; kAcol++) {
      i = 6 * kAcol + nz;
      _mm_storeu_pd(&tmp_4[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(TaskKv[i],
        TaskKp_0[i]), _mm_set_pd(lambda[kAcol], lambda_next[kAcol])), _mm_set_pd
        (omgtheta_idx_0, T_l)));
      T_l = tmp_4[0];
      omgtheta_idx_0 = tmp_4[1];
    }

    T_0[nz] = T_l - omgtheta_idx_0;
  }

  for (nz = 0; nz < 6; nz++) {
    T_l = 0.0;
    lambda_next[nz] = 0.0;
    for (kAcol = 0; kAcol < 6; kAcol++) {
      i = 6 * kAcol + nz;
      T_l += JlambdaT[i] * T_0[kAcol];
      lambda_next[nz] += (real_T)lambda_tmp[i] * acosinput *
        TaskSpaceController_U.qdot[kAcol];
      omgtheta_idx_0 = 0.0;
      for (i_0 = 0; i_0 < 6; i_0++) {
        omgtheta_idx_0 += JlambdaT[6 * i_0 + nz] * TaskKv[6 * kAcol + i_0];
      }

      TaskKp[i] = omgtheta_idx_0;
    }

    lambda[nz] = (T_l - TaskSpaceController_U.c[nz]) - lambda_next[nz];
  }

  for (nz = 0; nz <= 34; nz += 2) {
    tmp_3 = _mm_loadu_pd(&TaskKp[nz]);
    _mm_storeu_pd(&JlambdaT[nz], _mm_mul_pd(tmp_3, _mm_set1_pd
      (TaskSpaceController_U.dt)));
  }

  for (nz = 0; nz < 6; nz++) {
    for (kAcol = 0; kAcol < 6; kAcol++) {
      T_l = 0.0;
      for (i = 0; i < 6; i++) {
        T_l += JlambdaT[6 * i + nz] * Jlambda[6 * kAcol + i];
      }

      a_tmp[nz + 6 * kAcol] = TaskSpaceController_U.M[12 * kAcol + nz] + T_l;
    }
  }

  TaskSpaceController_xzgetrf(a_tmp, ipiv, &nz);
  for (nz = 0; nz < 5; nz++) {
    kAcol = ipiv[nz];
    if (nz + 1 != kAcol) {
      acosinput = lambda[nz];
      lambda[nz] = lambda[kAcol - 1];
      lambda[kAcol - 1] = acosinput;
    }
  }

  for (nz = 0; nz < 6; nz++) {
    kAcol = 6 * nz;
    if (lambda[nz] != 0.0) {
      for (i = nz + 2; i < 7; i++) {
        lambda[i - 1] -= a_tmp[(i + kAcol) - 1] * lambda[nz];
      }
    }
  }

  for (nz = 5; nz >= 0; nz--) {
    kAcol = 6 * nz;
    acosinput = lambda[nz];
    if (acosinput != 0.0) {
      lambda[nz] = acosinput / a_tmp[nz + kAcol];
      for (i = 0; i < nz; i++) {
        lambda[i] -= a_tmp[i + kAcol] * lambda[nz];
      }
    }
  }

  for (nz = 0; nz < 6; nz++) {
    acosinput = lambda[nz];
    rtb_qddot_m[nz] = acosinput;
    b_x[nz] = rtIsNaN(acosinput);
  }

  nz = b_x[0];
  for (kAcol = 0; kAcol < 5; kAcol++) {
    nz += b_x[kAcol + 1];
  }

  for (i = 0; i < 6; i++) {
    if (nz > 1) {
      rtb_qddot_m[i] = 0.0;
    }

    T_l = TaskSpaceController_U.qdot[i];
    acosinput = -11.0 * T_l;
    T_0[i] = -2.0 * rtb_qddot_m[i];
    tmp[i] = (TaskSpaceController_U.qdot_max[i] - T_l) * 10.0;
    tmp[i + 6] = -((T_l - TaskSpaceController_U.qdot_min[i]) * -10.0);
    T_l = TaskSpaceController_U.q[i];
    tmp[i + 12] = (TaskSpaceController_U.q_max[i] - T_l) * 10.0 + acosinput;
    tmp[i + 18] = -(acosinput - (T_l - TaskSpaceController_U.q_min[i]) * 10.0);
  }

  TaskSpaceController_quadprog(T_0, tmp, rtb_qddot_m, lambda, &acosinput,
    &omgtheta_idx_0, expl_temp, &d_a, &omgtheta_idx_1, &T_l, &b_lambda);
  if (omgtheta_idx_0 > 0.0) {
    for (nz = 0; nz < 6; nz++) {
      rtb_qddot_m[nz] = lambda[nz];
    }
  } else {
    for (i = 0; i <= 4; i += 2) {
      tmp_3 = _mm_loadu_pd(&TaskSpaceController_U.qdot[i]);
      tmp_0 = _mm_set1_pd(TaskSpaceController_U.dt);
      _mm_storeu_pd(&q[i], _mm_add_pd(_mm_mul_pd(tmp_3, tmp_0), _mm_loadu_pd
        (&TaskSpaceController_U.q[i])));
      tmp_1 = _mm_loadu_pd(&rtb_qddot_m[i]);
      _mm_storeu_pd(&qdot[i], _mm_add_pd(_mm_mul_pd(tmp_1, tmp_0), tmp_3));
    }
  }

  /* MATLAB Function: '<S1>/SPDControllerWithQP_Right1' incorporates:
   *  Inport: '<Root>/Jb_r'
   *  Inport: '<Root>/Jbdot_r'
   *  Inport: '<Root>/M'
   *  Inport: '<Root>/T_des_r'
   *  Inport: '<Root>/T_r'
   *  Inport: '<Root>/TaskKp'
   *  Inport: '<Root>/TaskKv'
   *  Inport: '<Root>/V_des_r'
   *  Inport: '<Root>/V_r'
   *  Inport: '<Root>/Vdot_des_r'
   *  Inport: '<Root>/a'
   *  Inport: '<Root>/b0'
   *  Inport: '<Root>/c'
   *  Inport: '<Root>/dt'
   *  Inport: '<Root>/q'
   *  Inport: '<Root>/q_max'
   *  Inport: '<Root>/q_min'
   *  Inport: '<Root>/qdot'
   *  Inport: '<Root>/qdot_max'
   *  Inport: '<Root>/qdot_min'
   *  MATLAB Function: '<S1>/SPDControllerWithQP_Left1'
   */
  for (i = 0; i < 6; i++) {
    qdot_0[i] = TaskSpaceController_U.qdot[i + 6];
    q_0[i] = TaskSpaceController_U.q[i + 6];
  }

  memset(&TaskKp[0], 0, 36U * sizeof(real_T));
  for (nz = 0; nz < 6; nz++) {
    TaskKp[nz + 6 * nz] = TaskSpaceController_U.TaskKp[nz + 6];
  }

  memset(&TaskKv[0], 0, 36U * sizeof(real_T));
  for (nz = 0; nz < 6; nz++) {
    TaskKv[nz + 6 * nz] = TaskSpaceController_U.TaskKv[nz + 6];
  }

  for (nz = 0; nz < 3; nz++) {
    omgmat[3 * nz] = TaskSpaceController_U.T_r[nz];
    omgmat[3 * nz + 1] = TaskSpaceController_U.T_r[nz + 4];
    omgmat[3 * nz + 2] = TaskSpaceController_U.T_r[nz + 8];
  }

  for (nz = 0; nz <= 6; nz += 2) {
    tmp_3 = _mm_loadu_pd(&omgmat[nz]);
    _mm_storeu_pd(&omgmat_0[nz], _mm_mul_pd(tmp_3, _mm_set1_pd(-1.0)));
  }

  for (nz = 8; nz < 9; nz++) {
    omgmat_0[nz] = -omgmat[nz];
  }

  acosinput = TaskSpaceController_U.T_r[13];
  T_l = TaskSpaceController_U.T_r[12];
  omgtheta_idx_0 = TaskSpaceController_U.T_r[14];
  for (nz = 0; nz < 3; nz++) {
    kAcol = nz << 2;
    omgmat_1[kAcol] = omgmat[3 * nz];
    omgmat_1[kAcol + 1] = omgmat[3 * nz + 1];
    omgmat_1[kAcol + 2] = omgmat[3 * nz + 2];
    omgmat_1[nz + 12] = (omgmat_0[nz + 3] * acosinput + omgmat_0[nz] * T_l) +
      omgmat_0[nz + 6] * omgtheta_idx_0;
  }

  omgmat_1[3] = 0.0;
  omgmat_1[7] = 0.0;
  omgmat_1[11] = 0.0;
  omgmat_1[15] = 1.0;
  for (nz = 0; nz < 4; nz++) {
    i = nz << 2;
    acosinput = TaskSpaceController_U.T_des_r[i + 1];
    T_l = TaskSpaceController_U.T_des_r[i];
    omgtheta_idx_0 = TaskSpaceController_U.T_des_r[i + 2];
    omgtheta_idx_1 = TaskSpaceController_U.T_des_r[i + 3];
    for (kAcol = 0; kAcol <= 2; kAcol += 2) {
      tmp_3 = _mm_loadu_pd(&omgmat_1[kAcol + 4]);
      tmp_0 = _mm_loadu_pd(&omgmat_1[kAcol]);
      tmp_1 = _mm_loadu_pd(&omgmat_1[kAcol + 8]);
      tmp_2 = _mm_loadu_pd(&omgmat_1[kAcol + 12]);
      _mm_storeu_pd(&Ttilde[kAcol + i], _mm_add_pd(_mm_add_pd(_mm_add_pd
        (_mm_mul_pd(_mm_set1_pd(acosinput), tmp_3), _mm_mul_pd(_mm_set1_pd(T_l),
        tmp_0)), _mm_mul_pd(_mm_set1_pd(omgtheta_idx_0), tmp_1)), _mm_mul_pd
        (_mm_set1_pd(omgtheta_idx_1), tmp_2)));
    }
  }

  T_l = (((Ttilde[0] + Ttilde[5]) + Ttilde[10]) - 1.0) / 2.0;
  if (T_l >= 1.0) {
    memset(&omgmat[0], 0, 9U * sizeof(real_T));
  } else if (T_l <= -1.0) {
    if (!(fabs(Ttilde[10] + 1.0) < 2.2204460492503131E-16)) {
      acosinput = 1.0 / sqrt((Ttilde[10] + 1.0) * 2.0);
      tmp_3 = _mm_mul_pd(_mm_set1_pd(acosinput), _mm_loadu_pd(&Ttilde[8]));
      _mm_storeu_pd(&tmp_4[0], tmp_3);
      omgtheta_idx_0 = tmp_4[0];
      omgtheta_idx_1 = tmp_4[1];
      acosinput *= Ttilde[10] + 1.0;
    } else if (!(fabs(Ttilde[5] + 1.0) < 2.2204460492503131E-16)) {
      acosinput = 1.0 / sqrt((Ttilde[5] + 1.0) * 2.0);
      omgtheta_idx_0 = acosinput * Ttilde[4];
      omgtheta_idx_1 = (Ttilde[5] + 1.0) * acosinput;
      acosinput *= Ttilde[6];
    } else {
      acosinput = 1.0 / sqrt((Ttilde[0] + 1.0) * 2.0);
      omgtheta_idx_0 = (Ttilde[0] + 1.0) * acosinput;
      tmp_3 = _mm_mul_pd(_mm_set1_pd(acosinput), _mm_loadu_pd(&Ttilde[1]));
      _mm_storeu_pd(&tmp_4[0], tmp_3);
      omgtheta_idx_1 = tmp_4[0];
      acosinput = tmp_4[1];
    }

    omgtheta_idx_0 *= 3.1415926535897931;
    omgtheta_idx_1 *= 3.1415926535897931;
    acosinput *= 3.1415926535897931;
    omgmat[0] = 0.0;
    omgmat[3] = -acosinput;
    omgmat[6] = omgtheta_idx_1;
    omgmat[1] = acosinput;
    omgmat[4] = 0.0;
    omgmat[7] = -omgtheta_idx_0;
    omgmat[2] = -omgtheta_idx_1;
    omgmat[5] = omgtheta_idx_0;
    omgmat[8] = 0.0;
  } else {
    acosinput = acos(T_l);
    acosinput *= 1.0 / (2.0 * sin(acosinput));
    for (nz = 0; nz < 3; nz++) {
      kAcol = nz << 2;
      tmp_3 = _mm_mul_pd(_mm_sub_pd(_mm_loadu_pd(&Ttilde[kAcol]), _mm_set_pd
        (Ttilde[nz + 4], Ttilde[nz])), _mm_set1_pd(acosinput));
      _mm_storeu_pd(&omgmat[3 * nz], tmp_3);
      omgmat[3 * nz + 2] = (Ttilde[kAcol + 2] - Ttilde[nz + 8]) * acosinput;
    }
  }

  isodd = false;
  b_p = true;
  nz = 0;
  exitg1 = false;
  while ((!exitg1) && (nz < 9)) {
    if (!(omgmat[nz] == 0.0)) {
      b_p = false;
      exitg1 = true;
    } else {
      nz++;
    }
  }

  if (b_p) {
    isodd = true;
  }

  if (isodd) {
    for (nz = 0; nz < 3; nz++) {
      kAcol = nz << 2;
      omgmat_1[kAcol] = 0.0;
      omgmat_1[kAcol + 1] = 0.0;
      omgmat_1[kAcol + 2] = 0.0;
      omgmat_1[nz + 12] = Ttilde[nz + 12];
    }
  } else {
    acosinput = acos(T_l);
    omgtheta_idx_0 = 1.0 / acosinput - 1.0 / tan(acosinput / 2.0) / 2.0;
    for (nz = 0; nz < 9; nz++) {
      b_I[nz] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (nz = 0; nz < 3; nz++) {
      T_l = 0.0;
      for (kAcol = 0; kAcol < 3; kAcol++) {
        i = 3 * kAcol + nz;
        T_l += (((omgmat[nz + 3] * omgtheta_idx_0 * omgmat[3 * kAcol + 1] +
                  omgtheta_idx_0 * omgmat[nz] * omgmat[3 * kAcol]) + omgmat[nz +
                 6] * omgtheta_idx_0 * omgmat[3 * kAcol + 2]) / acosinput +
                ((real_T)b_I[i] - omgmat[i] / 2.0)) * Ttilde[kAcol + 12];
        omgmat_1[kAcol + (nz << 2)] = omgmat[3 * nz + kAcol];
      }

      omgmat_1[nz + 12] = T_l;
    }
  }

  lambda[0] = omgmat_1[12];
  lambda[1] = omgmat_1[13];
  lambda[2] = omgmat_1[14];
  lambda[3] = omgmat_1[6];
  lambda[4] = omgmat_1[8];
  lambda[5] = omgmat_1[1];
  TaskSpaceController_dlog6(lambda, a_tmp);
  for (nz = 0; nz <= 34; nz += 2) {
    tmp_3 = _mm_loadu_pd(&a_tmp[nz]);
    _mm_storeu_pd(&a_tmp_0[nz], _mm_mul_pd(tmp_3, _mm_set1_pd(-1.0)));
  }

  for (nz = 0; nz < 6; nz++) {
    for (kAcol = 0; kAcol < 6; kAcol++) {
      acosinput = 0.0;
      for (i = 0; i < 6; i++) {
        acosinput += a_tmp_0[6 * i + kAcol] * TaskSpaceController_U.Jb_r[6 * nz
          + i];
      }

      Jlambda[kAcol + 6 * nz] = acosinput;
    }
  }

  for (nz = 0; nz < 3; nz++) {
    omgmat[3 * nz] = Ttilde[nz];
    omgmat[3 * nz + 1] = Ttilde[nz + 4];
    omgmat[3 * nz + 2] = Ttilde[nz + 8];
  }

  for (nz = 0; nz <= 6; nz += 2) {
    tmp_3 = _mm_loadu_pd(&omgmat[nz]);
    _mm_storeu_pd(&omgmat_0[nz], _mm_mul_pd(tmp_3, _mm_set1_pd(-1.0)));
  }

  for (nz = 8; nz < 9; nz++) {
    omgmat_0[nz] = -omgmat[nz];
  }

  acosinput = Ttilde[13];
  T_l = Ttilde[12];
  omgtheta_idx_0 = Ttilde[14];
  for (nz = 0; nz < 3; nz++) {
    kAcol = nz << 2;
    Ttilde[kAcol] = omgmat[3 * nz];
    Ttilde[kAcol + 1] = omgmat[3 * nz + 1];
    Ttilde[kAcol + 2] = omgmat[3 * nz + 2];
    Ttilde[nz + 12] = (omgmat_0[nz + 3] * acosinput + omgmat_0[nz] * T_l) +
      omgmat_0[nz + 6] * omgtheta_idx_0;
  }

  Ttilde[3] = 0.0;
  Ttilde[7] = 0.0;
  Ttilde[11] = 0.0;
  Ttilde[15] = 1.0;
  omgmat[0] = 0.0;
  omgmat[3] = -Ttilde[14];
  omgmat[6] = Ttilde[13];
  omgmat[1] = Ttilde[14];
  omgmat[4] = 0.0;
  omgmat[7] = -Ttilde[12];
  omgmat[2] = -Ttilde[13];
  omgmat[5] = Ttilde[12];
  omgmat[8] = 0.0;
  for (nz = 0; nz < 3; nz++) {
    T_l = omgmat[nz + 3];
    omgtheta_idx_0 = omgmat[nz];
    omgtheta_idx_1 = omgmat[nz + 6];
    for (kAcol = 0; kAcol < 3; kAcol++) {
      i = kAcol << 2;
      omgmat_0[nz + 3 * kAcol] = (Ttilde[i + 1] * T_l + Ttilde[i] *
        omgtheta_idx_0) + Ttilde[i + 2] * omgtheta_idx_1;
      T[kAcol + 6 * nz] = Ttilde[(nz << 2) + kAcol];
    }
  }

  for (nz = 0; nz < 3; nz++) {
    kAcol = (nz + 3) * 6;
    T[kAcol] = omgmat_0[3 * nz];
    T[6 * nz + 3] = 0.0;
    i = nz << 2;
    T[kAcol + 3] = Ttilde[i];
    T[kAcol + 1] = omgmat_0[3 * nz + 1];
    T[6 * nz + 4] = 0.0;
    T[kAcol + 4] = Ttilde[i + 1];
    T[kAcol + 2] = omgmat_0[3 * nz + 2];
    T[6 * nz + 5] = 0.0;
    T[kAcol + 5] = Ttilde[i + 2];
  }

  for (nz = 0; nz < 6; nz++) {
    T_l = 0.0;
    for (kAcol = 0; kAcol < 6; kAcol++) {
      T_l += T[6 * kAcol + nz] * TaskSpaceController_U.V_des_r[kAcol];
    }

    T_0[nz] = T_l - TaskSpaceController_U.V_r[nz];
  }

  for (nz = 0; nz < 6; nz++) {
    omgtheta_idx_0 = 0.0;
    for (kAcol = 0; kAcol < 6; kAcol++) {
      omgtheta_idx_0 += a_tmp[6 * kAcol + nz] * T_0[kAcol];
    }

    lambda_next[nz] = omgtheta_idx_0 * TaskSpaceController_U.dt + lambda[nz];
  }

  memcpy(&a_tmp[0], &TaskSpaceController_U.Jb_r[0], 36U * sizeof(real_T));
  TaskSpaceController_xzgetrf(a_tmp, ipiv, &nz);
  acosinput = a_tmp[0];
  isodd = false;
  for (nz = 0; nz < 5; nz++) {
    acosinput *= a_tmp[((nz + 1) * 6 + nz) + 1];
    if (ipiv[nz] > nz + 1) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    acosinput = -acosinput;
  }

  memcpy(&a_tmp[0], &TaskSpaceController_U.Jb_r[0], 36U * sizeof(real_T));
  TaskSpaceController_xzgetrf(a_tmp, ipiv, &nz);
  omgtheta_idx_0 = a_tmp[0];
  isodd = false;
  for (nz = 0; nz < 5; nz++) {
    omgtheta_idx_0 *= a_tmp[((nz + 1) * 6 + nz) + 1];
    if (ipiv[nz] > nz + 1) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    omgtheta_idx_0 = -omgtheta_idx_0;
  }

  acosinput = exp(-acosinput * omgtheta_idx_0 / TaskSpaceController_U.a[1] /
                  TaskSpaceController_U.a[1]) * TaskSpaceController_U.b0[1];
  TaskSpaceController_dlog6(lambda_next, a_tmp);
  for (i = 0; i < 6; i++) {
    for (nz = 0; nz < 6; nz++) {
      JlambdaT[nz + 6 * i] = Jlambda[6 * nz + i];
    }

    lambda[i] = -lambda_next[i];
  }

  omgmat_1[0] = 0.0;
  omgmat_1[4] = -lambda[5];
  omgmat_1[8] = lambda[4];
  omgmat_1[1] = lambda[5];
  omgmat_1[5] = 0.0;
  omgmat_1[9] = -lambda[3];
  omgmat_1[2] = -lambda[4];
  omgmat_1[6] = lambda[3];
  omgmat_1[10] = 0.0;
  omgmat_1[12] = lambda[0];
  omgmat_1[13] = lambda[1];
  omgmat_1[14] = lambda[2];
  omgmat_1[3] = 0.0;
  omgmat_1[7] = 0.0;
  omgmat_1[11] = 0.0;
  omgmat_1[15] = 0.0;
  se3mat[0] = lambda[3];
  se3mat[1] = lambda[4];
  se3mat[2] = lambda[5];
  omgtheta_idx_0 = TaskSpaceController_norm(se3mat);
  if (fabs(omgtheta_idx_0) < 2.2204460492503131E-16) {
    for (nz = 0; nz < 9; nz++) {
      b_I[nz] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (nz = 0; nz < 3; nz++) {
      kAcol = nz << 2;
      Ttilde[kAcol] = b_I[3 * nz];
      Ttilde[kAcol + 1] = b_I[3 * nz + 1];
      Ttilde[kAcol + 2] = b_I[3 * nz + 2];
      Ttilde[nz + 12] = omgmat_1[nz + 12];
    }

    Ttilde[3] = 0.0;
    Ttilde[7] = 0.0;
    Ttilde[11] = 0.0;
    Ttilde[15] = 1.0;
  } else {
    for (nz = 0; nz < 3; nz++) {
      kAcol = nz << 2;
      tmp_3 = _mm_div_pd(_mm_loadu_pd(&omgmat_1[kAcol]), _mm_set1_pd
                         (omgtheta_idx_0));
      _mm_storeu_pd(&omgmat[3 * nz], tmp_3);
      omgmat[3 * nz + 2] = omgmat_1[kAcol + 2] / omgtheta_idx_0;
    }

    omgtheta_idx_1 = cos(omgtheta_idx_0);
    T_l = sin(omgtheta_idx_0);
    d_a = omgtheta_idx_0 - T_l;
    for (nz = 0; nz < 9; nz++) {
      b_I[nz] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (nz = 0; nz < 3; nz++) {
      for (kAcol = 0; kAcol < 3; kAcol++) {
        i = 3 * kAcol + nz;
        omgmat_0[i] = (((1.0 - omgtheta_idx_1) * omgmat[nz] * omgmat[3 * kAcol]
                        + (1.0 - omgtheta_idx_1) * omgmat[nz + 3] * omgmat[3 *
                        kAcol + 1]) + (1.0 - omgtheta_idx_1) * omgmat[nz + 6] *
                       omgmat[3 * kAcol + 2]) + (omgmat[i] * T_l + (real_T)b_I[i]);
      }
    }

    for (nz = 0; nz < 3; nz++) {
      T_l = 0.0;
      for (kAcol = 0; kAcol < 3; kAcol++) {
        i = 3 * kAcol + nz;
        T_l += (((omgmat[nz + 3] * d_a * omgmat[3 * kAcol + 1] + d_a * omgmat[nz]
                  * omgmat[3 * kAcol]) + omgmat[nz + 6] * d_a * omgmat[3 * kAcol
                 + 2]) + ((1.0 - omgtheta_idx_1) * omgmat[i] + (real_T)j_a[i] *
                          omgtheta_idx_0)) * omgmat_1[kAcol + 12];
        Ttilde[kAcol + (nz << 2)] = omgmat_0[3 * nz + kAcol];
      }

      Ttilde[nz + 12] = T_l / omgtheta_idx_0;
    }

    Ttilde[3] = 0.0;
    Ttilde[7] = 0.0;
    Ttilde[11] = 0.0;
    Ttilde[15] = 1.0;
  }

  for (nz = 0; nz <= 34; nz += 2) {
    tmp_3 = _mm_loadu_pd(&TaskKp[nz]);
    _mm_storeu_pd(&TaskKp_0[nz], _mm_mul_pd(tmp_3, _mm_set1_pd(-1.0)));
  }

  omgmat[0] = 0.0;
  omgmat[3] = -Ttilde[14];
  omgmat[6] = Ttilde[13];
  omgmat[1] = Ttilde[14];
  omgmat[4] = 0.0;
  omgmat[7] = -Ttilde[12];
  omgmat[2] = -Ttilde[13];
  omgmat[5] = Ttilde[12];
  omgmat[8] = 0.0;
  for (nz = 0; nz < 3; nz++) {
    T_l = omgmat[nz + 3];
    omgtheta_idx_0 = omgmat[nz];
    omgtheta_idx_1 = omgmat[nz + 6];
    for (kAcol = 0; kAcol < 3; kAcol++) {
      i = kAcol << 2;
      omgmat_0[nz + 3 * kAcol] = (Ttilde[i + 1] * T_l + Ttilde[i] *
        omgtheta_idx_0) + Ttilde[i + 2] * omgtheta_idx_1;
      T[kAcol + 6 * nz] = Ttilde[(nz << 2) + kAcol];
    }
  }

  for (nz = 0; nz < 3; nz++) {
    kAcol = (nz + 3) * 6;
    T[kAcol] = omgmat_0[3 * nz];
    T[6 * nz + 3] = 0.0;
    i = nz << 2;
    T[kAcol + 3] = Ttilde[i];
    T[kAcol + 1] = omgmat_0[3 * nz + 1];
    T[6 * nz + 4] = 0.0;
    T[kAcol + 4] = Ttilde[i + 1];
    T[kAcol + 2] = omgmat_0[3 * nz + 2];
    T[6 * nz + 5] = 0.0;
    T[kAcol + 5] = Ttilde[i + 2];
  }

  for (nz = 0; nz < 6; nz++) {
    for (kAcol = 0; kAcol < 6; kAcol++) {
      omgtheta_idx_0 = 0.0;
      for (i = 0; i < 6; i++) {
        omgtheta_idx_0 += a_tmp[6 * i + nz] * T[6 * kAcol + i];
      }

      a_tmp_0[nz + 6 * kAcol] = omgtheta_idx_0;
    }

    T_0[nz] = TaskSpaceController_U.dt * TaskSpaceController_U.Vdot_des_r[nz] +
      TaskSpaceController_U.V_des_r[nz];
  }

  for (nz = 0; nz < 6; nz++) {
    a_tmp_1[nz] = 0.0;
    a_tmp_2[nz] = 0.0;
    T_l = 0.0;
    for (kAcol = 0; kAcol < 6; kAcol++) {
      i = 6 * kAcol + nz;
      _mm_storeu_pd(&tmp_4[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(a_tmp[i],
        a_tmp_0[i]), _mm_set_pd(TaskSpaceController_U.V_r[kAcol], T_0[kAcol])),
        _mm_set_pd(a_tmp_2[nz], a_tmp_1[nz])));
      a_tmp_1[nz] = tmp_4[0];
      a_tmp_2[nz] = tmp_4[1];
      omgtheta_idx_0 = 0.0;
      for (i = 0; i < 6; i++) {
        omgtheta_idx_0 += a_tmp[6 * i + nz] * TaskSpaceController_U.dt *
          TaskSpaceController_U.Jbdot_r[6 * kAcol + i];
      }

      T_l += TaskSpaceController_U.qdot[kAcol + 6] * omgtheta_idx_0;
    }

    lambda[nz] = (a_tmp_1[nz] - a_tmp_2[nz]) - T_l;
  }

  for (nz = 0; nz < 6; nz++) {
    T_l = 0.0;
    omgtheta_idx_0 = 0.0;
    for (kAcol = 0; kAcol < 6; kAcol++) {
      i = 6 * kAcol + nz;
      _mm_storeu_pd(&tmp_4[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(TaskKv[i],
        TaskKp_0[i]), _mm_set_pd(lambda[kAcol], lambda_next[kAcol])), _mm_set_pd
        (omgtheta_idx_0, T_l)));
      T_l = tmp_4[0];
      omgtheta_idx_0 = tmp_4[1];
    }

    T_0[nz] = T_l - omgtheta_idx_0;
  }

  for (nz = 0; nz < 6; nz++) {
    T_l = 0.0;
    lambda_next[nz] = 0.0;
    for (kAcol = 0; kAcol < 6; kAcol++) {
      i = 6 * kAcol + nz;
      T_l += JlambdaT[i] * T_0[kAcol];
      lambda_next[nz] += (real_T)lambda_tmp[i] * acosinput *
        TaskSpaceController_U.qdot[kAcol + 6];
      omgtheta_idx_0 = 0.0;
      for (i_0 = 0; i_0 < 6; i_0++) {
        omgtheta_idx_0 += JlambdaT[6 * i_0 + nz] * TaskKv[6 * kAcol + i_0];
      }

      TaskKp[i] = omgtheta_idx_0;
    }

    lambda[nz] = (T_l - TaskSpaceController_U.c[nz + 6]) - lambda_next[nz];
  }

  for (nz = 0; nz <= 34; nz += 2) {
    tmp_3 = _mm_loadu_pd(&TaskKp[nz]);
    _mm_storeu_pd(&JlambdaT[nz], _mm_mul_pd(tmp_3, _mm_set1_pd
      (TaskSpaceController_U.dt)));
  }

  for (nz = 0; nz < 6; nz++) {
    for (kAcol = 0; kAcol < 6; kAcol++) {
      T_l = 0.0;
      for (i = 0; i < 6; i++) {
        T_l += JlambdaT[6 * i + nz] * Jlambda[6 * kAcol + i];
      }

      a_tmp[nz + 6 * kAcol] = TaskSpaceController_U.M[((kAcol + 6) * 12 + nz) +
        6] + T_l;
    }
  }

  TaskSpaceController_xzgetrf(a_tmp, ipiv, &nz);
  for (nz = 0; nz < 5; nz++) {
    kAcol = ipiv[nz];
    if (nz + 1 != kAcol) {
      acosinput = lambda[nz];
      lambda[nz] = lambda[kAcol - 1];
      lambda[kAcol - 1] = acosinput;
    }
  }

  for (nz = 0; nz < 6; nz++) {
    kAcol = 6 * nz;
    if (lambda[nz] != 0.0) {
      for (i = nz + 2; i < 7; i++) {
        lambda[i - 1] -= a_tmp[(i + kAcol) - 1] * lambda[nz];
      }
    }
  }

  for (nz = 5; nz >= 0; nz--) {
    kAcol = 6 * nz;
    acosinput = lambda[nz];
    if (acosinput != 0.0) {
      lambda[nz] = acosinput / a_tmp[nz + kAcol];
      for (i = 0; i < nz; i++) {
        lambda[i] -= a_tmp[i + kAcol] * lambda[nz];
      }
    }
  }

  for (nz = 0; nz < 6; nz++) {
    acosinput = lambda[nz];
    lambda_next[nz] = acosinput;
    b_x[nz] = rtIsNaN(acosinput);
  }

  nz = b_x[0];
  for (kAcol = 0; kAcol < 5; kAcol++) {
    nz += b_x[kAcol + 1];
  }

  for (i = 0; i < 6; i++) {
    if (nz > 1) {
      lambda_next[i] = 0.0;
    }

    T_l = TaskSpaceController_U.qdot[i + 6];
    acosinput = -11.0 * T_l;
    T_0[i] = -2.0 * lambda_next[i];
    tmp[i] = (TaskSpaceController_U.qdot_max[i + 6] - T_l) * 10.0;
    tmp[i + 6] = -((T_l - TaskSpaceController_U.qdot_min[i + 6]) * -10.0);
    T_l = TaskSpaceController_U.q[i + 6];
    tmp[i + 12] = (TaskSpaceController_U.q_max[i + 6] - T_l) * 10.0 + acosinput;
    tmp[i + 18] = -(acosinput - (T_l - TaskSpaceController_U.q_min[i + 6]) *
                    10.0);
  }

  TaskSpaceController_quadprog(T_0, tmp, lambda_next, lambda, &acosinput,
    &omgtheta_idx_0, expl_temp, &d_a, &omgtheta_idx_1, &T_l, &b_lambda);
  if (omgtheta_idx_0 > 0.0) {
    for (nz = 0; nz < 6; nz++) {
      lambda_next[nz] = lambda[nz];
    }
  } else {
    for (i = 0; i <= 4; i += 2) {
      tmp_3 = _mm_loadu_pd(&TaskSpaceController_U.qdot[i + 6]);
      tmp_0 = _mm_set1_pd(TaskSpaceController_U.dt);
      _mm_storeu_pd(&q_0[i], _mm_add_pd(_mm_loadu_pd(&TaskSpaceController_U.q[i
        + 6]), _mm_mul_pd(tmp_3, tmp_0)));
      tmp_1 = _mm_loadu_pd(&lambda_next[i]);
      _mm_storeu_pd(&qdot_0[i], _mm_add_pd(_mm_mul_pd(tmp_1, tmp_0), tmp_3));
    }
  }

  /* MATLAB Function: '<S1>/HinfController' incorporates:
   *  Delay: '<S1>/Delay'
   *  Inport: '<Root>/HinfK'
   *  Inport: '<Root>/dt'
   *  Inport: '<Root>/q'
   *  Inport: '<Root>/qdot'
   *  SignalConversion generated from: '<S2>/ SFunction '
   */
  for (i = 0; i <= 4; i += 2) {
    tmp_3 = _mm_loadu_pd(&q[i]);
    _mm_storeu_pd(&e[i], _mm_sub_pd(tmp_3, _mm_loadu_pd
      (&TaskSpaceController_U.q[i])));
    tmp_3 = _mm_loadu_pd(&q_0[i]);
    _mm_storeu_pd(&e[i + 6], _mm_sub_pd(tmp_3, _mm_loadu_pd
      (&TaskSpaceController_U.q[i + 6])));
    tmp_3 = _mm_loadu_pd(&qdot[i]);
    _mm_storeu_pd(&edot[i], _mm_sub_pd(tmp_3, _mm_loadu_pd
      (&TaskSpaceController_U.qdot[i])));
    tmp_3 = _mm_loadu_pd(&qdot_0[i]);
    _mm_storeu_pd(&edot[i + 6], _mm_sub_pd(tmp_3, _mm_loadu_pd
      (&TaskSpaceController_U.qdot[i + 6])));
  }

  for (nz = 0; nz <= 10; nz += 2) {
    tmp_3 = _mm_loadu_pd(&e[nz]);
    tmp_0 = _mm_loadu_pd(&TaskSpaceController_DW.Delay_DSTATE[nz]);
    _mm_storeu_pd(&rtb_eint[nz], _mm_add_pd(_mm_mul_pd(tmp_3, _mm_set1_pd
      (TaskSpaceController_U.dt)), tmp_0));
  }

  memset(&a[0], 0, 144U * sizeof(real_T));
  for (nz = 0; nz < 12; nz++) {
    a[nz + 12 * nz] = TaskSpaceController_U.HinfK[nz];
  }

  for (nz = 0; nz < 6; nz++) {
    /* SignalConversion generated from: '<S2>/ SFunction ' incorporates:
     *  MATLAB Function: '<S1>/HinfController'
     */
    rtb_qddot_b[nz] = rtb_qddot_m[nz];
    rtb_qddot_b[nz + 6] = lambda_next[nz];
  }

  /* MATLAB Function: '<S1>/HinfController' incorporates:
   *  Inport: '<Root>/M'
   */
  for (nz = 0; nz <= 10; nz += 2) {
    tmp_3 = _mm_loadu_pd(&e[nz]);
    tmp_0 = _mm_set1_pd(100.0);
    tmp_1 = _mm_loadu_pd(&rtb_qddot_b[nz]);
    tmp_2 = _mm_loadu_pd(&edot[nz]);
    tmp_5 = _mm_set1_pd(20.0);
    _mm_storeu_pd(&rtb_qddot_b_0[nz], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_0,
      tmp_3), tmp_1), _mm_mul_pd(tmp_5, tmp_2)));
    tmp_1 = _mm_loadu_pd(&rtb_eint[nz]);
    _mm_storeu_pd(&edot_0[nz], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_5, tmp_3),
      tmp_2), _mm_mul_pd(tmp_0, tmp_1)));
  }

  for (i = 0; i < 12; i++) {
    T_l = 0.0;
    acosinput = 0.0;
    for (nz = 0; nz < 12; nz++) {
      kAcol = 12 * nz + i;
      _mm_storeu_pd(&tmp_4[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(a[kAcol],
        TaskSpaceController_U.M[kAcol]), _mm_set_pd(edot_0[nz], rtb_qddot_b_0[nz])),
        _mm_set_pd(acosinput, T_l)));
      T_l = tmp_4[0];
      acosinput = tmp_4[1];
    }

    /* Outport: '<Root>/tau' incorporates:
     *  Inport: '<Root>/M'
     *  Inport: '<Root>/c'
     *  Inport: '<Root>/g'
     */
    TaskSpaceController_Y.tau[i] = ((T_l + TaskSpaceController_U.c[i]) +
      TaskSpaceController_U.g[i]) + acosinput;

    /* Update for Delay: '<S1>/Delay' */
    TaskSpaceController_DW.Delay_DSTATE[i] = rtb_eint[i];
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(TaskSpaceController_M->rtwLogInfo,
                      (&TaskSpaceController_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.001s, 0.0s] */
    if ((rtmGetTFinal(TaskSpaceController_M)!=-1) &&
        !((rtmGetTFinal(TaskSpaceController_M)-
           TaskSpaceController_M->Timing.taskTime0) >
          TaskSpaceController_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(TaskSpaceController_M, "Simulation finished");
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
  if (!(++TaskSpaceController_M->Timing.clockTick0)) {
    ++TaskSpaceController_M->Timing.clockTickH0;
  }

  TaskSpaceController_M->Timing.taskTime0 =
    TaskSpaceController_M->Timing.clockTick0 *
    TaskSpaceController_M->Timing.stepSize0 +
    TaskSpaceController_M->Timing.clockTickH0 *
    TaskSpaceController_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void TaskSpaceController_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)TaskSpaceController_M, 0,
                sizeof(RT_MODEL_TaskSpaceController_T));
  rtmSetTFinal(TaskSpaceController_M, -1);
  TaskSpaceController_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    TaskSpaceController_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(TaskSpaceController_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(TaskSpaceController_M->rtwLogInfo, (NULL));
    rtliSetLogT(TaskSpaceController_M->rtwLogInfo, "tout");
    rtliSetLogX(TaskSpaceController_M->rtwLogInfo, "");
    rtliSetLogXFinal(TaskSpaceController_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(TaskSpaceController_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(TaskSpaceController_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(TaskSpaceController_M->rtwLogInfo, 0);
    rtliSetLogDecimation(TaskSpaceController_M->rtwLogInfo, 1);
    rtliSetLogY(TaskSpaceController_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(TaskSpaceController_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(TaskSpaceController_M->rtwLogInfo, (NULL));
  }

  /* states (dwork) */
  (void) memset((void *)&TaskSpaceController_DW, 0,
                sizeof(DW_TaskSpaceController_T));

  /* external inputs */
  (void)memset(&TaskSpaceController_U, 0, sizeof(ExtU_TaskSpaceController_T));

  /* external outputs */
  (void)memset(&TaskSpaceController_Y, 0, sizeof(ExtY_TaskSpaceController_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(TaskSpaceController_M->rtwLogInfo, 0.0,
    rtmGetTFinal(TaskSpaceController_M), TaskSpaceController_M->Timing.stepSize0,
    (&rtmGetErrorStatus(TaskSpaceController_M)));

  {
    int32_T i;

    /* InitializeConditions for Delay: '<S1>/Delay' */
    for (i = 0; i < 12; i++) {
      TaskSpaceController_DW.Delay_DSTATE[i] =
        TaskSpaceController_P.Delay_InitialCondition;
    }

    /* End of InitializeConditions for Delay: '<S1>/Delay' */
  }
}

/* Model terminate function */
void TaskSpaceController_terminate(void)
{
  /* (no terminate code required) */
}
