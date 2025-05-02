/*
 * FK.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "FK".
 *
 * Model version              : 1.8
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Fri May  2 20:22:47 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "FK.h"
#include "rtwtypes.h"
#include "FK_types.h"
#include "FK_private.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

/* Block signals (default storage) */
B_FK_T FK_B;

/* Block states (default storage) */
DW_FK_T FK_DW;

/* External inputs (root inport signals with default storage) */
ExtU_FK_T FK_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_FK_T FK_Y;

/* Real-time model */
static RT_MODEL_FK_T FK_M_;
RT_MODEL_FK_T *const FK_M = &FK_M_;

/* Forward declaration for local functions */
static real_T FK_norm(const real_T x[3]);

/* Forward declaration for local functions */
static void emxInit_f_robotics_manip_intern(emxArray_f_robotics_manip_int_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_g_robotics_manip_(g_robotics_manip_internal_Col_T
  *pStruct);
static void emxInitStruct_e_robotics_manip_(e_robotics_manip_internal_R_m_T
  *pStruct);
static void emxInitMatrix_e_robotics_manip_(e_robotics_manip_internal_R_m_T
  pMatrix[34]);
static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_R_m_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_b_m_T
  *pStruct);
static void FK_rand_mlc3(real_T r[5]);
static void rigidBodyJoint_set_MotionSubspa(b_rigidBodyJoint_FK_T *obj, const
  real_T msubspace_data[]);
static void emxEnsureCapacity_f_robotics_ma(emxArray_f_robotics_manip_int_T
  *emxArray, int32_T oldNumel);
static g_robotics_manip_internal_Col_T *FK_CollisionSet_CollisionSet
  (g_robotics_manip_internal_Col_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_k
  (e_robotics_manip_internal_R_m_T *obj, const char_T bodyInput[10]);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_d
  (e_robotics_manip_internal_R_m_T *obj, const char_T bodyInput[11]);
static void rigidBodyJoint_get_MotionSubspa(const b_rigidBodyJoint_FK_T *obj,
  real_T msubspace_data[], int32_T msubspace_size[2]);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_ov
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_ad
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_kz
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_f3
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_fo
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_g3
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_ns
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_ng
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_c4
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_lr
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_aw
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_jo
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_d2
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_fj
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_ke
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_bj
  (e_robotics_manip_internal_R_m_T *obj);
static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_op
  (e_robotics_manip_internal_R_m_T *obj);
static void GetTransformBlock_setupImpl_ml(robotics_slmanip_internal_b_m_T *obj);
static void FK_rand_mlc(real_T r[5]);
static void F_GetTransformBlock_setupImpl_m(robotics_slmanip_internal_b_m_T *obj);
static void emxInitStruct_e_robotics_mani_m(e_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitMatrix_e_robotics_mani_m(e_robotics_manip_internal_Rig_T
  pMatrix[34]);
static void emxInitStruct_f_robotics_mani_m(f_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmani_m(robotics_slmanip_internal_blo_T
  *pStruct);
static void FK_rand(real_T r[5]);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody
  (e_robotics_manip_internal_Rig_T *obj, const char_T bodyInput[10]);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_m
  (e_robotics_manip_internal_Rig_T *obj, const char_T bodyInput[11]);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_ml
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc3
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc3p
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc3pz
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc3pz0
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc3pz0z
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *F_RigidBody_RigidBody_mlc3pz0z4
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_RigidBody_mlc3pz0z4i
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_RigidBody_mlc3pz0z4in
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_RigidBod_mlc3pz0z4in4
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_RigidBo_mlc3pz0z4in4a
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_RigidB_mlc3pz0z4in4ay
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_Rigid_mlc3pz0z4in4ayx
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_i
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_c
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_b
  (e_robotics_manip_internal_Rig_T *obj);
static void FK_GetJacobianBlock_setupImpl(robotics_slmanip_internal_blo_T *obj);
static void FK_rand_m(real_T r[5]);
static void FK_GetJacobianBlock_setupImpl_m(robotics_slmanip_internal_blo_T *obj);
static void FK_rand_ml(real_T r[5]);
static void FK_GetTransformBlock_setupImpl(robotics_slmanip_internal_b_m_T *obj);
static void FK_rigidBodyJoint_get_JointAxis(const b_rigidBodyJoint_FK_T *obj,
  real_T ax[3]);
static void FK_cat(real_T varargin_1, real_T varargin_2, real_T varargin_3,
                   real_T varargin_4, real_T varargin_5, real_T varargin_6,
                   real_T varargin_7, real_T varargin_8, real_T varargin_9,
                   real_T y[9]);
static void RigidBodyTree_forwardKinemat_ml(f_robotics_manip_internal_R_m_T *obj,
  const real_T qvec[12], h_cell_wrap_FK_T Ttree_data[], int32_T Ttree_size[2]);
static void RigidBodyTree_forwardKinematics(f_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[12], h_cell_wrap_FK_T Ttree_data[], int32_T Ttree_size[2]);
static void emxFree_f_robotics_manip_intern(emxArray_f_robotics_manip_int_T
  **pEmxArray);
static void emxFreeStruct_g_robotics_manip_(g_robotics_manip_internal_Col_T
  *pStruct);
static void emxFreeStruct_e_robotics_manip_(e_robotics_manip_internal_R_m_T
  *pStruct);
static void emxFreeMatrix_e_robotics_manip_(e_robotics_manip_internal_R_m_T
  pMatrix[34]);
static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_R_m_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_b_m_T
  *pStruct);
static void emxFreeStruct_e_robotics_mani_m(e_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeMatrix_e_robotics_mani_m(e_robotics_manip_internal_Rig_T
  pMatrix[34]);
static void emxFreeStruct_f_robotics_mani_m(f_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_m(robotics_slmanip_internal_blo_T
  *pStruct);

/*
 * Output and update for atomic system:
 *    '<S1>/MATLAB Function1'
 *    '<S1>/MATLAB Function2'
 */
void FK_MATLABFunction1(const real_T rtu_q[12], B_MATLABFunction1_FK_T *localB)
{
  real_T x[6];
  int32_T i;
  for (i = 0; i < 6; i++) {
    x[i] = rtu_q[i];
  }

  real_T tmp;
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
    __m128d tmp_0;
    tmp_0 = _mm_loadu_pd(&x[i]);
    _mm_storeu_pd(&localB->q_lr[i], _mm_mul_pd(tmp_0, _mm_set1_pd(-1.0)));
    tmp_0 = _mm_loadu_pd(&rtu_q[i + 6]);
    _mm_storeu_pd(&localB->q_lr[i + 6], tmp_0);
  }
}

/*
 * Output and update for atomic system:
 *    '<S1>/MATLAB Function5'
 *    '<S1>/MATLAB Function6'
 */
void FK_MATLABFunction5(const real_T rtu_Jb[36], const real_T rtu_qdot[6],
  B_MATLABFunction5_FK_T *localB)
{
  real_T omgmat_0[36];
  real_T omgmat[9];
  real_T a[6];
  real_T dJidt[6];
  int32_T i;
  int32_T i_0;
  int32_T j;
  int32_T omgmat_tmp;
  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      dJidt[i_0] = 0.0;
    }

    for (j = 0; j < 6; j++) {
      real_T rtu_Jb_0;
      for (i_0 = 0; i_0 < 6; i_0++) {
        a[i_0] = 0.0;
      }

      if (i < j) {
        real_T rtu_Jb_1;
        omgmat[0] = 0.0;
        rtu_Jb_0 = rtu_Jb[6 * i + 5];
        omgmat[3] = -rtu_Jb_0;
        rtu_Jb_1 = rtu_Jb[6 * i + 4];
        omgmat[6] = rtu_Jb_1;
        omgmat[1] = rtu_Jb_0;
        omgmat[4] = 0.0;
        rtu_Jb_0 = rtu_Jb[6 * i + 3];
        omgmat[7] = -rtu_Jb_0;
        omgmat[2] = -rtu_Jb_1;
        omgmat[5] = rtu_Jb_0;
        omgmat[8] = 0.0;
        omgmat_0[18] = 0.0;
        rtu_Jb_0 = rtu_Jb[6 * i + 2];
        omgmat_0[24] = -rtu_Jb_0;
        rtu_Jb_1 = rtu_Jb[6 * i + 1];
        omgmat_0[30] = rtu_Jb_1;
        omgmat_0[19] = rtu_Jb_0;
        omgmat_0[25] = 0.0;
        rtu_Jb_0 = rtu_Jb[6 * i];
        omgmat_0[31] = -rtu_Jb_0;
        omgmat_0[20] = -rtu_Jb_1;
        omgmat_0[26] = rtu_Jb_0;
        omgmat_0[32] = 0.0;
        for (i_0 = 0; i_0 < 3; i_0++) {
          rtu_Jb_0 = omgmat[3 * i_0];
          omgmat_0[6 * i_0] = rtu_Jb_0;
          omgmat_0[6 * i_0 + 3] = 0.0;
          omgmat_tmp = (i_0 + 3) * 6;
          omgmat_0[omgmat_tmp + 3] = rtu_Jb_0;
          rtu_Jb_0 = omgmat[3 * i_0 + 1];
          omgmat_0[6 * i_0 + 1] = rtu_Jb_0;
          omgmat_0[6 * i_0 + 4] = 0.0;
          omgmat_0[omgmat_tmp + 4] = rtu_Jb_0;
          rtu_Jb_0 = omgmat[3 * i_0 + 2];
          omgmat_0[6 * i_0 + 2] = rtu_Jb_0;
          omgmat_0[6 * i_0 + 5] = 0.0;
          omgmat_0[omgmat_tmp + 5] = rtu_Jb_0;
        }

        for (i_0 = 0; i_0 < 6; i_0++) {
          rtu_Jb_0 = 0.0;
          for (omgmat_tmp = 0; omgmat_tmp < 6; omgmat_tmp++) {
            rtu_Jb_0 += omgmat_0[6 * omgmat_tmp + i_0] * rtu_Jb[6 * j +
              omgmat_tmp];
          }

          a[i_0] = rtu_Jb_0;
        }
      }

      rtu_Jb_0 = rtu_qdot[j];
      for (i_0 = 0; i_0 <= 4; i_0 += 2) {
        __m128d tmp;
        __m128d tmp_0;
        tmp = _mm_loadu_pd(&a[i_0]);
        tmp_0 = _mm_loadu_pd(&dJidt[i_0]);
        _mm_storeu_pd(&dJidt[i_0], _mm_add_pd(_mm_mul_pd(tmp, _mm_set1_pd
          (rtu_Jb_0)), tmp_0));
      }
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      localB->Jbdot[i_0 + 6 * i] = dJidt[i_0];
    }
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function7' */
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

static void emxInit_f_robotics_manip_intern(emxArray_f_robotics_manip_int_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_robotics_manip_int_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_f_robotics_manip_int_T *)malloc(sizeof
    (emxArray_f_robotics_manip_int_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_robotics_manip_internal_Col_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInitStruct_g_robotics_manip_(g_robotics_manip_internal_Col_T
  *pStruct)
{
  emxInit_f_robotics_manip_intern(&pStruct->CollisionGeometries, 2);
}

static void emxInitStruct_e_robotics_manip_(e_robotics_manip_internal_R_m_T
  *pStruct)
{
  emxInitStruct_g_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxInitMatrix_e_robotics_manip_(e_robotics_manip_internal_R_m_T
  pMatrix[34])
{
  int32_T i;
  for (i = 0; i < 34; i++) {
    emxInitStruct_e_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_R_m_T
  *pStruct)
{
  emxInitStruct_e_robotics_manip_(&pStruct->Base);
  emxInitMatrix_e_robotics_manip_(pStruct->_pobj0);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_b_m_T
  *pStruct)
{
  emxInitStruct_f_robotics_manip_(&pStruct->TreeInternal);
}

static void FK_rand_mlc3(real_T r[5])
{
  int32_T b_k;
  int32_T b_kk;
  int32_T k;
  uint32_T b_u[2];
  for (b_k = 0; b_k < 5; b_k++) {
    uint32_T mti;
    uint32_T y;

    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on: */
    /*  */
    /*  A C-program for MT19937, with initialization improved 2002/1/26. */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto. */
    /*  */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura, */
    /*  All rights reserved. */
    /*  */
    /*  Redistribution and use in source and binary forms, with or without */
    /*  modification, are permitted provided that the following conditions */
    /*  are met: */
    /*  */
    /*    1. Redistributions of source code must retain the above copyright */
    /*       notice, this list of conditions and the following disclaimer. */
    /*  */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer */
    /*       in the documentation and/or other materials provided with the */
    /*       distribution. */
    /*  */
    /*    3. The names of its contributors may not be used to endorse or */
    /*       promote products derived from this software without specific */
    /*       prior written permission. */
    /*  */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
    /*  */
    /* =============================   END   ================================= */
    int32_T exitg1;
    do {
      exitg1 = 0;
      for (k = 0; k < 2; k++) {
        mti = FK_DW.state_g[624] + 1U;
        if (FK_DW.state_g[624] + 1U >= 625U) {
          for (b_kk = 0; b_kk < 227; b_kk++) {
            y = (FK_DW.state_g[b_kk + 1] & 2147483647U) | (FK_DW.state_g[b_kk] &
              2147483648U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            FK_DW.state_g[b_kk] = FK_DW.state_g[b_kk + 397] ^ mti;
          }

          for (b_kk = 0; b_kk < 396; b_kk++) {
            y = (FK_DW.state_g[b_kk + 227] & 2147483648U) | (FK_DW.state_g[b_kk
              + 228] & 2147483647U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            FK_DW.state_g[b_kk + 227] = FK_DW.state_g[b_kk] ^ mti;
          }

          y = (FK_DW.state_g[623] & 2147483648U) | (FK_DW.state_g[0] &
            2147483647U);
          if ((y & 1U) == 0U) {
            mti = y >> 1U;
          } else {
            mti = y >> 1U ^ 2567483615U;
          }

          FK_DW.state_g[623] = FK_DW.state_g[396] ^ mti;
          mti = 1U;
        }

        y = FK_DW.state_g[(int32_T)mti - 1];
        FK_DW.state_g[624] = mti;
        y ^= y >> 11U;
        y ^= y << 7U & 2636928640U;
        y ^= y << 15U & 4022730752U;
        b_u[k] = y >> 18U ^ y;
      }

      mti = b_u[0] >> 5U;
      y = b_u[1] >> 6U;
      if ((mti == 0U) && (y == 0U)) {
        boolean_T b_isvalid;
        if ((FK_DW.state_g[624] >= 1U) && (FK_DW.state_g[624] < 625U)) {
          boolean_T exitg2;
          b_isvalid = false;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k + 1 < 625)) {
            if (FK_DW.state_g[k] == 0U) {
              k++;
            } else {
              b_isvalid = true;
              exitg2 = true;
            }
          }
        } else {
          b_isvalid = false;
        }

        if (!b_isvalid) {
          FK_DW.state_g[0] = 5489U;
          FK_DW.state_g[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    r[b_k] = ((real_T)mti * 6.7108864E+7 + (real_T)y) * 1.1102230246251565E-16;
  }
}

static void rigidBodyJoint_set_MotionSubspa(b_rigidBodyJoint_FK_T *obj, const
  real_T msubspace_data[])
{
  int32_T b_kstr;
  int32_T i;
  boolean_T b_bool;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  b_bool = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (obj->TypeInternal.Length < 1.0) {
    b_kstr = 0;
  } else {
    b_kstr = (int32_T)obj->TypeInternal.Length;
  }

  if (b_kstr == 5) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    b_kstr = 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 5) {
        if (obj->TypeInternal.Vector[b_kstr - 1] != tmp[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (!b_bool) {
    int32_T c;

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    if (obj->VelocityNumber < 1.0) {
      c = 0;
    } else {
      c = (int32_T)obj->VelocityNumber;
    }

    for (b_kstr = 0; b_kstr < c; b_kstr++) {
      for (i = 0; i < 6; i++) {
        obj->MotionSubspaceInternal[i + 6 * b_kstr] = msubspace_data[6 * b_kstr
          + i];
      }
    }
  } else {
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      obj->MotionSubspaceInternal[b_kstr] = 0.0;
    }
  }
}

static void emxEnsureCapacity_f_robotics_ma(emxArray_f_robotics_manip_int_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(f_robotics_manip_internal_Col_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_robotics_manip_internal_Col_T)
             * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_robotics_manip_internal_Col_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static g_robotics_manip_internal_Col_T *FK_CollisionSet_CollisionSet
  (g_robotics_manip_internal_Col_T *obj)
{
  static const void *t2_GeometryInternal = NULL;
  g_robotics_manip_internal_Col_T *b_obj;
  real_T c;
  int32_T b_i;
  int32_T d;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->Size = 0.0;
  b_obj = obj;
  obj->MaxElements = 0.0;
  b_i = (int32_T)obj->MaxElements;
  d = obj->CollisionGeometries->size[0] * obj->CollisionGeometries->size[1];
  obj->CollisionGeometries->size[0] = 1;
  obj->CollisionGeometries->size[1] = b_i;
  emxEnsureCapacity_f_robotics_ma(obj->CollisionGeometries, d);
  c = obj->MaxElements;
  d = (int32_T)c;
  for (b_i = 0; b_i < d; b_i++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->CollisionGeometries->data[b_i].CollisionPrimitive = (void *)
      t2_GeometryInternal;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_k
  (e_robotics_manip_internal_R_m_T *obj, const char_T bodyInput[10])
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 10.0;
  for (c = 0; c < 10; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 14.0;
  for (c = 0; c < 10; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[10] = '_';
  s.Vector[11] = 'j';
  s.Vector[12] = 'n';
  s.Vector[13] = 't';
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->ParentIndex = -1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_d
  (e_robotics_manip_internal_R_m_T *obj, const char_T bodyInput[11])
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 15.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[11] = '_';
  s.Vector[12] = 'j';
  s.Vector[13] = 'n';
  s.Vector[14] = 't';
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->ParentIndex = -1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static void rigidBodyJoint_get_MotionSubspa(const b_rigidBodyJoint_FK_T *obj,
  real_T msubspace_data[], int32_T msubspace_size[2])
{
  int32_T b_kstr;
  int32_T i;
  boolean_T b_bool;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  b_bool = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (obj->TypeInternal.Length < 1.0) {
    b_kstr = 0;
  } else {
    b_kstr = (int32_T)obj->TypeInternal.Length;
  }

  if (b_kstr == 5) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    b_kstr = 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 5) {
        if (obj->TypeInternal.Vector[b_kstr - 1] != tmp[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (!b_bool) {
    int32_T loop_ub;

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    if (obj->VelocityNumber < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = (int32_T)obj->VelocityNumber;
    }

    msubspace_size[0] = 6;
    msubspace_size[1] = loop_ub;
    for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
      for (i = 0; i < 6; i++) {
        msubspace_data[i + 6 * b_kstr] = obj->MotionSubspaceInternal[6 * b_kstr
          + i];
      }
    }
  } else {
    msubspace_size[0] = 6;
    msubspace_size[1] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0.0;
    }
  }
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_ov
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[12] = { 'l', '_', 'b', 'o', 'd', 'y', '_', 'f', 'i',
    'x', 'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { -0.99999999999978639, 6.5358979307624187E-7,
    -0.0, 0.0, 3.248713027256125E-7, 0.49705687905020046, 0.867717960508349, 0.0,
    5.6713160225719045E-7, 0.86771796050816374, -0.49705687905030665, 0.0, 0.0,
    0.1551, 1.2924, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '0';
  obj->NameInternal = s;
  obj->ParentIndex = 1.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 12.0;
  for (c = 0; c < 12; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_2[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_ad
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '0' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.08, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '1';
  obj->NameInternal = s;
  obj->ParentIndex = 2.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_kz
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '1' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.109, 0.222, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '2';
  obj->NameInternal = s;
  obj->ParentIndex = 3.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_f3
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '2' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, -0.45, 0.0, -0.0305, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '3';
  obj->NameInternal = s;
  obj->ParentIndex = 4.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_fo
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '3' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.267, 0.0, -0.075, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '4';
  obj->NameInternal = s;
  obj->ParentIndex = 5.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_g3
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '4' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.114, 0.083, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '5';
  obj->NameInternal = s;
  obj->ParentIndex = 6.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_ns
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '5' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.168, 0.0, 0.069, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '6';
  obj->NameInternal = s;
  obj->ParentIndex = 7.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_ng
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[5] = { 'l', '_', 't', 'c', 'p' };

  static const char_T tmp_0[11] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', 't',
    'c', 'p' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_3[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_4[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_5[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_6[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.06, 1.0 };

  static const int8_T tmp_7[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  obj->ParentIndex = 8.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_2[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_3[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_5[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_6[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_7[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_c4
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[12] = { 'r', '_', 'b', 'o', 'd', 'y', '_', 'f', 'i',
    'x', 'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -0.49705687905030665, 0.867717960508349, 0.0, 0.0, -0.867717960508349,
    -0.49705687905030665, 0.0, 0.0, -0.1551, 1.2924, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '0';
  obj->NameInternal = s;
  obj->ParentIndex = 1.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 12.0;
  for (c = 0; c < 12; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_2[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_lr
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '0' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0775, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '1';
  obj->NameInternal = s;
  obj->ParentIndex = 10.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_aw
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '1' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.109, 0.222, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '2';
  obj->NameInternal = s;
  obj->ParentIndex = 11.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_jo
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '2' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, -0.45, 0.0, -0.0305, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '3';
  obj->NameInternal = s;
  obj->ParentIndex = 12.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_d2
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '3' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.267, 0.0, -0.075, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '4';
  obj->NameInternal = s;
  obj->ParentIndex = 13.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_fj
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '4' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.114, 0.083, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '5';
  obj->NameInternal = s;
  obj->ParentIndex = 14.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_ke
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '5' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.168, 0.0, 0.069, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '6';
  obj->NameInternal = s;
  obj->ParentIndex = 15.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_bj
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[5] = { 'r', '_', 't', 'c', 'p' };

  static const char_T tmp_0[11] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 't',
    'c', 'p' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_3[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_4[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_5[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_6[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.06, 1.0 };

  static const int8_T tmp_7[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  obj->ParentIndex = 16.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_2[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_3[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_5[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_6[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_7[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_R_m_T *FK_RigidBody_RigidBody_op
  (e_robotics_manip_internal_R_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_R_m_T *b_obj;
  real_T msubspace_data[36];
  int32_T c;
  int32_T loop_ub;
  char_T jname_data[204];
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[5] = { 'w', 'o', 'r', 'l', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->NameInternal = s;
  obj->ParentIndex = -1.0;
  s = obj->NameInternal;
  if (s.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int32_T)s.Length;
  }

  if (loop_ub - 1 >= 0) {
    memcpy(&jname_data[0], &s.Vector[0], (uint32_T)loop_ub * sizeof(char_T));
  }

  jname_data[loop_ub] = '_';
  jname_data[loop_ub + 1] = 'j';
  jname_data[loop_ub + 2] = 'n';
  jname_data[loop_ub + 3] = 't';
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = loop_ub + 4;
  if ((loop_ub + 4) - 1 >= 0) {
    memcpy(&s.Vector[0], &jname_data[0], (uint32_T)(loop_ub + 4) * sizeof(char_T));
  }

  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (c == 8) {
    loop_ub = 1;
    do {
      exitg1 = 0;
      if (loop_ub - 1 < 8) {
        if (tmp_0[loop_ub - 1] != s.Vector[loop_ub - 1]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      loop_ub = 1;
      do {
        exitg1 = 0;
        if (loop_ub - 1 < 9) {
          if (tmp_1[loop_ub - 1] != s.Vector[loop_ub - 1]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
         *  MATLABSystem: '<S5>/MATLAB System'
         *  MATLABSystem: '<S6>/MATLAB System'
         */
        loop_ub = 1;
        do {
          exitg1 = 0;
          if (loop_ub - 1 < 8) {
            if (tmp_2[loop_ub - 1] != s.Vector[loop_ub - 1]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static void GetTransformBlock_setupImpl_ml(robotics_slmanip_internal_b_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1' };

  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2' };

  static const char_T tmp_1[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3' };

  static const char_T tmp_2[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4' };

  static const char_T tmp_3[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5' };

  static const char_T tmp_4[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '6' };

  static const char_T tmp_5[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '7' };

  static const char_T tmp_6[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '8' };

  static const char_T tmp_7[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '9' };

  static const char_T tmp_8[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '0' };

  static const char_T tmp_9[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '1' };

  static const char_T tmp_a[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '2' };

  static const char_T tmp_b[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '3' };

  static const char_T tmp_c[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '4' };

  static const char_T tmp_d[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '5' };

  static const char_T tmp_e[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '6' };

  static const char_T tmp_f[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '7' };

  static const char_T tmp_g[11] = { 'w', 'o', 'r', 'l', 'd', '_', 'f', 'i', 'x',
    'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_h[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_i[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_j[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_k[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_l[6] = { 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_m[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  real_T unusedExpr[5];
  int32_T msubspace_size[2];
  int32_T exitg1;
  FK_rand_mlc3(unusedExpr);

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  obj->TreeInternal.NumBodies = 17.0;
  obj->TreeInternal.Bodies[0] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[0], tmp);
  obj->TreeInternal.Bodies[1] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[1], tmp_0);
  obj->TreeInternal.Bodies[2] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[2], tmp_1);
  obj->TreeInternal.Bodies[3] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[3], tmp_2);
  obj->TreeInternal.Bodies[4] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[4], tmp_3);
  obj->TreeInternal.Bodies[5] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[5], tmp_4);
  obj->TreeInternal.Bodies[6] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[6], tmp_5);
  obj->TreeInternal.Bodies[7] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[7], tmp_6);
  obj->TreeInternal.Bodies[8] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[8], tmp_7);
  obj->TreeInternal.Bodies[9] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[9], tmp_8);
  obj->TreeInternal.Bodies[10] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[10], tmp_9);
  obj->TreeInternal.Bodies[11] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[11], tmp_a);
  obj->TreeInternal.Bodies[12] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[12], tmp_b);
  obj->TreeInternal.Bodies[13] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[13], tmp_c);
  obj->TreeInternal.Bodies[14] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[14], tmp_d);
  obj->TreeInternal.Bodies[15] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[15], tmp_e);
  obj->TreeInternal.Bodies[16] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[16], tmp_f);
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  obj->TreeInternal._pobj0[17].NameInternal = s;
  s = obj->TreeInternal._pobj0[17].NameInternal;
  s.Length = 4.0;

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  s.Vector[0] = 'b';
  s.Vector[1] = 'o';
  s.Vector[2] = 'd';
  s.Vector[3] = 'y';
  obj->TreeInternal._pobj0[17].NameInternal = s;
  obj->TreeInternal._pobj0[17].ParentIndex = 0.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.TypeInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    s.Vector[c] = tmp_g[c];
  }

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.NameInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.TypeInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_h[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S6>/MATLAB System' */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_i[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S6>/MATLAB System' */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_j[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S6>/MATLAB System' */
      msubspace_data[c] = tmp_k[c];
    }

    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S6>/MATLAB System' */
      msubspace_data[c] = tmp_l[c];
    }

    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 6.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 7.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  rigidBodyJoint_set_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.JointToParentTransform[c] =
      tmp_m[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.ChildToJointTransform[c] =
      tmp_m[c];
  }

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  rigidBodyJoint_get_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data, msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  rigidBodyJoint_set_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data);
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->TreeInternal._pobj0[17].CollisionsInternal);
  obj->TreeInternal._pobj0[17].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[0] = &obj->TreeInternal._pobj0[17];
  obj->TreeInternal.Bodies[1] = FK_RigidBody_RigidBody_ov
    (&obj->TreeInternal._pobj0[18]);
  obj->TreeInternal.Bodies[2] = FK_RigidBody_RigidBody_ad
    (&obj->TreeInternal._pobj0[19]);
  obj->TreeInternal.Bodies[3] = FK_RigidBody_RigidBody_kz
    (&obj->TreeInternal._pobj0[20]);
  obj->TreeInternal.Bodies[4] = FK_RigidBody_RigidBody_f3
    (&obj->TreeInternal._pobj0[21]);
  obj->TreeInternal.Bodies[5] = FK_RigidBody_RigidBody_fo
    (&obj->TreeInternal._pobj0[22]);
  obj->TreeInternal.Bodies[6] = FK_RigidBody_RigidBody_g3
    (&obj->TreeInternal._pobj0[23]);
  obj->TreeInternal.Bodies[7] = FK_RigidBody_RigidBody_ns
    (&obj->TreeInternal._pobj0[24]);
  obj->TreeInternal.Bodies[8] = FK_RigidBody_RigidBody_ng
    (&obj->TreeInternal._pobj0[25]);
  obj->TreeInternal.Bodies[9] = FK_RigidBody_RigidBody_c4
    (&obj->TreeInternal._pobj0[26]);
  obj->TreeInternal.Bodies[10] = FK_RigidBody_RigidBody_lr
    (&obj->TreeInternal._pobj0[27]);
  obj->TreeInternal.Bodies[11] = FK_RigidBody_RigidBody_aw
    (&obj->TreeInternal._pobj0[28]);
  obj->TreeInternal.Bodies[12] = FK_RigidBody_RigidBody_jo
    (&obj->TreeInternal._pobj0[29]);
  obj->TreeInternal.Bodies[13] = FK_RigidBody_RigidBody_d2
    (&obj->TreeInternal._pobj0[30]);
  obj->TreeInternal.Bodies[14] = FK_RigidBody_RigidBody_fj
    (&obj->TreeInternal._pobj0[31]);
  obj->TreeInternal.Bodies[15] = FK_RigidBody_RigidBody_ke
    (&obj->TreeInternal._pobj0[32]);
  obj->TreeInternal.Bodies[16] = FK_RigidBody_RigidBody_bj
    (&obj->TreeInternal._pobj0[33]);
  obj->TreeInternal.PositionNumber = 12.0;
  FK_RigidBody_RigidBody_op(&obj->TreeInternal.Base);
  obj->TreeInternal.matlabCodegenIsDeleted = false;
}

static void FK_rand_mlc(real_T r[5])
{
  int32_T b_k;
  int32_T b_kk;
  int32_T k;
  uint32_T b_u[2];
  for (b_k = 0; b_k < 5; b_k++) {
    uint32_T mti;
    uint32_T y;

    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on: */
    /*  */
    /*  A C-program for MT19937, with initialization improved 2002/1/26. */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto. */
    /*  */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura, */
    /*  All rights reserved. */
    /*  */
    /*  Redistribution and use in source and binary forms, with or without */
    /*  modification, are permitted provided that the following conditions */
    /*  are met: */
    /*  */
    /*    1. Redistributions of source code must retain the above copyright */
    /*       notice, this list of conditions and the following disclaimer. */
    /*  */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer */
    /*       in the documentation and/or other materials provided with the */
    /*       distribution. */
    /*  */
    /*    3. The names of its contributors may not be used to endorse or */
    /*       promote products derived from this software without specific */
    /*       prior written permission. */
    /*  */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
    /*  */
    /* =============================   END   ================================= */
    int32_T exitg1;
    do {
      exitg1 = 0;
      for (k = 0; k < 2; k++) {
        mti = FK_DW.state_j[624] + 1U;
        if (FK_DW.state_j[624] + 1U >= 625U) {
          for (b_kk = 0; b_kk < 227; b_kk++) {
            y = (FK_DW.state_j[b_kk + 1] & 2147483647U) | (FK_DW.state_j[b_kk] &
              2147483648U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            FK_DW.state_j[b_kk] = FK_DW.state_j[b_kk + 397] ^ mti;
          }

          for (b_kk = 0; b_kk < 396; b_kk++) {
            y = (FK_DW.state_j[b_kk + 227] & 2147483648U) | (FK_DW.state_j[b_kk
              + 228] & 2147483647U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            FK_DW.state_j[b_kk + 227] = FK_DW.state_j[b_kk] ^ mti;
          }

          y = (FK_DW.state_j[623] & 2147483648U) | (FK_DW.state_j[0] &
            2147483647U);
          if ((y & 1U) == 0U) {
            mti = y >> 1U;
          } else {
            mti = y >> 1U ^ 2567483615U;
          }

          FK_DW.state_j[623] = FK_DW.state_j[396] ^ mti;
          mti = 1U;
        }

        y = FK_DW.state_j[(int32_T)mti - 1];
        FK_DW.state_j[624] = mti;
        y ^= y >> 11U;
        y ^= y << 7U & 2636928640U;
        y ^= y << 15U & 4022730752U;
        b_u[k] = y >> 18U ^ y;
      }

      mti = b_u[0] >> 5U;
      y = b_u[1] >> 6U;
      if ((mti == 0U) && (y == 0U)) {
        boolean_T b_isvalid;
        if ((FK_DW.state_j[624] >= 1U) && (FK_DW.state_j[624] < 625U)) {
          boolean_T exitg2;
          b_isvalid = false;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k + 1 < 625)) {
            if (FK_DW.state_j[k] == 0U) {
              k++;
            } else {
              b_isvalid = true;
              exitg2 = true;
            }
          }
        } else {
          b_isvalid = false;
        }

        if (!b_isvalid) {
          FK_DW.state_j[0] = 5489U;
          FK_DW.state_j[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    r[b_k] = ((real_T)mti * 6.7108864E+7 + (real_T)y) * 1.1102230246251565E-16;
  }
}

static void F_GetTransformBlock_setupImpl_m(robotics_slmanip_internal_b_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1' };

  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2' };

  static const char_T tmp_1[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3' };

  static const char_T tmp_2[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4' };

  static const char_T tmp_3[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5' };

  static const char_T tmp_4[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '6' };

  static const char_T tmp_5[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '7' };

  static const char_T tmp_6[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '8' };

  static const char_T tmp_7[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '9' };

  static const char_T tmp_8[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '0' };

  static const char_T tmp_9[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '1' };

  static const char_T tmp_a[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '2' };

  static const char_T tmp_b[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '3' };

  static const char_T tmp_c[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '4' };

  static const char_T tmp_d[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '5' };

  static const char_T tmp_e[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '6' };

  static const char_T tmp_f[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '7' };

  static const char_T tmp_g[11] = { 'w', 'o', 'r', 'l', 'd', '_', 'f', 'i', 'x',
    'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_h[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_i[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_j[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_k[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_l[6] = { 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_m[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  real_T unusedExpr[5];
  int32_T msubspace_size[2];
  int32_T exitg1;
  FK_rand_mlc(unusedExpr);

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  obj->TreeInternal.NumBodies = 17.0;
  obj->TreeInternal.Bodies[0] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[0], tmp);
  obj->TreeInternal.Bodies[1] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[1], tmp_0);
  obj->TreeInternal.Bodies[2] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[2], tmp_1);
  obj->TreeInternal.Bodies[3] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[3], tmp_2);
  obj->TreeInternal.Bodies[4] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[4], tmp_3);
  obj->TreeInternal.Bodies[5] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[5], tmp_4);
  obj->TreeInternal.Bodies[6] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[6], tmp_5);
  obj->TreeInternal.Bodies[7] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[7], tmp_6);
  obj->TreeInternal.Bodies[8] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[8], tmp_7);
  obj->TreeInternal.Bodies[9] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[9], tmp_8);
  obj->TreeInternal.Bodies[10] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[10], tmp_9);
  obj->TreeInternal.Bodies[11] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[11], tmp_a);
  obj->TreeInternal.Bodies[12] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[12], tmp_b);
  obj->TreeInternal.Bodies[13] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[13], tmp_c);
  obj->TreeInternal.Bodies[14] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[14], tmp_d);
  obj->TreeInternal.Bodies[15] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[15], tmp_e);
  obj->TreeInternal.Bodies[16] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[16], tmp_f);
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  obj->TreeInternal._pobj0[17].NameInternal = s;
  s = obj->TreeInternal._pobj0[17].NameInternal;
  s.Length = 4.0;

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  s.Vector[0] = 'b';
  s.Vector[1] = 'o';
  s.Vector[2] = 'd';
  s.Vector[3] = 'y';
  obj->TreeInternal._pobj0[17].NameInternal = s;
  obj->TreeInternal._pobj0[17].ParentIndex = 0.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.TypeInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    s.Vector[c] = tmp_g[c];
  }

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.NameInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.TypeInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_h[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_i[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_j[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      msubspace_data[c] = tmp_k[c];
    }

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      msubspace_data[c] = tmp_l[c];
    }

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 6.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 7.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  rigidBodyJoint_set_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.JointToParentTransform[c] =
      tmp_m[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.ChildToJointTransform[c] =
      tmp_m[c];
  }

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  rigidBodyJoint_get_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data, msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  rigidBodyJoint_set_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data);
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->TreeInternal._pobj0[17].CollisionsInternal);
  obj->TreeInternal._pobj0[17].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[0] = &obj->TreeInternal._pobj0[17];
  obj->TreeInternal.Bodies[1] = FK_RigidBody_RigidBody_ov
    (&obj->TreeInternal._pobj0[18]);
  obj->TreeInternal.Bodies[2] = FK_RigidBody_RigidBody_ad
    (&obj->TreeInternal._pobj0[19]);
  obj->TreeInternal.Bodies[3] = FK_RigidBody_RigidBody_kz
    (&obj->TreeInternal._pobj0[20]);
  obj->TreeInternal.Bodies[4] = FK_RigidBody_RigidBody_f3
    (&obj->TreeInternal._pobj0[21]);
  obj->TreeInternal.Bodies[5] = FK_RigidBody_RigidBody_fo
    (&obj->TreeInternal._pobj0[22]);
  obj->TreeInternal.Bodies[6] = FK_RigidBody_RigidBody_g3
    (&obj->TreeInternal._pobj0[23]);
  obj->TreeInternal.Bodies[7] = FK_RigidBody_RigidBody_ns
    (&obj->TreeInternal._pobj0[24]);
  obj->TreeInternal.Bodies[8] = FK_RigidBody_RigidBody_ng
    (&obj->TreeInternal._pobj0[25]);
  obj->TreeInternal.Bodies[9] = FK_RigidBody_RigidBody_c4
    (&obj->TreeInternal._pobj0[26]);
  obj->TreeInternal.Bodies[10] = FK_RigidBody_RigidBody_lr
    (&obj->TreeInternal._pobj0[27]);
  obj->TreeInternal.Bodies[11] = FK_RigidBody_RigidBody_aw
    (&obj->TreeInternal._pobj0[28]);
  obj->TreeInternal.Bodies[12] = FK_RigidBody_RigidBody_jo
    (&obj->TreeInternal._pobj0[29]);
  obj->TreeInternal.Bodies[13] = FK_RigidBody_RigidBody_d2
    (&obj->TreeInternal._pobj0[30]);
  obj->TreeInternal.Bodies[14] = FK_RigidBody_RigidBody_fj
    (&obj->TreeInternal._pobj0[31]);
  obj->TreeInternal.Bodies[15] = FK_RigidBody_RigidBody_ke
    (&obj->TreeInternal._pobj0[32]);
  obj->TreeInternal.Bodies[16] = FK_RigidBody_RigidBody_bj
    (&obj->TreeInternal._pobj0[33]);
  obj->TreeInternal.PositionNumber = 12.0;
  FK_RigidBody_RigidBody_op(&obj->TreeInternal.Base);
  obj->TreeInternal.matlabCodegenIsDeleted = false;
}

static void emxInitStruct_e_robotics_mani_m(e_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_g_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxInitMatrix_e_robotics_mani_m(e_robotics_manip_internal_Rig_T
  pMatrix[34])
{
  int32_T i;
  for (i = 0; i < 34; i++) {
    emxInitStruct_e_robotics_mani_m(&pMatrix[i]);
  }
}

static void emxInitStruct_f_robotics_mani_m(f_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_e_robotics_mani_m(&pStruct->Base);
  emxInitMatrix_e_robotics_mani_m(pStruct->_pobj0);
}

static void emxInitStruct_robotics_slmani_m(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_f_robotics_mani_m(&pStruct->TreeInternal);
}

static void FK_rand(real_T r[5])
{
  int32_T b_k;
  int32_T b_kk;
  int32_T k;
  uint32_T b_u[2];
  for (b_k = 0; b_k < 5; b_k++) {
    uint32_T mti;
    uint32_T y;

    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on: */
    /*  */
    /*  A C-program for MT19937, with initialization improved 2002/1/26. */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto. */
    /*  */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura, */
    /*  All rights reserved. */
    /*  */
    /*  Redistribution and use in source and binary forms, with or without */
    /*  modification, are permitted provided that the following conditions */
    /*  are met: */
    /*  */
    /*    1. Redistributions of source code must retain the above copyright */
    /*       notice, this list of conditions and the following disclaimer. */
    /*  */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer */
    /*       in the documentation and/or other materials provided with the */
    /*       distribution. */
    /*  */
    /*    3. The names of its contributors may not be used to endorse or */
    /*       promote products derived from this software without specific */
    /*       prior written permission. */
    /*  */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
    /*  */
    /* =============================   END   ================================= */
    int32_T exitg1;
    do {
      exitg1 = 0;
      for (k = 0; k < 2; k++) {
        mti = FK_DW.state_l[624] + 1U;
        if (FK_DW.state_l[624] + 1U >= 625U) {
          for (b_kk = 0; b_kk < 227; b_kk++) {
            y = (FK_DW.state_l[b_kk + 1] & 2147483647U) | (FK_DW.state_l[b_kk] &
              2147483648U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            FK_DW.state_l[b_kk] = FK_DW.state_l[b_kk + 397] ^ mti;
          }

          for (b_kk = 0; b_kk < 396; b_kk++) {
            y = (FK_DW.state_l[b_kk + 227] & 2147483648U) | (FK_DW.state_l[b_kk
              + 228] & 2147483647U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            FK_DW.state_l[b_kk + 227] = FK_DW.state_l[b_kk] ^ mti;
          }

          y = (FK_DW.state_l[623] & 2147483648U) | (FK_DW.state_l[0] &
            2147483647U);
          if ((y & 1U) == 0U) {
            mti = y >> 1U;
          } else {
            mti = y >> 1U ^ 2567483615U;
          }

          FK_DW.state_l[623] = FK_DW.state_l[396] ^ mti;
          mti = 1U;
        }

        y = FK_DW.state_l[(int32_T)mti - 1];
        FK_DW.state_l[624] = mti;
        y ^= y >> 11U;
        y ^= y << 7U & 2636928640U;
        y ^= y << 15U & 4022730752U;
        b_u[k] = y >> 18U ^ y;
      }

      mti = b_u[0] >> 5U;
      y = b_u[1] >> 6U;
      if ((mti == 0U) && (y == 0U)) {
        boolean_T b_isvalid;
        if ((FK_DW.state_l[624] >= 1U) && (FK_DW.state_l[624] < 625U)) {
          boolean_T exitg2;
          b_isvalid = false;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k + 1 < 625)) {
            if (FK_DW.state_l[k] == 0U) {
              k++;
            } else {
              b_isvalid = true;
              exitg2 = true;
            }
          }
        } else {
          b_isvalid = false;
        }

        if (!b_isvalid) {
          FK_DW.state_l[0] = 5489U;
          FK_DW.state_l[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    r[b_k] = ((real_T)mti * 6.7108864E+7 + (real_T)y) * 1.1102230246251565E-16;
  }
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody
  (e_robotics_manip_internal_Rig_T *obj, const char_T bodyInput[10])
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 10.0;
  for (c = 0; c < 10; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 14.0;
  for (c = 0; c < 10; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[10] = '_';
  s.Vector[11] = 'j';
  s.Vector[12] = 'n';
  s.Vector[13] = 't';
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_m
  (e_robotics_manip_internal_Rig_T *obj, const char_T bodyInput[11])
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 15.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[11] = '_';
  s.Vector[12] = 'j';
  s.Vector[13] = 'n';
  s.Vector[14] = 't';
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_ml
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[12] = { 'l', '_', 'b', 'o', 'd', 'y', '_', 'f', 'i',
    'x', 'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { -0.99999999999978639, 6.5358979307624187E-7,
    -0.0, 0.0, 3.248713027256125E-7, 0.49705687905020046, 0.867717960508349, 0.0,
    5.6713160225719045E-7, 0.86771796050816374, -0.49705687905030665, 0.0, 0.0,
    0.1551, 1.2924, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '0';
  obj->NameInternal = s;
  obj->ParentIndex = 1.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 12.0;
  for (c = 0; c < 12; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_2[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '0' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.08, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '1';
  obj->NameInternal = s;
  obj->ParentIndex = 2.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc3
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '1' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.109, 0.222, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '2';
  obj->NameInternal = s;
  obj->ParentIndex = 3.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc3p
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '2' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, -0.45, 0.0, -0.0305, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '3';
  obj->NameInternal = s;
  obj->ParentIndex = 4.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc3pz
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '3' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.267, 0.0, -0.075, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '4';
  obj->NameInternal = s;
  obj->ParentIndex = 5.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc3pz0
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '4' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.114, 0.083, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '5';
  obj->NameInternal = s;
  obj->ParentIndex = 6.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_mlc3pz0z
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '5' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.168, 0.0, 0.069, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '6';
  obj->NameInternal = s;
  obj->ParentIndex = 7.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *F_RigidBody_RigidBody_mlc3pz0z4
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[5] = { 'l', '_', 't', 'c', 'p' };

  static const char_T tmp_0[11] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', 't',
    'c', 'p' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_3[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_4[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_5[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_6[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.06, 1.0 };

  static const int8_T tmp_7[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  obj->ParentIndex = 8.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_2[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_3[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_5[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_6[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_7[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_RigidBody_mlc3pz0z4i
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[12] = { 'r', '_', 'b', 'o', 'd', 'y', '_', 'f', 'i',
    'x', 'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -0.49705687905030665, 0.867717960508349, 0.0, 0.0, -0.867717960508349,
    -0.49705687905030665, 0.0, 0.0, -0.1551, 1.2924, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '0';
  obj->NameInternal = s;
  obj->ParentIndex = 1.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 12.0;
  for (c = 0; c < 12; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_2[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_RigidBody_mlc3pz0z4in
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '0' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0775, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '1';
  obj->NameInternal = s;
  obj->ParentIndex = 10.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_RigidBod_mlc3pz0z4in4
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '1' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.109, 0.222, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '2';
  obj->NameInternal = s;
  obj->ParentIndex = 11.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_RigidBo_mlc3pz0z4in4a
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '2' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, -0.45, 0.0, -0.0305, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '3';
  obj->NameInternal = s;
  obj->ParentIndex = 12.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_RigidB_mlc3pz0z4in4ay
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '3' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.267, 0.0, -0.075, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '4';
  obj->NameInternal = s;
  obj->ParentIndex = 13.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_Rigid_mlc3pz0z4in4ayx
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '4' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.114, 0.083, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '5';
  obj->NameInternal = s;
  obj->ParentIndex = 14.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_i
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '5' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_2[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_3[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_4[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.168, 0.0, 0.069, 1.0 };

  static const int8_T tmp_5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '6';
  obj->NameInternal = s;
  obj->ParentIndex = 15.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_0[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_4[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_5[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = tmp_2[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_c
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[5] = { 'r', '_', 't', 'c', 'p' };

  static const char_T tmp_0[11] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 't',
    'c', 'p' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_3[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_4[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_5[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_6[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.06, 1.0 };

  static const int8_T tmp_7[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  obj->ParentIndex = 16.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_1[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_2[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_3[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_5[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_6[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_7[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *FK_RigidBody_RigidBody_b
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T c;
  int32_T loop_ub;
  char_T jname_data[204];
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[5] = { 'w', 'o', 'r', 'l', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->NameInternal = s;
  obj->ParentIndex = -1.0;
  s = obj->NameInternal;
  if (s.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int32_T)s.Length;
  }

  if (loop_ub - 1 >= 0) {
    memcpy(&jname_data[0], &s.Vector[0], (uint32_T)loop_ub * sizeof(char_T));
  }

  jname_data[loop_ub] = '_';
  jname_data[loop_ub + 1] = 'j';
  jname_data[loop_ub + 2] = 'n';
  jname_data[loop_ub + 3] = 't';
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = loop_ub + 4;
  if ((loop_ub + 4) - 1 >= 0) {
    memcpy(&s.Vector[0], &jname_data[0], (uint32_T)(loop_ub + 4) * sizeof(char_T));
  }

  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  if (c == 8) {
    loop_ub = 1;
    do {
      exitg1 = 0;
      if (loop_ub - 1 < 8) {
        if (tmp_0[loop_ub - 1] != s.Vector[loop_ub - 1]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      loop_ub = 1;
      do {
        exitg1 = 0;
        if (loop_ub - 1 < 9) {
          if (tmp_1[loop_ub - 1] != s.Vector[loop_ub - 1]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
         *  MATLABSystem: '<S3>/MATLAB System'
         */
        loop_ub = 1;
        do {
          exitg1 = 0;
          if (loop_ub - 1 < 8) {
            if (tmp_2[loop_ub - 1] != s.Vector[loop_ub - 1]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 6.0;
    obj->JointInternal.PositionNumber = 7.0;
    obj->JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  FK_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static void FK_GetJacobianBlock_setupImpl(robotics_slmanip_internal_blo_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1' };

  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2' };

  static const char_T tmp_1[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3' };

  static const char_T tmp_2[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4' };

  static const char_T tmp_3[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5' };

  static const char_T tmp_4[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '6' };

  static const char_T tmp_5[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '7' };

  static const char_T tmp_6[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '8' };

  static const char_T tmp_7[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '9' };

  static const char_T tmp_8[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '0' };

  static const char_T tmp_9[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '1' };

  static const char_T tmp_a[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '2' };

  static const char_T tmp_b[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '3' };

  static const char_T tmp_c[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '4' };

  static const char_T tmp_d[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '5' };

  static const char_T tmp_e[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '6' };

  static const char_T tmp_f[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '7' };

  static const char_T tmp_g[11] = { 'w', 'o', 'r', 'l', 'd', '_', 'f', 'i', 'x',
    'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_h[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_i[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_j[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_k[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_l[6] = { 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_m[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const int8_T tmp_n[34] = { 0, 0, 1, 2, 3, 4, 5, 6, 0, 0, 7, 8, 9, 10,
    11, 12, 0, -1, -1, 1, 2, 3, 4, 5, 6, -1, -1, 7, 8, 9, 10, 11, 12, -1 };

  real_T unusedExpr[5];
  int32_T msubspace_size[2];
  int32_T exitg1;
  FK_rand(unusedExpr);

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  obj->TreeInternal.NumBodies = 17.0;
  obj->TreeInternal.Bodies[0] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[0], tmp);
  obj->TreeInternal.Bodies[1] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[1], tmp_0);
  obj->TreeInternal.Bodies[2] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[2], tmp_1);
  obj->TreeInternal.Bodies[3] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[3], tmp_2);
  obj->TreeInternal.Bodies[4] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[4], tmp_3);
  obj->TreeInternal.Bodies[5] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[5], tmp_4);
  obj->TreeInternal.Bodies[6] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[6], tmp_5);
  obj->TreeInternal.Bodies[7] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[7], tmp_6);
  obj->TreeInternal.Bodies[8] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[8], tmp_7);
  obj->TreeInternal.Bodies[9] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[9], tmp_8);
  obj->TreeInternal.Bodies[10] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[10], tmp_9);
  obj->TreeInternal.Bodies[11] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[11], tmp_a);
  obj->TreeInternal.Bodies[12] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[12], tmp_b);
  obj->TreeInternal.Bodies[13] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[13], tmp_c);
  obj->TreeInternal.Bodies[14] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[14], tmp_d);
  obj->TreeInternal.Bodies[15] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[15], tmp_e);
  obj->TreeInternal.Bodies[16] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[16], tmp_f);
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  obj->TreeInternal._pobj0[17].NameInternal = s;
  s = obj->TreeInternal._pobj0[17].NameInternal;
  s.Length = 4.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  s.Vector[0] = 'b';
  s.Vector[1] = 'o';
  s.Vector[2] = 'd';
  s.Vector[3] = 'y';
  obj->TreeInternal._pobj0[17].NameInternal = s;
  obj->TreeInternal._pobj0[17].ParentIndex = 0.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.TypeInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    s.Vector[c] = tmp_g[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.NameInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.TypeInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_h[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_i[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S2>/MATLAB System' */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_j[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' */
      msubspace_data[c] = tmp_k[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' */
      msubspace_data[c] = tmp_l[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 6.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 7.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  rigidBodyJoint_set_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.JointToParentTransform[c] =
      tmp_m[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.ChildToJointTransform[c] =
      tmp_m[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  rigidBodyJoint_get_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data, msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  rigidBodyJoint_set_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data);
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->TreeInternal._pobj0[17].CollisionsInternal);
  obj->TreeInternal._pobj0[17].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[0] = &obj->TreeInternal._pobj0[17];
  obj->TreeInternal.Bodies[0]->Index = 1.0;
  obj->TreeInternal.Bodies[1] = FK_RigidBody_RigidBody_ml
    (&obj->TreeInternal._pobj0[18]);
  obj->TreeInternal.Bodies[1]->Index = 2.0;
  obj->TreeInternal.Bodies[2] = FK_RigidBody_RigidBody_mlc
    (&obj->TreeInternal._pobj0[19]);
  obj->TreeInternal.Bodies[2]->Index = 3.0;
  obj->TreeInternal.Bodies[3] = FK_RigidBody_RigidBody_mlc3
    (&obj->TreeInternal._pobj0[20]);
  obj->TreeInternal.Bodies[3]->Index = 4.0;
  obj->TreeInternal.Bodies[4] = FK_RigidBody_RigidBody_mlc3p
    (&obj->TreeInternal._pobj0[21]);
  obj->TreeInternal.Bodies[4]->Index = 5.0;
  obj->TreeInternal.Bodies[5] = FK_RigidBody_RigidBody_mlc3pz
    (&obj->TreeInternal._pobj0[22]);
  obj->TreeInternal.Bodies[5]->Index = 6.0;
  obj->TreeInternal.Bodies[6] = FK_RigidBody_RigidBody_mlc3pz0
    (&obj->TreeInternal._pobj0[23]);
  obj->TreeInternal.Bodies[6]->Index = 7.0;
  obj->TreeInternal.Bodies[7] = FK_RigidBody_RigidBody_mlc3pz0z
    (&obj->TreeInternal._pobj0[24]);
  obj->TreeInternal.Bodies[7]->Index = 8.0;
  obj->TreeInternal.Bodies[8] = F_RigidBody_RigidBody_mlc3pz0z4
    (&obj->TreeInternal._pobj0[25]);
  obj->TreeInternal.Bodies[8]->Index = 9.0;
  obj->TreeInternal.Bodies[9] = RigidBody_RigidBody_mlc3pz0z4i
    (&obj->TreeInternal._pobj0[26]);
  obj->TreeInternal.Bodies[9]->Index = 10.0;
  obj->TreeInternal.Bodies[10] = RigidBody_RigidBody_mlc3pz0z4in
    (&obj->TreeInternal._pobj0[27]);
  obj->TreeInternal.Bodies[10]->Index = 11.0;
  obj->TreeInternal.Bodies[11] = RigidBody_RigidBod_mlc3pz0z4in4
    (&obj->TreeInternal._pobj0[28]);
  obj->TreeInternal.Bodies[11]->Index = 12.0;
  obj->TreeInternal.Bodies[12] = RigidBody_RigidBo_mlc3pz0z4in4a
    (&obj->TreeInternal._pobj0[29]);
  obj->TreeInternal.Bodies[12]->Index = 13.0;
  obj->TreeInternal.Bodies[13] = RigidBody_RigidB_mlc3pz0z4in4ay
    (&obj->TreeInternal._pobj0[30]);
  obj->TreeInternal.Bodies[13]->Index = 14.0;
  obj->TreeInternal.Bodies[14] = RigidBody_Rigid_mlc3pz0z4in4ayx
    (&obj->TreeInternal._pobj0[31]);
  obj->TreeInternal.Bodies[14]->Index = 15.0;
  obj->TreeInternal.Bodies[15] = FK_RigidBody_RigidBody_i
    (&obj->TreeInternal._pobj0[32]);
  obj->TreeInternal.Bodies[15]->Index = 16.0;
  obj->TreeInternal.Bodies[16] = FK_RigidBody_RigidBody_c
    (&obj->TreeInternal._pobj0[33]);
  obj->TreeInternal.Bodies[16]->Index = 17.0;
  obj->TreeInternal.PositionNumber = 12.0;
  obj->TreeInternal.VelocityNumber = 12.0;
  for (c = 0; c < 34; c++) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    obj->TreeInternal.VelocityDoFMap[c] = tmp_n[c];
  }

  /* Start for MATLABSystem: '<S2>/MATLAB System' */
  FK_RigidBody_RigidBody_b(&obj->TreeInternal.Base);
  obj->TreeInternal.Base.Index = 0.0;
  obj->TreeInternal.matlabCodegenIsDeleted = false;
}

static void FK_rand_m(real_T r[5])
{
  int32_T b_k;
  int32_T b_kk;
  int32_T k;
  uint32_T b_u[2];
  for (b_k = 0; b_k < 5; b_k++) {
    uint32_T mti;
    uint32_T y;

    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on: */
    /*  */
    /*  A C-program for MT19937, with initialization improved 2002/1/26. */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto. */
    /*  */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura, */
    /*  All rights reserved. */
    /*  */
    /*  Redistribution and use in source and binary forms, with or without */
    /*  modification, are permitted provided that the following conditions */
    /*  are met: */
    /*  */
    /*    1. Redistributions of source code must retain the above copyright */
    /*       notice, this list of conditions and the following disclaimer. */
    /*  */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer */
    /*       in the documentation and/or other materials provided with the */
    /*       distribution. */
    /*  */
    /*    3. The names of its contributors may not be used to endorse or */
    /*       promote products derived from this software without specific */
    /*       prior written permission. */
    /*  */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
    /*  */
    /* =============================   END   ================================= */
    int32_T exitg1;
    do {
      exitg1 = 0;
      for (k = 0; k < 2; k++) {
        mti = FK_DW.state_d[624] + 1U;
        if (FK_DW.state_d[624] + 1U >= 625U) {
          for (b_kk = 0; b_kk < 227; b_kk++) {
            y = (FK_DW.state_d[b_kk + 1] & 2147483647U) | (FK_DW.state_d[b_kk] &
              2147483648U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            FK_DW.state_d[b_kk] = FK_DW.state_d[b_kk + 397] ^ mti;
          }

          for (b_kk = 0; b_kk < 396; b_kk++) {
            y = (FK_DW.state_d[b_kk + 227] & 2147483648U) | (FK_DW.state_d[b_kk
              + 228] & 2147483647U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            FK_DW.state_d[b_kk + 227] = FK_DW.state_d[b_kk] ^ mti;
          }

          y = (FK_DW.state_d[623] & 2147483648U) | (FK_DW.state_d[0] &
            2147483647U);
          if ((y & 1U) == 0U) {
            mti = y >> 1U;
          } else {
            mti = y >> 1U ^ 2567483615U;
          }

          FK_DW.state_d[623] = FK_DW.state_d[396] ^ mti;
          mti = 1U;
        }

        y = FK_DW.state_d[(int32_T)mti - 1];
        FK_DW.state_d[624] = mti;
        y ^= y >> 11U;
        y ^= y << 7U & 2636928640U;
        y ^= y << 15U & 4022730752U;
        b_u[k] = y >> 18U ^ y;
      }

      mti = b_u[0] >> 5U;
      y = b_u[1] >> 6U;
      if ((mti == 0U) && (y == 0U)) {
        boolean_T b_isvalid;
        if ((FK_DW.state_d[624] >= 1U) && (FK_DW.state_d[624] < 625U)) {
          boolean_T exitg2;
          b_isvalid = false;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k + 1 < 625)) {
            if (FK_DW.state_d[k] == 0U) {
              k++;
            } else {
              b_isvalid = true;
              exitg2 = true;
            }
          }
        } else {
          b_isvalid = false;
        }

        if (!b_isvalid) {
          FK_DW.state_d[0] = 5489U;
          FK_DW.state_d[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    r[b_k] = ((real_T)mti * 6.7108864E+7 + (real_T)y) * 1.1102230246251565E-16;
  }
}

static void FK_GetJacobianBlock_setupImpl_m(robotics_slmanip_internal_blo_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1' };

  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2' };

  static const char_T tmp_1[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3' };

  static const char_T tmp_2[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4' };

  static const char_T tmp_3[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5' };

  static const char_T tmp_4[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '6' };

  static const char_T tmp_5[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '7' };

  static const char_T tmp_6[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '8' };

  static const char_T tmp_7[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '9' };

  static const char_T tmp_8[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '0' };

  static const char_T tmp_9[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '1' };

  static const char_T tmp_a[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '2' };

  static const char_T tmp_b[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '3' };

  static const char_T tmp_c[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '4' };

  static const char_T tmp_d[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '5' };

  static const char_T tmp_e[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '6' };

  static const char_T tmp_f[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '7' };

  static const char_T tmp_g[11] = { 'w', 'o', 'r', 'l', 'd', '_', 'f', 'i', 'x',
    'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_h[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_i[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_j[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_k[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_l[6] = { 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_m[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const int8_T tmp_n[34] = { 0, 0, 1, 2, 3, 4, 5, 6, 0, 0, 7, 8, 9, 10,
    11, 12, 0, -1, -1, 1, 2, 3, 4, 5, 6, -1, -1, 7, 8, 9, 10, 11, 12, -1 };

  real_T unusedExpr[5];
  int32_T msubspace_size[2];
  int32_T exitg1;
  FK_rand_m(unusedExpr);

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  obj->TreeInternal.NumBodies = 17.0;
  obj->TreeInternal.Bodies[0] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[0], tmp);
  obj->TreeInternal.Bodies[1] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[1], tmp_0);
  obj->TreeInternal.Bodies[2] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[2], tmp_1);
  obj->TreeInternal.Bodies[3] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[3], tmp_2);
  obj->TreeInternal.Bodies[4] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[4], tmp_3);
  obj->TreeInternal.Bodies[5] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[5], tmp_4);
  obj->TreeInternal.Bodies[6] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[6], tmp_5);
  obj->TreeInternal.Bodies[7] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[7], tmp_6);
  obj->TreeInternal.Bodies[8] = FK_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[8], tmp_7);
  obj->TreeInternal.Bodies[9] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[9], tmp_8);
  obj->TreeInternal.Bodies[10] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[10], tmp_9);
  obj->TreeInternal.Bodies[11] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[11], tmp_a);
  obj->TreeInternal.Bodies[12] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[12], tmp_b);
  obj->TreeInternal.Bodies[13] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[13], tmp_c);
  obj->TreeInternal.Bodies[14] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[14], tmp_d);
  obj->TreeInternal.Bodies[15] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[15], tmp_e);
  obj->TreeInternal.Bodies[16] = FK_RigidBody_RigidBody_m
    (&obj->TreeInternal._pobj0[16], tmp_f);
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  obj->TreeInternal._pobj0[17].NameInternal = s;
  s = obj->TreeInternal._pobj0[17].NameInternal;
  s.Length = 4.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  s.Vector[0] = 'b';
  s.Vector[1] = 'o';
  s.Vector[2] = 'd';
  s.Vector[3] = 'y';
  obj->TreeInternal._pobj0[17].NameInternal = s;
  obj->TreeInternal._pobj0[17].ParentIndex = 0.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.TypeInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    s.Vector[c] = tmp_g[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.NameInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.TypeInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_h[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_i[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S3>/MATLAB System' */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_j[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' */
      msubspace_data[c] = tmp_k[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' */
      msubspace_data[c] = tmp_l[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 6.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 7.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  rigidBodyJoint_set_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.JointToParentTransform[c] =
      tmp_m[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.ChildToJointTransform[c] =
      tmp_m[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  rigidBodyJoint_get_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data, msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  rigidBodyJoint_set_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data);
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->TreeInternal._pobj0[17].CollisionsInternal);
  obj->TreeInternal._pobj0[17].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[0] = &obj->TreeInternal._pobj0[17];
  obj->TreeInternal.Bodies[0]->Index = 1.0;
  obj->TreeInternal.Bodies[1] = FK_RigidBody_RigidBody_ml
    (&obj->TreeInternal._pobj0[18]);
  obj->TreeInternal.Bodies[1]->Index = 2.0;
  obj->TreeInternal.Bodies[2] = FK_RigidBody_RigidBody_mlc
    (&obj->TreeInternal._pobj0[19]);
  obj->TreeInternal.Bodies[2]->Index = 3.0;
  obj->TreeInternal.Bodies[3] = FK_RigidBody_RigidBody_mlc3
    (&obj->TreeInternal._pobj0[20]);
  obj->TreeInternal.Bodies[3]->Index = 4.0;
  obj->TreeInternal.Bodies[4] = FK_RigidBody_RigidBody_mlc3p
    (&obj->TreeInternal._pobj0[21]);
  obj->TreeInternal.Bodies[4]->Index = 5.0;
  obj->TreeInternal.Bodies[5] = FK_RigidBody_RigidBody_mlc3pz
    (&obj->TreeInternal._pobj0[22]);
  obj->TreeInternal.Bodies[5]->Index = 6.0;
  obj->TreeInternal.Bodies[6] = FK_RigidBody_RigidBody_mlc3pz0
    (&obj->TreeInternal._pobj0[23]);
  obj->TreeInternal.Bodies[6]->Index = 7.0;
  obj->TreeInternal.Bodies[7] = FK_RigidBody_RigidBody_mlc3pz0z
    (&obj->TreeInternal._pobj0[24]);
  obj->TreeInternal.Bodies[7]->Index = 8.0;
  obj->TreeInternal.Bodies[8] = F_RigidBody_RigidBody_mlc3pz0z4
    (&obj->TreeInternal._pobj0[25]);
  obj->TreeInternal.Bodies[8]->Index = 9.0;
  obj->TreeInternal.Bodies[9] = RigidBody_RigidBody_mlc3pz0z4i
    (&obj->TreeInternal._pobj0[26]);
  obj->TreeInternal.Bodies[9]->Index = 10.0;
  obj->TreeInternal.Bodies[10] = RigidBody_RigidBody_mlc3pz0z4in
    (&obj->TreeInternal._pobj0[27]);
  obj->TreeInternal.Bodies[10]->Index = 11.0;
  obj->TreeInternal.Bodies[11] = RigidBody_RigidBod_mlc3pz0z4in4
    (&obj->TreeInternal._pobj0[28]);
  obj->TreeInternal.Bodies[11]->Index = 12.0;
  obj->TreeInternal.Bodies[12] = RigidBody_RigidBo_mlc3pz0z4in4a
    (&obj->TreeInternal._pobj0[29]);
  obj->TreeInternal.Bodies[12]->Index = 13.0;
  obj->TreeInternal.Bodies[13] = RigidBody_RigidB_mlc3pz0z4in4ay
    (&obj->TreeInternal._pobj0[30]);
  obj->TreeInternal.Bodies[13]->Index = 14.0;
  obj->TreeInternal.Bodies[14] = RigidBody_Rigid_mlc3pz0z4in4ayx
    (&obj->TreeInternal._pobj0[31]);
  obj->TreeInternal.Bodies[14]->Index = 15.0;
  obj->TreeInternal.Bodies[15] = FK_RigidBody_RigidBody_i
    (&obj->TreeInternal._pobj0[32]);
  obj->TreeInternal.Bodies[15]->Index = 16.0;
  obj->TreeInternal.Bodies[16] = FK_RigidBody_RigidBody_c
    (&obj->TreeInternal._pobj0[33]);
  obj->TreeInternal.Bodies[16]->Index = 17.0;
  obj->TreeInternal.PositionNumber = 12.0;
  obj->TreeInternal.VelocityNumber = 12.0;
  for (c = 0; c < 34; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    obj->TreeInternal.VelocityDoFMap[c] = tmp_n[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  FK_RigidBody_RigidBody_b(&obj->TreeInternal.Base);
  obj->TreeInternal.Base.Index = 0.0;
  obj->TreeInternal.matlabCodegenIsDeleted = false;
}

static void FK_rand_ml(real_T r[5])
{
  int32_T b_k;
  int32_T b_kk;
  int32_T k;
  uint32_T b_u[2];
  for (b_k = 0; b_k < 5; b_k++) {
    uint32_T mti;
    uint32_T y;

    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on: */
    /*  */
    /*  A C-program for MT19937, with initialization improved 2002/1/26. */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto. */
    /*  */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura, */
    /*  All rights reserved. */
    /*  */
    /*  Redistribution and use in source and binary forms, with or without */
    /*  modification, are permitted provided that the following conditions */
    /*  are met: */
    /*  */
    /*    1. Redistributions of source code must retain the above copyright */
    /*       notice, this list of conditions and the following disclaimer. */
    /*  */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer */
    /*       in the documentation and/or other materials provided with the */
    /*       distribution. */
    /*  */
    /*    3. The names of its contributors may not be used to endorse or */
    /*       promote products derived from this software without specific */
    /*       prior written permission. */
    /*  */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
    /*  */
    /* =============================   END   ================================= */
    int32_T exitg1;
    do {
      exitg1 = 0;
      for (k = 0; k < 2; k++) {
        mti = FK_DW.state_i[624] + 1U;
        if (FK_DW.state_i[624] + 1U >= 625U) {
          for (b_kk = 0; b_kk < 227; b_kk++) {
            y = (FK_DW.state_i[b_kk + 1] & 2147483647U) | (FK_DW.state_i[b_kk] &
              2147483648U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            FK_DW.state_i[b_kk] = FK_DW.state_i[b_kk + 397] ^ mti;
          }

          for (b_kk = 0; b_kk < 396; b_kk++) {
            y = (FK_DW.state_i[b_kk + 227] & 2147483648U) | (FK_DW.state_i[b_kk
              + 228] & 2147483647U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            FK_DW.state_i[b_kk + 227] = FK_DW.state_i[b_kk] ^ mti;
          }

          y = (FK_DW.state_i[623] & 2147483648U) | (FK_DW.state_i[0] &
            2147483647U);
          if ((y & 1U) == 0U) {
            mti = y >> 1U;
          } else {
            mti = y >> 1U ^ 2567483615U;
          }

          FK_DW.state_i[623] = FK_DW.state_i[396] ^ mti;
          mti = 1U;
        }

        y = FK_DW.state_i[(int32_T)mti - 1];
        FK_DW.state_i[624] = mti;
        y ^= y >> 11U;
        y ^= y << 7U & 2636928640U;
        y ^= y << 15U & 4022730752U;
        b_u[k] = y >> 18U ^ y;
      }

      mti = b_u[0] >> 5U;
      y = b_u[1] >> 6U;
      if ((mti == 0U) && (y == 0U)) {
        boolean_T b_isvalid;
        if ((FK_DW.state_i[624] >= 1U) && (FK_DW.state_i[624] < 625U)) {
          boolean_T exitg2;
          b_isvalid = false;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k + 1 < 625)) {
            if (FK_DW.state_i[k] == 0U) {
              k++;
            } else {
              b_isvalid = true;
              exitg2 = true;
            }
          }
        } else {
          b_isvalid = false;
        }

        if (!b_isvalid) {
          FK_DW.state_i[0] = 5489U;
          FK_DW.state_i[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    r[b_k] = ((real_T)mti * 6.7108864E+7 + (real_T)y) * 1.1102230246251565E-16;
  }
}

static void FK_GetTransformBlock_setupImpl(robotics_slmanip_internal_b_m_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const char_T tmp[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1' };

  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2' };

  static const char_T tmp_1[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3' };

  static const char_T tmp_2[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4' };

  static const char_T tmp_3[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5' };

  static const char_T tmp_4[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '6' };

  static const char_T tmp_5[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '7' };

  static const char_T tmp_6[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '8' };

  static const char_T tmp_7[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '9' };

  static const char_T tmp_8[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '0' };

  static const char_T tmp_9[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '1' };

  static const char_T tmp_a[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '2' };

  static const char_T tmp_b[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '3' };

  static const char_T tmp_c[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '4' };

  static const char_T tmp_d[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '5' };

  static const char_T tmp_e[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '6' };

  static const char_T tmp_f[11] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '7' };

  static const char_T tmp_g[11] = { 'w', 'o', 'r', 'l', 'd', '_', 'f', 'i', 'x',
    'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_h[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_i[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_j[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_k[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_l[6] = { 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_m[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  real_T unusedExpr[5];
  int32_T msubspace_size[2];
  int32_T exitg1;
  FK_rand_ml(unusedExpr);

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  obj->TreeInternal.NumBodies = 17.0;
  obj->TreeInternal.Bodies[0] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[0], tmp);
  obj->TreeInternal.Bodies[1] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[1], tmp_0);
  obj->TreeInternal.Bodies[2] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[2], tmp_1);
  obj->TreeInternal.Bodies[3] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[3], tmp_2);
  obj->TreeInternal.Bodies[4] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[4], tmp_3);
  obj->TreeInternal.Bodies[5] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[5], tmp_4);
  obj->TreeInternal.Bodies[6] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[6], tmp_5);
  obj->TreeInternal.Bodies[7] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[7], tmp_6);
  obj->TreeInternal.Bodies[8] = FK_RigidBody_RigidBody_k
    (&obj->TreeInternal._pobj0[8], tmp_7);
  obj->TreeInternal.Bodies[9] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[9], tmp_8);
  obj->TreeInternal.Bodies[10] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[10], tmp_9);
  obj->TreeInternal.Bodies[11] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[11], tmp_a);
  obj->TreeInternal.Bodies[12] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[12], tmp_b);
  obj->TreeInternal.Bodies[13] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[13], tmp_c);
  obj->TreeInternal.Bodies[14] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[14], tmp_d);
  obj->TreeInternal.Bodies[15] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[15], tmp_e);
  obj->TreeInternal.Bodies[16] = FK_RigidBody_RigidBody_d
    (&obj->TreeInternal._pobj0[16], tmp_f);
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  obj->TreeInternal._pobj0[17].NameInternal = s;
  s = obj->TreeInternal._pobj0[17].NameInternal;
  s.Length = 4.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  s.Vector[0] = 'b';
  s.Vector[1] = 'o';
  s.Vector[2] = 'd';
  s.Vector[3] = 'y';
  obj->TreeInternal._pobj0[17].NameInternal = s;
  obj->TreeInternal._pobj0[17].ParentIndex = 0.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.TypeInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    s.Vector[c] = tmp_g[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.NameInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  obj->TreeInternal._pobj0[17].JointInternal.TypeInternal = s;
  s = obj->TreeInternal._pobj0[17].JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  if (c == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_h[b_kstr - 1] != s.Vector[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_i[b_kstr - 1] != s.Vector[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_j[b_kstr - 1] != s.Vector[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        c = 2;
      } else {
        c = -1;
      }
    }
  }

  switch (c) {
   case 0:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' */
      msubspace_data[c] = tmp_k[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' */
      msubspace_data[c] = tmp_l[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 6.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 7.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = (rtNaN);
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = (rtNaN);
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = (rtNaN);
    break;

   default:
    for (c = 0; c < 6; c++) {
      msubspace_data[c] = 0.0;
    }

    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.VelocityNumber = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.PositionNumber = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  rigidBodyJoint_set_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.JointToParentTransform[c] =
      tmp_m[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.ChildToJointTransform[c] =
      tmp_m[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  rigidBodyJoint_get_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data, msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  rigidBodyJoint_set_MotionSubspa(&obj->TreeInternal._pobj0[17].JointInternal,
    msubspace_data);
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[0] = 0.0;
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[1] = 0.0;
  obj->TreeInternal._pobj0[17].JointInternal.JointAxisInternal[2] = 0.0;
  FK_CollisionSet_CollisionSet(&obj->TreeInternal._pobj0[17].CollisionsInternal);
  obj->TreeInternal._pobj0[17].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[0] = &obj->TreeInternal._pobj0[17];
  obj->TreeInternal.Bodies[1] = FK_RigidBody_RigidBody_ov
    (&obj->TreeInternal._pobj0[18]);
  obj->TreeInternal.Bodies[2] = FK_RigidBody_RigidBody_ad
    (&obj->TreeInternal._pobj0[19]);
  obj->TreeInternal.Bodies[3] = FK_RigidBody_RigidBody_kz
    (&obj->TreeInternal._pobj0[20]);
  obj->TreeInternal.Bodies[4] = FK_RigidBody_RigidBody_f3
    (&obj->TreeInternal._pobj0[21]);
  obj->TreeInternal.Bodies[5] = FK_RigidBody_RigidBody_fo
    (&obj->TreeInternal._pobj0[22]);
  obj->TreeInternal.Bodies[6] = FK_RigidBody_RigidBody_g3
    (&obj->TreeInternal._pobj0[23]);
  obj->TreeInternal.Bodies[7] = FK_RigidBody_RigidBody_ns
    (&obj->TreeInternal._pobj0[24]);
  obj->TreeInternal.Bodies[8] = FK_RigidBody_RigidBody_ng
    (&obj->TreeInternal._pobj0[25]);
  obj->TreeInternal.Bodies[9] = FK_RigidBody_RigidBody_c4
    (&obj->TreeInternal._pobj0[26]);
  obj->TreeInternal.Bodies[10] = FK_RigidBody_RigidBody_lr
    (&obj->TreeInternal._pobj0[27]);
  obj->TreeInternal.Bodies[11] = FK_RigidBody_RigidBody_aw
    (&obj->TreeInternal._pobj0[28]);
  obj->TreeInternal.Bodies[12] = FK_RigidBody_RigidBody_jo
    (&obj->TreeInternal._pobj0[29]);
  obj->TreeInternal.Bodies[13] = FK_RigidBody_RigidBody_d2
    (&obj->TreeInternal._pobj0[30]);
  obj->TreeInternal.Bodies[14] = FK_RigidBody_RigidBody_fj
    (&obj->TreeInternal._pobj0[31]);
  obj->TreeInternal.Bodies[15] = FK_RigidBody_RigidBody_ke
    (&obj->TreeInternal._pobj0[32]);
  obj->TreeInternal.Bodies[16] = FK_RigidBody_RigidBody_bj
    (&obj->TreeInternal._pobj0[33]);
  obj->TreeInternal.PositionNumber = 12.0;
  FK_RigidBody_RigidBody_op(&obj->TreeInternal.Base);
  obj->TreeInternal.matlabCodegenIsDeleted = false;
}

static void FK_rigidBodyJoint_get_JointAxis(const b_rigidBodyJoint_FK_T *obj,
  real_T ax[3])
{
  int32_T b_kstr;
  boolean_T b_bool;
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  boolean_T guard1;
  b_bool = false;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  if (obj->TypeInternal.Length < 1.0) {
    b_kstr = 0;
  } else {
    b_kstr = (int32_T)obj->TypeInternal.Length;
  }

  if (b_kstr == 8) {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (obj->TypeInternal.Vector[b_kstr - 1] != tmp[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (b_bool) {
    guard1 = true;
  } else {
    /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
     *  MATLABSystem: '<S3>/MATLAB System'
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    if (obj->TypeInternal.Length < 1.0) {
      b_kstr = 0;
    } else {
      b_kstr = (int32_T)obj->TypeInternal.Length;
    }

    if (b_kstr == 9) {
      /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
       *  MATLABSystem: '<S3>/MATLAB System'
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       *  MATLABSystem: '<S6>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (obj->TypeInternal.Vector[b_kstr - 1] != tmp_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void FK_cat(real_T varargin_1, real_T varargin_2, real_T varargin_3,
                   real_T varargin_4, real_T varargin_5, real_T varargin_6,
                   real_T varargin_7, real_T varargin_8, real_T varargin_9,
                   real_T y[9])
{
  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  y[0] = varargin_1;
  y[1] = varargin_2;
  y[2] = varargin_3;
  y[3] = varargin_4;
  y[4] = varargin_5;
  y[5] = varargin_6;
  y[6] = varargin_7;
  y[7] = varargin_8;
  y[8] = varargin_9;
}

static void RigidBodyTree_forwardKinemat_ml(f_robotics_manip_internal_R_m_T *obj,
  const real_T qvec[12], h_cell_wrap_FK_T Ttree_data[], int32_T Ttree_size[2])
{
  __m128d tmp_0;
  e_robotics_manip_internal_R_m_T *body;
  real_T a[16];
  real_T b[16];
  real_T b_0[16];
  real_T b_I[16];
  real_T R[9];
  real_T tempR[9];
  real_T result_data[4];
  real_T v[3];
  real_T tmp[2];
  real_T b_q_idx_0;
  real_T b_q_idx_1;
  real_T cth;
  real_T k;
  real_T n;
  real_T tempR_tmp;
  real_T theta;
  int32_T b_jtilecol;
  int32_T c;
  int32_T d;
  int32_T e;
  int32_T i;
  char_T obj_Vector[200];
  boolean_T result;
  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_4[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  real_T cth_tmp;
  real_T tempR_tmp_tmp;
  real_T tmp_5;
  int32_T b_tmp;
  int32_T exitg1;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  n = obj->NumBodies;
  Ttree_size[0] = 1;
  b_tmp = (int32_T)n;
  Ttree_size[1] = (int32_T)n;
  if ((int32_T)n != 0) {
    c = (uint8_T)(int32_T)n;
    for (b_jtilecol = 0; b_jtilecol < c; b_jtilecol++) {
      for (i = 0; i < 16; i++) {
        Ttree_data[b_jtilecol].f1[i] = tmp_1[i];
      }
    }
  }

  k = 1.0;

  /* Start for MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  for (b_jtilecol = 0; b_jtilecol < b_tmp; b_jtilecol++) {
    body = obj->Bodies[b_jtilecol];
    n = body->JointInternal.PositionNumber;
    n += k;
    if (k > n - 1.0) {
      e = 0;
      d = 0;
    } else {
      e = (int32_T)k - 1;
      d = (int32_T)(n - 1.0);
    }

    for (i = 0; i < 16; i++) {
      a[i] = body->JointInternal.JointToParentTransform[i];
    }

    cth = body->JointInternal.TypeInternal.Length;
    for (i = 0; i < 200; i++) {
      obj_Vector[i] = body->JointInternal.TypeInternal.Vector[i];
    }

    if (cth < 1.0) {
      c = 0;
    } else {
      c = (int32_T)cth;
    }

    result = false;
    if (c == 8) {
      i = 1;
      do {
        exitg1 = 0;
        if (i - 1 < 8) {
          if (tmp_2[i - 1] != obj_Vector[i - 1]) {
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      cth = 0.0;
    } else {
      if (c == 9) {
        i = 1;
        do {
          exitg1 = 0;
          if (i - 1 < 9) {
            if (tmp_3[i - 1] != obj_Vector[i - 1]) {
              exitg1 = 1;
            } else {
              i++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        cth = 1.0;
      } else {
        if (c == 8) {
          i = 1;
          do {
            exitg1 = 0;
            if (i - 1 < 8) {
              if (tmp_4[i - 1] != obj_Vector[i - 1]) {
                exitg1 = 1;
              } else {
                i++;
              }
            } else {
              result = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (result) {
          cth = 2.0;
        } else {
          cth = -1.0;
        }
      }
    }

    switch ((int32_T)cth) {
     case 0:
      FK_rigidBodyJoint_get_JointAxis(&body->JointInternal, v);
      result_data[0] = v[0];
      result_data[1] = v[1];
      result_data[2] = v[2];
      if ((d - e != 0) - 1 >= 0) {
        result_data[3] = qvec[e];
      }

      k = result_data[0];
      theta = result_data[1];
      cth_tmp = result_data[2];
      cth = 1.0 / sqrt((k * k + theta * theta) + cth_tmp * cth_tmp);
      _mm_storeu_pd(&v[0], _mm_mul_pd(_mm_set_pd(theta, k), _mm_set1_pd(cth)));
      v[2] = cth_tmp * cth;
      theta = result_data[3];
      cth = cos(theta);
      theta = sin(theta);
      k = v[0] * v[1] * (1.0 - cth);
      cth_tmp = v[2] * theta;
      b_q_idx_0 = v[0] * v[2] * (1.0 - cth);
      b_q_idx_1 = v[1] * theta;
      tempR_tmp = v[1] * v[2] * (1.0 - cth);
      theta *= v[0];
      FK_cat(v[0] * v[0] * (1.0 - cth) + cth, k - cth_tmp, b_q_idx_0 + b_q_idx_1,
             k + cth_tmp, v[1] * v[1] * (1.0 - cth) + cth, tempR_tmp - theta,
             b_q_idx_0 - b_q_idx_1, tempR_tmp + theta, v[2] * v[2] * (1.0 - cth)
             + cth, tempR);
      for (c = 0; c < 3; c++) {
        R[c] = tempR[c * 3];
        R[c + 3] = tempR[c * 3 + 1];
        R[c + 6] = tempR[c * 3 + 2];
      }

      memset(&b[0], 0, sizeof(real_T) << 4U);
      for (i = 0; i < 3; i++) {
        c = i << 2;
        b[c] = R[3 * i];
        b[c + 1] = R[3 * i + 1];
        b[c + 2] = R[3 * i + 2];
      }

      b[15] = 1.0;
      break;

     case 1:
      FK_rigidBodyJoint_get_JointAxis(&body->JointInternal, v);
      memset(&tempR[0], 0, 9U * sizeof(real_T));
      tempR[0] = 1.0;
      tempR[4] = 1.0;
      tempR[8] = 1.0;
      k = qvec[e];
      for (i = 0; i < 3; i++) {
        c = i << 2;
        b[c] = tempR[3 * i];
        b[c + 1] = tempR[3 * i + 1];
        b[c + 2] = tempR[3 * i + 2];
        b[i + 12] = v[i] * k;
      }

      b[3] = 0.0;
      b[7] = 0.0;
      b[11] = 0.0;
      b[15] = 1.0;
      break;

     case 2:
      memset(&b_I[0], 0, sizeof(real_T) << 4U);
      b_I[0] = 1.0;
      b_I[5] = 1.0;
      b_I[10] = 1.0;
      b_I[15] = 1.0;
      b_I[12] = qvec[e + 4];
      b_I[13] = qvec[e + 5];
      b_I[14] = qvec[e + 6];
      cth = qvec[e];
      b_q_idx_0 = cth * cth;
      cth = qvec[e + 1];
      b_q_idx_1 = cth * cth;
      cth = qvec[e + 2];
      k = cth * cth;
      cth = qvec[e + 3];
      tmp_0 = _mm_set1_pd(1.0 / sqrt(((b_q_idx_0 + b_q_idx_1) + k) + cth * cth));
      _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_loadu_pd(&qvec[e]), tmp_0));
      b_q_idx_0 = tmp[0];
      b_q_idx_1 = tmp[1];
      _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_loadu_pd(&qvec[e + 2]), tmp_0));
      k = b_q_idx_1 * tmp[0];
      cth = b_q_idx_0 * tmp[1];
      tempR_tmp_tmp = tmp[1] * tmp[1];
      cth_tmp = b_q_idx_1 * tmp[1];
      tempR_tmp = b_q_idx_0 * tmp[0];
      theta = tmp[0] * tmp[1];
      tmp_5 = b_q_idx_0 * b_q_idx_1;
      b_q_idx_1 *= b_q_idx_1;
      b_q_idx_0 = tmp[0] * tmp[0];
      FK_cat(1.0 - (b_q_idx_0 + tempR_tmp_tmp) * 2.0, (k - cth) * 2.0, (cth_tmp
              + tempR_tmp) * 2.0, (k + cth) * 2.0, 1.0 - (b_q_idx_1 +
              tempR_tmp_tmp) * 2.0, (theta - tmp_5) * 2.0, (cth_tmp - tempR_tmp)
             * 2.0, (theta + tmp_5) * 2.0, 1.0 - (b_q_idx_1 + b_q_idx_0) * 2.0,
             tempR);
      for (c = 0; c < 3; c++) {
        R[c] = tempR[c * 3];
        R[c + 3] = tempR[c * 3 + 1];
        R[c + 6] = tempR[c * 3 + 2];
      }

      memset(&b_0[0], 0, sizeof(real_T) << 4U);
      for (i = 0; i < 3; i++) {
        c = i << 2;
        b_0[c] = R[3 * i];
        b_0[c + 1] = R[3 * i + 1];
        b_0[c + 2] = R[3 * i + 2];
      }

      b_0[15] = 1.0;
      for (i = 0; i < 4; i++) {
        k = b_I[i + 4];
        cth_tmp = b_I[i];
        b_q_idx_0 = b_I[i + 8];
        b_q_idx_1 = b_I[i + 12];
        for (c = 0; c <= 2; c += 2) {
          e = (c + 1) << 2;
          d = c << 2;
          _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(b_0[e + 1], b_0[d + 1]), _mm_set1_pd(k)), _mm_mul_pd
            (_mm_set_pd(b_0[e], b_0[d]), _mm_set1_pd(cth_tmp))), _mm_mul_pd
            (_mm_set_pd(b_0[e + 2], b_0[d + 2]), _mm_set1_pd(b_q_idx_0))),
            _mm_mul_pd(_mm_set_pd(b_0[e + 3], b_0[d + 3]), _mm_set1_pd(b_q_idx_1))));
          b[i + d] = tmp[0];
          b[i + e] = tmp[1];
        }
      }
      break;

     default:
      memset(&b[0], 0, sizeof(real_T) << 4U);
      b[0] = 1.0;
      b[5] = 1.0;
      b[10] = 1.0;
      b[15] = 1.0;
      break;
    }

    for (i = 0; i < 16; i++) {
      b_0[i] = body->JointInternal.ChildToJointTransform[i];
    }

    for (i = 0; i < 4; i++) {
      cth_tmp = a[i + 4];
      b_q_idx_0 = a[i];
      b_q_idx_1 = a[i + 8];
      tempR_tmp = a[i + 12];
      for (c = 0; c <= 2; c += 2) {
        e = (c + 1) << 2;
        d = c << 2;
        _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(b[e + 1], b[d + 1]), _mm_set1_pd(cth_tmp)), _mm_mul_pd
          (_mm_set_pd(b[e], b[d]), _mm_set1_pd(b_q_idx_0))), _mm_mul_pd
          (_mm_set_pd(b[e + 2], b[d + 2]), _mm_set1_pd(b_q_idx_1))), _mm_mul_pd
          (_mm_set_pd(b[e + 3], b[d + 3]), _mm_set1_pd(tempR_tmp))));
        b_I[i + d] = tmp[0];
        b_I[i + e] = tmp[1];
      }

      cth_tmp = b_I[i + 4];
      b_q_idx_0 = b_I[i];
      b_q_idx_1 = b_I[i + 8];
      tempR_tmp = b_I[i + 12];
      for (c = 0; c <= 2; c += 2) {
        e = (c + 1) << 2;
        d = c << 2;
        _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(b_0[e + 1], b_0[d + 1]), _mm_set1_pd(cth_tmp)), _mm_mul_pd
          (_mm_set_pd(b_0[e], b_0[d]), _mm_set1_pd(b_q_idx_0))), _mm_mul_pd
          (_mm_set_pd(b_0[e + 2], b_0[d + 2]), _mm_set1_pd(b_q_idx_1))),
          _mm_mul_pd(_mm_set_pd(b_0[e + 3], b_0[d + 3]), _mm_set1_pd(tempR_tmp))));
        Ttree_data[b_jtilecol].f1[i + d] = tmp[0];
        Ttree_data[b_jtilecol].f1[i + e] = tmp[1];
      }
    }

    k = n;
    if (body->ParentIndex > 0.0) {
      for (i = 0; i < 16; i++) {
        a[i] = Ttree_data[(int32_T)body->ParentIndex - 1].f1[i];
      }

      for (i = 0; i < 4; i++) {
        cth_tmp = a[i + 4];
        b_q_idx_0 = a[i];
        b_q_idx_1 = a[i + 8];
        tempR_tmp = a[i + 12];
        for (c = 0; c <= 2; c += 2) {
          e = (c + 1) << 2;
          d = c << 2;
          _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(Ttree_data[b_jtilecol].f1[e + 1], Ttree_data[b_jtilecol]
                        .f1[d + 1]), _mm_set1_pd(cth_tmp)), _mm_mul_pd
            (_mm_set_pd(Ttree_data[b_jtilecol].f1[e], Ttree_data[b_jtilecol]
                        .f1[d]), _mm_set1_pd(b_q_idx_0))), _mm_mul_pd(_mm_set_pd
            (Ttree_data[b_jtilecol].f1[e + 2], Ttree_data[b_jtilecol].f1[d + 2]),
            _mm_set1_pd(b_q_idx_1))), _mm_mul_pd(_mm_set_pd
            (Ttree_data[b_jtilecol].f1[e + 3], Ttree_data[b_jtilecol].f1[d + 3]),
            _mm_set1_pd(tempR_tmp))));
          b_I[i + d] = tmp[0];
          b_I[i + e] = tmp[1];
        }
      }

      memcpy(&Ttree_data[b_jtilecol].f1[0], &b_I[0], sizeof(real_T) << 4U);
    }
  }
}

static void RigidBodyTree_forwardKinematics(f_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[12], h_cell_wrap_FK_T Ttree_data[], int32_T Ttree_size[2])
{
  __m128d tmp_0;
  e_robotics_manip_internal_Rig_T *body;
  real_T a[16];
  real_T b[16];
  real_T b_0[16];
  real_T b_I[16];
  real_T R[9];
  real_T tempR[9];
  real_T result_data[4];
  real_T v[3];
  real_T tmp[2];
  real_T b_q_idx_0;
  real_T b_q_idx_1;
  real_T cth;
  real_T k;
  real_T n;
  real_T tempR_tmp;
  real_T theta;
  int32_T b_jtilecol;
  int32_T c;
  int32_T d;
  int32_T e;
  int32_T i;
  char_T obj_Vector[200];
  boolean_T result;
  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_4[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  real_T cth_tmp;
  real_T tempR_tmp_tmp;
  real_T tmp_5;
  int32_T b_tmp;
  int32_T exitg1;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  n = obj->NumBodies;
  Ttree_size[0] = 1;
  b_tmp = (int32_T)n;
  Ttree_size[1] = (int32_T)n;
  if ((int32_T)n != 0) {
    c = (uint8_T)(int32_T)n;
    for (b_jtilecol = 0; b_jtilecol < c; b_jtilecol++) {
      for (i = 0; i < 16; i++) {
        Ttree_data[b_jtilecol].f1[i] = tmp_1[i];
      }
    }
  }

  k = 1.0;

  /* Start for MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  for (b_jtilecol = 0; b_jtilecol < b_tmp; b_jtilecol++) {
    body = obj->Bodies[b_jtilecol];
    n = body->JointInternal.PositionNumber;
    n += k;
    if (k > n - 1.0) {
      e = 0;
      d = 0;
    } else {
      e = (int32_T)k - 1;
      d = (int32_T)(n - 1.0);
    }

    for (i = 0; i < 16; i++) {
      a[i] = body->JointInternal.JointToParentTransform[i];
    }

    cth = body->JointInternal.TypeInternal.Length;
    for (i = 0; i < 200; i++) {
      obj_Vector[i] = body->JointInternal.TypeInternal.Vector[i];
    }

    if (cth < 1.0) {
      c = 0;
    } else {
      c = (int32_T)cth;
    }

    result = false;
    if (c == 8) {
      i = 1;
      do {
        exitg1 = 0;
        if (i - 1 < 8) {
          if (tmp_2[i - 1] != obj_Vector[i - 1]) {
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      cth = 0.0;
    } else {
      if (c == 9) {
        i = 1;
        do {
          exitg1 = 0;
          if (i - 1 < 9) {
            if (tmp_3[i - 1] != obj_Vector[i - 1]) {
              exitg1 = 1;
            } else {
              i++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        cth = 1.0;
      } else {
        if (c == 8) {
          i = 1;
          do {
            exitg1 = 0;
            if (i - 1 < 8) {
              if (tmp_4[i - 1] != obj_Vector[i - 1]) {
                exitg1 = 1;
              } else {
                i++;
              }
            } else {
              result = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (result) {
          cth = 2.0;
        } else {
          cth = -1.0;
        }
      }
    }

    switch ((int32_T)cth) {
     case 0:
      FK_rigidBodyJoint_get_JointAxis(&body->JointInternal, v);
      result_data[0] = v[0];
      result_data[1] = v[1];
      result_data[2] = v[2];
      if ((d - e != 0) - 1 >= 0) {
        result_data[3] = qvec[e];
      }

      k = result_data[0];
      theta = result_data[1];
      cth_tmp = result_data[2];
      cth = 1.0 / sqrt((k * k + theta * theta) + cth_tmp * cth_tmp);
      _mm_storeu_pd(&v[0], _mm_mul_pd(_mm_set_pd(theta, k), _mm_set1_pd(cth)));
      v[2] = cth_tmp * cth;
      theta = result_data[3];
      cth = cos(theta);
      theta = sin(theta);
      k = v[0] * v[1] * (1.0 - cth);
      cth_tmp = v[2] * theta;
      b_q_idx_0 = v[0] * v[2] * (1.0 - cth);
      b_q_idx_1 = v[1] * theta;
      tempR_tmp = v[1] * v[2] * (1.0 - cth);
      theta *= v[0];
      FK_cat(v[0] * v[0] * (1.0 - cth) + cth, k - cth_tmp, b_q_idx_0 + b_q_idx_1,
             k + cth_tmp, v[1] * v[1] * (1.0 - cth) + cth, tempR_tmp - theta,
             b_q_idx_0 - b_q_idx_1, tempR_tmp + theta, v[2] * v[2] * (1.0 - cth)
             + cth, tempR);
      for (c = 0; c < 3; c++) {
        R[c] = tempR[c * 3];
        R[c + 3] = tempR[c * 3 + 1];
        R[c + 6] = tempR[c * 3 + 2];
      }

      memset(&b[0], 0, sizeof(real_T) << 4U);
      for (i = 0; i < 3; i++) {
        c = i << 2;
        b[c] = R[3 * i];
        b[c + 1] = R[3 * i + 1];
        b[c + 2] = R[3 * i + 2];
      }

      b[15] = 1.0;
      break;

     case 1:
      FK_rigidBodyJoint_get_JointAxis(&body->JointInternal, v);
      memset(&tempR[0], 0, 9U * sizeof(real_T));
      tempR[0] = 1.0;
      tempR[4] = 1.0;
      tempR[8] = 1.0;
      k = qvec[e];
      for (i = 0; i < 3; i++) {
        c = i << 2;
        b[c] = tempR[3 * i];
        b[c + 1] = tempR[3 * i + 1];
        b[c + 2] = tempR[3 * i + 2];
        b[i + 12] = v[i] * k;
      }

      b[3] = 0.0;
      b[7] = 0.0;
      b[11] = 0.0;
      b[15] = 1.0;
      break;

     case 2:
      memset(&b_I[0], 0, sizeof(real_T) << 4U);
      b_I[0] = 1.0;
      b_I[5] = 1.0;
      b_I[10] = 1.0;
      b_I[15] = 1.0;
      b_I[12] = qvec[e + 4];
      b_I[13] = qvec[e + 5];
      b_I[14] = qvec[e + 6];
      cth = qvec[e];
      b_q_idx_0 = cth * cth;
      cth = qvec[e + 1];
      b_q_idx_1 = cth * cth;
      cth = qvec[e + 2];
      k = cth * cth;
      cth = qvec[e + 3];
      tmp_0 = _mm_set1_pd(1.0 / sqrt(((b_q_idx_0 + b_q_idx_1) + k) + cth * cth));
      _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_loadu_pd(&qvec[e]), tmp_0));
      b_q_idx_0 = tmp[0];
      b_q_idx_1 = tmp[1];
      _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_loadu_pd(&qvec[e + 2]), tmp_0));
      k = b_q_idx_1 * tmp[0];
      cth = b_q_idx_0 * tmp[1];
      tempR_tmp_tmp = tmp[1] * tmp[1];
      cth_tmp = b_q_idx_1 * tmp[1];
      tempR_tmp = b_q_idx_0 * tmp[0];
      theta = tmp[0] * tmp[1];
      tmp_5 = b_q_idx_0 * b_q_idx_1;
      b_q_idx_1 *= b_q_idx_1;
      b_q_idx_0 = tmp[0] * tmp[0];
      FK_cat(1.0 - (b_q_idx_0 + tempR_tmp_tmp) * 2.0, (k - cth) * 2.0, (cth_tmp
              + tempR_tmp) * 2.0, (k + cth) * 2.0, 1.0 - (b_q_idx_1 +
              tempR_tmp_tmp) * 2.0, (theta - tmp_5) * 2.0, (cth_tmp - tempR_tmp)
             * 2.0, (theta + tmp_5) * 2.0, 1.0 - (b_q_idx_1 + b_q_idx_0) * 2.0,
             tempR);
      for (c = 0; c < 3; c++) {
        R[c] = tempR[c * 3];
        R[c + 3] = tempR[c * 3 + 1];
        R[c + 6] = tempR[c * 3 + 2];
      }

      memset(&b_0[0], 0, sizeof(real_T) << 4U);
      for (i = 0; i < 3; i++) {
        c = i << 2;
        b_0[c] = R[3 * i];
        b_0[c + 1] = R[3 * i + 1];
        b_0[c + 2] = R[3 * i + 2];
      }

      b_0[15] = 1.0;
      for (i = 0; i < 4; i++) {
        k = b_I[i + 4];
        cth_tmp = b_I[i];
        b_q_idx_0 = b_I[i + 8];
        b_q_idx_1 = b_I[i + 12];
        for (c = 0; c <= 2; c += 2) {
          e = (c + 1) << 2;
          d = c << 2;
          _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(b_0[e + 1], b_0[d + 1]), _mm_set1_pd(k)), _mm_mul_pd
            (_mm_set_pd(b_0[e], b_0[d]), _mm_set1_pd(cth_tmp))), _mm_mul_pd
            (_mm_set_pd(b_0[e + 2], b_0[d + 2]), _mm_set1_pd(b_q_idx_0))),
            _mm_mul_pd(_mm_set_pd(b_0[e + 3], b_0[d + 3]), _mm_set1_pd(b_q_idx_1))));
          b[i + d] = tmp[0];
          b[i + e] = tmp[1];
        }
      }
      break;

     default:
      memset(&b[0], 0, sizeof(real_T) << 4U);
      b[0] = 1.0;
      b[5] = 1.0;
      b[10] = 1.0;
      b[15] = 1.0;
      break;
    }

    for (i = 0; i < 16; i++) {
      b_0[i] = body->JointInternal.ChildToJointTransform[i];
    }

    for (i = 0; i < 4; i++) {
      cth_tmp = a[i + 4];
      b_q_idx_0 = a[i];
      b_q_idx_1 = a[i + 8];
      tempR_tmp = a[i + 12];
      for (c = 0; c <= 2; c += 2) {
        e = (c + 1) << 2;
        d = c << 2;
        _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(b[e + 1], b[d + 1]), _mm_set1_pd(cth_tmp)), _mm_mul_pd
          (_mm_set_pd(b[e], b[d]), _mm_set1_pd(b_q_idx_0))), _mm_mul_pd
          (_mm_set_pd(b[e + 2], b[d + 2]), _mm_set1_pd(b_q_idx_1))), _mm_mul_pd
          (_mm_set_pd(b[e + 3], b[d + 3]), _mm_set1_pd(tempR_tmp))));
        b_I[i + d] = tmp[0];
        b_I[i + e] = tmp[1];
      }

      cth_tmp = b_I[i + 4];
      b_q_idx_0 = b_I[i];
      b_q_idx_1 = b_I[i + 8];
      tempR_tmp = b_I[i + 12];
      for (c = 0; c <= 2; c += 2) {
        e = (c + 1) << 2;
        d = c << 2;
        _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(b_0[e + 1], b_0[d + 1]), _mm_set1_pd(cth_tmp)), _mm_mul_pd
          (_mm_set_pd(b_0[e], b_0[d]), _mm_set1_pd(b_q_idx_0))), _mm_mul_pd
          (_mm_set_pd(b_0[e + 2], b_0[d + 2]), _mm_set1_pd(b_q_idx_1))),
          _mm_mul_pd(_mm_set_pd(b_0[e + 3], b_0[d + 3]), _mm_set1_pd(tempR_tmp))));
        Ttree_data[b_jtilecol].f1[i + d] = tmp[0];
        Ttree_data[b_jtilecol].f1[i + e] = tmp[1];
      }
    }

    k = n;
    if (body->ParentIndex > 0.0) {
      for (i = 0; i < 16; i++) {
        a[i] = Ttree_data[(int32_T)body->ParentIndex - 1].f1[i];
      }

      for (i = 0; i < 4; i++) {
        cth_tmp = a[i + 4];
        b_q_idx_0 = a[i];
        b_q_idx_1 = a[i + 8];
        tempR_tmp = a[i + 12];
        for (c = 0; c <= 2; c += 2) {
          e = (c + 1) << 2;
          d = c << 2;
          _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(Ttree_data[b_jtilecol].f1[e + 1], Ttree_data[b_jtilecol]
                        .f1[d + 1]), _mm_set1_pd(cth_tmp)), _mm_mul_pd
            (_mm_set_pd(Ttree_data[b_jtilecol].f1[e], Ttree_data[b_jtilecol]
                        .f1[d]), _mm_set1_pd(b_q_idx_0))), _mm_mul_pd(_mm_set_pd
            (Ttree_data[b_jtilecol].f1[e + 2], Ttree_data[b_jtilecol].f1[d + 2]),
            _mm_set1_pd(b_q_idx_1))), _mm_mul_pd(_mm_set_pd
            (Ttree_data[b_jtilecol].f1[e + 3], Ttree_data[b_jtilecol].f1[d + 3]),
            _mm_set1_pd(tempR_tmp))));
          b_I[i + d] = tmp[0];
          b_I[i + e] = tmp[1];
        }
      }

      memcpy(&Ttree_data[b_jtilecol].f1[0], &b_I[0], sizeof(real_T) << 4U);
    }
  }
}

static void emxFree_f_robotics_manip_intern(emxArray_f_robotics_manip_int_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_robotics_manip_int_T *)NULL) {
    if (((*pEmxArray)->data != (f_robotics_manip_internal_Col_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_robotics_manip_int_T *)NULL;
  }
}

static void emxFreeStruct_g_robotics_manip_(g_robotics_manip_internal_Col_T
  *pStruct)
{
  emxFree_f_robotics_manip_intern(&pStruct->CollisionGeometries);
}

static void emxFreeStruct_e_robotics_manip_(e_robotics_manip_internal_R_m_T
  *pStruct)
{
  emxFreeStruct_g_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxFreeMatrix_e_robotics_manip_(e_robotics_manip_internal_R_m_T
  pMatrix[34])
{
  int32_T i;
  for (i = 0; i < 34; i++) {
    emxFreeStruct_e_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_R_m_T
  *pStruct)
{
  emxFreeStruct_e_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_e_robotics_manip_(pStruct->_pobj0);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_b_m_T
  *pStruct)
{
  emxFreeStruct_f_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_e_robotics_mani_m(e_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_g_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxFreeMatrix_e_robotics_mani_m(e_robotics_manip_internal_Rig_T
  pMatrix[34])
{
  int32_T i;
  for (i = 0; i < 34; i++) {
    emxFreeStruct_e_robotics_mani_m(&pMatrix[i]);
  }
}

static void emxFreeStruct_f_robotics_mani_m(f_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_e_robotics_mani_m(&pStruct->Base);
  emxFreeMatrix_e_robotics_mani_m(pStruct->_pobj0);
}

static void emxFreeStruct_robotics_slmani_m(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_f_robotics_mani_m(&pStruct->TreeInternal);
}

/* Model step function */
void FK_step(void)
{
  __m128d tmp_4;
  __m128d tmp_5;
  __m128d tmp_6;
  __m128d tmp_7;
  e_robotics_manip_internal_R_m_T *obj;
  e_robotics_manip_internal_Rig_T *body;
  h_cell_wrap_FK_T Ttree_data[17];
  real_T rtb_J[72];
  real_T rtb_Jb_out_a_tmp_0[72];
  real_T rtb_Jdot[72];
  real_T JacSlice_data[36];
  real_T X[36];
  real_T b_data[36];
  real_T rtb_Jb_out_n[36];
  real_T T2[16];
  real_T T2inv[16];
  real_T T2inv_0[16];
  real_T Tdh[16];
  real_T rtb_MATLABSystem[16];
  real_T rtb_MATLABSystem_g[16];
  real_T rtb_T[16];
  real_T omgmat[9];
  real_T omgmat_0[9];
  real_T c_0[6];
  real_T dJidt[6];
  real_T rtb_T_0[3];
  real_T tmp_8[2];
  real_T T2_0;
  real_T bid1;
  real_T idx_idx_1;
  real_T tmp_0;
  real_T velnum;
  int32_T b;
  int32_T c;
  int32_T coffset_tmp;
  int32_T i;
  int32_T i_0;
  int32_T loop_ub_tmp;
  int32_T n;
  int32_T omgmat_tmp;
  char_T obj_Vector[200];
  int8_T rtb_Jb_out_a_tmp[36];
  int8_T chainmask[17];
  int8_T tmp[3];
  int8_T tmp_1;
  int8_T tmp_2;
  int8_T tmp_3;
  boolean_T b_bool;
  static const int8_T y[9] = { 0, 1, 0, -1, 0, 0, 0, 0, 0 };

  static const int8_T b_y[3] = { 0, 0, 1 };

  static const int8_T c_1[6] = { 0, 0, 0, 0, 0, 1 };

  static const char_T tmp_9[5] = { 'r', '_', 't', 'c', 'p' };

  static const char_T tmp_a[5] = { 'w', 'o', 'r', 'l', 'd' };

  static const char_T tmp_b[5] = { 'l', '_', 't', 'c', 'p' };

  static const char_T tmp_c[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const int8_T a[36] = { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };

  emxArray_h_cell_wrap_1x17_FK_T obj_i;
  int32_T Ttree_size[2];
  int32_T B_tmp;
  int32_T exitg1;
  boolean_T exitg2;

  /* MATLAB Function: '<S1>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/q'
   */
  FK_MATLABFunction1(FK_U.q, &FK_B.sf_MATLABFunction1);

  /* MATLAB Function: '<S1>/MATLAB Function2' incorporates:
   *  Inport: '<Root>/qdot'
   */
  FK_MATLABFunction1(FK_U.qdot, &FK_B.sf_MATLABFunction2);

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Inport: '<Root>/lambda_lr'
   */
  memset(&rtb_J[0], 0, 72U * sizeof(real_T));
  FK_exp6(&FK_U.lambda_lr[72], rtb_T);
  for (i_0 = 0; i_0 < 9; i_0++) {
    omgmat[i_0] = y[i_0];
  }

  for (i = 0; i < 3; i++) {
    tmp[i] = b_y[i];
    bid1 = 0.0;
    for (i_0 = 0; i_0 < 3; i_0++) {
      n = i << 2;
      bid1 += ((omgmat[3 * i_0 + 1] * rtb_T[n + 1] + omgmat[3 * i_0] * rtb_T[n])
               + omgmat[3 * i_0 + 2] * rtb_T[n + 2]) * rtb_T[i_0 + 12];
    }

    rtb_T_0[i] = bid1;
  }

  tmp_1 = tmp[1];
  tmp_2 = tmp[0];
  tmp_3 = tmp[2];
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_J[i_0 + 66] = rtb_T_0[i_0];
    n = i_0 << 2;
    rtb_J[i_0 + 69] = (rtb_T[n + 1] * (real_T)tmp_1 + rtb_T[n] * (real_T)tmp_2)
      + rtb_T[n + 2] * (real_T)tmp_3;
  }

  for (i = 0; i < 12; i++) {
    FK_exp6(&FK_U.lambda_lr[(11 - i) * 6], T2);
    velnum = FK_B.sf_MATLABFunction1.q_lr[11 - i];
    for (i_0 = 0; i_0 <= 4; i_0 += 2) {
      _mm_storeu_pd(&c_0[i_0], _mm_mul_pd(_mm_set_pd(c_1[i_0 + 1], c_1[i_0]),
        _mm_set1_pd(velnum)));
    }

    FK_exp6(c_0, rtb_MATLABSystem);
    for (i_0 = 0; i_0 < 4; i_0++) {
      bid1 = T2[i_0 + 4];
      idx_idx_1 = T2[i_0];
      velnum = T2[i_0 + 8];
      T2_0 = T2[i_0 + 12];
      for (b = 0; b <= 2; b += 2) {
        omgmat_tmp = (b + 1) << 2;
        coffset_tmp = b << 2;
        _mm_storeu_pd(&tmp_8[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(rtb_MATLABSystem[omgmat_tmp + 1],
                      rtb_MATLABSystem[coffset_tmp + 1]), _mm_set1_pd(bid1)),
          _mm_mul_pd(_mm_set_pd(rtb_MATLABSystem[omgmat_tmp],
          rtb_MATLABSystem[coffset_tmp]), _mm_set1_pd(idx_idx_1))), _mm_mul_pd
          (_mm_set_pd(rtb_MATLABSystem[omgmat_tmp + 2],
                      rtb_MATLABSystem[coffset_tmp + 2]), _mm_set1_pd(velnum))),
          _mm_mul_pd(_mm_set_pd(rtb_MATLABSystem[omgmat_tmp + 3],
          rtb_MATLABSystem[coffset_tmp + 3]), _mm_set1_pd(T2_0))));
        rtb_MATLABSystem_g[i_0 + coffset_tmp] = tmp_8[0];
        rtb_MATLABSystem_g[i_0 + omgmat_tmp] = tmp_8[1];
      }

      bid1 = rtb_MATLABSystem_g[i_0 + 4];
      idx_idx_1 = rtb_MATLABSystem_g[i_0];
      velnum = rtb_MATLABSystem_g[i_0 + 8];
      T2_0 = rtb_MATLABSystem_g[i_0 + 12];
      for (b = 0; b <= 2; b += 2) {
        omgmat_tmp = (b + 1) << 2;
        coffset_tmp = b << 2;
        _mm_storeu_pd(&tmp_8[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set_pd(rtb_T[omgmat_tmp + 1], rtb_T[coffset_tmp + 1]),
           _mm_set1_pd(bid1)), _mm_mul_pd(_mm_set_pd(rtb_T[omgmat_tmp],
          rtb_T[coffset_tmp]), _mm_set1_pd(idx_idx_1))), _mm_mul_pd(_mm_set_pd
          (rtb_T[omgmat_tmp + 2], rtb_T[coffset_tmp + 2]), _mm_set1_pd(velnum))),
          _mm_mul_pd(_mm_set_pd(rtb_T[omgmat_tmp + 3], rtb_T[coffset_tmp + 3]),
                     _mm_set1_pd(T2_0))));
        T2inv[i_0 + coffset_tmp] = tmp_8[0];
        T2inv[i_0 + omgmat_tmp] = tmp_8[1];
      }
    }

    memcpy(&rtb_T[0], &T2inv[0], sizeof(real_T) << 4U);
    if (12 - i > 1) {
      for (i_0 = 0; i_0 < 3; i_0++) {
        bid1 = 0.0;
        idx_idx_1 = 0.0;
        for (b = 0; b < 3; b++) {
          n = i_0 << 2;
          bid1 += ((omgmat[3 * b + 1] * rtb_T[n + 1] + omgmat[3 * b] * rtb_T[n])
                   + omgmat[3 * b + 2] * rtb_T[n + 2]) * rtb_T[b + 12];
          idx_idx_1 += rtb_T[n + b] * (real_T)tmp[b];
        }

        n = (10 - i) * 6 + i_0;
        rtb_J[n] = bid1;
        rtb_J[n + 3] = idx_idx_1;
      }
    }
  }

  for (n = 0; n < 12; n++) {
    for (i = 0; i < 6; i++) {
      dJidt[i] = 0.0;
    }

    for (i = 0; i < 12; i++) {
      for (i_0 = 0; i_0 < 6; i_0++) {
        c_0[i_0] = 0.0;
      }

      if (n < i) {
        omgmat[0] = 0.0;
        bid1 = rtb_J[6 * n + 5];
        omgmat[3] = -bid1;
        idx_idx_1 = rtb_J[6 * n + 4];
        omgmat[6] = idx_idx_1;
        omgmat[1] = bid1;
        omgmat[4] = 0.0;
        bid1 = rtb_J[6 * n + 3];
        omgmat[7] = -bid1;
        omgmat[2] = -idx_idx_1;
        omgmat[5] = bid1;
        omgmat[8] = 0.0;
        X[18] = 0.0;
        bid1 = rtb_J[6 * n + 2];
        X[24] = -bid1;
        idx_idx_1 = rtb_J[6 * n + 1];
        X[30] = idx_idx_1;
        X[19] = bid1;
        X[25] = 0.0;
        bid1 = rtb_J[6 * n];
        X[31] = -bid1;
        X[20] = -idx_idx_1;
        X[26] = bid1;
        X[32] = 0.0;
        for (i_0 = 0; i_0 < 3; i_0++) {
          bid1 = omgmat[3 * i_0];
          X[6 * i_0] = bid1;
          X[6 * i_0 + 3] = 0.0;
          omgmat_tmp = (i_0 + 3) * 6;
          X[omgmat_tmp + 3] = bid1;
          bid1 = omgmat[3 * i_0 + 1];
          X[6 * i_0 + 1] = bid1;
          X[6 * i_0 + 4] = 0.0;
          X[omgmat_tmp + 4] = bid1;
          bid1 = omgmat[3 * i_0 + 2];
          X[6 * i_0 + 2] = bid1;
          X[6 * i_0 + 5] = 0.0;
          X[omgmat_tmp + 5] = bid1;
        }

        for (i_0 = 0; i_0 < 6; i_0++) {
          bid1 = 0.0;
          for (b = 0; b < 6; b++) {
            bid1 += X[6 * b + i_0] * rtb_J[6 * i + b];
          }

          c_0[i_0] = bid1;
        }
      }

      velnum = FK_B.sf_MATLABFunction2.q_lr[i];
      for (i_0 = 0; i_0 <= 4; i_0 += 2) {
        tmp_6 = _mm_loadu_pd(&c_0[i_0]);
        tmp_7 = _mm_loadu_pd(&dJidt[i_0]);
        _mm_storeu_pd(&dJidt[i_0], _mm_add_pd(_mm_mul_pd(tmp_6, _mm_set1_pd
          (velnum)), tmp_7));
      }
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      rtb_Jdot[i_0 + 6 * n] = dJidt[i_0];
    }
  }

  /* End of MATLAB Function: '<S1>/MATLAB Function' */

  /* Outport: '<Root>/T_lr' */
  memcpy(&FK_Y.T_lr[0], &rtb_T[0], sizeof(real_T) << 4U);

  /* MATLABSystem: '<S6>/MATLAB System' incorporates:
   *  Inport: '<Root>/q'
   */
  RigidBodyTree_forwardKinemat_ml(&FK_DW.obj_g.TreeInternal, FK_U.q, Ttree_data,
    Ttree_size);
  bid1 = -1.0;
  velnum = FK_DW.obj_g.TreeInternal.Base.NameInternal.Length;
  memcpy(&obj_Vector[0], &FK_DW.obj_g.TreeInternal.Base.NameInternal.Vector[0],
         200U * sizeof(char_T));
  b_bool = false;
  if (velnum < 1.0) {
    i_0 = 0;
  } else {
    i_0 = (int32_T)velnum;
  }

  if (i_0 == 5) {
    omgmat_tmp = 1;
    do {
      exitg1 = 0;
      if (omgmat_tmp - 1 < 5) {
        if (obj_Vector[omgmat_tmp - 1] != tmp_9[omgmat_tmp - 1]) {
          exitg1 = 1;
        } else {
          omgmat_tmp++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    bid1 = 0.0;
  } else {
    idx_idx_1 = FK_DW.obj_g.TreeInternal.NumBodies;
    n = 0;
    exitg2 = false;
    while ((!exitg2) && (n <= (int32_T)idx_idx_1 - 1)) {
      obj = FK_DW.obj_g.TreeInternal.Bodies[n];
      velnum = obj->NameInternal.Length;
      for (i_0 = 0; i_0 < 200; i_0++) {
        obj_Vector[i_0] = obj->NameInternal.Vector[i_0];
      }

      if (velnum < 1.0) {
        i_0 = 0;
      } else {
        i_0 = (int32_T)velnum;
      }

      if (i_0 == 5) {
        omgmat_tmp = 1;
        do {
          exitg1 = 0;
          if (omgmat_tmp - 1 < 5) {
            if (obj_Vector[omgmat_tmp - 1] != tmp_9[omgmat_tmp - 1]) {
              exitg1 = 1;
            } else {
              omgmat_tmp++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_bool) {
        bid1 = (real_T)n + 1.0;
        exitg2 = true;
      } else {
        n++;
      }
    }
  }

  if (bid1 == 0.0) {
    memset(&rtb_T[0], 0, sizeof(real_T) << 4U);
    rtb_T[0] = 1.0;
    rtb_T[5] = 1.0;
    rtb_T[10] = 1.0;
    rtb_T[15] = 1.0;
  } else {
    for (i_0 = 0; i_0 < 16; i_0++) {
      rtb_T[i_0] = Ttree_data[(int32_T)bid1 - 1].f1[i_0];
    }
  }

  bid1 = -1.0;
  velnum = FK_DW.obj_g.TreeInternal.Base.NameInternal.Length;
  memcpy(&obj_Vector[0], &FK_DW.obj_g.TreeInternal.Base.NameInternal.Vector[0],
         200U * sizeof(char_T));
  b_bool = false;
  if (velnum < 1.0) {
    i_0 = 0;
  } else {
    i_0 = (int32_T)velnum;
  }

  if (i_0 == 5) {
    omgmat_tmp = 1;
    do {
      exitg1 = 0;
      if (omgmat_tmp - 1 < 5) {
        if (obj_Vector[omgmat_tmp - 1] != tmp_a[omgmat_tmp - 1]) {
          exitg1 = 1;
        } else {
          omgmat_tmp++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    bid1 = 0.0;
  } else {
    idx_idx_1 = FK_DW.obj_g.TreeInternal.NumBodies;
    n = 0;
    exitg2 = false;
    while ((!exitg2) && (n <= (int32_T)idx_idx_1 - 1)) {
      obj = FK_DW.obj_g.TreeInternal.Bodies[n];
      velnum = obj->NameInternal.Length;
      for (i_0 = 0; i_0 < 200; i_0++) {
        obj_Vector[i_0] = obj->NameInternal.Vector[i_0];
      }

      if (velnum < 1.0) {
        i_0 = 0;
      } else {
        i_0 = (int32_T)velnum;
      }

      if (i_0 == 5) {
        omgmat_tmp = 1;
        do {
          exitg1 = 0;
          if (omgmat_tmp - 1 < 5) {
            if (obj_Vector[omgmat_tmp - 1] != tmp_a[omgmat_tmp - 1]) {
              exitg1 = 1;
            } else {
              omgmat_tmp++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_bool) {
        bid1 = (real_T)n + 1.0;
        exitg2 = true;
      } else {
        n++;
      }
    }
  }

  if (bid1 == 0.0) {
    memset(&T2[0], 0, sizeof(real_T) << 4U);
    T2[0] = 1.0;
    T2[5] = 1.0;
    T2[10] = 1.0;
    T2[15] = 1.0;
  } else {
    for (i_0 = 0; i_0 < 16; i_0++) {
      T2[i_0] = Ttree_data[(int32_T)bid1 - 1].f1[i_0];
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    omgmat[3 * i_0] = T2[i_0];
    omgmat[3 * i_0 + 1] = T2[i_0 + 4];
    omgmat[3 * i_0 + 2] = T2[i_0 + 8];
  }

  for (i_0 = 0; i_0 <= 6; i_0 += 2) {
    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    tmp_6 = _mm_loadu_pd(&omgmat[i_0]);
    _mm_storeu_pd(&omgmat_0[i_0], _mm_mul_pd(tmp_6, _mm_set1_pd(-1.0)));
  }

  for (i_0 = 8; i_0 < 9; i_0++) {
    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    omgmat_0[i_0] = -omgmat[i_0];
  }

  /* Start for MATLABSystem: '<S6>/MATLAB System' */
  bid1 = T2[13];
  idx_idx_1 = T2[12];
  velnum = T2[14];
  for (i_0 = 0; i_0 < 3; i_0++) {
    omgmat_tmp = i_0 << 2;
    Tdh[omgmat_tmp] = omgmat[3 * i_0];
    Tdh[omgmat_tmp + 1] = omgmat[3 * i_0 + 1];
    Tdh[omgmat_tmp + 2] = omgmat[3 * i_0 + 2];
    Tdh[i_0 + 12] = (omgmat_0[i_0 + 3] * bid1 + omgmat_0[i_0] * idx_idx_1) +
      omgmat_0[i_0 + 6] * velnum;
  }

  Tdh[3] = 0.0;
  Tdh[7] = 0.0;
  Tdh[11] = 0.0;
  Tdh[15] = 1.0;

  /* MATLABSystem: '<S6>/MATLAB System' */
  for (i_0 = 0; i_0 < 4; i_0++) {
    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    n = i_0 << 2;
    bid1 = rtb_T[n + 1];
    idx_idx_1 = rtb_T[n];
    velnum = rtb_T[n + 2];
    T2_0 = rtb_T[n + 3];
    for (b = 0; b <= 2; b += 2) {
      tmp_6 = _mm_loadu_pd(&Tdh[b + 4]);
      tmp_7 = _mm_loadu_pd(&Tdh[b]);
      tmp_4 = _mm_loadu_pd(&Tdh[b + 8]);
      tmp_5 = _mm_loadu_pd(&Tdh[b + 12]);
      _mm_storeu_pd(&rtb_MATLABSystem[b + n], _mm_add_pd(_mm_add_pd(_mm_add_pd
        (_mm_mul_pd(_mm_set1_pd(bid1), tmp_6), _mm_mul_pd(_mm_set1_pd(idx_idx_1),
        tmp_7)), _mm_mul_pd(_mm_set1_pd(velnum), tmp_4)), _mm_mul_pd(_mm_set1_pd
        (T2_0), tmp_5)));
    }
  }

  /* Outport: '<Root>/T_r' incorporates:
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  memcpy(&FK_Y.T_r[0], &rtb_MATLABSystem[0], sizeof(real_T) << 4U);

  /* MATLABSystem: '<S5>/MATLAB System' incorporates:
   *  Inport: '<Root>/q'
   */
  RigidBodyTree_forwardKinemat_ml(&FK_DW.obj_b.TreeInternal, FK_U.q, Ttree_data,
    Ttree_size);
  bid1 = -1.0;
  velnum = FK_DW.obj_b.TreeInternal.Base.NameInternal.Length;
  memcpy(&obj_Vector[0], &FK_DW.obj_b.TreeInternal.Base.NameInternal.Vector[0],
         200U * sizeof(char_T));
  b_bool = false;
  if (velnum < 1.0) {
    i_0 = 0;
  } else {
    i_0 = (int32_T)velnum;
  }

  if (i_0 == 5) {
    omgmat_tmp = 1;
    do {
      exitg1 = 0;
      if (omgmat_tmp - 1 < 5) {
        if (obj_Vector[omgmat_tmp - 1] != tmp_b[omgmat_tmp - 1]) {
          exitg1 = 1;
        } else {
          omgmat_tmp++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    bid1 = 0.0;
  } else {
    idx_idx_1 = FK_DW.obj_b.TreeInternal.NumBodies;
    n = 0;
    exitg2 = false;
    while ((!exitg2) && (n <= (int32_T)idx_idx_1 - 1)) {
      obj = FK_DW.obj_b.TreeInternal.Bodies[n];
      velnum = obj->NameInternal.Length;
      for (i_0 = 0; i_0 < 200; i_0++) {
        obj_Vector[i_0] = obj->NameInternal.Vector[i_0];
      }

      if (velnum < 1.0) {
        i_0 = 0;
      } else {
        i_0 = (int32_T)velnum;
      }

      if (i_0 == 5) {
        omgmat_tmp = 1;
        do {
          exitg1 = 0;
          if (omgmat_tmp - 1 < 5) {
            if (obj_Vector[omgmat_tmp - 1] != tmp_b[omgmat_tmp - 1]) {
              exitg1 = 1;
            } else {
              omgmat_tmp++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_bool) {
        bid1 = (real_T)n + 1.0;
        exitg2 = true;
      } else {
        n++;
      }
    }
  }

  if (bid1 == 0.0) {
    memset(&rtb_T[0], 0, sizeof(real_T) << 4U);
    rtb_T[0] = 1.0;
    rtb_T[5] = 1.0;
    rtb_T[10] = 1.0;
    rtb_T[15] = 1.0;
  } else {
    for (i_0 = 0; i_0 < 16; i_0++) {
      rtb_T[i_0] = Ttree_data[(int32_T)bid1 - 1].f1[i_0];
    }
  }

  bid1 = -1.0;
  velnum = FK_DW.obj_b.TreeInternal.Base.NameInternal.Length;
  memcpy(&obj_Vector[0], &FK_DW.obj_b.TreeInternal.Base.NameInternal.Vector[0],
         200U * sizeof(char_T));
  b_bool = false;
  if (velnum < 1.0) {
    i_0 = 0;
  } else {
    i_0 = (int32_T)velnum;
  }

  if (i_0 == 5) {
    omgmat_tmp = 1;
    do {
      exitg1 = 0;
      if (omgmat_tmp - 1 < 5) {
        if (obj_Vector[omgmat_tmp - 1] != tmp_a[omgmat_tmp - 1]) {
          exitg1 = 1;
        } else {
          omgmat_tmp++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    bid1 = 0.0;
  } else {
    idx_idx_1 = FK_DW.obj_b.TreeInternal.NumBodies;
    n = 0;
    exitg2 = false;
    while ((!exitg2) && (n <= (int32_T)idx_idx_1 - 1)) {
      obj = FK_DW.obj_b.TreeInternal.Bodies[n];
      velnum = obj->NameInternal.Length;
      for (i_0 = 0; i_0 < 200; i_0++) {
        obj_Vector[i_0] = obj->NameInternal.Vector[i_0];
      }

      if (velnum < 1.0) {
        i_0 = 0;
      } else {
        i_0 = (int32_T)velnum;
      }

      if (i_0 == 5) {
        omgmat_tmp = 1;
        do {
          exitg1 = 0;
          if (omgmat_tmp - 1 < 5) {
            if (obj_Vector[omgmat_tmp - 1] != tmp_a[omgmat_tmp - 1]) {
              exitg1 = 1;
            } else {
              omgmat_tmp++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_bool) {
        bid1 = (real_T)n + 1.0;
        exitg2 = true;
      } else {
        n++;
      }
    }
  }

  if (bid1 == 0.0) {
    memset(&T2[0], 0, sizeof(real_T) << 4U);
    T2[0] = 1.0;
    T2[5] = 1.0;
    T2[10] = 1.0;
    T2[15] = 1.0;
  } else {
    for (i_0 = 0; i_0 < 16; i_0++) {
      T2[i_0] = Ttree_data[(int32_T)bid1 - 1].f1[i_0];
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    omgmat[3 * i_0] = T2[i_0];
    omgmat[3 * i_0 + 1] = T2[i_0 + 4];
    omgmat[3 * i_0 + 2] = T2[i_0 + 8];
  }

  for (i_0 = 0; i_0 <= 6; i_0 += 2) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    tmp_6 = _mm_loadu_pd(&omgmat[i_0]);
    _mm_storeu_pd(&omgmat_0[i_0], _mm_mul_pd(tmp_6, _mm_set1_pd(-1.0)));
  }

  for (i_0 = 8; i_0 < 9; i_0++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    omgmat_0[i_0] = -omgmat[i_0];
  }

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  bid1 = T2[13];
  idx_idx_1 = T2[12];
  velnum = T2[14];
  for (i_0 = 0; i_0 < 3; i_0++) {
    omgmat_tmp = i_0 << 2;
    Tdh[omgmat_tmp] = omgmat[3 * i_0];
    Tdh[omgmat_tmp + 1] = omgmat[3 * i_0 + 1];
    Tdh[omgmat_tmp + 2] = omgmat[3 * i_0 + 2];
    Tdh[i_0 + 12] = (omgmat_0[i_0 + 3] * bid1 + omgmat_0[i_0] * idx_idx_1) +
      omgmat_0[i_0 + 6] * velnum;
  }

  Tdh[3] = 0.0;
  Tdh[7] = 0.0;
  Tdh[11] = 0.0;
  Tdh[15] = 1.0;

  /* MATLABSystem: '<S5>/MATLAB System' */
  for (i_0 = 0; i_0 < 4; i_0++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    n = i_0 << 2;
    bid1 = rtb_T[n + 1];
    idx_idx_1 = rtb_T[n];
    velnum = rtb_T[n + 2];
    T2_0 = rtb_T[n + 3];
    for (b = 0; b <= 2; b += 2) {
      tmp_6 = _mm_loadu_pd(&Tdh[b + 4]);
      tmp_7 = _mm_loadu_pd(&Tdh[b]);
      tmp_4 = _mm_loadu_pd(&Tdh[b + 8]);
      tmp_5 = _mm_loadu_pd(&Tdh[b + 12]);
      _mm_storeu_pd(&rtb_MATLABSystem_g[b + n], _mm_add_pd(_mm_add_pd(_mm_add_pd
        (_mm_mul_pd(_mm_set1_pd(bid1), tmp_6), _mm_mul_pd(_mm_set1_pd(idx_idx_1),
        tmp_7)), _mm_mul_pd(_mm_set1_pd(velnum), tmp_4)), _mm_mul_pd(_mm_set1_pd
        (T2_0), tmp_5)));
    }
  }

  /* Outport: '<Root>/T_l' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  memcpy(&FK_Y.T_l[0], &rtb_MATLABSystem_g[0], sizeof(real_T) << 4U);

  /* MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  Inport: '<Root>/q'
   */
  RigidBodyTree_forwardKinematics(&FK_DW.obj_k.TreeInternal, FK_U.q, Ttree_data,
    Ttree_size);
  velnum = FK_DW.obj_k.TreeInternal.VelocityNumber;
  B_tmp = (int32_T)velnum;
  loop_ub_tmp = 6 * (int32_T)velnum;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&FK_B.b_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
  }

  for (i_0 = 0; i_0 < 17; i_0++) {
    chainmask[i_0] = 0;
  }

  velnum = FK_DW.obj_k.TreeInternal.Base.NameInternal.Length;
  memcpy(&obj_Vector[0], &FK_DW.obj_k.TreeInternal.Base.NameInternal.Vector[0],
         200U * sizeof(char_T));
  b_bool = false;
  if (velnum < 1.0) {
    i_0 = 0;
  } else {
    i_0 = (int32_T)velnum;
  }

  if (i_0 == 5) {
    omgmat_tmp = 1;
    do {
      exitg1 = 0;
      if (omgmat_tmp - 1 < 5) {
        if (tmp_b[omgmat_tmp - 1] != obj_Vector[omgmat_tmp - 1]) {
          exitg1 = 1;
        } else {
          omgmat_tmp++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    memset(&T2inv[0], 0, sizeof(real_T) << 4U);
    T2inv[0] = 1.0;
    T2inv[5] = 1.0;
    T2inv[10] = 1.0;
    T2inv[15] = 1.0;
    memset(&T2[0], 0, sizeof(real_T) << 4U);
    T2[0] = 1.0;
    T2[5] = 1.0;
    T2[10] = 1.0;
    T2[15] = 1.0;
  } else {
    bid1 = -1.0;
    velnum = FK_DW.obj_k.TreeInternal.Base.NameInternal.Length;
    memcpy(&obj_Vector[0], &FK_DW.obj_k.TreeInternal.Base.NameInternal.Vector[0],
           200U * sizeof(char_T));
    if (velnum < 1.0) {
      i_0 = 0;
    } else {
      i_0 = (int32_T)velnum;
    }

    if (i_0 == 5) {
      omgmat_tmp = 1;
      do {
        exitg1 = 0;
        if (omgmat_tmp - 1 < 5) {
          if (obj_Vector[omgmat_tmp - 1] != tmp_b[omgmat_tmp - 1]) {
            exitg1 = 1;
          } else {
            omgmat_tmp++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      bid1 = 0.0;
    } else {
      idx_idx_1 = FK_DW.obj_k.TreeInternal.NumBodies;
      n = 0;
      exitg2 = false;
      while ((!exitg2) && (n <= (int32_T)idx_idx_1 - 1)) {
        body = FK_DW.obj_k.TreeInternal.Bodies[n];
        velnum = body->NameInternal.Length;
        for (i_0 = 0; i_0 < 200; i_0++) {
          obj_Vector[i_0] = body->NameInternal.Vector[i_0];
        }

        if (velnum < 1.0) {
          i_0 = 0;
        } else {
          i_0 = (int32_T)velnum;
        }

        if (i_0 == 5) {
          omgmat_tmp = 1;
          do {
            exitg1 = 0;
            if (omgmat_tmp - 1 < 5) {
              if (obj_Vector[omgmat_tmp - 1] != tmp_b[omgmat_tmp - 1]) {
                exitg1 = 1;
              } else {
                omgmat_tmp++;
              }
            } else {
              b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (b_bool) {
          bid1 = (real_T)n + 1.0;
          exitg2 = true;
        } else {
          n++;
        }
      }
    }

    body = FK_DW.obj_k.TreeInternal.Bodies[(int32_T)bid1 - 1];
    for (i_0 = 0; i_0 < 16; i_0++) {
      T2[i_0] = Ttree_data[(int32_T)bid1 - 1].f1[i_0];
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      omgmat[3 * i_0] = Ttree_data[(int32_T)bid1 - 1].f1[i_0];
      omgmat[3 * i_0 + 1] = Ttree_data[(int32_T)bid1 - 1].f1[i_0 + 4];
      omgmat[3 * i_0 + 2] = Ttree_data[(int32_T)bid1 - 1].f1[i_0 + 8];
    }

    for (i_0 = 0; i_0 <= 6; i_0 += 2) {
      tmp_6 = _mm_loadu_pd(&omgmat[i_0]);
      _mm_storeu_pd(&omgmat_0[i_0], _mm_mul_pd(tmp_6, _mm_set1_pd(-1.0)));
    }

    for (i_0 = 8; i_0 < 9; i_0++) {
      omgmat_0[i_0] = -omgmat[i_0];
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      n = i_0 << 2;
      T2inv[n] = omgmat[3 * i_0];
      T2inv[n + 1] = omgmat[3 * i_0 + 1];
      T2inv[n + 2] = omgmat[3 * i_0 + 2];
      T2inv[i_0 + 12] = (Ttree_data[(int32_T)bid1 - 1].f1[12] * omgmat_0[i_0] +
                         omgmat_0[i_0 + 3] * Ttree_data[(int32_T)bid1 - 1].f1[13])
        + omgmat_0[i_0 + 6] * Ttree_data[(int32_T)bid1 - 1].f1[14];
    }

    T2inv[3] = 0.0;
    T2inv[7] = 0.0;
    T2inv[11] = 0.0;
    T2inv[15] = 1.0;
    chainmask[(int32_T)bid1 - 1] = 1;
    while (body->ParentIndex > 0.0) {
      body = FK_DW.obj_k.TreeInternal.Bodies[(int32_T)body->ParentIndex - 1];
      chainmask[(int32_T)body->Index - 1] = 1;
    }
  }

  idx_idx_1 = FK_DW.obj_k.TreeInternal.NumBodies;
  c = (int32_T)idx_idx_1;
  for (n = 0; n < c; n++) {
    body = FK_DW.obj_k.TreeInternal.Bodies[n];
    velnum = body->JointInternal.TypeInternal.Length;
    for (i_0 = 0; i_0 < 200; i_0++) {
      obj_Vector[i_0] = body->JointInternal.TypeInternal.Vector[i_0];
    }

    b_bool = false;
    if (velnum < 1.0) {
      i_0 = 0;
    } else {
      i_0 = (int32_T)velnum;
    }

    if (i_0 == 5) {
      omgmat_tmp = 1;
      do {
        exitg1 = 0;
        if (omgmat_tmp - 1 < 5) {
          if (obj_Vector[omgmat_tmp - 1] != tmp_c[omgmat_tmp - 1]) {
            exitg1 = 1;
          } else {
            omgmat_tmp++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if ((!b_bool) && (chainmask[n] != 0)) {
      for (i_0 = 0; i_0 < 16; i_0++) {
        rtb_T[i_0] = Ttree_data[(int32_T)body->Index - 1].f1[i_0];
      }

      for (i_0 = 0; i_0 < 16; i_0++) {
        Tdh[i_0] = body->JointInternal.ChildToJointTransform[i_0];
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        omgmat[3 * i_0] = Tdh[i_0];
        omgmat[3 * i_0 + 1] = Tdh[i_0 + 4];
        omgmat[3 * i_0 + 2] = Tdh[i_0 + 8];
      }

      for (i_0 = 0; i_0 <= 6; i_0 += 2) {
        tmp_6 = _mm_loadu_pd(&omgmat[i_0]);
        _mm_storeu_pd(&omgmat_0[i_0], _mm_mul_pd(tmp_6, _mm_set1_pd(-1.0)));
      }

      for (i_0 = 8; i_0 < 9; i_0++) {
        omgmat_0[i_0] = -omgmat[i_0];
      }

      bid1 = Tdh[13];
      idx_idx_1 = Tdh[12];
      velnum = Tdh[14];
      for (i_0 = 0; i_0 <= 0; i_0 += 2) {
        tmp_6 = _mm_loadu_pd(&omgmat_0[i_0 + 3]);
        tmp_7 = _mm_loadu_pd(&omgmat_0[i_0]);
        tmp_4 = _mm_loadu_pd(&omgmat_0[i_0 + 6]);
        _mm_storeu_pd(&rtb_T_0[i_0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_6,
          _mm_set1_pd(bid1)), _mm_mul_pd(tmp_7, _mm_set1_pd(idx_idx_1))),
          _mm_mul_pd(tmp_4, _mm_set1_pd(velnum))));
      }

      for (i_0 = 2; i_0 < 3; i_0++) {
        rtb_T_0[i_0] = (omgmat_0[i_0 + 3] * bid1 + omgmat_0[i_0] * idx_idx_1) +
          omgmat_0[i_0 + 6] * velnum;
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        bid1 = T2inv[i_0 + 4];
        idx_idx_1 = T2inv[i_0];
        velnum = T2inv[i_0 + 8];
        T2_0 = T2inv[i_0 + 12];
        for (b = 0; b <= 2; b += 2) {
          omgmat_tmp = (b + 1) << 2;
          coffset_tmp = b << 2;
          _mm_storeu_pd(&tmp_8[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(rtb_T[omgmat_tmp + 1], rtb_T[coffset_tmp + 1]),
             _mm_set1_pd(bid1)), _mm_mul_pd(_mm_set_pd(rtb_T[omgmat_tmp],
            rtb_T[coffset_tmp]), _mm_set1_pd(idx_idx_1))), _mm_mul_pd(_mm_set_pd
            (rtb_T[omgmat_tmp + 2], rtb_T[coffset_tmp + 2]), _mm_set1_pd(velnum))),
            _mm_mul_pd(_mm_set_pd(rtb_T[omgmat_tmp + 3], rtb_T[coffset_tmp + 3]),
                       _mm_set1_pd(T2_0))));
          T2inv_0[i_0 + coffset_tmp] = tmp_8[0];
          T2inv_0[i_0 + omgmat_tmp] = tmp_8[1];
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        omgmat_tmp = i_0 << 2;
        Tdh[omgmat_tmp] = omgmat[3 * i_0];
        Tdh[omgmat_tmp + 1] = omgmat[3 * i_0 + 1];
        Tdh[omgmat_tmp + 2] = omgmat[3 * i_0 + 2];
        Tdh[i_0 + 12] = rtb_T_0[i_0];
      }

      Tdh[3] = 0.0;
      Tdh[7] = 0.0;
      Tdh[11] = 0.0;
      Tdh[15] = 1.0;
      for (i_0 = 0; i_0 < 4; i_0++) {
        bid1 = T2inv_0[i_0 + 4];
        idx_idx_1 = T2inv_0[i_0];
        velnum = T2inv_0[i_0 + 8];
        T2_0 = T2inv_0[i_0 + 12];
        for (b = 0; b <= 2; b += 2) {
          omgmat_tmp = (b + 1) << 2;
          coffset_tmp = b << 2;
          _mm_storeu_pd(&tmp_8[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(Tdh[omgmat_tmp + 1], Tdh[coffset_tmp + 1]), _mm_set1_pd
             (bid1)), _mm_mul_pd(_mm_set_pd(Tdh[omgmat_tmp], Tdh[coffset_tmp]),
            _mm_set1_pd(idx_idx_1))), _mm_mul_pd(_mm_set_pd(Tdh[omgmat_tmp + 2],
            Tdh[coffset_tmp + 2]), _mm_set1_pd(velnum))), _mm_mul_pd(_mm_set_pd
            (Tdh[omgmat_tmp + 3], Tdh[coffset_tmp + 3]), _mm_set1_pd(T2_0))));
          rtb_T[i_0 + coffset_tmp] = tmp_8[0];
          rtb_T[i_0 + omgmat_tmp] = tmp_8[1];
        }
      }

      bid1 = FK_DW.obj_k.TreeInternal.VelocityDoFMap[n];
      idx_idx_1 = FK_DW.obj_k.TreeInternal.VelocityDoFMap[n + 17];
      omgmat[0] = 0.0;
      omgmat[3] = -rtb_T[14];
      omgmat[6] = rtb_T[13];
      omgmat[1] = rtb_T[14];
      omgmat[4] = 0.0;
      omgmat[7] = -rtb_T[12];
      omgmat[2] = -rtb_T[13];
      omgmat[5] = rtb_T[12];
      omgmat[8] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        velnum = omgmat[i_0 + 3];
        T2_0 = omgmat[i_0];
        tmp_0 = omgmat[i_0 + 6];
        for (b = 0; b < 3; b++) {
          omgmat_tmp = b << 2;
          omgmat_0[i_0 + 3 * b] = (rtb_T[omgmat_tmp + 1] * velnum +
            rtb_T[omgmat_tmp] * T2_0) + rtb_T[omgmat_tmp + 2] * tmp_0;
          X[b + 6 * i_0] = rtb_T[(i_0 << 2) + b];
          X[b + 6 * (i_0 + 3)] = 0.0;
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        X[6 * i_0 + 3] = omgmat_0[3 * i_0];
        i = i_0 << 2;
        omgmat_tmp = (i_0 + 3) * 6;
        X[omgmat_tmp + 3] = rtb_T[i];
        X[6 * i_0 + 4] = omgmat_0[3 * i_0 + 1];
        X[omgmat_tmp + 4] = rtb_T[i + 1];
        X[6 * i_0 + 5] = omgmat_0[3 * i_0 + 2];
        X[omgmat_tmp + 5] = rtb_T[i + 2];
      }

      rigidBodyJoint_get_MotionSubspa(&body->JointInternal, b_data, Ttree_size);
      b = Ttree_size[1];
      for (omgmat_tmp = 0; omgmat_tmp < b; omgmat_tmp++) {
        coffset_tmp = omgmat_tmp * 6 - 1;
        for (i_0 = 0; i_0 < 6; i_0++) {
          velnum = 0.0;
          for (i = 0; i < 6; i++) {
            velnum += X[i * 6 + i_0] * b_data[(coffset_tmp + i) + 1];
          }

          JacSlice_data[(coffset_tmp + i_0) + 1] = velnum;
        }
      }

      if (bid1 > idx_idx_1) {
        omgmat_tmp = 0;
        i = 0;
      } else {
        omgmat_tmp = (int32_T)bid1 - 1;
        i = (int32_T)idx_idx_1;
      }

      i -= omgmat_tmp;
      for (i_0 = 0; i_0 < i; i_0++) {
        for (b = 0; b < 6; b++) {
          FK_B.b_data[b + 6 * (omgmat_tmp + i_0)] = JacSlice_data[6 * i_0 + b];
        }
      }
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    n = i_0 << 2;
    bid1 = T2[n];
    X[6 * i_0] = bid1;
    i = (i_0 + 3) * 6;
    X[i] = 0.0;
    X[6 * i_0 + 3] = 0.0;
    X[i + 3] = bid1;
    bid1 = T2[n + 1];
    X[6 * i_0 + 1] = bid1;
    X[i + 1] = 0.0;
    X[6 * i_0 + 4] = 0.0;
    X[i + 4] = bid1;
    bid1 = T2[n + 2];
    X[6 * i_0 + 2] = bid1;
    X[i + 2] = 0.0;
    X[6 * i_0 + 5] = 0.0;
    X[i + 5] = bid1;
  }

  if (loop_ub_tmp - 1 >= 0) {
    memcpy(&FK_B.B_data[0], &FK_B.b_data[0], (uint32_T)loop_ub_tmp * sizeof
           (real_T));
  }

  for (omgmat_tmp = 0; omgmat_tmp < B_tmp; omgmat_tmp++) {
    coffset_tmp = omgmat_tmp * 6 - 1;
    for (n = 0; n < 6; n++) {
      velnum = 0.0;
      for (i = 0; i < 6; i++) {
        velnum += X[i * 6 + n] * FK_B.B_data[(coffset_tmp + i) + 1];
      }

      FK_B.b_data[(coffset_tmp + n) + 1] = velnum;
    }
  }

  /* MATLAB Function: '<S1>/MATLAB Function3' incorporates:
   *  MATLABSystem: '<S2>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  for (i_0 = 0; i_0 < 3; i_0++) {
    omgmat[3 * i_0] = rtb_MATLABSystem_g[i_0];
    omgmat[3 * i_0 + 1] = rtb_MATLABSystem_g[i_0 + 4];
    omgmat[3 * i_0 + 2] = rtb_MATLABSystem_g[i_0 + 8];
  }

  for (i_0 = 0; i_0 < 36; i_0++) {
    /* MATLAB Function: '<S1>/MATLAB Function4' */
    rtb_Jb_out_a_tmp[i_0] = a[i_0];
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    bid1 = omgmat[3 * i_0];
    X[6 * i_0] = bid1;
    omgmat_tmp = (i_0 + 3) * 6;
    X[omgmat_tmp] = 0.0;
    X[6 * i_0 + 3] = 0.0;
    X[omgmat_tmp + 3] = bid1;
    bid1 = omgmat[3 * i_0 + 1];
    X[6 * i_0 + 1] = bid1;
    X[omgmat_tmp + 1] = 0.0;
    X[6 * i_0 + 4] = 0.0;
    X[omgmat_tmp + 4] = bid1;
    bid1 = omgmat[3 * i_0 + 2];
    X[6 * i_0 + 2] = bid1;
    X[omgmat_tmp + 2] = 0.0;
    X[6 * i_0 + 5] = 0.0;
    X[omgmat_tmp + 5] = bid1;
  }

  for (i_0 = 0; i_0 < 6; i_0++) {
    for (b = 0; b < 6; b++) {
      bid1 = 0.0;
      for (omgmat_tmp = 0; omgmat_tmp < 6; omgmat_tmp++) {
        bid1 += (real_T)rtb_Jb_out_a_tmp[6 * omgmat_tmp + i_0] * X[6 * b +
          omgmat_tmp];
      }

      b_data[i_0 + 6 * b] = bid1;
    }

    for (b = 0; b < 12; b++) {
      bid1 = 0.0;
      for (omgmat_tmp = 0; omgmat_tmp < 6; omgmat_tmp++) {
        bid1 += b_data[6 * omgmat_tmp + i_0] * FK_B.b_data[6 * b + omgmat_tmp];
      }

      rtb_Jb_out_a_tmp_0[i_0 + 6 * b] = bid1;
    }
  }

  for (i = 0; i < 36; i++) {
    bid1 = rtb_Jb_out_a_tmp_0[i];
    rtb_Jb_out_n[i] = bid1;

    /* Outport: '<Root>/Jb_l' */
    FK_Y.Jb_l[i] = bid1;
  }

  /* End of MATLAB Function: '<S1>/MATLAB Function3' */

  /* MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  Inport: '<Root>/q'
   *  MATLABSystem: '<S2>/MATLAB System'
   */
  RigidBodyTree_forwardKinematics(&FK_DW.obj.TreeInternal, FK_U.q, Ttree_data,
    Ttree_size);
  velnum = FK_DW.obj.TreeInternal.VelocityNumber;
  B_tmp = (int32_T)velnum;
  loop_ub_tmp = 6 * (int32_T)velnum;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&FK_B.b_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
  }

  for (i_0 = 0; i_0 < 17; i_0++) {
    chainmask[i_0] = 0;
  }

  velnum = FK_DW.obj.TreeInternal.Base.NameInternal.Length;
  memcpy(&obj_Vector[0], &FK_DW.obj.TreeInternal.Base.NameInternal.Vector[0],
         200U * sizeof(char_T));
  b_bool = false;
  if (velnum < 1.0) {
    i_0 = 0;
  } else {
    i_0 = (int32_T)velnum;
  }

  if (i_0 == 5) {
    omgmat_tmp = 1;
    do {
      exitg1 = 0;
      if (omgmat_tmp - 1 < 5) {
        if (tmp_9[omgmat_tmp - 1] != obj_Vector[omgmat_tmp - 1]) {
          exitg1 = 1;
        } else {
          omgmat_tmp++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    memset(&T2inv[0], 0, sizeof(real_T) << 4U);
    T2inv[0] = 1.0;
    T2inv[5] = 1.0;
    T2inv[10] = 1.0;
    T2inv[15] = 1.0;
    memset(&T2[0], 0, sizeof(real_T) << 4U);
    T2[0] = 1.0;
    T2[5] = 1.0;
    T2[10] = 1.0;
    T2[15] = 1.0;
  } else {
    bid1 = -1.0;
    velnum = FK_DW.obj.TreeInternal.Base.NameInternal.Length;
    memcpy(&obj_Vector[0], &FK_DW.obj.TreeInternal.Base.NameInternal.Vector[0],
           200U * sizeof(char_T));
    if (velnum < 1.0) {
      i_0 = 0;
    } else {
      i_0 = (int32_T)velnum;
    }

    if (i_0 == 5) {
      omgmat_tmp = 1;
      do {
        exitg1 = 0;
        if (omgmat_tmp - 1 < 5) {
          if (obj_Vector[omgmat_tmp - 1] != tmp_9[omgmat_tmp - 1]) {
            exitg1 = 1;
          } else {
            omgmat_tmp++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      bid1 = 0.0;
    } else {
      idx_idx_1 = FK_DW.obj.TreeInternal.NumBodies;
      n = 0;
      exitg2 = false;
      while ((!exitg2) && (n <= (int32_T)idx_idx_1 - 1)) {
        body = FK_DW.obj.TreeInternal.Bodies[n];
        velnum = body->NameInternal.Length;
        for (i_0 = 0; i_0 < 200; i_0++) {
          obj_Vector[i_0] = body->NameInternal.Vector[i_0];
        }

        if (velnum < 1.0) {
          i_0 = 0;
        } else {
          i_0 = (int32_T)velnum;
        }

        if (i_0 == 5) {
          omgmat_tmp = 1;
          do {
            exitg1 = 0;
            if (omgmat_tmp - 1 < 5) {
              if (obj_Vector[omgmat_tmp - 1] != tmp_9[omgmat_tmp - 1]) {
                exitg1 = 1;
              } else {
                omgmat_tmp++;
              }
            } else {
              b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (b_bool) {
          bid1 = (real_T)n + 1.0;
          exitg2 = true;
        } else {
          n++;
        }
      }
    }

    body = FK_DW.obj.TreeInternal.Bodies[(int32_T)bid1 - 1];
    for (i_0 = 0; i_0 < 16; i_0++) {
      T2[i_0] = Ttree_data[(int32_T)bid1 - 1].f1[i_0];
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      omgmat[3 * i_0] = Ttree_data[(int32_T)bid1 - 1].f1[i_0];
      omgmat[3 * i_0 + 1] = Ttree_data[(int32_T)bid1 - 1].f1[i_0 + 4];
      omgmat[3 * i_0 + 2] = Ttree_data[(int32_T)bid1 - 1].f1[i_0 + 8];
    }

    for (i_0 = 0; i_0 <= 6; i_0 += 2) {
      tmp_6 = _mm_loadu_pd(&omgmat[i_0]);
      _mm_storeu_pd(&omgmat_0[i_0], _mm_mul_pd(tmp_6, _mm_set1_pd(-1.0)));
    }

    for (i_0 = 8; i_0 < 9; i_0++) {
      omgmat_0[i_0] = -omgmat[i_0];
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      n = i_0 << 2;
      T2inv[n] = omgmat[3 * i_0];
      T2inv[n + 1] = omgmat[3 * i_0 + 1];
      T2inv[n + 2] = omgmat[3 * i_0 + 2];
      T2inv[i_0 + 12] = (Ttree_data[(int32_T)bid1 - 1].f1[12] * omgmat_0[i_0] +
                         omgmat_0[i_0 + 3] * Ttree_data[(int32_T)bid1 - 1].f1[13])
        + omgmat_0[i_0 + 6] * Ttree_data[(int32_T)bid1 - 1].f1[14];
    }

    T2inv[3] = 0.0;
    T2inv[7] = 0.0;
    T2inv[11] = 0.0;
    T2inv[15] = 1.0;
    chainmask[(int32_T)bid1 - 1] = 1;
    while (body->ParentIndex > 0.0) {
      body = FK_DW.obj.TreeInternal.Bodies[(int32_T)body->ParentIndex - 1];
      chainmask[(int32_T)body->Index - 1] = 1;
    }
  }

  idx_idx_1 = FK_DW.obj.TreeInternal.NumBodies;
  c = (int32_T)idx_idx_1;
  for (n = 0; n < c; n++) {
    body = FK_DW.obj.TreeInternal.Bodies[n];
    velnum = body->JointInternal.TypeInternal.Length;
    for (i_0 = 0; i_0 < 200; i_0++) {
      obj_Vector[i_0] = body->JointInternal.TypeInternal.Vector[i_0];
    }

    b_bool = false;
    if (velnum < 1.0) {
      i_0 = 0;
    } else {
      i_0 = (int32_T)velnum;
    }

    if (i_0 == 5) {
      omgmat_tmp = 1;
      do {
        exitg1 = 0;
        if (omgmat_tmp - 1 < 5) {
          if (obj_Vector[omgmat_tmp - 1] != tmp_c[omgmat_tmp - 1]) {
            exitg1 = 1;
          } else {
            omgmat_tmp++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if ((!b_bool) && (chainmask[n] != 0)) {
      for (i_0 = 0; i_0 < 16; i_0++) {
        rtb_T[i_0] = Ttree_data[(int32_T)body->Index - 1].f1[i_0];
      }

      for (i_0 = 0; i_0 < 16; i_0++) {
        Tdh[i_0] = body->JointInternal.ChildToJointTransform[i_0];
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        omgmat[3 * i_0] = Tdh[i_0];
        omgmat[3 * i_0 + 1] = Tdh[i_0 + 4];
        omgmat[3 * i_0 + 2] = Tdh[i_0 + 8];
      }

      for (i_0 = 0; i_0 <= 6; i_0 += 2) {
        tmp_6 = _mm_loadu_pd(&omgmat[i_0]);
        _mm_storeu_pd(&omgmat_0[i_0], _mm_mul_pd(tmp_6, _mm_set1_pd(-1.0)));
      }

      for (i_0 = 8; i_0 < 9; i_0++) {
        omgmat_0[i_0] = -omgmat[i_0];
      }

      bid1 = Tdh[13];
      idx_idx_1 = Tdh[12];
      velnum = Tdh[14];
      for (i_0 = 0; i_0 <= 0; i_0 += 2) {
        tmp_6 = _mm_loadu_pd(&omgmat_0[i_0 + 3]);
        tmp_7 = _mm_loadu_pd(&omgmat_0[i_0]);
        tmp_4 = _mm_loadu_pd(&omgmat_0[i_0 + 6]);
        _mm_storeu_pd(&rtb_T_0[i_0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_6,
          _mm_set1_pd(bid1)), _mm_mul_pd(tmp_7, _mm_set1_pd(idx_idx_1))),
          _mm_mul_pd(tmp_4, _mm_set1_pd(velnum))));
      }

      for (i_0 = 2; i_0 < 3; i_0++) {
        rtb_T_0[i_0] = (omgmat_0[i_0 + 3] * bid1 + omgmat_0[i_0] * idx_idx_1) +
          omgmat_0[i_0 + 6] * velnum;
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        bid1 = T2inv[i_0 + 4];
        idx_idx_1 = T2inv[i_0];
        velnum = T2inv[i_0 + 8];
        T2_0 = T2inv[i_0 + 12];
        for (b = 0; b <= 2; b += 2) {
          omgmat_tmp = (b + 1) << 2;
          coffset_tmp = b << 2;
          _mm_storeu_pd(&tmp_8[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(rtb_T[omgmat_tmp + 1], rtb_T[coffset_tmp + 1]),
             _mm_set1_pd(bid1)), _mm_mul_pd(_mm_set_pd(rtb_T[omgmat_tmp],
            rtb_T[coffset_tmp]), _mm_set1_pd(idx_idx_1))), _mm_mul_pd(_mm_set_pd
            (rtb_T[omgmat_tmp + 2], rtb_T[coffset_tmp + 2]), _mm_set1_pd(velnum))),
            _mm_mul_pd(_mm_set_pd(rtb_T[omgmat_tmp + 3], rtb_T[coffset_tmp + 3]),
                       _mm_set1_pd(T2_0))));
          T2inv_0[i_0 + coffset_tmp] = tmp_8[0];
          T2inv_0[i_0 + omgmat_tmp] = tmp_8[1];
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        omgmat_tmp = i_0 << 2;
        Tdh[omgmat_tmp] = omgmat[3 * i_0];
        Tdh[omgmat_tmp + 1] = omgmat[3 * i_0 + 1];
        Tdh[omgmat_tmp + 2] = omgmat[3 * i_0 + 2];
        Tdh[i_0 + 12] = rtb_T_0[i_0];
      }

      Tdh[3] = 0.0;
      Tdh[7] = 0.0;
      Tdh[11] = 0.0;
      Tdh[15] = 1.0;
      for (i_0 = 0; i_0 < 4; i_0++) {
        bid1 = T2inv_0[i_0 + 4];
        idx_idx_1 = T2inv_0[i_0];
        velnum = T2inv_0[i_0 + 8];
        T2_0 = T2inv_0[i_0 + 12];
        for (b = 0; b <= 2; b += 2) {
          omgmat_tmp = (b + 1) << 2;
          coffset_tmp = b << 2;
          _mm_storeu_pd(&tmp_8[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
            (_mm_set_pd(Tdh[omgmat_tmp + 1], Tdh[coffset_tmp + 1]), _mm_set1_pd
             (bid1)), _mm_mul_pd(_mm_set_pd(Tdh[omgmat_tmp], Tdh[coffset_tmp]),
            _mm_set1_pd(idx_idx_1))), _mm_mul_pd(_mm_set_pd(Tdh[omgmat_tmp + 2],
            Tdh[coffset_tmp + 2]), _mm_set1_pd(velnum))), _mm_mul_pd(_mm_set_pd
            (Tdh[omgmat_tmp + 3], Tdh[coffset_tmp + 3]), _mm_set1_pd(T2_0))));
          rtb_T[i_0 + coffset_tmp] = tmp_8[0];
          rtb_T[i_0 + omgmat_tmp] = tmp_8[1];
        }
      }

      bid1 = FK_DW.obj.TreeInternal.VelocityDoFMap[n];
      idx_idx_1 = FK_DW.obj.TreeInternal.VelocityDoFMap[n + 17];
      omgmat[0] = 0.0;
      omgmat[3] = -rtb_T[14];
      omgmat[6] = rtb_T[13];
      omgmat[1] = rtb_T[14];
      omgmat[4] = 0.0;
      omgmat[7] = -rtb_T[12];
      omgmat[2] = -rtb_T[13];
      omgmat[5] = rtb_T[12];
      omgmat[8] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        velnum = omgmat[i_0 + 3];
        T2_0 = omgmat[i_0];
        tmp_0 = omgmat[i_0 + 6];
        for (b = 0; b < 3; b++) {
          omgmat_tmp = b << 2;
          omgmat_0[i_0 + 3 * b] = (rtb_T[omgmat_tmp + 1] * velnum +
            rtb_T[omgmat_tmp] * T2_0) + rtb_T[omgmat_tmp + 2] * tmp_0;
          X[b + 6 * i_0] = rtb_T[(i_0 << 2) + b];
          X[b + 6 * (i_0 + 3)] = 0.0;
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        X[6 * i_0 + 3] = omgmat_0[3 * i_0];
        i = i_0 << 2;
        omgmat_tmp = (i_0 + 3) * 6;
        X[omgmat_tmp + 3] = rtb_T[i];
        X[6 * i_0 + 4] = omgmat_0[3 * i_0 + 1];
        X[omgmat_tmp + 4] = rtb_T[i + 1];
        X[6 * i_0 + 5] = omgmat_0[3 * i_0 + 2];
        X[omgmat_tmp + 5] = rtb_T[i + 2];
      }

      rigidBodyJoint_get_MotionSubspa(&body->JointInternal, b_data, Ttree_size);
      b = Ttree_size[1];
      for (omgmat_tmp = 0; omgmat_tmp < b; omgmat_tmp++) {
        coffset_tmp = omgmat_tmp * 6 - 1;
        for (i_0 = 0; i_0 < 6; i_0++) {
          velnum = 0.0;
          for (i = 0; i < 6; i++) {
            velnum += X[i * 6 + i_0] * b_data[(coffset_tmp + i) + 1];
          }

          JacSlice_data[(coffset_tmp + i_0) + 1] = velnum;
        }
      }

      if (bid1 > idx_idx_1) {
        omgmat_tmp = 0;
        i = 0;
      } else {
        omgmat_tmp = (int32_T)bid1 - 1;
        i = (int32_T)idx_idx_1;
      }

      i -= omgmat_tmp;
      for (i_0 = 0; i_0 < i; i_0++) {
        for (b = 0; b < 6; b++) {
          FK_B.b_data[b + 6 * (omgmat_tmp + i_0)] = JacSlice_data[6 * i_0 + b];
        }
      }
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    n = i_0 << 2;
    bid1 = T2[n];
    X[6 * i_0] = bid1;
    i = (i_0 + 3) * 6;
    X[i] = 0.0;
    X[6 * i_0 + 3] = 0.0;
    X[i + 3] = bid1;
    bid1 = T2[n + 1];
    X[6 * i_0 + 1] = bid1;
    X[i + 1] = 0.0;
    X[6 * i_0 + 4] = 0.0;
    X[i + 4] = bid1;
    bid1 = T2[n + 2];
    X[6 * i_0 + 2] = bid1;
    X[i + 2] = 0.0;
    X[6 * i_0 + 5] = 0.0;
    X[i + 5] = bid1;
  }

  if (loop_ub_tmp - 1 >= 0) {
    memcpy(&FK_B.B_data[0], &FK_B.b_data[0], (uint32_T)loop_ub_tmp * sizeof
           (real_T));
  }

  for (omgmat_tmp = 0; omgmat_tmp < B_tmp; omgmat_tmp++) {
    coffset_tmp = omgmat_tmp * 6 - 1;
    for (n = 0; n < 6; n++) {
      velnum = 0.0;
      for (i = 0; i < 6; i++) {
        velnum += X[i * 6 + n] * FK_B.B_data[(coffset_tmp + i) + 1];
      }

      FK_B.b_data[(coffset_tmp + n) + 1] = velnum;
    }
  }

  /* MATLAB Function: '<S1>/MATLAB Function4' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   *  MATLABSystem: '<S6>/MATLAB System'
   */
  for (i_0 = 0; i_0 < 3; i_0++) {
    bid1 = rtb_MATLABSystem[i_0];
    X[6 * i_0] = bid1;
    omgmat_tmp = (i_0 + 3) * 6;
    X[omgmat_tmp] = 0.0;
    X[6 * i_0 + 3] = 0.0;
    X[omgmat_tmp + 3] = bid1;
    bid1 = rtb_MATLABSystem[i_0 + 4];
    X[6 * i_0 + 1] = bid1;
    X[omgmat_tmp + 1] = 0.0;
    X[6 * i_0 + 4] = 0.0;
    X[omgmat_tmp + 4] = bid1;
    bid1 = rtb_MATLABSystem[i_0 + 8];
    X[6 * i_0 + 2] = bid1;
    X[omgmat_tmp + 2] = 0.0;
    X[6 * i_0 + 5] = 0.0;
    X[omgmat_tmp + 5] = bid1;
  }

  for (i_0 = 0; i_0 < 6; i_0++) {
    for (b = 0; b < 6; b++) {
      bid1 = 0.0;
      for (omgmat_tmp = 0; omgmat_tmp < 6; omgmat_tmp++) {
        bid1 += (real_T)rtb_Jb_out_a_tmp[6 * omgmat_tmp + i_0] * X[6 * b +
          omgmat_tmp];
      }

      b_data[i_0 + 6 * b] = bid1;
    }

    for (b = 0; b < 12; b++) {
      bid1 = 0.0;
      for (omgmat_tmp = 0; omgmat_tmp < 6; omgmat_tmp++) {
        bid1 += b_data[6 * omgmat_tmp + i_0] * FK_B.b_data[6 * b + omgmat_tmp];
      }

      rtb_Jb_out_a_tmp_0[i_0 + 6 * b] = bid1;
    }
  }

  for (i_0 = 0; i_0 < 36; i_0++) {
    bid1 = rtb_Jb_out_a_tmp_0[i_0 + 36];
    X[i_0] = bid1;

    /* Outport: '<Root>/Jb_r' */
    FK_Y.Jb_r[i_0] = bid1;
  }

  /* MATLAB Function: '<S1>/MATLAB Function5' incorporates:
   *  Inport: '<Root>/q'
   */
  FK_MATLABFunction5(rtb_Jb_out_n, &FK_U.q[0], &FK_B.sf_MATLABFunction5);

  /* Outport: '<Root>/Jbdot_l' */
  memcpy(&FK_Y.Jbdot_l[0], &FK_B.sf_MATLABFunction5.Jbdot[0], 36U * sizeof
         (real_T));

  /* MATLAB Function: '<S1>/MATLAB Function6' incorporates:
   *  Inport: '<Root>/q'
   */
  FK_MATLABFunction5(X, &FK_U.q[6], &FK_B.sf_MATLABFunction6);

  /* Outport: '<Root>/Jbdot_r' */
  memcpy(&FK_Y.Jbdot_r[0], &FK_B.sf_MATLABFunction6.Jbdot[0], 36U * sizeof
         (real_T));

  /* Outport: '<Root>/Jb_lr' */
  memcpy(&FK_Y.Jb_lr[0], &rtb_J[0], 72U * sizeof(real_T));

  /* Outport: '<Root>/Jbdot_lr' */
  memcpy(&FK_Y.Jbdot_lr[0], &rtb_Jdot[0], 72U * sizeof(real_T));
  for (i_0 = 0; i_0 < 6; i_0++) {
    /* Outport: '<Root>/V_r' incorporates:
     *  Product: '<S1>/Matrix Multiply1'
     */
    bid1 = 0.0;

    /* Outport: '<Root>/V_l' incorporates:
     *  Product: '<S1>/Matrix Multiply'
     */
    idx_idx_1 = 0.0;
    for (b = 0; b < 6; b++) {
      /* Product: '<S1>/Matrix Multiply' incorporates:
       *  Inport: '<Root>/qdot'
       *  Outport: '<Root>/V_l'
       *  Outport: '<Root>/V_r'
       *  Product: '<S1>/Matrix Multiply1'
       */
      omgmat_tmp = 6 * b + i_0;
      _mm_storeu_pd(&tmp_8[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
        (rtb_Jb_out_n[omgmat_tmp], X[omgmat_tmp]), _mm_set_pd(FK_U.qdot[b],
        FK_U.qdot[b + 6])), _mm_set_pd(idx_idx_1, bid1)));

      /* Outport: '<Root>/V_r' incorporates:
       *  Product: '<S1>/Matrix Multiply1'
       */
      bid1 = tmp_8[0];

      /* Outport: '<Root>/V_l' incorporates:
       *  Product: '<S1>/Matrix Multiply'
       */
      idx_idx_1 = tmp_8[1];
    }

    /* Outport: '<Root>/V_l' incorporates:
     *  Product: '<S1>/Matrix Multiply'
     */
    FK_Y.V_l[i_0] = idx_idx_1;

    /* Outport: '<Root>/V_r' incorporates:
     *  Product: '<S1>/Matrix Multiply1'
     */
    FK_Y.V_r[i_0] = bid1;

    /* Product: '<S1>/Matrix Multiply2' incorporates:
     *  Outport: '<Root>/V_lr'
     */
    bid1 = 0.0;

    /* Outport: '<Root>/V_lr' incorporates:
     *  Product: '<S1>/Matrix Multiply2'
     */
    for (b = 0; b < 12; b++) {
      bid1 += rtb_J[6 * b + i_0] * FK_B.sf_MATLABFunction2.q_lr[b];
    }

    FK_Y.V_lr[i_0] = bid1;
  }

  /* MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  Inport: '<Root>/q'
   */
  RigidBodyTree_forwardKinemat_ml(&FK_DW.obj_i.TreeInternal, FK_U.q, obj_i.data,
    obj_i.size);

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

  /* block I/O */
  (void) memset(((void *) &FK_B), 0,
                sizeof(B_FK_T));

  /* states (dwork) */
  (void) memset((void *)&FK_DW, 0,
                sizeof(DW_FK_T));

  /* external inputs */
  (void)memset(&FK_U, 0, sizeof(ExtU_FK_T));

  /* external outputs */
  (void)memset(&FK_Y, 0, sizeof(ExtY_FK_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(FK_M->rtwLogInfo, 0.0, rtmGetTFinal(FK_M),
    FK_M->Timing.stepSize0, (&rtmGetErrorStatus(FK_M)));

  {
    int32_T i;
    static const uint32_T tmp[625] = { 5489U, 1301868182U, 2938499221U,
      2950281878U, 1875628136U, 751856242U, 944701696U, 2243192071U, 694061057U,
      219885934U, 2066767472U, 3182869408U, 485472502U, 2336857883U, 1071588843U,
      3418470598U, 951210697U, 3693558366U, 2923482051U, 1793174584U,
      2982310801U, 1586906132U, 1951078751U, 1808158765U, 1733897588U,
      431328322U, 4202539044U, 530658942U, 1714810322U, 3025256284U, 3342585396U,
      1937033938U, 2640572511U, 1654299090U, 3692403553U, 4233871309U,
      3497650794U, 862629010U, 2943236032U, 2426458545U, 1603307207U,
      1133453895U, 3099196360U, 2208657629U, 2747653927U, 931059398U, 761573964U,
      3157853227U, 785880413U, 730313442U, 124945756U, 2937117055U, 3295982469U,
      1724353043U, 3021675344U, 3884886417U, 4010150098U, 4056961966U,
      699635835U, 2681338818U, 1339167484U, 720757518U, 2800161476U, 2376097373U,
      1532957371U, 3902664099U, 1238982754U, 3725394514U, 3449176889U,
      3570962471U, 4287636090U, 4087307012U, 3603343627U, 202242161U,
      2995682783U, 1620962684U, 3704723357U, 371613603U, 2814834333U,
      2111005706U, 624778151U, 2094172212U, 4284947003U, 1211977835U, 991917094U,
      1570449747U, 2962370480U, 1259410321U, 170182696U, 146300961U, 2836829791U,
      619452428U, 2723670296U, 1881399711U, 1161269684U, 1675188680U,
      4132175277U, 780088327U, 3409462821U, 1036518241U, 1834958505U,
      3048448173U, 161811569U, 618488316U, 44795092U, 3918322701U, 1924681712U,
      3239478144U, 383254043U, 4042306580U, 2146983041U, 3992780527U,
      3518029708U, 3545545436U, 3901231469U, 1896136409U, 2028528556U,
      2339662006U, 501326714U, 2060962201U, 2502746480U, 561575027U, 581893337U,
      3393774360U, 1778912547U, 3626131687U, 2175155826U, 319853231U, 986875531U,
      819755096U, 2915734330U, 2688355739U, 3482074849U, 2736559U, 2296975761U,
      1029741190U, 2876812646U, 690154749U, 579200347U, 4027461746U, 1285330465U,
      2701024045U, 4117700889U, 759495121U, 3332270341U, 2313004527U,
      2277067795U, 4131855432U, 2722057515U, 1264804546U, 3848622725U,
      2211267957U, 4100593547U, 959123777U, 2130745407U, 3194437393U, 486673947U,
      1377371204U, 17472727U, 352317554U, 3955548058U, 159652094U, 1232063192U,
      3835177280U, 49423123U, 3083993636U, 733092U, 2120519771U, 2573409834U,
      1112952433U, 3239502554U, 761045320U, 1087580692U, 2540165110U, 641058802U,
      1792435497U, 2261799288U, 1579184083U, 627146892U, 2165744623U,
      2200142389U, 2167590760U, 2381418376U, 1793358889U, 3081659520U,
      1663384067U, 2009658756U, 2689600308U, 739136266U, 2304581039U,
      3529067263U, 591360555U, 525209271U, 3131882996U, 294230224U, 2076220115U,
      3113580446U, 1245621585U, 1386885462U, 3203270426U, 123512128U, 12350217U,
      354956375U, 4282398238U, 3356876605U, 3888857667U, 157639694U, 2616064085U,
      1563068963U, 2762125883U, 4045394511U, 4180452559U, 3294769488U,
      1684529556U, 1002945951U, 3181438866U, 22506664U, 691783457U, 2685221343U,
      171579916U, 3878728600U, 2475806724U, 2030324028U, 3331164912U,
      1708711359U, 1970023127U, 2859691344U, 2588476477U, 2748146879U,
      136111222U, 2967685492U, 909517429U, 2835297809U, 3206906216U, 3186870716U,
      341264097U, 2542035121U, 3353277068U, 548223577U, 3170936588U, 1678403446U,
      297435620U, 2337555430U, 466603495U, 1132321815U, 1208589219U, 696392160U,
      894244439U, 2562678859U, 470224582U, 3306867480U, 201364898U, 2075966438U,
      1767227936U, 2929737987U, 3674877796U, 2654196643U, 3692734598U,
      3528895099U, 2796780123U, 3048728353U, 842329300U, 191554730U, 2922459673U,
      3489020079U, 3979110629U, 1022523848U, 2202932467U, 3583655201U,
      3565113719U, 587085778U, 4176046313U, 3013713762U, 950944241U, 396426791U,
      3784844662U, 3477431613U, 3594592395U, 2782043838U, 3392093507U,
      3106564952U, 2829419931U, 1358665591U, 2206918825U, 3170783123U, 31522386U,
      2988194168U, 1782249537U, 1105080928U, 843500134U, 1225290080U,
      1521001832U, 3605886097U, 2802786495U, 2728923319U, 3996284304U,
      903417639U, 1171249804U, 1020374987U, 2824535874U, 423621996U, 1988534473U,
      2493544470U, 1008604435U, 1756003503U, 1488867287U, 1386808992U,
      732088248U, 1780630732U, 2482101014U, 976561178U, 1543448953U, 2602866064U,
      2021139923U, 1952599828U, 2360242564U, 2117959962U, 2753061860U,
      2388623612U, 4138193781U, 2962920654U, 2284970429U, 766920861U,
      3457264692U, 2879611383U, 815055854U, 2332929068U, 1254853997U,
      3740375268U, 3799380844U, 4091048725U, 2006331129U, 1982546212U,
      686850534U, 1907447564U, 2682801776U, 2780821066U, 998290361U, 1342433871U,
      4195430425U, 607905174U, 3902331779U, 2454067926U, 1708133115U,
      1170874362U, 2008609376U, 3260320415U, 2211196135U, 433538229U,
      2728786374U, 2189520818U, 262554063U, 1182318347U, 3710237267U,
      1221022450U, 715966018U, 2417068910U, 2591870721U, 2870691989U,
      3418190842U, 4238214053U, 1540704231U, 1575580968U, 2095917976U,
      4078310857U, 2313532447U, 2110690783U, 4056346629U, 4061784526U,
      1123218514U, 551538993U, 597148360U, 4120175196U, 3581618160U, 3181170517U,
      422862282U, 3227524138U, 1713114790U, 662317149U, 1230418732U, 928171837U,
      1324564878U, 1928816105U, 1786535431U, 2878099422U, 3290185549U,
      539474248U, 1657512683U, 552370646U, 1671741683U, 3655312128U, 1552739510U,
      2605208763U, 1441755014U, 181878989U, 3124053868U, 1447103986U,
      3183906156U, 1728556020U, 3502241336U, 3055466967U, 1013272474U,
      818402132U, 1715099063U, 2900113506U, 397254517U, 4194863039U, 1009068739U,
      232864647U, 2540223708U, 2608288560U, 2415367765U, 478404847U, 3455100648U,
      3182600021U, 2115988978U, 434269567U, 4117179324U, 3461774077U, 887256537U,
      3545801025U, 286388911U, 3451742129U, 1981164769U, 786667016U, 3310123729U,
      3097811076U, 2224235657U, 2959658883U, 3370969234U, 2514770915U,
      3345656436U, 2677010851U, 2206236470U, 271648054U, 2342188545U,
      4292848611U, 3646533909U, 3754009956U, 3803931226U, 4160647125U,
      1477814055U, 4043852216U, 1876372354U, 3133294443U, 3871104810U,
      3177020907U, 2074304428U, 3479393793U, 759562891U, 164128153U, 1839069216U,
      2114162633U, 3989947309U, 3611054956U, 1333547922U, 835429831U, 494987340U,
      171987910U, 1252001001U, 370809172U, 3508925425U, 2535703112U, 1276855041U,
      1922855120U, 835673414U, 3030664304U, 613287117U, 171219893U, 3423096126U,
      3376881639U, 2287770315U, 1658692645U, 1262815245U, 3957234326U,
      1168096164U, 2968737525U, 2655813712U, 2132313144U, 3976047964U,
      326516571U, 353088456U, 3679188938U, 3205649712U, 2654036126U, 1249024881U,
      880166166U, 691800469U, 2229503665U, 1673458056U, 4032208375U, 1851778863U,
      2563757330U, 376742205U, 1794655231U, 340247333U, 1505873033U, 396524441U,
      879666767U, 3335579166U, 3260764261U, 3335999539U, 506221798U, 4214658741U,
      975887814U, 2080536343U, 3360539560U, 571586418U, 138896374U, 4234352651U,
      2737620262U, 3928362291U, 1516365296U, 38056726U, 3599462320U, 3585007266U,
      3850961033U, 471667319U, 1536883193U, 2310166751U, 1861637689U,
      2530999841U, 4139843801U, 2710569485U, 827578615U, 2012334720U,
      2907369459U, 3029312804U, 2820112398U, 1965028045U, 35518606U, 2478379033U,
      643747771U, 1924139484U, 4123405127U, 3811735531U, 3429660832U,
      3285177704U, 1948416081U, 1311525291U, 1183517742U, 1739192232U,
      3979815115U, 2567840007U, 4116821529U, 213304419U, 4125718577U,
      1473064925U, 2442436592U, 1893310111U, 4195361916U, 3747569474U,
      828465101U, 2991227658U, 750582866U, 1205170309U, 1409813056U, 678418130U,
      1171531016U, 3821236156U, 354504587U, 4202874632U, 3882511497U,
      1893248677U, 1903078632U, 26340130U, 2069166240U, 3657122492U, 3725758099U,
      831344905U, 811453383U, 3447711422U, 2434543565U, 4166886888U, 3358210805U,
      4142984013U, 2988152326U, 3527824853U, 982082992U, 2809155763U, 190157081U,
      3340214818U, 2365432395U, 2548636180U, 2894533366U, 3474657421U,
      2372634704U, 2845748389U, 43024175U, 2774226648U, 1987702864U, 3186502468U,
      453610222U, 4204736567U, 1392892630U, 2471323686U, 2470534280U,
      3541393095U, 4269885866U, 3909911300U, 759132955U, 1482612480U, 667715263U,
      1795580598U, 2337923983U, 3390586366U, 581426223U, 1515718634U, 476374295U,
      705213300U, 363062054U, 2084697697U, 2407503428U, 2292957699U, 2426213835U,
      2199989172U, 1987356470U, 4026755612U, 2147252133U, 270400031U,
      1367820199U, 2369854699U, 2844269403U, 79981964U, 624U };

    emxInitStruct_robotics_slmanip_(&FK_DW.obj_g);

    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    for (i = 0; i < 34; i++) {
      FK_DW.obj_g.TreeInternal._pobj0[i].
        CollisionsInternal.matlabCodegenIsDeleted = true;
    }

    FK_DW.obj_g.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    for (i = 0; i < 34; i++) {
      FK_DW.obj_g.TreeInternal._pobj0[i].matlabCodegenIsDeleted = true;
    }

    FK_DW.obj_g.TreeInternal.Base.matlabCodegenIsDeleted = true;
    FK_DW.obj_g.TreeInternal.matlabCodegenIsDeleted = true;
    FK_DW.method = 7U;
    FK_DW.method_not_empty = true;
    FK_DW.state = 1144108930U;
    FK_DW.state_not_empty_m = true;
    FK_DW.state_p[0] = 362436069U;
    FK_DW.state_p[1] = 521288629U;
    FK_DW.state_not_empty_n = true;
    memcpy(&FK_DW.state_g[0], &tmp[0], 625U * sizeof(uint32_T));
    FK_DW.state_not_empty = true;
    FK_DW.obj_g.matlabCodegenIsDeleted = false;
    FK_DW.objisempty = true;
    FK_DW.obj_g.isInitialized = 1;
    GetTransformBlock_setupImpl_ml(&FK_DW.obj_g);

    /* End of Start for MATLABSystem: '<S6>/MATLAB System' */
    emxInitStruct_robotics_slmanip_(&FK_DW.obj_b);

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    for (i = 0; i < 34; i++) {
      FK_DW.obj_b.TreeInternal._pobj0[i].
        CollisionsInternal.matlabCodegenIsDeleted = true;
    }

    FK_DW.obj_b.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    for (i = 0; i < 34; i++) {
      FK_DW.obj_b.TreeInternal._pobj0[i].matlabCodegenIsDeleted = true;
    }

    FK_DW.obj_b.TreeInternal.Base.matlabCodegenIsDeleted = true;
    FK_DW.obj_b.TreeInternal.matlabCodegenIsDeleted = true;
    FK_DW.method_e = 7U;
    FK_DW.method_not_empty_b = true;
    FK_DW.state_o = 1144108930U;
    FK_DW.state_not_empty_p = true;
    FK_DW.state_a[0] = 362436069U;
    FK_DW.state_a[1] = 521288629U;
    FK_DW.state_not_empty_m3 = true;
    memcpy(&FK_DW.state_j[0], &tmp[0], 625U * sizeof(uint32_T));
    FK_DW.state_not_empty_b = true;
    FK_DW.obj_b.matlabCodegenIsDeleted = false;
    FK_DW.objisempty_j = true;
    FK_DW.obj_b.isInitialized = 1;
    F_GetTransformBlock_setupImpl_m(&FK_DW.obj_b);

    /* End of Start for MATLABSystem: '<S5>/MATLAB System' */
    emxInitStruct_robotics_slmani_m(&FK_DW.obj_k);

    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    for (i = 0; i < 34; i++) {
      FK_DW.obj_k.TreeInternal._pobj0[i].
        CollisionsInternal.matlabCodegenIsDeleted = true;
    }

    FK_DW.obj_k.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    for (i = 0; i < 34; i++) {
      FK_DW.obj_k.TreeInternal._pobj0[i].matlabCodegenIsDeleted = true;
    }

    FK_DW.obj_k.TreeInternal.Base.matlabCodegenIsDeleted = true;
    FK_DW.obj_k.TreeInternal.matlabCodegenIsDeleted = true;
    FK_DW.method_b = 7U;
    FK_DW.method_not_empty_px = true;
    FK_DW.state_f = 1144108930U;
    FK_DW.state_not_empty_l = true;
    FK_DW.state_e[0] = 362436069U;
    FK_DW.state_e[1] = 521288629U;
    FK_DW.state_not_empty_ju = true;
    memcpy(&FK_DW.state_l[0], &tmp[0], 625U * sizeof(uint32_T));
    FK_DW.state_not_empty_c = true;
    FK_DW.obj_k.matlabCodegenIsDeleted = false;
    FK_DW.objisempty_i = true;
    FK_DW.obj_k.isInitialized = 1;
    FK_GetJacobianBlock_setupImpl(&FK_DW.obj_k);

    /* End of Start for MATLABSystem: '<S2>/MATLAB System' */
    emxInitStruct_robotics_slmani_m(&FK_DW.obj);

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    for (i = 0; i < 34; i++) {
      FK_DW.obj.TreeInternal._pobj0[i].CollisionsInternal.matlabCodegenIsDeleted
        = true;
    }

    FK_DW.obj.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
    for (i = 0; i < 34; i++) {
      FK_DW.obj.TreeInternal._pobj0[i].matlabCodegenIsDeleted = true;
    }

    FK_DW.obj.TreeInternal.Base.matlabCodegenIsDeleted = true;
    FK_DW.obj.TreeInternal.matlabCodegenIsDeleted = true;
    FK_DW.method_f = 7U;
    FK_DW.method_not_empty_m = true;
    FK_DW.state_px = 1144108930U;
    FK_DW.state_not_empty_d = true;
    FK_DW.state_b[0] = 362436069U;
    FK_DW.state_b[1] = 521288629U;
    FK_DW.state_not_empty_iq = true;
    memcpy(&FK_DW.state_d[0], &tmp[0], 625U * sizeof(uint32_T));
    FK_DW.state_not_empty_i = true;
    FK_DW.obj.matlabCodegenIsDeleted = false;
    FK_DW.objisempty_o = true;
    FK_DW.obj.isInitialized = 1;
    FK_GetJacobianBlock_setupImpl_m(&FK_DW.obj);

    /* End of Start for MATLABSystem: '<S3>/MATLAB System' */
    emxInitStruct_robotics_slmanip_(&FK_DW.obj_i);

    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    for (i = 0; i < 34; i++) {
      FK_DW.obj_i.TreeInternal._pobj0[i].
        CollisionsInternal.matlabCodegenIsDeleted = true;
    }

    FK_DW.obj_i.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    for (i = 0; i < 34; i++) {
      FK_DW.obj_i.TreeInternal._pobj0[i].matlabCodegenIsDeleted = true;
    }

    FK_DW.obj_i.TreeInternal.Base.matlabCodegenIsDeleted = true;
    FK_DW.obj_i.TreeInternal.matlabCodegenIsDeleted = true;
    FK_DW.method_i = 7U;
    FK_DW.method_not_empty_p = true;
    FK_DW.state_m = 1144108930U;
    FK_DW.state_not_empty_j = true;
    FK_DW.state_c[0] = 362436069U;
    FK_DW.state_c[1] = 521288629U;
    FK_DW.state_not_empty_e = true;
    memcpy(&FK_DW.state_i[0], &tmp[0], 625U * sizeof(uint32_T));
    FK_DW.state_not_empty_bp = true;
    FK_DW.obj_i.matlabCodegenIsDeleted = false;
    FK_DW.objisempty_l = true;
    FK_DW.obj_i.isInitialized = 1;
    FK_GetTransformBlock_setupImpl(&FK_DW.obj_i);

    /* End of Start for MATLABSystem: '<S4>/MATLAB System' */
  }
}

/* Model terminate function */
void FK_terminate(void)
{
  e_robotics_manip_internal_R_m_T *obj_0;
  e_robotics_manip_internal_Rig_T *obj_2;
  f_robotics_manip_internal_Col_T obj;
  g_robotics_manip_internal_Col_T *obj_1;
  real_T b_0;
  int32_T b;
  int32_T b_i;
  int32_T c;

  /* Terminate for MATLABSystem: '<S6>/MATLAB System' */
  if (!FK_DW.obj_g.matlabCodegenIsDeleted) {
    FK_DW.obj_g.matlabCodegenIsDeleted = true;
  }

  if (!FK_DW.obj_g.TreeInternal.matlabCodegenIsDeleted) {
    FK_DW.obj_g.TreeInternal.matlabCodegenIsDeleted = true;
  }

  if (!FK_DW.obj_g.TreeInternal.Base.matlabCodegenIsDeleted) {
    FK_DW.obj_g.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }

  for (b = 0; b < 34; b++) {
    obj_0 = &FK_DW.obj_g.TreeInternal._pobj0[b];
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
    }
  }

  if (!FK_DW.obj_g.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted)
  {
    FK_DW.obj_g.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    b_0 = FK_DW.obj_g.TreeInternal.Base.CollisionsInternal.Size;
    b = (int32_T)b_0;
    for (b_i = 0; b_i < b; b_i++) {
      obj =
        FK_DW.obj_g.TreeInternal.Base.CollisionsInternal.CollisionGeometries->data
        [b_i];
      collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
      FK_DW.obj_g.TreeInternal.Base.CollisionsInternal.CollisionGeometries->
        data[b_i] = obj;
    }
  }

  for (b = 0; b < 34; b++) {
    obj_1 = &FK_DW.obj_g.TreeInternal._pobj0[b].CollisionsInternal;
    if (!obj_1->matlabCodegenIsDeleted) {
      obj_1->matlabCodegenIsDeleted = true;
      b_0 = obj_1->Size;
      c = (int32_T)b_0;
      for (b_i = 0; b_i < c; b_i++) {
        obj = obj_1->CollisionGeometries->data[b_i];
        collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
        obj_1->CollisionGeometries->data[b_i] = obj;
      }
    }
  }

  /* End of Terminate for MATLABSystem: '<S6>/MATLAB System' */
  emxFreeStruct_robotics_slmanip_(&FK_DW.obj_g);

  /* Terminate for MATLABSystem: '<S5>/MATLAB System' */
  if (!FK_DW.obj_b.matlabCodegenIsDeleted) {
    FK_DW.obj_b.matlabCodegenIsDeleted = true;
  }

  if (!FK_DW.obj_b.TreeInternal.matlabCodegenIsDeleted) {
    FK_DW.obj_b.TreeInternal.matlabCodegenIsDeleted = true;
  }

  if (!FK_DW.obj_b.TreeInternal.Base.matlabCodegenIsDeleted) {
    FK_DW.obj_b.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }

  for (b = 0; b < 34; b++) {
    obj_0 = &FK_DW.obj_b.TreeInternal._pobj0[b];
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
    }
  }

  if (!FK_DW.obj_b.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted)
  {
    FK_DW.obj_b.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    b_0 = FK_DW.obj_b.TreeInternal.Base.CollisionsInternal.Size;
    b = (int32_T)b_0;
    for (b_i = 0; b_i < b; b_i++) {
      obj =
        FK_DW.obj_b.TreeInternal.Base.CollisionsInternal.CollisionGeometries->data
        [b_i];
      collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
      FK_DW.obj_b.TreeInternal.Base.CollisionsInternal.CollisionGeometries->
        data[b_i] = obj;
    }
  }

  for (b = 0; b < 34; b++) {
    obj_1 = &FK_DW.obj_b.TreeInternal._pobj0[b].CollisionsInternal;
    if (!obj_1->matlabCodegenIsDeleted) {
      obj_1->matlabCodegenIsDeleted = true;
      b_0 = obj_1->Size;
      c = (int32_T)b_0;
      for (b_i = 0; b_i < c; b_i++) {
        obj = obj_1->CollisionGeometries->data[b_i];
        collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
        obj_1->CollisionGeometries->data[b_i] = obj;
      }
    }
  }

  /* End of Terminate for MATLABSystem: '<S5>/MATLAB System' */
  emxFreeStruct_robotics_slmanip_(&FK_DW.obj_b);

  /* Terminate for MATLABSystem: '<S2>/MATLAB System' */
  if (!FK_DW.obj_k.matlabCodegenIsDeleted) {
    FK_DW.obj_k.matlabCodegenIsDeleted = true;
  }

  if (!FK_DW.obj_k.TreeInternal.matlabCodegenIsDeleted) {
    FK_DW.obj_k.TreeInternal.matlabCodegenIsDeleted = true;
  }

  if (!FK_DW.obj_k.TreeInternal.Base.matlabCodegenIsDeleted) {
    FK_DW.obj_k.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }

  for (b = 0; b < 34; b++) {
    obj_2 = &FK_DW.obj_k.TreeInternal._pobj0[b];
    if (!obj_2->matlabCodegenIsDeleted) {
      obj_2->matlabCodegenIsDeleted = true;
    }
  }

  if (!FK_DW.obj_k.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted)
  {
    FK_DW.obj_k.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    b_0 = FK_DW.obj_k.TreeInternal.Base.CollisionsInternal.Size;
    b = (int32_T)b_0;
    for (b_i = 0; b_i < b; b_i++) {
      obj =
        FK_DW.obj_k.TreeInternal.Base.CollisionsInternal.CollisionGeometries->data
        [b_i];
      collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
      FK_DW.obj_k.TreeInternal.Base.CollisionsInternal.CollisionGeometries->
        data[b_i] = obj;
    }
  }

  for (b = 0; b < 34; b++) {
    obj_1 = &FK_DW.obj_k.TreeInternal._pobj0[b].CollisionsInternal;
    if (!obj_1->matlabCodegenIsDeleted) {
      obj_1->matlabCodegenIsDeleted = true;
      b_0 = obj_1->Size;
      c = (int32_T)b_0;
      for (b_i = 0; b_i < c; b_i++) {
        obj = obj_1->CollisionGeometries->data[b_i];
        collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
        obj_1->CollisionGeometries->data[b_i] = obj;
      }
    }
  }

  /* End of Terminate for MATLABSystem: '<S2>/MATLAB System' */
  emxFreeStruct_robotics_slmani_m(&FK_DW.obj_k);

  /* Terminate for MATLABSystem: '<S3>/MATLAB System' */
  if (!FK_DW.obj.matlabCodegenIsDeleted) {
    FK_DW.obj.matlabCodegenIsDeleted = true;
  }

  if (!FK_DW.obj.TreeInternal.matlabCodegenIsDeleted) {
    FK_DW.obj.TreeInternal.matlabCodegenIsDeleted = true;
  }

  if (!FK_DW.obj.TreeInternal.Base.matlabCodegenIsDeleted) {
    FK_DW.obj.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }

  for (b = 0; b < 34; b++) {
    obj_2 = &FK_DW.obj.TreeInternal._pobj0[b];
    if (!obj_2->matlabCodegenIsDeleted) {
      obj_2->matlabCodegenIsDeleted = true;
    }
  }

  if (!FK_DW.obj.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted) {
    FK_DW.obj.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
    b_0 = FK_DW.obj.TreeInternal.Base.CollisionsInternal.Size;
    b = (int32_T)b_0;
    for (b_i = 0; b_i < b; b_i++) {
      obj =
        FK_DW.obj.TreeInternal.Base.CollisionsInternal.CollisionGeometries->
        data[b_i];
      collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
      FK_DW.obj.TreeInternal.Base.CollisionsInternal.CollisionGeometries->
        data[b_i] = obj;
    }
  }

  for (b = 0; b < 34; b++) {
    obj_1 = &FK_DW.obj.TreeInternal._pobj0[b].CollisionsInternal;
    if (!obj_1->matlabCodegenIsDeleted) {
      obj_1->matlabCodegenIsDeleted = true;
      b_0 = obj_1->Size;
      c = (int32_T)b_0;
      for (b_i = 0; b_i < c; b_i++) {
        obj = obj_1->CollisionGeometries->data[b_i];
        collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
        obj_1->CollisionGeometries->data[b_i] = obj;
      }
    }
  }

  /* End of Terminate for MATLABSystem: '<S3>/MATLAB System' */
  emxFreeStruct_robotics_slmani_m(&FK_DW.obj);

  /* Terminate for MATLABSystem: '<S4>/MATLAB System' */
  if (!FK_DW.obj_i.matlabCodegenIsDeleted) {
    FK_DW.obj_i.matlabCodegenIsDeleted = true;
  }

  if (!FK_DW.obj_i.TreeInternal.matlabCodegenIsDeleted) {
    FK_DW.obj_i.TreeInternal.matlabCodegenIsDeleted = true;
  }

  if (!FK_DW.obj_i.TreeInternal.Base.matlabCodegenIsDeleted) {
    FK_DW.obj_i.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }

  for (b = 0; b < 34; b++) {
    obj_0 = &FK_DW.obj_i.TreeInternal._pobj0[b];
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
    }
  }

  if (!FK_DW.obj_i.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted)
  {
    FK_DW.obj_i.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    b_0 = FK_DW.obj_i.TreeInternal.Base.CollisionsInternal.Size;
    b = (int32_T)b_0;
    for (b_i = 0; b_i < b; b_i++) {
      obj =
        FK_DW.obj_i.TreeInternal.Base.CollisionsInternal.CollisionGeometries->data
        [b_i];
      collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
      FK_DW.obj_i.TreeInternal.Base.CollisionsInternal.CollisionGeometries->
        data[b_i] = obj;
    }
  }

  for (b = 0; b < 34; b++) {
    obj_1 = &FK_DW.obj_i.TreeInternal._pobj0[b].CollisionsInternal;
    if (!obj_1->matlabCodegenIsDeleted) {
      obj_1->matlabCodegenIsDeleted = true;
      b_0 = obj_1->Size;
      c = (int32_T)b_0;
      for (b_i = 0; b_i < c; b_i++) {
        obj = obj_1->CollisionGeometries->data[b_i];
        collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
        obj_1->CollisionGeometries->data[b_i] = obj;
      }
    }
  }

  /* End of Terminate for MATLABSystem: '<S4>/MATLAB System' */
  emxFreeStruct_robotics_slmanip_(&FK_DW.obj_i);
}
