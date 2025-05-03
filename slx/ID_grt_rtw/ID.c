/*
 * ID.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "ID".
 *
 * Model version              : 2.2
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Sat May  3 16:27:29 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "ID.h"
#include "ID_types.h"
#include "rtwtypes.h"
#include <string.h>
#include <emmintrin.h>
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include "ID_private.h"

/* Block signals (default storage) */
B_ID_T ID_B;

/* Block states (default storage) */
DW_ID_T ID_DW;

/* External inputs (root inport signals with default storage) */
ExtU_ID_T ID_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_ID_T ID_Y;

/* Real-time model */
static RT_MODEL_ID_T ID_M_;
RT_MODEL_ID_T *const ID_M = &ID_M_;

/* Forward declaration for local functions */
static void emxInit_f_robotics_manip_intern(emxArray_f_robotics_manip_int_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_g_robotics_manip_(g_robotics_manip_internal_Col_T
  *pStruct);
static void emxInitStruct_e_robotics_manip_(e_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitMatrix_e_robotics_manip_(e_robotics_manip_internal_Rig_T
  pMatrix[34]);
static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_R_h_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_b_h_T
  *pStruct);
static void ID_rand_h(real_T r[5]);
static void rigidBodyJoint_set_MotionSubspa(b_rigidBodyJoint_ID_T *obj, const
  real_T msubspace_data[]);
static void emxEnsureCapacity_f_robotics_ma(emxArray_f_robotics_manip_int_T
  *emxArray, int32_T oldNumel);
static g_robotics_manip_internal_Col_T *ID_CollisionSet_CollisionSet
  (g_robotics_manip_internal_Col_T *obj);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody
  (e_robotics_manip_internal_Rig_T *obj, const char_T bodyInput[10]);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_h
  (e_robotics_manip_internal_Rig_T *obj, const char_T bodyInput[11]);
static void rigidBodyJoint_get_MotionSubspa(const b_rigidBodyJoint_ID_T *obj,
  real_T msubspace_data[], int32_T msubspace_size[2]);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hy
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hyn
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hynm
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hynmt
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hynmta
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hynmtam
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hynmtamd
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *I_RigidBody_RigidBody_hynmtamdm
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_RigidBody_hynmtamdms
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_RigidBody_hynmtamdmsn
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_RigidBod_hynmtamdmsnr
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_RigidBo_hynmtamdmsnr0
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_RigidB_hynmtamdmsnr0g
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *RigidBody_Rigid_hynmtamdmsnr0gd
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_a
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_l
  (e_robotics_manip_internal_Rig_T *obj);
static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_au
  (e_robotics_manip_internal_Rig_T *obj);
static void ID_MassMatrixBlock_setupImpl(robotics_slmanip_internal_b_h_T *obj);
static void emxInitStruct_f_robotics_mani_h(f_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmani_h(robotics_slmanip_internal__hy_T
  *pStruct);
static void ID_rand_hy(real_T r[5]);
static void VelocityProductTorqueBlock_setu(robotics_slmanip_internal__hy_T *obj);
static void emxInitStruct_robotics_slman_hy(robotics_slmanip_internal_blo_T
  *pStruct);
static void ID_rand(real_T r[5]);
static void ID_GravityTorqueBlock_setupImpl(robotics_slmanip_internal_blo_T *obj);
static void ID_emxInit_real_T(emxArray_real_T_ID_T **pEmxArray, int32_T
  numDimensions);
static void ID_emxEnsureCapacity_real_T(emxArray_real_T_ID_T *emxArray, int32_T
  oldNumel);
static void ID_rigidBodyJoint_get_JointAxis(const b_rigidBodyJoint_ID_T *obj,
  real_T ax[3]);
static void ID_cat(real_T varargin_1, real_T varargin_2, real_T varargin_3,
                   real_T varargin_4, real_T varargin_5, real_T varargin_6,
                   real_T varargin_7, real_T varargin_8, real_T varargin_9,
                   real_T y[9]);
static void rigidBodyJoint_transformBodyT_h(const b_rigidBodyJoint_ID_T *obj,
  const real_T q_data[], const int32_T *q_size, real_T T[16]);
static void rigidBodyJoint_transformBodyToP(const b_rigidBodyJoint_ID_T *obj,
  real_T T[16]);
static void ID_mtimes(const real_T A[36], const real_T B_data[], const int32_T
                      B_size[2], real_T C_data[], int32_T C_size[2]);
static void ID_mtimes_hyn(const real_T A_data[], const int32_T A_size[2], const
  real_T B_data[], const int32_T B_size[2], real_T C_data[], int32_T C_size[2]);
static void ID_mtimes_hynm(const real_T A[36], const real_T B_data[], const
  int32_T B_size[2], real_T C_data[], int32_T C_size[2]);
static void ID_emxFree_real_T(emxArray_real_T_ID_T **pEmxArray);
static void ID_mtimes_hynmtam(const real_T A_data[], const int32_T A_size[2],
  const real_T B[6], real_T C_data[], int32_T *C_size);
static void RigidBodyTreeDynamics_inverseDy(f_robotics_manip_internal_Rig_T
  *robot, const real_T q[12], real_T tau[12]);
static void emxFree_f_robotics_manip_intern(emxArray_f_robotics_manip_int_T
  **pEmxArray);
static void emxFreeStruct_g_robotics_manip_(g_robotics_manip_internal_Col_T
  *pStruct);
static void emxFreeStruct_e_robotics_manip_(e_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeMatrix_e_robotics_manip_(e_robotics_manip_internal_Rig_T
  pMatrix[34]);
static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_R_h_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_b_h_T
  *pStruct);
static void emxFreeStruct_f_robotics_mani_h(f_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_h(robotics_slmanip_internal__hy_T
  *pStruct);
static void emxFreeStruct_robotics_slman_hy(robotics_slmanip_internal_blo_T
  *pStruct);
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

static void emxInitStruct_e_robotics_manip_(e_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_g_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxInitMatrix_e_robotics_manip_(e_robotics_manip_internal_Rig_T
  pMatrix[34])
{
  int32_T i;
  for (i = 0; i < 34; i++) {
    emxInitStruct_e_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_R_h_T
  *pStruct)
{
  emxInitStruct_e_robotics_manip_(&pStruct->Base);
  emxInitMatrix_e_robotics_manip_(pStruct->_pobj0);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_b_h_T
  *pStruct)
{
  emxInitStruct_f_robotics_manip_(&pStruct->TreeInternal);
}

static void ID_rand_h(real_T r[5])
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
        mti = ID_DW.state_l[624] + 1U;
        if (ID_DW.state_l[624] + 1U >= 625U) {
          for (b_kk = 0; b_kk < 227; b_kk++) {
            y = (ID_DW.state_l[b_kk + 1] & 2147483647U) | (ID_DW.state_l[b_kk] &
              2147483648U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            ID_DW.state_l[b_kk] = ID_DW.state_l[b_kk + 397] ^ mti;
          }

          for (b_kk = 0; b_kk < 396; b_kk++) {
            y = (ID_DW.state_l[b_kk + 227] & 2147483648U) | (ID_DW.state_l[b_kk
              + 228] & 2147483647U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            ID_DW.state_l[b_kk + 227] = ID_DW.state_l[b_kk] ^ mti;
          }

          y = (ID_DW.state_l[623] & 2147483648U) | (ID_DW.state_l[0] &
            2147483647U);
          if ((y & 1U) == 0U) {
            mti = y >> 1U;
          } else {
            mti = y >> 1U ^ 2567483615U;
          }

          ID_DW.state_l[623] = ID_DW.state_l[396] ^ mti;
          mti = 1U;
        }

        y = ID_DW.state_l[(int32_T)mti - 1];
        ID_DW.state_l[624] = mti;
        y ^= y >> 11U;
        y ^= y << 7U & 2636928640U;
        y ^= y << 15U & 4022730752U;
        b_u[k] = y >> 18U ^ y;
      }

      mti = b_u[0] >> 5U;
      y = b_u[1] >> 6U;
      if ((mti == 0U) && (y == 0U)) {
        boolean_T b_isvalid;
        if ((ID_DW.state_l[624] >= 1U) && (ID_DW.state_l[624] < 625U)) {
          boolean_T exitg2;
          b_isvalid = false;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k + 1 < 625)) {
            if (ID_DW.state_l[k] == 0U) {
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
          ID_DW.state_l[0] = 5489U;
          ID_DW.state_l[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    r[b_k] = ((real_T)mti * 6.7108864E+7 + (real_T)y) * 1.1102230246251565E-16;
  }
}

static void rigidBodyJoint_set_MotionSubspa(b_rigidBodyJoint_ID_T *obj, const
  real_T msubspace_data[])
{
  int32_T b_kstr;
  int32_T i;
  boolean_T b_bool;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  b_bool = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  if (obj->TypeInternal.Length < 1.0) {
    b_kstr = 0;
  } else {
    b_kstr = (int32_T)obj->TypeInternal.Length;
  }

  if (b_kstr == 5) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

static g_robotics_manip_internal_Col_T *ID_CollisionSet_CollisionSet
  (g_robotics_manip_internal_Col_T *obj)
{
  static const void *t0_GeometryInternal = NULL;
  g_robotics_manip_internal_Col_T *b_obj;
  real_T c;
  int32_T b_i;
  int32_T d;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->CollisionGeometries->data[b_i].CollisionPrimitive = (void *)
      t0_GeometryInternal;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody
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

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 10.0;
  for (c = 0; c < 10; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 14.0;
  for (c = 0; c < 10; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[10] = '_';
  s.Vector[11] = 'j';
  s.Vector[12] = 'n';
  s.Vector[13] = 't';
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  for (c = 0; c < 36; c++) {
    b_I[c] = 0;
  }

  for (c = 0; c < 6; c++) {
    b_I[c + 6 * c] = 1;
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = b_I[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_h
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

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 15.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = bodyInput[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[11] = '_';
  s.Vector[12] = 'j';
  s.Vector[13] = 'n';
  s.Vector[14] = 't';
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_2[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  for (c = 0; c < 36; c++) {
    b_I[c] = 0;
  }

  for (c = 0; c < 6; c++) {
    b_I[c + 6 * c] = 1;
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = b_I[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static void rigidBodyJoint_get_MotionSubspa(const b_rigidBodyJoint_ID_T *obj,
  real_T msubspace_data[], int32_T msubspace_size[2])
{
  int32_T b_kstr;
  int32_T i;
  boolean_T b_bool;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  b_bool = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  if (obj->TypeInternal.Length < 1.0) {
    b_kstr = 0;
  } else {
    b_kstr = (int32_T)obj->TypeInternal.Length;
  }

  if (b_kstr == 5) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hy
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.00572623, 2.51E-6, -0.0001138, 0.0, -0.0,
    0.0, 2.51E-6, 0.00558959, -1.4E-7, 0.0, 0.0, -0.0, -0.0001138, -1.4E-7,
    0.00966674, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 1.59306955, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 1.59306955, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 1.59306955 };

  static const char_T tmp_0[12] = { 'l', '_', 'b', 'o', 'd', 'y', '_', 'f', 'i',
    'x', 'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_3[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_4[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_5[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_6[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -0.49705687905030665, -0.867717960508349, 0.0, -0.0, 0.867717960508349,
    -0.49705687905030665, 0.0, 0.0, 0.1551, 1.2924, 1.0 };

  static const int8_T tmp_7[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '0';
  obj->NameInternal = s;
  obj->ParentIndex = 1.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 12.0;
  for (c = 0; c < 12; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_5[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_6[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_7[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hyn
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.15418559, -2.35E-6, 1.739E-5, 0.0, -0.0, 0.0,
    -2.35E-6, 0.12937017, -0.04854267, 0.0, 0.0, -0.0, 1.739E-5, -0.04854267,
    0.05964415, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 11.8030102, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 11.8030102, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 11.8030102 };

  static const char_T tmp_0[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '0' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.08, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '1';
  obj->NameInternal = s;
  obj->ParentIndex = 2.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hynm
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.2935698, -4.0E-7, 1.441E-5, 0.0, -0.0, 0.0,
    -4.0E-7, 0.28094142, 0.03727972, 0.0, 0.0, -0.0, 1.441E-5, 0.03727972,
    0.03620609, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 7.99292141, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 7.99292141, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 7.99292141 };

  static const char_T tmp_0[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '1' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.109, 0.222, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '2';
  obj->NameInternal = s;
  obj->ParentIndex = 3.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hynmt
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.03424593, 1.49E-6, 7.24E-6, 0.0, -0.0, 0.0,
    1.49E-6, 0.03406024, 0.00186009, 0.0, 0.0, -0.0, 7.24E-6, 0.00186009,
    0.00450477, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 2.99134127, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 2.99134127, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 2.99134127 };

  static const char_T tmp_0[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '2' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, -0.45, 0.0, -0.0305, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '3';
  obj->NameInternal = s;
  obj->ParentIndex = 4.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hynmta
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.00670405, 3.75E-6, 1.5E-6, 0.0, -0.0, 0.0,
    3.75E-6, 0.00279246, -0.00127967, 0.0, 0.0, -0.0, 1.5E-6, -0.00127967,
    0.00619341, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 2.12317035, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 2.12317035, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 2.12317035 };

  static const char_T tmp_0[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '3' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.267, 0.0, -0.075, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '4';
  obj->NameInternal = s;
  obj->ParentIndex = 5.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hynmtam
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.00994891, 1.4E-7, 3.21E-6, 0.0, -0.0, 0.0,
    1.4E-7, 0.00978189, -0.00093546, 0.0, 0.0, -0.0, 3.21E-6, -0.00093546,
    0.00271492, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 2.28865091, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 2.28865091, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 2.28865091 };

  static const char_T tmp_0[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '4' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.114, 0.083, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '5';
  obj->NameInternal = s;
  obj->ParentIndex = 6.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_hynmtamd
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.00043534, 1.3E-7, -2.0E-8, 0.0, -0.0, 0.0,
    1.3E-7, 0.00044549, 5.1E-7, 0.0, 0.0, -0.0, -2.0E-8, 5.1E-7, 0.00059634,
    -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.40083918, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    0.40083918, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.40083918 };

  static const char_T tmp_0[9] = { 'l', '_', 'j', 'o', 'i', 'n', 't', '_', '5' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.168, 0.0, 0.069, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'l';
  s.Vector[1] = '_';
  s.Vector[2] = '6';
  obj->NameInternal = s;
  obj->ParentIndex = 7.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *I_RigidBody_RigidBody_hynmtamdm
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

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  obj->ParentIndex = 8.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = 0.0;
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_5[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_6[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_7[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_RigidBody_hynmtamdms
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.00572623, 2.51E-6, -0.0001138, 0.0, -0.0,
    0.0, 2.51E-6, 0.00558959, -1.4E-7, 0.0, 0.0, -0.0, -0.0001138, -1.4E-7,
    0.00966674, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 1.59306955, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 1.59306955, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 1.59306955 };

  static const char_T tmp_0[12] = { 'r', '_', 'b', 'o', 'd', 'y', '_', 'f', 'i',
    'x', 'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_3[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_4[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_5[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_6[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -0.49705687905030665, 0.867717960508349, 0.0, 0.0, -0.867717960508349,
    -0.49705687905030665, 0.0, 0.0, -0.1551, 1.2924, 1.0 };

  static const int8_T tmp_7[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '0';
  obj->NameInternal = s;
  obj->ParentIndex = 1.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 12.0;
  for (c = 0; c < 12; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_5[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_6[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_7[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_RigidBody_hynmtamdmsn
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.15418559, -2.35E-6, 1.739E-5, 0.0, -0.0, 0.0,
    -2.35E-6, 0.12937017, -0.04854267, 0.0, 0.0, -0.0, 1.739E-5, -0.04854267,
    0.05964415, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 11.8030102, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 11.8030102, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 11.8030102 };

  static const char_T tmp_0[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '0' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0775, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '1';
  obj->NameInternal = s;
  obj->ParentIndex = 10.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_RigidBod_hynmtamdmsnr
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.2935698, -4.0E-7, 1.441E-5, 0.0, -0.0, 0.0,
    -4.0E-7, 0.28094142, 0.03727972, 0.0, 0.0, -0.0, 1.441E-5, 0.03727972,
    0.03620609, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 7.99292141, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 7.99292141, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 7.99292141 };

  static const char_T tmp_0[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '1' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.109, 0.222, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '2';
  obj->NameInternal = s;
  obj->ParentIndex = 11.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_RigidBo_hynmtamdmsnr0
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.03424593, 1.49E-6, 7.24E-6, 0.0, -0.0, 0.0,
    1.49E-6, 0.03406024, 0.00186009, 0.0, 0.0, -0.0, 7.24E-6, 0.00186009,
    0.00450477, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 2.99134127, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 2.99134127, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 2.99134127 };

  static const char_T tmp_0[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '2' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, -0.45, 0.0, -0.0305, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '3';
  obj->NameInternal = s;
  obj->ParentIndex = 12.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_RigidB_hynmtamdmsnr0g
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.00670405, 3.75E-6, 1.5E-6, 0.0, -0.0, 0.0,
    3.75E-6, 0.00279246, -0.00127967, 0.0, 0.0, -0.0, 1.5E-6, -0.00127967,
    0.00619341, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 2.12317035, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 2.12317035, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 2.12317035 };

  static const char_T tmp_0[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '3' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.267, 0.0, -0.075, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '4';
  obj->NameInternal = s;
  obj->ParentIndex = 13.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *RigidBody_Rigid_hynmtamdmsnr0gd
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.00994891, 1.4E-7, 3.21E-6, 0.0, -0.0, 0.0,
    1.4E-7, 0.00978189, -0.00093546, 0.0, 0.0, -0.0, 3.21E-6, -0.00093546,
    0.00271492, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 2.28865091, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 2.28865091, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 2.28865091 };

  static const char_T tmp_0[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '4' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 9.6326794747667144E-5, 0.0,
    -0.99999999536057427, 0.0, 0.99999999072114854, 9.6326794747667144E-5,
    9.6326794300766137E-5, 0.0, 9.6326794300766137E-5, -0.99999999536057427,
    9.278851386359195E-9, 0.0, 0.0, -0.114, 0.083, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '5';
  obj->NameInternal = s;
  obj->ParentIndex = 14.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_a
  (e_robotics_manip_internal_Rig_T *obj)
{
  e_robotics_manip_internal_Cha_T s;
  e_robotics_manip_internal_Rig_T *b_obj;
  real_T msubspace_data[36];
  int32_T b_kstr;
  int32_T c;
  int8_T b_I[36];
  boolean_T result;
  static const real_T tmp[36] = { 0.00043534, 1.3E-7, -2.0E-8, 0.0, -0.0, 0.0,
    1.3E-7, 0.00044549, 5.1E-7, 0.0, 0.0, -0.0, -2.0E-8, 5.1E-7, 0.00059634,
    -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.40083918, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    0.40083918, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.40083918 };

  static const char_T tmp_0[9] = { 'r', '_', 'j', 'o', 'i', 'n', 't', '_', '5' };

  static const char_T a[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_3[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_4[6] = { 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[16] = { 9.6326794747667144E-5, 0.99999999536057427,
    -0.0, 0.0, -9.6326794300766137E-5, 9.278851386359195E-9,
    -0.99999999536057427, 0.0, -0.99999999072114854, 9.6326794300766137E-5,
    9.6326794747667144E-5, 0.0, -0.168, 0.0, 0.069, 1.0 };

  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int32_T msubspace_size[2];
  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 3.0;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  s.Vector[0] = 'r';
  s.Vector[1] = '_';
  s.Vector[2] = '6';
  obj->NameInternal = s;
  obj->ParentIndex = 15.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = tmp[c];
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 9.0;
  for (c = 0; c < 9; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 8.0;
  for (c = 0; c < 8; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = a[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_5[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_6[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = tmp_3[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_l
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

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  obj->ParentIndex = 16.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = 0.0;
  }

  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.NameInternal;
  s.Length = 11.0;
  for (c = 0; c < 11; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp_0[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s = obj->JointInternal.TypeInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_5[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.JointToParentTransform[c] = tmp_6[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.ChildToJointTransform[c] = tmp_7[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, msubspace_data,
    msubspace_size);
  for (c = 0; c < 6; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    msubspace_data[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static e_robotics_manip_internal_Rig_T *ID_RigidBody_RigidBody_au
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

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  b_obj = obj;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  s = obj->NameInternal;
  s.Length = 5.0;
  for (c = 0; c < 5; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = tmp[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->NameInternal = s;
  obj->ParentIndex = -1.0;
  for (c = 0; c < 36; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->SpatialInertia[c] = 0.0;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
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
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.MotionSubspaceInternal[c] = 0.0;
  }

  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (c = 0; c < 200; c++) {
    s.Vector[c] = ' ';
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    s.Vector[c] = b[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  obj->JointInternal.TypeInternal = s;
  s = obj->JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    c = 0;
  } else {
    c = (int32_T)s.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
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
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_3[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (c = 0; c < 6; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      msubspace_data[c] = tmp_4[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 2:
    for (c = 0; c < 36; c++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      b_I[c] = 0;
    }

    for (c = 0; c < 6; c++) {
      b_I[c + 6 * c] = 1;
    }

    for (c = 0; c < 36; c++) {
      msubspace_data[c] = b_I[c];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  rigidBodyJoint_set_MotionSubspa(&obj->JointInternal, msubspace_data);
  ID_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static void ID_MassMatrixBlock_setupImpl(robotics_slmanip_internal_b_h_T *obj)
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

  static const real_T tmp_g[36] = { 0.00572623, 2.51E-6, -0.0001138, 0.0, -0.0,
    0.0, 2.51E-6, 0.00558959, -1.4E-7, 0.0, 0.0, -0.0, -0.0001138, -1.4E-7,
    0.00966674, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 1.59306955, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 1.59306955, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 1.59306955 };

  static const char_T tmp_h[11] = { 'w', 'o', 'r', 'l', 'd', '_', 'f', 'i', 'x',
    'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_i[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_j[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_k[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_l[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_m[6] = { 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_n[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const int8_T tmp_o[34] = { 0, 0, 1, 2, 3, 4, 5, 6, 0, 0, 7, 8, 9, 10,
    11, 12, 0, -1, -1, 1, 2, 3, 4, 5, 6, -1, -1, 7, 8, 9, 10, 11, 12, -1 };

  real_T unusedExpr[5];
  int32_T msubspace_size[2];
  int32_T exitg1;
  ID_rand_h(unusedExpr);

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  obj->TreeInternal.NumBodies = 17.0;
  obj->TreeInternal.Bodies[0] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[0], tmp);
  obj->TreeInternal.Bodies[1] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[1], tmp_0);
  obj->TreeInternal.Bodies[2] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[2], tmp_1);
  obj->TreeInternal.Bodies[3] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[3], tmp_2);
  obj->TreeInternal.Bodies[4] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[4], tmp_3);
  obj->TreeInternal.Bodies[5] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[5], tmp_4);
  obj->TreeInternal.Bodies[6] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[6], tmp_5);
  obj->TreeInternal.Bodies[7] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[7], tmp_6);
  obj->TreeInternal.Bodies[8] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[8], tmp_7);
  obj->TreeInternal.Bodies[9] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[9], tmp_8);
  obj->TreeInternal.Bodies[10] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[10], tmp_9);
  obj->TreeInternal.Bodies[11] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[11], tmp_a);
  obj->TreeInternal.Bodies[12] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[12], tmp_b);
  obj->TreeInternal.Bodies[13] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[13], tmp_c);
  obj->TreeInternal.Bodies[14] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[14], tmp_d);
  obj->TreeInternal.Bodies[15] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[15], tmp_e);
  obj->TreeInternal.Bodies[16] = ID_RigidBody_RigidBody_h
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
    obj->TreeInternal._pobj0[17].SpatialInertia[c] = tmp_g[c];
  }

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
    s.Vector[c] = tmp_h[c];
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
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S4>/MATLAB System' */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
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
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_k[b_kstr - 1] != s.Vector[b_kstr - 1]) {
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
      msubspace_data[c] = tmp_l[c];
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
      msubspace_data[c] = tmp_m[c];
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
      tmp_n[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.ChildToJointTransform[c] =
      tmp_n[c];
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
  ID_CollisionSet_CollisionSet(&obj->TreeInternal._pobj0[17].CollisionsInternal);
  obj->TreeInternal._pobj0[17].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[0] = &obj->TreeInternal._pobj0[17];
  obj->TreeInternal.Bodies[0]->Index = 1.0;
  obj->TreeInternal.Bodies[1] = ID_RigidBody_RigidBody_hy
    (&obj->TreeInternal._pobj0[18]);
  obj->TreeInternal.Bodies[1]->Index = 2.0;
  obj->TreeInternal.Bodies[2] = ID_RigidBody_RigidBody_hyn
    (&obj->TreeInternal._pobj0[19]);
  obj->TreeInternal.Bodies[2]->Index = 3.0;
  obj->TreeInternal.Bodies[3] = ID_RigidBody_RigidBody_hynm
    (&obj->TreeInternal._pobj0[20]);
  obj->TreeInternal.Bodies[3]->Index = 4.0;
  obj->TreeInternal.Bodies[4] = ID_RigidBody_RigidBody_hynmt
    (&obj->TreeInternal._pobj0[21]);
  obj->TreeInternal.Bodies[4]->Index = 5.0;
  obj->TreeInternal.Bodies[5] = ID_RigidBody_RigidBody_hynmta
    (&obj->TreeInternal._pobj0[22]);
  obj->TreeInternal.Bodies[5]->Index = 6.0;
  obj->TreeInternal.Bodies[6] = ID_RigidBody_RigidBody_hynmtam
    (&obj->TreeInternal._pobj0[23]);
  obj->TreeInternal.Bodies[6]->Index = 7.0;
  obj->TreeInternal.Bodies[7] = ID_RigidBody_RigidBody_hynmtamd
    (&obj->TreeInternal._pobj0[24]);
  obj->TreeInternal.Bodies[7]->Index = 8.0;
  obj->TreeInternal.Bodies[8] = I_RigidBody_RigidBody_hynmtamdm
    (&obj->TreeInternal._pobj0[25]);
  obj->TreeInternal.Bodies[8]->Index = 9.0;
  obj->TreeInternal.Bodies[9] = RigidBody_RigidBody_hynmtamdms
    (&obj->TreeInternal._pobj0[26]);
  obj->TreeInternal.Bodies[9]->Index = 10.0;
  obj->TreeInternal.Bodies[10] = RigidBody_RigidBody_hynmtamdmsn
    (&obj->TreeInternal._pobj0[27]);
  obj->TreeInternal.Bodies[10]->Index = 11.0;
  obj->TreeInternal.Bodies[11] = RigidBody_RigidBod_hynmtamdmsnr
    (&obj->TreeInternal._pobj0[28]);
  obj->TreeInternal.Bodies[11]->Index = 12.0;
  obj->TreeInternal.Bodies[12] = RigidBody_RigidBo_hynmtamdmsnr0
    (&obj->TreeInternal._pobj0[29]);
  obj->TreeInternal.Bodies[12]->Index = 13.0;
  obj->TreeInternal.Bodies[13] = RigidBody_RigidB_hynmtamdmsnr0g
    (&obj->TreeInternal._pobj0[30]);
  obj->TreeInternal.Bodies[13]->Index = 14.0;
  obj->TreeInternal.Bodies[14] = RigidBody_Rigid_hynmtamdmsnr0gd
    (&obj->TreeInternal._pobj0[31]);
  obj->TreeInternal.Bodies[14]->Index = 15.0;
  obj->TreeInternal.Bodies[15] = ID_RigidBody_RigidBody_a
    (&obj->TreeInternal._pobj0[32]);
  obj->TreeInternal.Bodies[15]->Index = 16.0;
  obj->TreeInternal.Bodies[16] = ID_RigidBody_RigidBody_l
    (&obj->TreeInternal._pobj0[33]);
  obj->TreeInternal.Bodies[16]->Index = 17.0;
  obj->TreeInternal.VelocityNumber = 12.0;
  for (c = 0; c < 34; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    obj->TreeInternal.PositionDoFMap[c] = tmp_o[c];
  }

  for (c = 0; c < 34; c++) {
    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    obj->TreeInternal.VelocityDoFMap[c] = tmp_o[c];
  }

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  ID_RigidBody_RigidBody_au(&obj->TreeInternal.Base);
  obj->TreeInternal.Base.Index = 0.0;
  obj->TreeInternal.matlabCodegenIsDeleted = false;
}

static void emxInitStruct_f_robotics_mani_h(f_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_e_robotics_manip_(&pStruct->Base);
  emxInitMatrix_e_robotics_manip_(pStruct->_pobj0);
}

static void emxInitStruct_robotics_slmani_h(robotics_slmanip_internal__hy_T
  *pStruct)
{
  emxInitStruct_f_robotics_mani_h(&pStruct->TreeInternal);
}

static void ID_rand_hy(real_T r[5])
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
        mti = ID_DW.state_m[624] + 1U;
        if (ID_DW.state_m[624] + 1U >= 625U) {
          for (b_kk = 0; b_kk < 227; b_kk++) {
            y = (ID_DW.state_m[b_kk + 1] & 2147483647U) | (ID_DW.state_m[b_kk] &
              2147483648U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            ID_DW.state_m[b_kk] = ID_DW.state_m[b_kk + 397] ^ mti;
          }

          for (b_kk = 0; b_kk < 396; b_kk++) {
            y = (ID_DW.state_m[b_kk + 227] & 2147483648U) | (ID_DW.state_m[b_kk
              + 228] & 2147483647U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            ID_DW.state_m[b_kk + 227] = ID_DW.state_m[b_kk] ^ mti;
          }

          y = (ID_DW.state_m[623] & 2147483648U) | (ID_DW.state_m[0] &
            2147483647U);
          if ((y & 1U) == 0U) {
            mti = y >> 1U;
          } else {
            mti = y >> 1U ^ 2567483615U;
          }

          ID_DW.state_m[623] = ID_DW.state_m[396] ^ mti;
          mti = 1U;
        }

        y = ID_DW.state_m[(int32_T)mti - 1];
        ID_DW.state_m[624] = mti;
        y ^= y >> 11U;
        y ^= y << 7U & 2636928640U;
        y ^= y << 15U & 4022730752U;
        b_u[k] = y >> 18U ^ y;
      }

      mti = b_u[0] >> 5U;
      y = b_u[1] >> 6U;
      if ((mti == 0U) && (y == 0U)) {
        boolean_T b_isvalid;
        if ((ID_DW.state_m[624] >= 1U) && (ID_DW.state_m[624] < 625U)) {
          boolean_T exitg2;
          b_isvalid = false;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k + 1 < 625)) {
            if (ID_DW.state_m[k] == 0U) {
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
          ID_DW.state_m[0] = 5489U;
          ID_DW.state_m[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    r[b_k] = ((real_T)mti * 6.7108864E+7 + (real_T)y) * 1.1102230246251565E-16;
  }
}

static void VelocityProductTorqueBlock_setu(robotics_slmanip_internal__hy_T *obj)
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

  static const real_T tmp_g[36] = { 0.00572623, 2.51E-6, -0.0001138, 0.0, -0.0,
    0.0, 2.51E-6, 0.00558959, -1.4E-7, 0.0, 0.0, -0.0, -0.0001138, -1.4E-7,
    0.00966674, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 1.59306955, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 1.59306955, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 1.59306955 };

  static const char_T tmp_h[11] = { 'w', 'o', 'r', 'l', 'd', '_', 'f', 'i', 'x',
    'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_i[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_j[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_k[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_l[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_m[6] = { 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_n[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const int8_T tmp_o[34] = { 0, 0, 1, 2, 3, 4, 5, 6, 0, 0, 7, 8, 9, 10,
    11, 12, 0, -1, -1, 1, 2, 3, 4, 5, 6, -1, -1, 7, 8, 9, 10, 11, 12, -1 };

  real_T unusedExpr[5];
  int32_T msubspace_size[2];
  int32_T exitg1;
  ID_rand_hy(unusedExpr);

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  obj->TreeInternal.NumBodies = 17.0;
  obj->TreeInternal.Bodies[0] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[0], tmp);
  obj->TreeInternal.Bodies[1] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[1], tmp_0);
  obj->TreeInternal.Bodies[2] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[2], tmp_1);
  obj->TreeInternal.Bodies[3] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[3], tmp_2);
  obj->TreeInternal.Bodies[4] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[4], tmp_3);
  obj->TreeInternal.Bodies[5] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[5], tmp_4);
  obj->TreeInternal.Bodies[6] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[6], tmp_5);
  obj->TreeInternal.Bodies[7] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[7], tmp_6);
  obj->TreeInternal.Bodies[8] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[8], tmp_7);
  obj->TreeInternal.Bodies[9] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[9], tmp_8);
  obj->TreeInternal.Bodies[10] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[10], tmp_9);
  obj->TreeInternal.Bodies[11] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[11], tmp_a);
  obj->TreeInternal.Bodies[12] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[12], tmp_b);
  obj->TreeInternal.Bodies[13] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[13], tmp_c);
  obj->TreeInternal.Bodies[14] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[14], tmp_d);
  obj->TreeInternal.Bodies[15] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[15], tmp_e);
  obj->TreeInternal.Bodies[16] = ID_RigidBody_RigidBody_h
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
    obj->TreeInternal._pobj0[17].SpatialInertia[c] = tmp_g[c];
  }

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
    s.Vector[c] = tmp_h[c];
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
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
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
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_k[b_kstr - 1] != s.Vector[b_kstr - 1]) {
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
      msubspace_data[c] = tmp_l[c];
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
      msubspace_data[c] = tmp_m[c];
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
      tmp_n[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.ChildToJointTransform[c] =
      tmp_n[c];
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
  ID_CollisionSet_CollisionSet(&obj->TreeInternal._pobj0[17].CollisionsInternal);
  obj->TreeInternal._pobj0[17].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[0] = &obj->TreeInternal._pobj0[17];
  obj->TreeInternal.Bodies[0]->Index = 1.0;
  obj->TreeInternal.Bodies[1] = ID_RigidBody_RigidBody_hy
    (&obj->TreeInternal._pobj0[18]);
  obj->TreeInternal.Bodies[1]->Index = 2.0;
  obj->TreeInternal.Bodies[2] = ID_RigidBody_RigidBody_hyn
    (&obj->TreeInternal._pobj0[19]);
  obj->TreeInternal.Bodies[2]->Index = 3.0;
  obj->TreeInternal.Bodies[3] = ID_RigidBody_RigidBody_hynm
    (&obj->TreeInternal._pobj0[20]);
  obj->TreeInternal.Bodies[3]->Index = 4.0;
  obj->TreeInternal.Bodies[4] = ID_RigidBody_RigidBody_hynmt
    (&obj->TreeInternal._pobj0[21]);
  obj->TreeInternal.Bodies[4]->Index = 5.0;
  obj->TreeInternal.Bodies[5] = ID_RigidBody_RigidBody_hynmta
    (&obj->TreeInternal._pobj0[22]);
  obj->TreeInternal.Bodies[5]->Index = 6.0;
  obj->TreeInternal.Bodies[6] = ID_RigidBody_RigidBody_hynmtam
    (&obj->TreeInternal._pobj0[23]);
  obj->TreeInternal.Bodies[6]->Index = 7.0;
  obj->TreeInternal.Bodies[7] = ID_RigidBody_RigidBody_hynmtamd
    (&obj->TreeInternal._pobj0[24]);
  obj->TreeInternal.Bodies[7]->Index = 8.0;
  obj->TreeInternal.Bodies[8] = I_RigidBody_RigidBody_hynmtamdm
    (&obj->TreeInternal._pobj0[25]);
  obj->TreeInternal.Bodies[8]->Index = 9.0;
  obj->TreeInternal.Bodies[9] = RigidBody_RigidBody_hynmtamdms
    (&obj->TreeInternal._pobj0[26]);
  obj->TreeInternal.Bodies[9]->Index = 10.0;
  obj->TreeInternal.Bodies[10] = RigidBody_RigidBody_hynmtamdmsn
    (&obj->TreeInternal._pobj0[27]);
  obj->TreeInternal.Bodies[10]->Index = 11.0;
  obj->TreeInternal.Bodies[11] = RigidBody_RigidBod_hynmtamdmsnr
    (&obj->TreeInternal._pobj0[28]);
  obj->TreeInternal.Bodies[11]->Index = 12.0;
  obj->TreeInternal.Bodies[12] = RigidBody_RigidBo_hynmtamdmsnr0
    (&obj->TreeInternal._pobj0[29]);
  obj->TreeInternal.Bodies[12]->Index = 13.0;
  obj->TreeInternal.Bodies[13] = RigidBody_RigidB_hynmtamdmsnr0g
    (&obj->TreeInternal._pobj0[30]);
  obj->TreeInternal.Bodies[13]->Index = 14.0;
  obj->TreeInternal.Bodies[14] = RigidBody_Rigid_hynmtamdmsnr0gd
    (&obj->TreeInternal._pobj0[31]);
  obj->TreeInternal.Bodies[14]->Index = 15.0;
  obj->TreeInternal.Bodies[15] = ID_RigidBody_RigidBody_a
    (&obj->TreeInternal._pobj0[32]);
  obj->TreeInternal.Bodies[15]->Index = 16.0;
  obj->TreeInternal.Bodies[16] = ID_RigidBody_RigidBody_l
    (&obj->TreeInternal._pobj0[33]);
  obj->TreeInternal.Bodies[16]->Index = 17.0;
  obj->TreeInternal.Gravity[0] = 0.0;
  obj->TreeInternal.Gravity[1] = 0.0;
  obj->TreeInternal.Gravity[2] = -9.80665;
  for (c = 0; c < 34; c++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    obj->TreeInternal.PositionDoFMap[c] = tmp_o[c];
  }

  for (c = 0; c < 34; c++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    obj->TreeInternal.VelocityDoFMap[c] = tmp_o[c];
  }

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  ID_RigidBody_RigidBody_au(&obj->TreeInternal.Base);
  obj->TreeInternal.Base.Index = 0.0;
  obj->TreeInternal.matlabCodegenIsDeleted = false;
}

static void emxInitStruct_robotics_slman_hy(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_f_robotics_mani_h(&pStruct->TreeInternal);
}

static void ID_rand(real_T r[5])
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
        mti = ID_DW.state_d[624] + 1U;
        if (ID_DW.state_d[624] + 1U >= 625U) {
          for (b_kk = 0; b_kk < 227; b_kk++) {
            y = (ID_DW.state_d[b_kk + 1] & 2147483647U) | (ID_DW.state_d[b_kk] &
              2147483648U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            ID_DW.state_d[b_kk] = ID_DW.state_d[b_kk + 397] ^ mti;
          }

          for (b_kk = 0; b_kk < 396; b_kk++) {
            y = (ID_DW.state_d[b_kk + 227] & 2147483648U) | (ID_DW.state_d[b_kk
              + 228] & 2147483647U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            ID_DW.state_d[b_kk + 227] = ID_DW.state_d[b_kk] ^ mti;
          }

          y = (ID_DW.state_d[623] & 2147483648U) | (ID_DW.state_d[0] &
            2147483647U);
          if ((y & 1U) == 0U) {
            mti = y >> 1U;
          } else {
            mti = y >> 1U ^ 2567483615U;
          }

          ID_DW.state_d[623] = ID_DW.state_d[396] ^ mti;
          mti = 1U;
        }

        y = ID_DW.state_d[(int32_T)mti - 1];
        ID_DW.state_d[624] = mti;
        y ^= y >> 11U;
        y ^= y << 7U & 2636928640U;
        y ^= y << 15U & 4022730752U;
        b_u[k] = y >> 18U ^ y;
      }

      mti = b_u[0] >> 5U;
      y = b_u[1] >> 6U;
      if ((mti == 0U) && (y == 0U)) {
        boolean_T b_isvalid;
        if ((ID_DW.state_d[624] >= 1U) && (ID_DW.state_d[624] < 625U)) {
          boolean_T exitg2;
          b_isvalid = false;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k + 1 < 625)) {
            if (ID_DW.state_d[k] == 0U) {
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
          ID_DW.state_d[0] = 5489U;
          ID_DW.state_d[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    r[b_k] = ((real_T)mti * 6.7108864E+7 + (real_T)y) * 1.1102230246251565E-16;
  }
}

static void ID_GravityTorqueBlock_setupImpl(robotics_slmanip_internal_blo_T *obj)
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

  static const real_T tmp_g[36] = { 0.00572623, 2.51E-6, -0.0001138, 0.0, -0.0,
    0.0, 2.51E-6, 0.00558959, -1.4E-7, 0.0, 0.0, -0.0, -0.0001138, -1.4E-7,
    0.00966674, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 1.59306955, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 1.59306955, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 1.59306955 };

  static const char_T tmp_h[11] = { 'w', 'o', 'r', 'l', 'd', '_', 'f', 'i', 'x',
    'e', 'd' };

  static const char_T b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_i[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_j[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_k[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  static const int8_T tmp_l[6] = { 0, 0, 1, 0, 0, 0 };

  static const int8_T tmp_m[6] = { 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_n[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const int8_T tmp_o[34] = { 0, 0, 1, 2, 3, 4, 5, 6, 0, 0, 7, 8, 9, 10,
    11, 12, 0, -1, -1, 1, 2, 3, 4, 5, 6, -1, -1, 7, 8, 9, 10, 11, 12, -1 };

  real_T unusedExpr[5];
  int32_T msubspace_size[2];
  int32_T exitg1;
  ID_rand(unusedExpr);

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  obj->TreeInternal.NumBodies = 17.0;
  obj->TreeInternal.Bodies[0] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[0], tmp);
  obj->TreeInternal.Bodies[1] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[1], tmp_0);
  obj->TreeInternal.Bodies[2] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[2], tmp_1);
  obj->TreeInternal.Bodies[3] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[3], tmp_2);
  obj->TreeInternal.Bodies[4] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[4], tmp_3);
  obj->TreeInternal.Bodies[5] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[5], tmp_4);
  obj->TreeInternal.Bodies[6] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[6], tmp_5);
  obj->TreeInternal.Bodies[7] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[7], tmp_6);
  obj->TreeInternal.Bodies[8] = ID_RigidBody_RigidBody(&obj->
    TreeInternal._pobj0[8], tmp_7);
  obj->TreeInternal.Bodies[9] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[9], tmp_8);
  obj->TreeInternal.Bodies[10] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[10], tmp_9);
  obj->TreeInternal.Bodies[11] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[11], tmp_a);
  obj->TreeInternal.Bodies[12] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[12], tmp_b);
  obj->TreeInternal.Bodies[13] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[13], tmp_c);
  obj->TreeInternal.Bodies[14] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[14], tmp_d);
  obj->TreeInternal.Bodies[15] = ID_RigidBody_RigidBody_h
    (&obj->TreeInternal._pobj0[15], tmp_e);
  obj->TreeInternal.Bodies[16] = ID_RigidBody_RigidBody_h
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
    obj->TreeInternal._pobj0[17].SpatialInertia[c] = tmp_g[c];
  }

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
    s.Vector[c] = tmp_h[c];
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
    c = 0;
  } else {
    if (c == 9) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
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
      c = 1;
    } else {
      if (c == 8) {
        /* Start for MATLABSystem: '<S3>/MATLAB System' */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_k[b_kstr - 1] != s.Vector[b_kstr - 1]) {
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
      msubspace_data[c] = tmp_l[c];
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
      msubspace_data[c] = tmp_m[c];
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
      tmp_n[c];
  }

  for (c = 0; c < 16; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    obj->TreeInternal._pobj0[17].JointInternal.ChildToJointTransform[c] =
      tmp_n[c];
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
  ID_CollisionSet_CollisionSet(&obj->TreeInternal._pobj0[17].CollisionsInternal);
  obj->TreeInternal._pobj0[17].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[0] = &obj->TreeInternal._pobj0[17];
  obj->TreeInternal.Bodies[0]->Index = 1.0;
  obj->TreeInternal.Bodies[1] = ID_RigidBody_RigidBody_hy
    (&obj->TreeInternal._pobj0[18]);
  obj->TreeInternal.Bodies[1]->Index = 2.0;
  obj->TreeInternal.Bodies[2] = ID_RigidBody_RigidBody_hyn
    (&obj->TreeInternal._pobj0[19]);
  obj->TreeInternal.Bodies[2]->Index = 3.0;
  obj->TreeInternal.Bodies[3] = ID_RigidBody_RigidBody_hynm
    (&obj->TreeInternal._pobj0[20]);
  obj->TreeInternal.Bodies[3]->Index = 4.0;
  obj->TreeInternal.Bodies[4] = ID_RigidBody_RigidBody_hynmt
    (&obj->TreeInternal._pobj0[21]);
  obj->TreeInternal.Bodies[4]->Index = 5.0;
  obj->TreeInternal.Bodies[5] = ID_RigidBody_RigidBody_hynmta
    (&obj->TreeInternal._pobj0[22]);
  obj->TreeInternal.Bodies[5]->Index = 6.0;
  obj->TreeInternal.Bodies[6] = ID_RigidBody_RigidBody_hynmtam
    (&obj->TreeInternal._pobj0[23]);
  obj->TreeInternal.Bodies[6]->Index = 7.0;
  obj->TreeInternal.Bodies[7] = ID_RigidBody_RigidBody_hynmtamd
    (&obj->TreeInternal._pobj0[24]);
  obj->TreeInternal.Bodies[7]->Index = 8.0;
  obj->TreeInternal.Bodies[8] = I_RigidBody_RigidBody_hynmtamdm
    (&obj->TreeInternal._pobj0[25]);
  obj->TreeInternal.Bodies[8]->Index = 9.0;
  obj->TreeInternal.Bodies[9] = RigidBody_RigidBody_hynmtamdms
    (&obj->TreeInternal._pobj0[26]);
  obj->TreeInternal.Bodies[9]->Index = 10.0;
  obj->TreeInternal.Bodies[10] = RigidBody_RigidBody_hynmtamdmsn
    (&obj->TreeInternal._pobj0[27]);
  obj->TreeInternal.Bodies[10]->Index = 11.0;
  obj->TreeInternal.Bodies[11] = RigidBody_RigidBod_hynmtamdmsnr
    (&obj->TreeInternal._pobj0[28]);
  obj->TreeInternal.Bodies[11]->Index = 12.0;
  obj->TreeInternal.Bodies[12] = RigidBody_RigidBo_hynmtamdmsnr0
    (&obj->TreeInternal._pobj0[29]);
  obj->TreeInternal.Bodies[12]->Index = 13.0;
  obj->TreeInternal.Bodies[13] = RigidBody_RigidB_hynmtamdmsnr0g
    (&obj->TreeInternal._pobj0[30]);
  obj->TreeInternal.Bodies[13]->Index = 14.0;
  obj->TreeInternal.Bodies[14] = RigidBody_Rigid_hynmtamdmsnr0gd
    (&obj->TreeInternal._pobj0[31]);
  obj->TreeInternal.Bodies[14]->Index = 15.0;
  obj->TreeInternal.Bodies[15] = ID_RigidBody_RigidBody_a
    (&obj->TreeInternal._pobj0[32]);
  obj->TreeInternal.Bodies[15]->Index = 16.0;
  obj->TreeInternal.Bodies[16] = ID_RigidBody_RigidBody_l
    (&obj->TreeInternal._pobj0[33]);
  obj->TreeInternal.Bodies[16]->Index = 17.0;
  obj->TreeInternal.Gravity[0] = 0.0;
  obj->TreeInternal.Gravity[1] = 0.0;
  obj->TreeInternal.Gravity[2] = -9.80665;
  for (c = 0; c < 34; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    obj->TreeInternal.PositionDoFMap[c] = tmp_o[c];
  }

  for (c = 0; c < 34; c++) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    obj->TreeInternal.VelocityDoFMap[c] = tmp_o[c];
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  ID_RigidBody_RigidBody_au(&obj->TreeInternal.Base);
  obj->TreeInternal.Base.Index = 0.0;
  obj->TreeInternal.matlabCodegenIsDeleted = false;
}

static void ID_emxInit_real_T(emxArray_real_T_ID_T **pEmxArray, int32_T
  numDimensions)
{
  emxArray_real_T_ID_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T_ID_T *)malloc(sizeof(emxArray_real_T_ID_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void ID_emxEnsureCapacity_real_T(emxArray_real_T_ID_T *emxArray, int32_T
  oldNumel)
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

    newData = malloc((uint32_T)i * sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void ID_rigidBodyJoint_get_JointAxis(const b_rigidBodyJoint_ID_T *obj,
  real_T ax[3])
{
  int32_T b_kstr;
  boolean_T b_bool;
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  boolean_T guard1;
  b_bool = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  if (obj->TypeInternal.Length < 1.0) {
    b_kstr = 0;
  } else {
    b_kstr = (int32_T)obj->TypeInternal.Length;
  }

  if (b_kstr == 8) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
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
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    if (obj->TypeInternal.Length < 1.0) {
      b_kstr = 0;
    } else {
      b_kstr = (int32_T)obj->TypeInternal.Length;
    }

    if (b_kstr == 9) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
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

static void ID_cat(real_T varargin_1, real_T varargin_2, real_T varargin_3,
                   real_T varargin_4, real_T varargin_5, real_T varargin_6,
                   real_T varargin_7, real_T varargin_8, real_T varargin_9,
                   real_T y[9])
{
  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
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

static void rigidBodyJoint_transformBodyT_h(const b_rigidBodyJoint_ID_T *obj,
  const real_T q_data[], const int32_T *q_size, real_T T[16])
{
  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_3;
  real_T b[16];
  real_T b_0[16];
  real_T b_I[16];
  real_T R[9];
  real_T tempR[9];
  real_T result_data[4];
  real_T v[3];
  real_T tmp_2[2];
  real_T b_q_idx_1;
  real_T cth;
  real_T tempR_tmp;
  real_T tempR_tmp_0;
  real_T tempR_tmp_1;
  real_T tempR_tmp_2;
  real_T theta;
  int32_T b_kstr;
  int32_T i;
  int32_T tmp_4;
  boolean_T result;
  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_7[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  real_T cth_tmp;
  real_T cth_tmp_0;
  int32_T exitg1;
  int32_T result_data_tmp;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  if (obj->TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = (int32_T)obj->TypeInternal.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  if (i == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_5[b_kstr - 1] != obj->TypeInternal.Vector[b_kstr - 1]) {
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
    cth = 0.0;
  } else {
    if (i == 9) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_6[b_kstr - 1] != obj->TypeInternal.Vector[b_kstr - 1]) {
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
      cth = 1.0;
    } else {
      if (i == 8) {
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_7[b_kstr - 1] != obj->TypeInternal.Vector[b_kstr - 1]) {
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
        cth = 2.0;
      } else {
        cth = -1.0;
      }
    }
  }

  switch ((int32_T)cth) {
   case 0:
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    ID_rigidBodyJoint_get_JointAxis(obj, v);
    result_data[0] = v[0];
    result_data[1] = v[1];
    result_data[2] = v[2];
    if ((*q_size != 0) - 1 >= 0) {
      result_data[3] = q_data[0];
    }

    theta = result_data[0];
    cth_tmp = result_data[1];
    cth_tmp_0 = result_data[2];
    cth = 1.0 / sqrt((theta * theta + cth_tmp * cth_tmp) + cth_tmp_0 * cth_tmp_0);
    _mm_storeu_pd(&v[0], _mm_mul_pd(_mm_set_pd(cth_tmp, theta), _mm_set1_pd(cth)));
    v[2] = cth_tmp_0 * cth;
    theta = result_data[3];
    cth = cos(theta);
    theta = sin(theta);
    cth_tmp = v[0] * v[1] * (1.0 - cth);
    cth_tmp_0 = v[2] * theta;
    tempR_tmp = v[0] * v[2] * (1.0 - cth);
    tempR_tmp_0 = v[1] * theta;
    tempR_tmp_1 = v[1] * v[2] * (1.0 - cth);
    theta *= v[0];
    ID_cat(v[0] * v[0] * (1.0 - cth) + cth, cth_tmp - cth_tmp_0, tempR_tmp +
           tempR_tmp_0, cth_tmp + cth_tmp_0, v[1] * v[1] * (1.0 - cth) + cth,
           tempR_tmp_1 - theta, tempR_tmp - tempR_tmp_0, tempR_tmp_1 + theta, v
           [2] * v[2] * (1.0 - cth) + cth, tempR);
    for (i = 0; i < 3; i++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      R[i] = tempR[i * 3];
      R[i + 3] = tempR[i * 3 + 1];
      R[i + 6] = tempR[i * 3 + 2];
    }

    memset(&b[0], 0, sizeof(real_T) << 4U);
    for (i = 0; i < 3; i++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      result_data_tmp = i << 2;
      b[result_data_tmp] = R[3 * i];
      b[result_data_tmp + 1] = R[3 * i + 1];
      b[result_data_tmp + 2] = R[3 * i + 2];
    }

    b[15] = 1.0;
    break;

   case 1:
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    ID_rigidBodyJoint_get_JointAxis(obj, v);
    memset(&tempR[0], 0, 9U * sizeof(real_T));
    tempR[0] = 1.0;
    tempR[4] = 1.0;
    tempR[8] = 1.0;
    cth_tmp = q_data[0];
    for (i = 0; i < 3; i++) {
      result_data_tmp = i << 2;

      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      b[result_data_tmp] = tempR[3 * i];
      b[result_data_tmp + 1] = tempR[3 * i + 1];
      b[result_data_tmp + 2] = tempR[3 * i + 2];
      b[i + 12] = v[i] * cth_tmp;
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

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    b_I[12] = q_data[4];
    b_I[13] = q_data[5];
    b_I[14] = q_data[6];
    tmp_3 = _mm_set1_pd(1.0 / sqrt(((q_data[0] * q_data[0] + q_data[1] * q_data
      [1]) + q_data[2] * q_data[2]) + q_data[3] * q_data[3]));
    _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(_mm_loadu_pd(&q_data[0]), tmp_3));
    cth = tmp_2[0];
    b_q_idx_1 = tmp_2[1];
    _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(_mm_loadu_pd(&q_data[2]), tmp_3));
    cth_tmp = b_q_idx_1 * tmp_2[0];
    cth_tmp_0 = cth * tmp_2[1];
    tempR_tmp = tmp_2[1] * tmp_2[1];
    tempR_tmp_0 = b_q_idx_1 * tmp_2[1];
    tempR_tmp_1 = cth * tmp_2[0];
    theta = tmp_2[0] * tmp_2[1];

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    cth *= b_q_idx_1;
    b_q_idx_1 *= b_q_idx_1;
    tempR_tmp_2 = tmp_2[0] * tmp_2[0];
    ID_cat(1.0 - (tempR_tmp_2 + tempR_tmp) * 2.0, (cth_tmp - cth_tmp_0) * 2.0,
           (tempR_tmp_0 + tempR_tmp_1) * 2.0, (cth_tmp + cth_tmp_0) * 2.0, 1.0 -
           (b_q_idx_1 + tempR_tmp) * 2.0, (theta - cth) * 2.0, (tempR_tmp_0 -
            tempR_tmp_1) * 2.0, (theta + cth) * 2.0, 1.0 - (b_q_idx_1 +
            tempR_tmp_2) * 2.0, tempR);
    for (i = 0; i < 3; i++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      R[i] = tempR[i * 3];
      R[i + 3] = tempR[i * 3 + 1];
      R[i + 6] = tempR[i * 3 + 2];
    }

    memset(&b_0[0], 0, sizeof(real_T) << 4U);
    for (i = 0; i < 3; i++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      result_data_tmp = i << 2;
      b_0[result_data_tmp] = R[3 * i];
      b_0[result_data_tmp + 1] = R[3 * i + 1];
      b_0[result_data_tmp + 2] = R[3 * i + 2];
    }

    b_0[15] = 1.0;
    for (i = 0; i < 4; i++) {
      result_data_tmp = i << 2;
      cth_tmp = b_0[result_data_tmp + 1];
      cth_tmp_0 = b_0[result_data_tmp];
      tempR_tmp = b_0[result_data_tmp + 2];
      tempR_tmp_0 = b_0[result_data_tmp + 3];
      for (b_kstr = 0; b_kstr <= 2; b_kstr += 2) {
        tmp_3 = _mm_loadu_pd(&b_I[b_kstr + 4]);
        tmp = _mm_loadu_pd(&b_I[b_kstr]);
        tmp_0 = _mm_loadu_pd(&b_I[b_kstr + 8]);
        tmp_1 = _mm_loadu_pd(&b_I[b_kstr + 12]);
        _mm_storeu_pd(&b[b_kstr + result_data_tmp], _mm_add_pd(_mm_add_pd
          (_mm_add_pd(_mm_mul_pd(_mm_set1_pd(cth_tmp), tmp_3), _mm_mul_pd
                      (_mm_set1_pd(cth_tmp_0), tmp)), _mm_mul_pd(_mm_set1_pd
          (tempR_tmp), tmp_0)), _mm_mul_pd(_mm_set1_pd(tempR_tmp_0), tmp_1)));
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

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  for (i = 0; i < 4; i++) {
    cth_tmp = obj->JointToParentTransform[i + 4];
    cth_tmp_0 = obj->JointToParentTransform[i];
    tempR_tmp = obj->JointToParentTransform[i + 8];
    tempR_tmp_0 = obj->JointToParentTransform[i + 12];
    for (b_kstr = 0; b_kstr <= 2; b_kstr += 2) {
      result_data_tmp = (b_kstr + 1) << 2;
      tmp_4 = b_kstr << 2;
      _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set_pd(b[result_data_tmp + 1], b[tmp_4 + 1]), _mm_set1_pd(cth_tmp)),
        _mm_mul_pd(_mm_set_pd(b[result_data_tmp], b[tmp_4]), _mm_set1_pd
                   (cth_tmp_0))), _mm_mul_pd(_mm_set_pd(b[result_data_tmp + 2],
        b[tmp_4 + 2]), _mm_set1_pd(tempR_tmp))), _mm_mul_pd(_mm_set_pd
        (b[result_data_tmp + 3], b[tmp_4 + 3]), _mm_set1_pd(tempR_tmp_0))));
      b_I[i + tmp_4] = tmp_2[0];
      b_I[i + result_data_tmp] = tmp_2[1];
    }

    cth_tmp = b_I[i + 4];
    cth_tmp_0 = b_I[i];
    tempR_tmp = b_I[i + 8];
    tempR_tmp_0 = b_I[i + 12];
    for (b_kstr = 0; b_kstr < 4; b_kstr++) {
      result_data_tmp = b_kstr << 2;
      T[i + result_data_tmp] = ((obj->ChildToJointTransform[result_data_tmp + 1]
        * cth_tmp + obj->ChildToJointTransform[result_data_tmp] * cth_tmp_0) +
        obj->ChildToJointTransform[result_data_tmp + 2] * tempR_tmp) +
        obj->ChildToJointTransform[result_data_tmp + 3] * tempR_tmp_0;
    }
  }
}

static void rigidBodyJoint_transformBodyToP(const b_rigidBodyJoint_ID_T *obj,
  real_T T[16])
{
  real_T b[16];
  real_T obj_0[16];
  real_T R[9];
  real_T tempR[9];
  real_T v[3];
  real_T tmp[2];
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  real_T b_index;
  int32_T T_tmp;
  int32_T b_kstr;
  int32_T i;
  int32_T tmp_0;
  boolean_T result;
  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_3[8] = { 'f', 'l', 'o', 'a', 't', 'i', 'n', 'g' };

  int32_T exitg1;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  if (obj->TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = (int32_T)obj->TypeInternal.Length;
  }

  result = false;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  if (i == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (tmp_1[b_kstr - 1] != obj->TypeInternal.Vector[b_kstr - 1]) {
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
    b_index = 0.0;
  } else {
    if (i == 9) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (tmp_2[b_kstr - 1] != obj->TypeInternal.Vector[b_kstr - 1]) {
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
      b_index = 1.0;
    } else {
      if (i == 8) {
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
         */
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 8) {
            if (tmp_3[b_kstr - 1] != obj->TypeInternal.Vector[b_kstr - 1]) {
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
        b_index = 2.0;
      } else {
        b_index = -1.0;
      }
    }
  }

  switch ((int32_T)b_index) {
   case 0:
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    ID_rigidBodyJoint_get_JointAxis(obj, v);
    axang_idx_0 = v[0];
    axang_idx_1 = v[1];
    axang_idx_2 = v[2];

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    b_index = 1.0 / sqrt((axang_idx_0 * axang_idx_0 + axang_idx_1 * axang_idx_1)
                         + axang_idx_2 * axang_idx_2);
    v[0] = axang_idx_0 * b_index;
    v[1] = axang_idx_1 * b_index;
    v[2] = axang_idx_2 * b_index;
    b_index = v[0] * v[1] * 0.0;
    axang_idx_0 = v[0] * v[2] * 0.0;
    axang_idx_1 = v[1] * v[2] * 0.0;
    ID_cat(v[0] * v[0] * 0.0 + 1.0, b_index - v[2] * 0.0, axang_idx_0 + v[1] *
           0.0, b_index + v[2] * 0.0, v[1] * v[1] * 0.0 + 1.0, axang_idx_1 - v[0]
           * 0.0, axang_idx_0 - v[1] * 0.0, axang_idx_1 + v[0] * 0.0, v[2] * v[2]
           * 0.0 + 1.0, tempR);
    for (i = 0; i < 3; i++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      R[i] = tempR[i * 3];
      R[i + 3] = tempR[i * 3 + 1];
      R[i + 6] = tempR[i * 3 + 2];
    }

    memset(&b[0], 0, sizeof(real_T) << 4U);
    for (i = 0; i < 3; i++) {
      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      b_kstr = i << 2;
      b[b_kstr] = R[3 * i];
      b[b_kstr + 1] = R[3 * i + 1];
      b[b_kstr + 2] = R[3 * i + 2];
    }

    b[15] = 1.0;
    break;

   case 1:
    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    ID_rigidBodyJoint_get_JointAxis(obj, v);
    memset(&tempR[0], 0, 9U * sizeof(real_T));
    tempR[0] = 1.0;
    tempR[4] = 1.0;
    tempR[8] = 1.0;
    for (i = 0; i < 3; i++) {
      b_kstr = i << 2;

      /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
       *  MATLABSystem: '<S4>/MATLAB System'
       *  MATLABSystem: '<S5>/MATLAB System'
       */
      b[b_kstr] = tempR[3 * i];
      b[b_kstr + 1] = tempR[3 * i + 1];
      b[b_kstr + 2] = tempR[3 * i + 2];
      b[i + 12] = v[i] * 0.0;
    }

    b[3] = 0.0;
    b[7] = 0.0;
    b[11] = 0.0;
    b[15] = 1.0;
    break;

   case 2:
    /* Check node always fails. would cause program termination and was eliminated */
    break;

   default:
    memset(&b[0], 0, sizeof(real_T) << 4U);
    b[0] = 1.0;
    b[5] = 1.0;
    b[10] = 1.0;
    b[15] = 1.0;
    break;
  }

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  for (i = 0; i < 4; i++) {
    b_index = obj->JointToParentTransform[i + 4];
    axang_idx_0 = obj->JointToParentTransform[i];
    axang_idx_1 = obj->JointToParentTransform[i + 8];
    axang_idx_2 = obj->JointToParentTransform[i + 12];
    for (b_kstr = 0; b_kstr <= 2; b_kstr += 2) {
      T_tmp = (b_kstr + 1) << 2;
      tmp_0 = b_kstr << 2;
      _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set_pd(b[T_tmp + 1], b[tmp_0 + 1]), _mm_set1_pd(b_index)),
        _mm_mul_pd(_mm_set_pd(b[T_tmp], b[tmp_0]), _mm_set1_pd(axang_idx_0))),
        _mm_mul_pd(_mm_set_pd(b[T_tmp + 2], b[tmp_0 + 2]), _mm_set1_pd
                   (axang_idx_1))), _mm_mul_pd(_mm_set_pd(b[T_tmp + 3], b[tmp_0
        + 3]), _mm_set1_pd(axang_idx_2))));
      obj_0[i + tmp_0] = tmp[0];
      obj_0[i + T_tmp] = tmp[1];
    }

    b_index = obj_0[i + 4];
    axang_idx_0 = obj_0[i];
    axang_idx_1 = obj_0[i + 8];
    axang_idx_2 = obj_0[i + 12];
    for (b_kstr = 0; b_kstr < 4; b_kstr++) {
      T_tmp = b_kstr << 2;
      T[i + T_tmp] = ((obj->ChildToJointTransform[T_tmp + 1] * b_index +
                       obj->ChildToJointTransform[T_tmp] * axang_idx_0) +
                      obj->ChildToJointTransform[T_tmp + 2] * axang_idx_1) +
        obj->ChildToJointTransform[T_tmp + 3] * axang_idx_2;
    }
  }
}

static void ID_mtimes(const real_T A[36], const real_T B_data[], const int32_T
                      B_size[2], real_T C_data[], int32_T C_size[2])
{
  int32_T b;
  int32_T b_i;
  int32_T b_j;
  int32_T b_k;
  C_size[0] = 6;

  /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  /* Start for MATLABSystem: '<S3>/MATLAB System' */
  b = B_size[1];
  C_size[1] = B_size[1];
  for (b_j = 0; b_j < b; b_j++) {
    int32_T coffset_tmp;

    /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
     *  MATLABSystem: '<S4>/MATLAB System'
     *  MATLABSystem: '<S5>/MATLAB System'
     */
    coffset_tmp = b_j * 6 - 1;
    for (b_i = 0; b_i < 6; b_i++) {
      real_T s;
      s = 0.0;
      for (b_k = 0; b_k < 6; b_k++) {
        /* Start for MATLABSystem: '<S3>/MATLAB System' incorporates:
         *  MATLABSystem: '<S4>/MATLAB System'
         *  MATLABSystem: '<S5>/MATLAB System'
         */
        s += A[b_k * 6 + b_i] * B_data[(coffset_tmp + b_k) + 1];
      }

      C_data[(coffset_tmp + b_i) + 1] = s;
    }
  }
}

static void ID_mtimes_hyn(const real_T A_data[], const int32_T A_size[2], const
  real_T B_data[], const int32_T B_size[2], real_T C_data[], int32_T C_size[2])
{
  int32_T b;
  int32_T b_i;
  int32_T b_j;
  int32_T b_k;
  int32_T m_tmp;

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  m_tmp = A_size[1];
  C_size[0] = A_size[1];

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  b = B_size[1];
  C_size[1] = B_size[1];
  for (b_j = 0; b_j < b; b_j++) {
    int32_T boffset;
    int32_T coffset;
    coffset = b_j * m_tmp - 1;

    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    boffset = b_j * 6 - 1;
    for (b_i = 0; b_i < m_tmp; b_i++) {
      real_T s;
      int32_T aoffset;

      /* Start for MATLABSystem: '<S4>/MATLAB System' */
      aoffset = b_i * 6 - 1;
      s = 0.0;
      for (b_k = 0; b_k < 6; b_k++) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' */
        s += A_data[(b_k + aoffset) + 1] * B_data[(b_k + boffset) + 1];
      }

      C_data[(coffset + b_i) + 1] = s;
    }
  }
}

static void ID_mtimes_hynm(const real_T A[36], const real_T B_data[], const
  int32_T B_size[2], real_T C_data[], int32_T C_size[2])
{
  int32_T b;
  int32_T b_i;
  int32_T b_j;
  int32_T b_k;
  C_size[0] = 6;

  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  /* Start for MATLABSystem: '<S4>/MATLAB System' */
  b = B_size[1];
  C_size[1] = B_size[1];
  for (b_j = 0; b_j < b; b_j++) {
    int32_T coffset_tmp;

    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    coffset_tmp = b_j * 6 - 1;
    for (b_i = 0; b_i < 6; b_i++) {
      real_T s;
      int32_T aoffset;

      /* Start for MATLABSystem: '<S4>/MATLAB System' */
      aoffset = b_i * 6 - 1;
      s = 0.0;
      for (b_k = 0; b_k < 6; b_k++) {
        /* Start for MATLABSystem: '<S4>/MATLAB System' */
        s += A[(b_k + aoffset) + 1] * B_data[(b_k + coffset_tmp) + 1];
      }

      C_data[(coffset_tmp + b_i) + 1] = s;
    }
  }
}

static void ID_emxFree_real_T(emxArray_real_T_ID_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_ID_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_ID_T *)NULL;
  }
}

static void ID_mtimes_hynmtam(const real_T A_data[], const int32_T A_size[2],
  const real_T B[6], real_T C_data[], int32_T *C_size)
{
  int32_T b;
  int32_T b_i;
  int32_T b_k;

  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  /* Start for MATLABSystem: '<S5>/MATLAB System' */
  b = A_size[1];
  *C_size = A_size[1];
  for (b_i = 0; b_i < b; b_i++) {
    real_T s;
    int32_T aoffset;

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    aoffset = b_i * 6 - 1;
    s = 0.0;
    for (b_k = 0; b_k < 6; b_k++) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      s += A_data[(aoffset + b_k) + 1] * B[b_k];
    }

    C_data[b_i] = s;
  }
}

static void RigidBodyTreeDynamics_inverseDy(f_robotics_manip_internal_Rig_T
  *robot, const real_T q[12], real_T tau[12])
{
  __m128d tmp_1;
  __m128d tmp_2;
  e_robotics_manip_internal_Rig_T *obj;
  real_T aB_data[102];
  real_T f_data[102];
  real_T vB_data[102];
  real_T vJ_data[102];
  real_T S_data[36];
  real_T XDHOffset[36];
  real_T y_data[36];
  real_T T[16];
  real_T TDHOffset[16];
  real_T q_data[12];
  real_T R[9];
  real_T R_0[9];
  real_T tmp[9];
  real_T R_1[6];
  real_T X[6];
  real_T a0[6];
  real_T y[6];
  real_T tmp_3[2];
  real_T TDHOffset_0;
  real_T T_0;
  real_T a_idx_0;
  real_T a_idx_1;
  real_T nb;
  real_T tmp_0;
  int32_T Tinv_tmp;
  int32_T aoffset;
  int32_T b;
  int32_T b_k;
  int32_T i;
  int32_T k;
  char_T obj_Vector[200];
  boolean_T b_bool;
  static const char_T tmp_4[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T S_size[2];
  int32_T y_size[2];
  int32_T c_tmp;
  int32_T exitg1;
  a0[0] = 0.0;
  a0[1] = 0.0;
  a0[2] = 0.0;
  a0[3] = -robot->Gravity[0];
  a0[4] = -robot->Gravity[1];
  a0[5] = -robot->Gravity[2];
  nb = robot->NumBodies;
  c_tmp = (int32_T)nb;
  i = 6 * (int32_T)nb;
  if (i - 1 >= 0) {
    memset(&vJ_data[0], 0, (uint32_T)i * sizeof(real_T));
  }

  if (i - 1 >= 0) {
    memset(&vB_data[0], 0, (uint32_T)i * sizeof(real_T));
  }

  if (i - 1 >= 0) {
    memset(&aB_data[0], 0, (uint32_T)i * sizeof(real_T));
  }

  memset(&tau[0], 0, 12U * sizeof(real_T));
  for (b_k = 0; b_k < c_tmp; b_k++) {
    memset(&XDHOffset[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      XDHOffset[i + 6 * i] = 1.0;
    }

    for (b = 0; b < 36; b++) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      ID_B.Xtree_data_k[b_k].f1[b] = XDHOffset[b];
      XDHOffset[b] = 0.0;
    }

    for (i = 0; i < 6; i++) {
      XDHOffset[i + 6 * i] = 1.0;
    }

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    memcpy(&ID_B.X_data_c[b_k].f1[0], &XDHOffset[0], 36U * sizeof(real_T));
  }

  if ((int32_T)nb - 1 >= 0) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    tmp[0] = 0.0;
    tmp[4] = 0.0;
    tmp[8] = 0.0;
  }

  for (i = 0; i < c_tmp; i++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    obj = robot->Bodies[i];
    rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, S_data, S_size);
    a_idx_0 = robot->PositionDoFMap[i];
    a_idx_1 = robot->PositionDoFMap[i + 17];
    memset(&XDHOffset[0], 0, 36U * sizeof(real_T));
    for (b_k = 0; b_k < 6; b_k++) {
      XDHOffset[b_k + 6 * b_k] = 1.0;
    }

    if (a_idx_1 < a_idx_0) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      obj = robot->Bodies[i];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal, T);
      for (b = 0; b < 6; b++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        vJ_data[b + 6 * i] = 0.0;
      }
    } else {
      if (a_idx_0 > a_idx_1) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        b_k = 0;
        k = 0;
      } else {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        b_k = (int32_T)a_idx_0 - 1;
        k = (int32_T)a_idx_1;
      }

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      obj = robot->Bodies[i];
      k -= b_k;
      for (b = 0; b < k; b++) {
        q_data[b] = q[b_k + b];
      }

      rigidBodyJoint_transformBodyT_h(&obj->JointInternal, q_data, &k, T);
      obj = robot->Bodies[i];
      for (b = 0; b < 16; b++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        TDHOffset[b] = obj->JointInternal.ChildToJointTransform[b];
      }

      for (b = 0; b < 3; b++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        R[3 * b] = TDHOffset[b];
        R[3 * b + 1] = TDHOffset[b + 4];
        R[3 * b + 2] = TDHOffset[b + 8];
      }

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      for (b = 0; b <= 6; b += 2) {
        tmp_2 = _mm_loadu_pd(&R[b]);
        _mm_storeu_pd(&R_0[b], _mm_mul_pd(tmp_2, _mm_set1_pd(-1.0)));
      }

      for (b = 8; b < 9; b++) {
        R_0[b] = -R[b];
      }

      a_idx_0 = TDHOffset[13];
      a_idx_1 = TDHOffset[12];
      TDHOffset_0 = TDHOffset[14];
      for (b = 0; b < 3; b++) {
        aoffset = b << 2;
        TDHOffset[aoffset] = R[3 * b];
        TDHOffset[aoffset + 1] = R[3 * b + 1];
        TDHOffset[aoffset + 2] = R[3 * b + 2];
        TDHOffset[b + 12] = (R_0[b + 3] * a_idx_0 + R_0[b] * a_idx_1) + R_0[b +
          6] * TDHOffset_0;
      }

      TDHOffset[3] = 0.0;
      TDHOffset[7] = 0.0;
      TDHOffset[11] = 0.0;
      TDHOffset[15] = 1.0;

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      R[0] = 0.0;
      R[3] = -TDHOffset[14];
      R[6] = TDHOffset[13];
      R[1] = TDHOffset[14];
      R[4] = 0.0;
      R[7] = -TDHOffset[12];
      R[2] = -TDHOffset[13];
      R[5] = TDHOffset[12];
      R[8] = 0.0;
      for (b = 0; b < 3; b++) {
        a_idx_1 = R[b + 3];
        TDHOffset_0 = R[b];
        tmp_0 = R[b + 6];
        for (aoffset = 0; aoffset < 3; aoffset++) {
          Tinv_tmp = aoffset << 2;
          R_0[b + 3 * aoffset] = (TDHOffset[Tinv_tmp + 1] * a_idx_1 +
            TDHOffset[Tinv_tmp] * TDHOffset_0) + TDHOffset[Tinv_tmp + 2] * tmp_0;
          XDHOffset[aoffset + 6 * b] = TDHOffset[(b << 2) + aoffset];
          XDHOffset[aoffset + 6 * (b + 3)] = 0.0;
        }
      }

      for (b = 0; b < 3; b++) {
        XDHOffset[6 * b + 3] = R_0[3 * b];

        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        b_k = b << 2;
        k = (b + 3) * 6;
        XDHOffset[k + 3] = TDHOffset[b_k];
        XDHOffset[6 * b + 4] = R_0[3 * b + 1];

        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        XDHOffset[k + 4] = TDHOffset[b_k + 1];
        XDHOffset[6 * b + 5] = R_0[3 * b + 2];

        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        XDHOffset[k + 5] = TDHOffset[b_k + 2];
      }

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      ID_mtimes(XDHOffset, S_data, S_size, y_data, y_size);
      for (b_k = 0; b_k < 6; b_k++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        y[b_k] = 0.0;
      }

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      b = y_size[1];
      for (b_k = 0; b_k < b; b_k++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        aoffset = b_k * 6 - 1;
        for (k = 0; k <= 4; k += 2) {
          /* Start for MATLABSystem: '<S5>/MATLAB System' */
          tmp_2 = _mm_loadu_pd(&y_data[(aoffset + k) + 1]);
          tmp_1 = _mm_loadu_pd(&y[k]);
          _mm_storeu_pd(&y[k], _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set1_pd(0.0)),
            tmp_1));
        }
      }

      for (b = 0; b < 6; b++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        vJ_data[b + 6 * i] = y[b];
      }
    }

    for (b = 0; b < 3; b++) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      R[3 * b] = T[b];
      R[3 * b + 1] = T[b + 4];
      R[3 * b + 2] = T[b + 8];
    }

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    for (b = 0; b <= 6; b += 2) {
      tmp_2 = _mm_loadu_pd(&R[b]);
      _mm_storeu_pd(&R_0[b], _mm_mul_pd(tmp_2, _mm_set1_pd(-1.0)));
    }

    for (b = 8; b < 9; b++) {
      R_0[b] = -R[b];
    }

    T_0 = T[13];
    a_idx_0 = T[12];
    a_idx_1 = T[14];
    for (b = 0; b < 3; b++) {
      aoffset = b << 2;
      TDHOffset[aoffset] = R[3 * b];
      TDHOffset[aoffset + 1] = R[3 * b + 1];
      TDHOffset[aoffset + 2] = R[3 * b + 2];
      TDHOffset[b + 12] = (R_0[b + 3] * T_0 + R_0[b] * a_idx_0) + R_0[b + 6] *
        a_idx_1;
    }

    TDHOffset[3] = 0.0;
    TDHOffset[7] = 0.0;
    TDHOffset[11] = 0.0;
    TDHOffset[15] = 1.0;

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    tmp[3] = -TDHOffset[14];
    tmp[6] = TDHOffset[13];
    tmp[1] = TDHOffset[14];
    tmp[7] = -TDHOffset[12];
    tmp[2] = -TDHOffset[13];
    tmp[5] = TDHOffset[12];
    for (b = 0; b < 3; b++) {
      a_idx_1 = tmp[b + 3];
      TDHOffset_0 = tmp[b];
      tmp_0 = tmp[b + 6];
      for (aoffset = 0; aoffset < 3; aoffset++) {
        Tinv_tmp = aoffset << 2;
        R[b + 3 * aoffset] = (TDHOffset[Tinv_tmp + 1] * a_idx_1 +
                              TDHOffset[Tinv_tmp] * TDHOffset_0) +
          TDHOffset[Tinv_tmp + 2] * tmp_0;
        ID_B.X_data_c[i].f1[aoffset + 6 * b] = TDHOffset[(b << 2) + aoffset];
        ID_B.X_data_c[i].f1[aoffset + 6 * (b + 3)] = 0.0;
      }
    }

    for (b = 0; b < 3; b++) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      ID_B.X_data_c[i].f1[6 * b + 3] = R[3 * b];
      aoffset = b << 2;
      Tinv_tmp = (b + 3) * 6;
      ID_B.X_data_c[i].f1[Tinv_tmp + 3] = TDHOffset[aoffset];
      ID_B.X_data_c[i].f1[6 * b + 4] = R[3 * b + 1];
      ID_B.X_data_c[i].f1[Tinv_tmp + 4] = TDHOffset[aoffset + 1];
      ID_B.X_data_c[i].f1[6 * b + 5] = R[3 * b + 2];
      ID_B.X_data_c[i].f1[Tinv_tmp + 5] = TDHOffset[aoffset + 2];
    }

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    a_idx_0 = robot->Bodies[i]->ParentIndex;
    if (a_idx_0 > 0.0) {
      for (b = 0; b < 6; b++) {
        a_idx_1 = 0.0;
        for (aoffset = 0; aoffset < 6; aoffset++) {
          /* Start for MATLABSystem: '<S5>/MATLAB System' */
          a_idx_1 += vB_data[((int32_T)a_idx_0 - 1) * 6 + aoffset] *
            ID_B.X_data_c[i].f1[6 * aoffset + b];
        }

        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        y[b] = vJ_data[6 * i + b] + a_idx_1;
      }

      for (b = 0; b < 6; b++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        vB_data[b + 6 * i] = y[b];
      }

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      ID_mtimes(XDHOffset, S_data, S_size, y_data, y_size);
      for (b_k = 0; b_k < 6; b_k++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        y[b_k] = 0.0;
      }

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      b = y_size[1];
      for (b_k = 0; b_k < b; b_k++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        aoffset = b_k * 6 - 1;
        for (k = 0; k <= 4; k += 2) {
          /* Start for MATLABSystem: '<S5>/MATLAB System' */
          tmp_2 = _mm_loadu_pd(&y_data[(aoffset + k) + 1]);
          tmp_1 = _mm_loadu_pd(&y[k]);
          _mm_storeu_pd(&y[k], _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set1_pd(0.0)),
            tmp_1));
        }
      }

      R[0] = 0.0;

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      T_0 = vB_data[6 * i + 2];
      R[3] = -T_0;
      a_idx_1 = vB_data[6 * i + 1];
      R[6] = a_idx_1;
      R[1] = T_0;
      R[4] = 0.0;

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      T_0 = vB_data[6 * i];
      R[7] = -T_0;
      R[2] = -a_idx_1;
      R[5] = T_0;
      R[8] = 0.0;

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      S_data[3] = 0.0;
      T_0 = vB_data[6 * i + 5];
      S_data[9] = -T_0;
      a_idx_1 = vB_data[6 * i + 4];
      S_data[15] = a_idx_1;
      S_data[4] = T_0;
      S_data[10] = 0.0;
      T_0 = vB_data[6 * i + 3];
      S_data[16] = -T_0;
      S_data[5] = -a_idx_1;
      S_data[11] = T_0;
      S_data[17] = 0.0;
      for (b = 0; b < 3; b++) {
        T_0 = R[3 * b];
        S_data[6 * b] = T_0;
        b_k = (b + 3) * 6;
        S_data[b_k] = 0.0;
        S_data[b_k + 3] = T_0;
        T_0 = R[3 * b + 1];
        S_data[6 * b + 1] = T_0;
        S_data[b_k + 1] = 0.0;
        S_data[b_k + 4] = T_0;
        T_0 = R[3 * b + 2];
        S_data[6 * b + 2] = T_0;
        S_data[b_k + 2] = 0.0;
        S_data[b_k + 5] = T_0;
      }

      for (b = 0; b < 6; b++) {
        a_idx_1 = 0.0;
        T_0 = 0.0;
        for (aoffset = 0; aoffset < 6; aoffset++) {
          Tinv_tmp = 6 * aoffset + b;
          _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
            (S_data[Tinv_tmp], ID_B.X_data_c[i].f1[Tinv_tmp]), _mm_set_pd
            (vJ_data[aoffset + 6 * i], aB_data[aoffset + 6 * ((int32_T)a_idx_0 -
            1)])), _mm_set_pd(T_0, a_idx_1)));
          a_idx_1 = tmp_3[0];
          T_0 = tmp_3[1];
        }

        R_1[b] = T_0;
        X[b] = a_idx_1 + y[b];
      }

      for (b = 0; b <= 4; b += 2) {
        tmp_2 = _mm_loadu_pd(&X[b]);
        tmp_1 = _mm_loadu_pd(&R_1[b]);

        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        _mm_storeu_pd(&aB_data[b + 6 * i], _mm_add_pd(tmp_2, tmp_1));
      }

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      R[0] = 0.0;
      R[3] = -T[14];
      R[6] = T[13];
      R[1] = T[14];
      R[4] = 0.0;
      R[7] = -T[12];
      R[2] = -T[13];
      R[5] = T[12];
      R[8] = 0.0;
      for (b = 0; b < 3; b++) {
        a_idx_1 = R[b + 3];
        TDHOffset_0 = R[b];
        tmp_0 = R[b + 6];
        for (aoffset = 0; aoffset < 3; aoffset++) {
          Tinv_tmp = aoffset << 2;
          R_0[b + 3 * aoffset] = (T[Tinv_tmp + 1] * a_idx_1 + T[Tinv_tmp] *
            TDHOffset_0) + T[Tinv_tmp + 2] * tmp_0;
          XDHOffset[aoffset + 6 * b] = T[(b << 2) + aoffset];
          XDHOffset[aoffset + 6 * (b + 3)] = 0.0;
        }
      }

      for (b = 0; b < 3; b++) {
        XDHOffset[6 * b + 3] = R_0[3 * b];
        b_k = b << 2;
        k = (b + 3) * 6;
        XDHOffset[k + 3] = T[b_k];
        XDHOffset[6 * b + 4] = R_0[3 * b + 1];
        XDHOffset[k + 4] = T[b_k + 1];
        XDHOffset[6 * b + 5] = R_0[3 * b + 2];
        XDHOffset[k + 5] = T[b_k + 2];
      }

      for (b = 0; b < 6; b++) {
        for (aoffset = 0; aoffset < 6; aoffset++) {
          T_0 = 0.0;
          for (Tinv_tmp = 0; Tinv_tmp < 6; Tinv_tmp++) {
            T_0 += ID_B.Xtree_data_k[(int32_T)a_idx_0 - 1].f1[6 * Tinv_tmp + b] *
              XDHOffset[6 * aoffset + Tinv_tmp];
          }

          S_data[b + 6 * aoffset] = T_0;
        }
      }

      memcpy(&ID_B.Xtree_data_k[i].f1[0], &S_data[0], 36U * sizeof(real_T));
    } else {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      ID_mtimes(XDHOffset, S_data, S_size, y_data, y_size);
      for (b_k = 0; b_k < 6; b_k++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        b = 6 * i + b_k;
        vB_data[b] = vJ_data[b];

        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        y[b_k] = 0.0;
      }

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      b = y_size[1];
      for (b_k = 0; b_k < b; b_k++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        aoffset = b_k * 6 - 1;
        for (k = 0; k <= 4; k += 2) {
          /* Start for MATLABSystem: '<S5>/MATLAB System' */
          tmp_2 = _mm_loadu_pd(&y_data[(aoffset + k) + 1]);
          tmp_1 = _mm_loadu_pd(&y[k]);
          _mm_storeu_pd(&y[k], _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set1_pd(0.0)),
            tmp_1));
        }
      }

      for (b = 0; b < 6; b++) {
        a_idx_1 = 0.0;
        for (aoffset = 0; aoffset < 6; aoffset++) {
          /* Start for MATLABSystem: '<S5>/MATLAB System' */
          a_idx_1 += ID_B.X_data_c[i].f1[6 * aoffset + b] * a0[aoffset];
        }

        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        aB_data[b + 6 * i] = a_idx_1 + y[b];
      }

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      R[0] = 0.0;
      R[3] = -T[14];
      R[6] = T[13];
      R[1] = T[14];
      R[4] = 0.0;
      R[7] = -T[12];
      R[2] = -T[13];
      R[5] = T[12];
      R[8] = 0.0;
      for (b = 0; b < 3; b++) {
        a_idx_1 = R[b + 3];
        TDHOffset_0 = R[b];
        tmp_0 = R[b + 6];
        for (aoffset = 0; aoffset < 3; aoffset++) {
          Tinv_tmp = aoffset << 2;
          R_0[b + 3 * aoffset] = (T[Tinv_tmp + 1] * a_idx_1 + T[Tinv_tmp] *
            TDHOffset_0) + T[Tinv_tmp + 2] * tmp_0;
          ID_B.Xtree_data_k[i].f1[aoffset + 6 * b] = T[(b << 2) + aoffset];
          ID_B.Xtree_data_k[i].f1[aoffset + 6 * (b + 3)] = 0.0;
        }
      }

      for (b = 0; b < 3; b++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        ID_B.Xtree_data_k[i].f1[6 * b + 3] = R_0[3 * b];
        aoffset = b << 2;
        Tinv_tmp = (b + 3) * 6;
        ID_B.Xtree_data_k[i].f1[Tinv_tmp + 3] = T[aoffset];
        ID_B.Xtree_data_k[i].f1[6 * b + 4] = R_0[3 * b + 1];
        ID_B.Xtree_data_k[i].f1[Tinv_tmp + 4] = T[aoffset + 1];
        ID_B.Xtree_data_k[i].f1[6 * b + 5] = R_0[3 * b + 2];
        ID_B.Xtree_data_k[i].f1[Tinv_tmp + 5] = T[aoffset + 2];
      }
    }

    for (b = 0; b < 36; b++) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      XDHOffset[b] = robot->Bodies[i]->SpatialInertia[b];
    }

    R[0] = 0.0;

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    T_0 = vB_data[6 * i + 2];
    R[3] = -T_0;
    a_idx_1 = vB_data[6 * i + 1];
    R[6] = a_idx_1;
    R[1] = T_0;
    R[4] = 0.0;

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    T_0 = vB_data[6 * i];
    R[7] = -T_0;
    R[2] = -a_idx_1;
    R[5] = T_0;
    R[8] = 0.0;

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    S_data[18] = 0.0;
    T_0 = vB_data[6 * i + 5];
    S_data[24] = -T_0;
    a_idx_1 = vB_data[6 * i + 4];
    S_data[30] = a_idx_1;
    S_data[19] = T_0;
    S_data[25] = 0.0;
    T_0 = vB_data[6 * i + 3];
    S_data[31] = -T_0;
    S_data[20] = -a_idx_1;
    S_data[26] = T_0;
    S_data[32] = 0.0;
    for (b = 0; b < 3; b++) {
      T_0 = R[3 * b];
      S_data[6 * b] = T_0;
      S_data[6 * b + 3] = 0.0;
      b_k = (b + 3) * 6;
      S_data[b_k + 3] = T_0;
      T_0 = R[3 * b + 1];
      S_data[6 * b + 1] = T_0;
      S_data[6 * b + 4] = 0.0;
      S_data[b_k + 4] = T_0;
      T_0 = R[3 * b + 2];
      S_data[6 * b + 2] = T_0;
      S_data[6 * b + 5] = 0.0;
      S_data[b_k + 5] = T_0;
    }

    for (b = 0; b < 6; b++) {
      T_0 = 0.0;
      a_idx_0 = 0.0;
      for (aoffset = 0; aoffset < 6; aoffset++) {
        b_k = 6 * i + aoffset;
        _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(XDHOffset[6 *
          aoffset + b]), _mm_set_pd(aB_data[b_k], vB_data[b_k])), _mm_set_pd
          (a_idx_0, T_0)));
        T_0 = tmp_3[0];
        a_idx_0 = tmp_3[1];
      }

      X[b] = a_idx_0;
      y[b] = T_0;
    }

    for (b = 0; b < 6; b++) {
      a_idx_1 = 0.0;
      T_0 = 0.0;
      for (aoffset = 0; aoffset < 6; aoffset++) {
        _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
          (ID_B.Xtree_data_k[i].f1[6 * b + aoffset], S_data[6 * aoffset + b]),
          _mm_set_pd(0.0, y[aoffset])), _mm_set_pd(a_idx_1, T_0)));
        T_0 = tmp_3[0];
        a_idx_1 = tmp_3[1];
      }

      f_data[b + 6 * i] = (X[b] + T_0) - a_idx_1;
    }
  }

  b_k = (int32_T)-((-1.0 - nb) + 1.0);
  for (i = 0; i < b_k; i++) {
    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    T_0 = nb - (real_T)i;
    obj = robot->Bodies[(int32_T)T_0 - 1];

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    a_idx_0 = obj->JointInternal.TypeInternal.Length;
    for (b = 0; b < 200; b++) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      obj_Vector[b] = obj->JointInternal.TypeInternal.Vector[b];
    }

    b_bool = false;

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    if (a_idx_0 < 1.0) {
      b = 0;
    } else {
      b = (int32_T)a_idx_0;
    }

    if (b == 5) {
      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      k = 1;
      do {
        exitg1 = 0;
        if (k - 1 < 5) {
          if (obj_Vector[k - 1] != tmp_4[k - 1]) {
            exitg1 = 1;
          } else {
            k++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!b_bool) {
      obj = robot->Bodies[(int32_T)T_0 - 1];
      for (b = 0; b < 16; b++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        TDHOffset[b] = obj->JointInternal.ChildToJointTransform[b];
      }

      for (b = 0; b < 3; b++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        R[3 * b] = TDHOffset[b];
        R[3 * b + 1] = TDHOffset[b + 4];
        R[3 * b + 2] = TDHOffset[b + 8];
      }

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      for (b = 0; b <= 6; b += 2) {
        tmp_2 = _mm_loadu_pd(&R[b]);
        _mm_storeu_pd(&R_0[b], _mm_mul_pd(tmp_2, _mm_set1_pd(-1.0)));
      }

      for (b = 8; b < 9; b++) {
        R_0[b] = -R[b];
      }

      a_idx_0 = TDHOffset[13];
      a_idx_1 = TDHOffset[12];
      TDHOffset_0 = TDHOffset[14];
      for (b = 0; b < 3; b++) {
        aoffset = b << 2;
        TDHOffset[aoffset] = R[3 * b];
        TDHOffset[aoffset + 1] = R[3 * b + 1];
        TDHOffset[aoffset + 2] = R[3 * b + 2];
        TDHOffset[b + 12] = (R_0[b + 3] * a_idx_0 + R_0[b] * a_idx_1) + R_0[b +
          6] * TDHOffset_0;
      }

      TDHOffset[3] = 0.0;
      TDHOffset[7] = 0.0;
      TDHOffset[11] = 0.0;
      TDHOffset[15] = 1.0;
      obj = robot->Bodies[(int32_T)T_0 - 1];

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, S_data, S_size);
      a_idx_0 = robot->VelocityDoFMap[(int32_T)T_0 - 1];
      a_idx_1 = robot->VelocityDoFMap[(int32_T)T_0 + 16];

      /* Start for MATLABSystem: '<S5>/MATLAB System' */
      if (a_idx_0 > a_idx_1) {
        c_tmp = 0;
        k = 0;
      } else {
        c_tmp = (int32_T)a_idx_0 - 1;
        k = (int32_T)a_idx_1;
      }

      tmp[0] = 0.0;
      tmp[3] = -TDHOffset[14];
      tmp[6] = TDHOffset[13];
      tmp[1] = TDHOffset[14];
      tmp[4] = 0.0;
      tmp[7] = -TDHOffset[12];
      tmp[2] = -TDHOffset[13];
      tmp[5] = TDHOffset[12];
      tmp[8] = 0.0;
      for (b = 0; b < 3; b++) {
        a_idx_1 = tmp[b + 3];
        TDHOffset_0 = tmp[b];
        tmp_0 = tmp[b + 6];
        for (aoffset = 0; aoffset < 3; aoffset++) {
          Tinv_tmp = aoffset << 2;
          R[b + 3 * aoffset] = (TDHOffset[Tinv_tmp + 1] * a_idx_1 +
                                TDHOffset[Tinv_tmp] * TDHOffset_0) +
            TDHOffset[Tinv_tmp + 2] * tmp_0;
          XDHOffset[aoffset + 6 * b] = TDHOffset[(b << 2) + aoffset];
          XDHOffset[aoffset + 6 * (b + 3)] = 0.0;
        }
      }

      for (b = 0; b < 3; b++) {
        XDHOffset[6 * b + 3] = R[3 * b];
        aoffset = b << 2;
        Tinv_tmp = (b + 3) * 6;
        XDHOffset[Tinv_tmp + 3] = TDHOffset[aoffset];
        XDHOffset[6 * b + 4] = R[3 * b + 1];
        XDHOffset[Tinv_tmp + 4] = TDHOffset[aoffset + 1];
        XDHOffset[6 * b + 5] = R[3 * b + 2];
        XDHOffset[Tinv_tmp + 5] = TDHOffset[aoffset + 2];
      }

      ID_mtimes(XDHOffset, S_data, S_size, y_data, y_size);
      ID_mtimes_hynmtam(y_data, y_size, &f_data[6 * ((int32_T)T_0 - 1)], a0, &b);
      k -= c_tmp;
      for (b = 0; b < k; b++) {
        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        tau[c_tmp + b] = a0[b];
      }
    }

    a_idx_0 = robot->Bodies[(int32_T)T_0 - 1]->ParentIndex;
    if (a_idx_0 > 0.0) {
      for (b = 0; b < 6; b++) {
        a_idx_1 = 0.0;
        for (aoffset = 0; aoffset < 6; aoffset++) {
          /* Start for MATLABSystem: '<S5>/MATLAB System' */
          a_idx_1 += f_data[((int32_T)T_0 - 1) * 6 + aoffset] * ID_B.X_data_c
            [(int32_T)T_0 - 1].f1[6 * b + aoffset];
        }

        /* Start for MATLABSystem: '<S5>/MATLAB System' */
        a0[b] = f_data[((int32_T)a_idx_0 - 1) * 6 + b] + a_idx_1;
      }

      for (b = 0; b < 6; b++) {
        f_data[b + 6 * ((int32_T)a_idx_0 - 1)] = a0[b];
      }
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

static void emxFreeStruct_e_robotics_manip_(e_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_g_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxFreeMatrix_e_robotics_manip_(e_robotics_manip_internal_Rig_T
  pMatrix[34])
{
  int32_T i;
  for (i = 0; i < 34; i++) {
    emxFreeStruct_e_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_R_h_T
  *pStruct)
{
  emxFreeStruct_e_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_e_robotics_manip_(pStruct->_pobj0);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_b_h_T
  *pStruct)
{
  emxFreeStruct_f_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_f_robotics_mani_h(f_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_e_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_e_robotics_manip_(pStruct->_pobj0);
}

static void emxFreeStruct_robotics_slmani_h(robotics_slmanip_internal__hy_T
  *pStruct)
{
  emxFreeStruct_f_robotics_mani_h(&pStruct->TreeInternal);
}

static void emxFreeStruct_robotics_slman_hy(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_f_robotics_mani_h(&pStruct->TreeInternal);
}

/* Model step function */
void ID_step(void)
{
  __m128d tmp_3;
  __m128d tmp_4;
  e_robotics_manip_internal_Rig_T *obj;
  emxArray_real_T_ID_T *b;
  real_T aB_data[102];
  real_T f_data[102];
  real_T vB_data[102];
  real_T vJ_data[102];
  real_T Fi_data[36];
  real_T S_data[36];
  real_T Si_data[36];
  real_T XDHOffset[36];
  real_T T[16];
  real_T Tinv[16];
  real_T jointTorqWithoutVel[12];
  real_T rtb_TmpSignalConversionAtMATLAB[12];
  real_T tmp[12];
  real_T R[9];
  real_T R_0[9];
  real_T tmp_0[9];
  real_T tmp_1[9];
  real_T tmp_2[9];
  real_T R_1[6];
  real_T X[6];
  real_T a0[6];
  real_T y[6];
  real_T tmp_5[2];
  real_T b_idx_0;
  real_T b_idx_1;
  real_T i;
  real_T nb;
  real_T pid;
  real_T s;
  int32_T b_0;
  int32_T c;
  int32_T coffset;
  int32_T f;
  int32_T i_0;
  int32_T k;
  int32_T loop_ub;
  int32_T loop_ub_0;
  char_T obj_Vector[200];
  boolean_T b_bool;
  static const char_T tmp_6[5] = { 'f', 'i', 'x', 'e', 'd' };

  real_T tmp_7;
  int32_T Fi_size[2];
  int32_T Fi_size_0[2];
  int32_T Fi_size_1[2];
  int32_T S_size[2];
  int32_T Si_size[2];
  int32_T Hji_size_idx_0;
  int32_T b_tmp;
  int32_T c_tmp;
  int32_T exitg1;

  /* SignalConversion generated from: '<S3>/MATLAB System' incorporates:
   *  Constant: '<S2>/Constant19'
   *  Inport: '<Root>/q'
   *  Sum: '<S2>/Sum1'
   */
  rtb_TmpSignalConversionAtMATLAB[0] = ID_U.q[0] + ID_P.Constant19_Value;
  memcpy(&rtb_TmpSignalConversionAtMATLAB[1], &ID_U.q[1], 11U * sizeof(real_T));
  ID_emxInit_real_T(&b, 2);

  /* MATLABSystem: '<S4>/MATLAB System' */
  b_tmp = (int32_T)ID_DW.obj_g.TreeInternal.VelocityNumber;
  b_0 = b->size[0] * b->size[1];
  b->size[0] = b_tmp;
  b->size[1] = b_tmp;
  ID_emxEnsureCapacity_real_T(b, b_0);
  loop_ub = b_tmp * b_tmp;
  if (loop_ub - 1 >= 0) {
    memset(&b->data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  c_tmp = (int32_T)ID_DW.obj_g.TreeInternal.NumBodies;
  if (c_tmp - 1 >= 0) {
    tmp_0[0] = 0.0;
    tmp_0[4] = 0.0;
    tmp_0[8] = 0.0;
  }

  for (b_tmp = 0; b_tmp < c_tmp; b_tmp++) {
    for (i_0 = 0; i_0 < 36; i_0++) {
      ID_B.Ic_data[b_tmp].f1[i_0] = ID_DW.obj_g.TreeInternal.Bodies[b_tmp]
        ->SpatialInertia[i_0];
    }

    nb = ID_DW.obj_g.TreeInternal.PositionDoFMap[b_tmp + 17];
    if (nb < ID_DW.obj_g.TreeInternal.PositionDoFMap[b_tmp]) {
      rigidBodyJoint_transformBodyToP(&ID_DW.obj_g.TreeInternal.Bodies[b_tmp]
        ->JointInternal, T);
    } else {
      if (ID_DW.obj_g.TreeInternal.PositionDoFMap[b_tmp] > nb) {
        b_0 = 0;
        f = 0;
      } else {
        b_0 = (int32_T)ID_DW.obj_g.TreeInternal.PositionDoFMap[b_tmp] - 1;
        f = (int32_T)nb;
      }

      loop_ub = f - b_0;
      for (i_0 = 0; i_0 < loop_ub; i_0++) {
        jointTorqWithoutVel[i_0] = rtb_TmpSignalConversionAtMATLAB[b_0 + i_0];
      }

      rigidBodyJoint_transformBodyT_h(&ID_DW.obj_g.TreeInternal.Bodies[b_tmp]
        ->JointInternal, jointTorqWithoutVel, &loop_ub, T);
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      R[3 * i_0] = T[i_0];
      R[3 * i_0 + 1] = T[i_0 + 4];
      R[3 * i_0 + 2] = T[i_0 + 8];
    }

    for (i_0 = 0; i_0 <= 6; i_0 += 2) {
      tmp_4 = _mm_loadu_pd(&R[i_0]);
      _mm_storeu_pd(&R_0[i_0], _mm_mul_pd(tmp_4, _mm_set1_pd(-1.0)));
    }

    for (i_0 = 8; i_0 < 9; i_0++) {
      R_0[i_0] = -R[i_0];
    }

    s = T[13];
    b_idx_0 = T[12];
    b_idx_1 = T[14];
    for (i_0 = 0; i_0 < 3; i_0++) {
      loop_ub = i_0 << 2;
      Tinv[loop_ub] = R[3 * i_0];
      Tinv[loop_ub + 1] = R[3 * i_0 + 1];
      Tinv[loop_ub + 2] = R[3 * i_0 + 2];
      Tinv[i_0 + 12] = (R_0[i_0 + 3] * s + R_0[i_0] * b_idx_0) + R_0[i_0 + 6] *
        b_idx_1;
    }

    Tinv[3] = 0.0;
    Tinv[7] = 0.0;
    Tinv[11] = 0.0;
    Tinv[15] = 1.0;
    tmp_0[3] = -Tinv[14];
    tmp_0[6] = Tinv[13];
    tmp_0[1] = Tinv[14];
    tmp_0[7] = -Tinv[12];
    tmp_0[2] = -Tinv[13];
    tmp_0[5] = Tinv[12];
    for (i_0 = 0; i_0 < 3; i_0++) {
      s = tmp_0[i_0 + 3];
      b_idx_0 = tmp_0[i_0];
      b_idx_1 = tmp_0[i_0 + 6];
      for (coffset = 0; coffset < 3; coffset++) {
        loop_ub = coffset << 2;
        R[i_0 + 3 * coffset] = (Tinv[loop_ub + 1] * s + Tinv[loop_ub] * b_idx_0)
          + Tinv[loop_ub + 2] * b_idx_1;
        ID_B.X_data[b_tmp].f1[coffset + 6 * i_0] = Tinv[(i_0 << 2) + coffset];
        ID_B.X_data[b_tmp].f1[coffset + 6 * (i_0 + 3)] = 0.0;
      }
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      ID_B.X_data[b_tmp].f1[6 * i_0 + 3] = R[3 * i_0];
      coffset = i_0 << 2;
      loop_ub = (i_0 + 3) * 6;
      ID_B.X_data[b_tmp].f1[loop_ub + 3] = Tinv[coffset];
      ID_B.X_data[b_tmp].f1[6 * i_0 + 4] = R[3 * i_0 + 1];
      ID_B.X_data[b_tmp].f1[loop_ub + 4] = Tinv[coffset + 1];
      ID_B.X_data[b_tmp].f1[6 * i_0 + 5] = R[3 * i_0 + 2];
      ID_B.X_data[b_tmp].f1[loop_ub + 5] = Tinv[coffset + 2];
    }
  }

  f = (int32_T)-((-1.0 - ID_DW.obj_g.TreeInternal.NumBodies) + 1.0);
  for (b_0 = 0; b_0 < f; b_0++) {
    i = ID_DW.obj_g.TreeInternal.NumBodies - (real_T)b_0;
    pid = ID_DW.obj_g.TreeInternal.Bodies[(int32_T)i - 1]->ParentIndex;
    if (pid > 0.0) {
      for (i_0 = 0; i_0 < 6; i_0++) {
        for (coffset = 0; coffset < 6; coffset++) {
          nb = 0.0;
          for (loop_ub = 0; loop_ub < 6; loop_ub++) {
            nb += ID_B.X_data[(int32_T)i - 1].f1[6 * i_0 + loop_ub] *
              ID_B.Ic_data[(int32_T)i - 1].f1[6 * coffset + loop_ub];
          }

          XDHOffset[i_0 + 6 * coffset] = nb;
        }
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        for (coffset = 0; coffset < 6; coffset++) {
          s = 0.0;
          for (loop_ub = 0; loop_ub < 6; loop_ub++) {
            s += XDHOffset[6 * loop_ub + i_0] * ID_B.X_data[(int32_T)i - 1].f1[6
              * coffset + loop_ub];
          }

          loop_ub = 6 * coffset + i_0;
          ID_B.Ic_data[(int32_T)pid - 1].f1[loop_ub] += s;
        }
      }
    }

    if (ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i - 1] <=
        ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i + 16]) {
      for (i_0 = 0; i_0 < 16; i_0++) {
        T[i_0] = ID_DW.obj_g.TreeInternal.Bodies[(int32_T)i - 1]
          ->JointInternal.ChildToJointTransform[i_0];
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        R[3 * i_0] = T[i_0];
        R[3 * i_0 + 1] = T[i_0 + 4];
        R[3 * i_0 + 2] = T[i_0 + 8];
      }

      for (i_0 = 0; i_0 <= 6; i_0 += 2) {
        tmp_4 = _mm_loadu_pd(&R[i_0]);
        _mm_storeu_pd(&R_0[i_0], _mm_mul_pd(tmp_4, _mm_set1_pd(-1.0)));
      }

      for (i_0 = 8; i_0 < 9; i_0++) {
        R_0[i_0] = -R[i_0];
      }

      s = T[13];
      b_idx_0 = T[12];
      b_idx_1 = T[14];
      for (i_0 = 0; i_0 < 3; i_0++) {
        loop_ub = i_0 << 2;
        Tinv[loop_ub] = R[3 * i_0];
        Tinv[loop_ub + 1] = R[3 * i_0 + 1];
        Tinv[loop_ub + 2] = R[3 * i_0 + 2];
        Tinv[i_0 + 12] = (R_0[i_0 + 3] * s + R_0[i_0] * b_idx_0) + R_0[i_0 + 6] *
          b_idx_1;
      }

      Tinv[3] = 0.0;
      Tinv[7] = 0.0;
      Tinv[11] = 0.0;
      Tinv[15] = 1.0;
      rigidBodyJoint_get_MotionSubspa(&ID_DW.obj_g.TreeInternal.Bodies[(int32_T)
        i - 1]->JointInternal, S_data, S_size);
      tmp_0[0] = 0.0;
      tmp_0[3] = -Tinv[14];
      tmp_0[6] = Tinv[13];
      tmp_0[1] = Tinv[14];
      tmp_0[4] = 0.0;
      tmp_0[7] = -Tinv[12];
      tmp_0[2] = -Tinv[13];
      tmp_0[5] = Tinv[12];
      tmp_0[8] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        s = tmp_0[i_0 + 3];
        b_idx_0 = tmp_0[i_0];
        b_idx_1 = tmp_0[i_0 + 6];
        for (coffset = 0; coffset < 3; coffset++) {
          loop_ub = coffset << 2;
          R[i_0 + 3 * coffset] = (Tinv[loop_ub + 1] * s + Tinv[loop_ub] *
            b_idx_0) + Tinv[loop_ub + 2] * b_idx_1;
          XDHOffset[coffset + 6 * i_0] = Tinv[(i_0 << 2) + coffset];
          XDHOffset[coffset + 6 * (i_0 + 3)] = 0.0;
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        XDHOffset[6 * i_0 + 3] = R[3 * i_0];
        loop_ub = i_0 << 2;
        c = (i_0 + 3) * 6;
        XDHOffset[c + 3] = Tinv[loop_ub];
        XDHOffset[6 * i_0 + 4] = R[3 * i_0 + 1];
        XDHOffset[c + 4] = Tinv[loop_ub + 1];
        XDHOffset[6 * i_0 + 5] = R[3 * i_0 + 2];
        XDHOffset[c + 5] = Tinv[loop_ub + 2];
      }

      ID_mtimes(XDHOffset, S_data, S_size, Si_data, Si_size);
      Fi_size[0] = 6;
      c = Si_size[1];
      Fi_size[1] = Si_size[1];
      for (i_0 = 0; i_0 < c; i_0++) {
        coffset = i_0 * 6 - 1;
        for (b_tmp = 0; b_tmp < 6; b_tmp++) {
          s = 0.0;
          for (c_tmp = 0; c_tmp < 6; c_tmp++) {
            s += ID_B.Ic_data[(int32_T)i - 1].f1[c_tmp * 6 + b_tmp] * Si_data
              [(coffset + c_tmp) + 1];
          }

          Fi_data[(coffset + b_tmp) + 1] = s;
        }
      }

      nb = ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i - 1];
      tmp_7 = ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i + 16];
      if (nb > tmp_7) {
        b_tmp = 0;
        c_tmp = 0;
      } else {
        b_tmp = (int32_T)nb - 1;
        c_tmp = (int32_T)tmp_7;
      }

      if (nb > tmp_7) {
        k = 0;
        c = 0;
      } else {
        k = (int32_T)ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i - 1] - 1;
        c = (int32_T)ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i + 16];
      }

      ID_mtimes_hyn(Si_data, Si_size, Fi_data, Fi_size, XDHOffset, S_size);
      loop_ub = c_tmp - b_tmp;
      c_tmp = c - k;
      for (i_0 = 0; i_0 < c_tmp; i_0++) {
        for (coffset = 0; coffset < loop_ub; coffset++) {
          b->data[(b_tmp + coffset) + b->size[0] * (k + i_0)] =
            XDHOffset[loop_ub * i_0 + coffset];
        }
      }

      Fi_size_0[0] = 6;
      Fi_size_0[1] = Si_size[1];
      loop_ub = 6 * Si_size[1] - 1;
      if (loop_ub >= 0) {
        memcpy(&XDHOffset[0], &Fi_data[0], (uint32_T)(loop_ub + 1) * sizeof
               (real_T));
      }

      ID_mtimes_hynm(ID_B.X_data[(int32_T)i - 1].f1, XDHOffset, Fi_size_0,
                     Fi_data, Fi_size);
      while (pid > 0.0) {
        for (i_0 = 0; i_0 < 16; i_0++) {
          T[i_0] = ID_DW.obj_g.TreeInternal.Bodies[(int32_T)pid - 1]
            ->JointInternal.ChildToJointTransform[i_0];
        }

        for (i_0 = 0; i_0 < 3; i_0++) {
          R[3 * i_0] = T[i_0];
          R[3 * i_0 + 1] = T[i_0 + 4];
          R[3 * i_0 + 2] = T[i_0 + 8];
        }

        for (i_0 = 0; i_0 <= 6; i_0 += 2) {
          tmp_4 = _mm_loadu_pd(&R[i_0]);
          _mm_storeu_pd(&R_0[i_0], _mm_mul_pd(tmp_4, _mm_set1_pd(-1.0)));
        }

        for (i_0 = 8; i_0 < 9; i_0++) {
          R_0[i_0] = -R[i_0];
        }

        s = T[13];
        b_idx_0 = T[12];
        b_idx_1 = T[14];
        for (i_0 = 0; i_0 < 3; i_0++) {
          loop_ub = i_0 << 2;
          Tinv[loop_ub] = R[3 * i_0];
          Tinv[loop_ub + 1] = R[3 * i_0 + 1];
          Tinv[loop_ub + 2] = R[3 * i_0 + 2];
          Tinv[i_0 + 12] = (R_0[i_0 + 3] * s + R_0[i_0] * b_idx_0) + R_0[i_0 + 6]
            * b_idx_1;
        }

        Tinv[3] = 0.0;
        Tinv[7] = 0.0;
        Tinv[11] = 0.0;
        Tinv[15] = 1.0;
        rigidBodyJoint_get_MotionSubspa(&ID_DW.obj_g.TreeInternal.Bodies
          [(int32_T)pid - 1]->JointInternal, S_data, S_size);
        nb = ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)pid - 1];
        tmp_7 = ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)pid + 16];
        if (nb <= tmp_7) {
          tmp_0[0] = 0.0;
          tmp_0[3] = -Tinv[14];
          tmp_0[6] = Tinv[13];
          tmp_0[1] = Tinv[14];
          tmp_0[4] = 0.0;
          tmp_0[7] = -Tinv[12];
          tmp_0[2] = -Tinv[13];
          tmp_0[5] = Tinv[12];
          tmp_0[8] = 0.0;
          for (i_0 = 0; i_0 < 3; i_0++) {
            s = tmp_0[i_0 + 3];
            b_idx_0 = tmp_0[i_0];
            b_idx_1 = tmp_0[i_0 + 6];
            for (coffset = 0; coffset < 3; coffset++) {
              loop_ub = coffset << 2;
              R[i_0 + 3 * coffset] = (Tinv[loop_ub + 1] * s + Tinv[loop_ub] *
                b_idx_0) + Tinv[loop_ub + 2] * b_idx_1;
              XDHOffset[coffset + 6 * i_0] = Tinv[(i_0 << 2) + coffset];
              XDHOffset[coffset + 6 * (i_0 + 3)] = 0.0;
            }
          }

          for (i_0 = 0; i_0 < 3; i_0++) {
            XDHOffset[6 * i_0 + 3] = R[3 * i_0];
            loop_ub = i_0 << 2;
            c = (i_0 + 3) * 6;
            XDHOffset[c + 3] = Tinv[loop_ub];
            XDHOffset[6 * i_0 + 4] = R[3 * i_0 + 1];
            XDHOffset[c + 4] = Tinv[loop_ub + 1];
            XDHOffset[6 * i_0 + 5] = R[3 * i_0 + 2];
            XDHOffset[c + 5] = Tinv[loop_ub + 2];
          }

          ID_mtimes(XDHOffset, S_data, S_size, Si_data, Si_size);
          ID_mtimes_hyn(Si_data, Si_size, Fi_data, Fi_size, XDHOffset, S_size);
          if (nb > tmp_7) {
            b_tmp = 0;
            c_tmp = 0;
          } else {
            b_tmp = (int32_T)nb - 1;
            c_tmp = (int32_T)tmp_7;
          }

          s = ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i - 1];
          if (s > ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i + 16]) {
            k = 0;
            c = 0;
          } else {
            k = (int32_T)s - 1;
            c = (int32_T)ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i + 16];
          }

          loop_ub = c_tmp - b_tmp;
          c_tmp = c - k;
          for (i_0 = 0; i_0 < c_tmp; i_0++) {
            for (coffset = 0; coffset < loop_ub; coffset++) {
              b->data[(b_tmp + coffset) + b->size[0] * (k + i_0)] =
                XDHOffset[loop_ub * i_0 + coffset];
            }
          }

          s = ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i - 1];
          if (s > ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i + 16]) {
            b_tmp = 0;
            c_tmp = 0;
          } else {
            b_tmp = (int32_T)s - 1;
            c_tmp = (int32_T)ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)i
              + 16];
          }

          if (nb > tmp_7) {
            k = 0;
            c = 0;
          } else {
            k = (int32_T)ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)pid -
              1] - 1;
            c = (int32_T)ID_DW.obj_g.TreeInternal.VelocityDoFMap[(int32_T)pid +
              16];
          }

          loop_ub = S_size[1];
          Hji_size_idx_0 = S_size[1];
          loop_ub_0 = S_size[0];
          for (i_0 = 0; i_0 < loop_ub_0; i_0++) {
            for (coffset = 0; coffset < loop_ub; coffset++) {
              S_data[coffset + Hji_size_idx_0 * i_0] = XDHOffset[S_size[0] *
                coffset + i_0];
            }
          }

          loop_ub = c_tmp - b_tmp;
          c_tmp = c - k;
          for (i_0 = 0; i_0 < c_tmp; i_0++) {
            for (coffset = 0; coffset < loop_ub; coffset++) {
              b->data[(b_tmp + coffset) + b->size[0] * (k + i_0)] =
                S_data[loop_ub * i_0 + coffset];
            }
          }
        }

        Fi_size_1[0] = 6;
        Fi_size_1[1] = Fi_size[1];
        loop_ub = Fi_size[0] * Fi_size[1] - 1;
        if (loop_ub >= 0) {
          memcpy(&XDHOffset[0], &Fi_data[0], (uint32_T)(loop_ub + 1) * sizeof
                 (real_T));
        }

        ID_mtimes_hynm(ID_B.X_data[(int32_T)pid - 1].f1, XDHOffset, Fi_size_1,
                       Fi_data, Fi_size);
        pid = ID_DW.obj_g.TreeInternal.Bodies[(int32_T)pid - 1]->ParentIndex;
      }
    }
  }

  /* Outport: '<Root>/M' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   */
  memcpy(&ID_Y.M[0], &b->data[0], 144U * sizeof(real_T));
  ID_emxFree_real_T(&b);

  /* MATLABSystem: '<S5>/MATLAB System' incorporates:
   *  Inport: '<Root>/qdot'
   */
  a0[0] = 0.0;
  a0[1] = 0.0;
  a0[2] = 0.0;
  a0[3] = -ID_DW.obj_h.TreeInternal.Gravity[0];
  a0[4] = -ID_DW.obj_h.TreeInternal.Gravity[1];
  a0[5] = -ID_DW.obj_h.TreeInternal.Gravity[2];
  nb = ID_DW.obj_h.TreeInternal.NumBodies;
  f = (int32_T)nb;
  b_tmp = 6 * (int32_T)nb;
  if (b_tmp - 1 >= 0) {
    memset(&vJ_data[0], 0, (uint32_T)b_tmp * sizeof(real_T));
  }

  if (b_tmp - 1 >= 0) {
    memset(&vB_data[0], 0, (uint32_T)b_tmp * sizeof(real_T));
  }

  if (b_tmp - 1 >= 0) {
    memset(&aB_data[0], 0, (uint32_T)b_tmp * sizeof(real_T));
  }

  memset(&tmp[0], 0, 12U * sizeof(real_T));
  for (c_tmp = 0; c_tmp < f; c_tmp++) {
    memset(&XDHOffset[0], 0, 36U * sizeof(real_T));
    for (b_0 = 0; b_0 < 6; b_0++) {
      XDHOffset[b_0 + 6 * b_0] = 1.0;
    }

    for (i_0 = 0; i_0 < 36; i_0++) {
      ID_B.Xtree_data[c_tmp].f1[i_0] = XDHOffset[i_0];
      XDHOffset[i_0] = 0.0;
    }

    for (b_0 = 0; b_0 < 6; b_0++) {
      XDHOffset[b_0 + 6 * b_0] = 1.0;
    }

    memcpy(&ID_B.X_data_m[c_tmp].f1[0], &XDHOffset[0], 36U * sizeof(real_T));
  }

  if ((int32_T)nb - 1 >= 0) {
    tmp_1[0] = 0.0;
    tmp_1[4] = 0.0;
    tmp_1[8] = 0.0;
  }

  for (b_tmp = 0; b_tmp < f; b_tmp++) {
    obj = ID_DW.obj_h.TreeInternal.Bodies[b_tmp];
    rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, S_data, S_size);
    pid = ID_DW.obj_h.TreeInternal.PositionDoFMap[b_tmp];
    s = ID_DW.obj_h.TreeInternal.PositionDoFMap[b_tmp + 17];
    b_idx_0 = ID_DW.obj_h.TreeInternal.VelocityDoFMap[b_tmp];
    b_idx_1 = ID_DW.obj_h.TreeInternal.VelocityDoFMap[b_tmp + 17];
    memset(&XDHOffset[0], 0, 36U * sizeof(real_T));
    for (c_tmp = 0; c_tmp < 6; c_tmp++) {
      XDHOffset[c_tmp + 6 * c_tmp] = 1.0;
    }

    if (s < pid) {
      obj = ID_DW.obj_h.TreeInternal.Bodies[b_tmp];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal, T);
      for (i_0 = 0; i_0 < 6; i_0++) {
        vJ_data[i_0 + 6 * b_tmp] = 0.0;
      }
    } else {
      if (pid > s) {
        b_0 = 0;
        c_tmp = 0;
      } else {
        b_0 = (int32_T)pid - 1;
        c_tmp = (int32_T)s;
      }

      if (b_idx_0 > b_idx_1) {
        i_0 = 1;
      } else {
        i_0 = (int32_T)b_idx_0;
      }

      obj = ID_DW.obj_h.TreeInternal.Bodies[b_tmp];
      loop_ub = c_tmp - b_0;
      for (coffset = 0; coffset < loop_ub; coffset++) {
        jointTorqWithoutVel[coffset] = rtb_TmpSignalConversionAtMATLAB[b_0 +
          coffset];
      }

      rigidBodyJoint_transformBodyT_h(&obj->JointInternal, jointTorqWithoutVel,
        &loop_ub, T);
      obj = ID_DW.obj_h.TreeInternal.Bodies[b_tmp];
      for (coffset = 0; coffset < 16; coffset++) {
        Tinv[coffset] = obj->JointInternal.ChildToJointTransform[coffset];
      }

      for (coffset = 0; coffset < 3; coffset++) {
        R[3 * coffset] = Tinv[coffset];
        R[3 * coffset + 1] = Tinv[coffset + 4];
        R[3 * coffset + 2] = Tinv[coffset + 8];
      }

      for (coffset = 0; coffset <= 6; coffset += 2) {
        tmp_4 = _mm_loadu_pd(&R[coffset]);
        _mm_storeu_pd(&R_0[coffset], _mm_mul_pd(tmp_4, _mm_set1_pd(-1.0)));
      }

      for (coffset = 8; coffset < 9; coffset++) {
        R_0[coffset] = -R[coffset];
      }

      pid = Tinv[13];
      s = Tinv[12];
      b_idx_0 = Tinv[14];
      for (coffset = 0; coffset < 3; coffset++) {
        loop_ub = coffset << 2;
        Tinv[loop_ub] = R[3 * coffset];
        Tinv[loop_ub + 1] = R[3 * coffset + 1];
        Tinv[loop_ub + 2] = R[3 * coffset + 2];
        Tinv[coffset + 12] = (R_0[coffset + 3] * pid + R_0[coffset] * s) +
          R_0[coffset + 6] * b_idx_0;
      }

      Tinv[3] = 0.0;
      Tinv[7] = 0.0;
      Tinv[11] = 0.0;
      Tinv[15] = 1.0;
      tmp_0[0] = 0.0;
      tmp_0[3] = -Tinv[14];
      tmp_0[6] = Tinv[13];
      tmp_0[1] = Tinv[14];
      tmp_0[4] = 0.0;
      tmp_0[7] = -Tinv[12];
      tmp_0[2] = -Tinv[13];
      tmp_0[5] = Tinv[12];
      tmp_0[8] = 0.0;
      for (coffset = 0; coffset < 3; coffset++) {
        s = tmp_0[coffset + 3];
        b_idx_0 = tmp_0[coffset];
        b_idx_1 = tmp_0[coffset + 6];
        for (loop_ub = 0; loop_ub < 3; loop_ub++) {
          b_0 = loop_ub << 2;
          R[coffset + 3 * loop_ub] = (Tinv[b_0 + 1] * s + Tinv[b_0] * b_idx_0) +
            Tinv[b_0 + 2] * b_idx_1;
          XDHOffset[loop_ub + 6 * coffset] = Tinv[(coffset << 2) + loop_ub];
          XDHOffset[loop_ub + 6 * (coffset + 3)] = 0.0;
        }
      }

      for (coffset = 0; coffset < 3; coffset++) {
        XDHOffset[6 * coffset + 3] = R[3 * coffset];
        b_0 = coffset << 2;
        c_tmp = (coffset + 3) * 6;
        XDHOffset[c_tmp + 3] = Tinv[b_0];
        XDHOffset[6 * coffset + 4] = R[3 * coffset + 1];
        XDHOffset[c_tmp + 4] = Tinv[b_0 + 1];
        XDHOffset[6 * coffset + 5] = R[3 * coffset + 2];
        XDHOffset[c_tmp + 5] = Tinv[b_0 + 2];
      }

      ID_mtimes(XDHOffset, S_data, S_size, Si_data, Si_size);
      for (b_0 = 0; b_0 < 6; b_0++) {
        y[b_0] = 0.0;
      }

      c = Si_size[1];
      for (c_tmp = 0; c_tmp < c; c_tmp++) {
        coffset = c_tmp * 6 - 1;
        for (b_0 = 0; b_0 <= 4; b_0 += 2) {
          tmp_4 = _mm_loadu_pd(&Si_data[(coffset + b_0) + 1]);
          tmp_3 = _mm_loadu_pd(&y[b_0]);
          _mm_storeu_pd(&y[b_0], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(ID_U.qdot
            [(i_0 + c_tmp) - 1]), tmp_4), tmp_3));
        }
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        vJ_data[i_0 + 6 * b_tmp] = y[i_0];
      }
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      R[3 * i_0] = T[i_0];
      R[3 * i_0 + 1] = T[i_0 + 4];
      R[3 * i_0 + 2] = T[i_0 + 8];
    }

    for (i_0 = 0; i_0 <= 6; i_0 += 2) {
      tmp_4 = _mm_loadu_pd(&R[i_0]);
      _mm_storeu_pd(&R_0[i_0], _mm_mul_pd(tmp_4, _mm_set1_pd(-1.0)));
    }

    for (i_0 = 8; i_0 < 9; i_0++) {
      R_0[i_0] = -R[i_0];
    }

    s = T[13];
    b_idx_0 = T[12];
    b_idx_1 = T[14];
    for (i_0 = 0; i_0 < 3; i_0++) {
      loop_ub = i_0 << 2;
      Tinv[loop_ub] = R[3 * i_0];
      Tinv[loop_ub + 1] = R[3 * i_0 + 1];
      Tinv[loop_ub + 2] = R[3 * i_0 + 2];
      Tinv[i_0 + 12] = (R_0[i_0 + 3] * s + R_0[i_0] * b_idx_0) + R_0[i_0 + 6] *
        b_idx_1;
    }

    Tinv[3] = 0.0;
    Tinv[7] = 0.0;
    Tinv[11] = 0.0;
    Tinv[15] = 1.0;
    tmp_1[3] = -Tinv[14];
    tmp_1[6] = Tinv[13];
    tmp_1[1] = Tinv[14];
    tmp_1[7] = -Tinv[12];
    tmp_1[2] = -Tinv[13];
    tmp_1[5] = Tinv[12];
    for (i_0 = 0; i_0 < 3; i_0++) {
      s = tmp_1[i_0 + 3];
      b_idx_0 = tmp_1[i_0];
      b_idx_1 = tmp_1[i_0 + 6];
      for (coffset = 0; coffset < 3; coffset++) {
        loop_ub = coffset << 2;
        tmp_0[i_0 + 3 * coffset] = (Tinv[loop_ub + 1] * s + Tinv[loop_ub] *
          b_idx_0) + Tinv[loop_ub + 2] * b_idx_1;
        ID_B.X_data_m[b_tmp].f1[coffset + 6 * i_0] = Tinv[(i_0 << 2) + coffset];
        ID_B.X_data_m[b_tmp].f1[coffset + 6 * (i_0 + 3)] = 0.0;
      }
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      ID_B.X_data_m[b_tmp].f1[6 * i_0 + 3] = tmp_0[3 * i_0];
      coffset = i_0 << 2;
      loop_ub = (i_0 + 3) * 6;
      ID_B.X_data_m[b_tmp].f1[loop_ub + 3] = Tinv[coffset];
      ID_B.X_data_m[b_tmp].f1[6 * i_0 + 4] = tmp_0[3 * i_0 + 1];
      ID_B.X_data_m[b_tmp].f1[loop_ub + 4] = Tinv[coffset + 1];
      ID_B.X_data_m[b_tmp].f1[6 * i_0 + 5] = tmp_0[3 * i_0 + 2];
      ID_B.X_data_m[b_tmp].f1[loop_ub + 5] = Tinv[coffset + 2];
    }

    pid = ID_DW.obj_h.TreeInternal.Bodies[b_tmp]->ParentIndex;
    if (pid > 0.0) {
      for (i_0 = 0; i_0 < 6; i_0++) {
        s = 0.0;
        for (coffset = 0; coffset < 6; coffset++) {
          s += vB_data[((int32_T)pid - 1) * 6 + coffset] * ID_B.X_data_m[b_tmp].
            f1[6 * coffset + i_0];
        }

        y[i_0] = vJ_data[6 * b_tmp + i_0] + s;
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        vB_data[i_0 + 6 * b_tmp] = y[i_0];
      }

      ID_mtimes(XDHOffset, S_data, S_size, Si_data, Si_size);
      for (b_0 = 0; b_0 < 6; b_0++) {
        y[b_0] = 0.0;
      }

      c = Si_size[1];
      for (c_tmp = 0; c_tmp < c; c_tmp++) {
        coffset = c_tmp * 6 - 1;
        for (b_0 = 0; b_0 <= 4; b_0 += 2) {
          tmp_4 = _mm_loadu_pd(&Si_data[(coffset + b_0) + 1]);
          tmp_3 = _mm_loadu_pd(&y[b_0]);
          _mm_storeu_pd(&y[b_0], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(0.0)),
            tmp_3));
        }
      }

      R[0] = 0.0;
      i = vB_data[6 * b_tmp + 2];
      R[3] = -i;
      s = vB_data[6 * b_tmp + 1];
      R[6] = s;
      R[1] = i;
      R[4] = 0.0;
      i = vB_data[6 * b_tmp];
      R[7] = -i;
      R[2] = -s;
      R[5] = i;
      R[8] = 0.0;
      Fi_data[3] = 0.0;
      i = vB_data[6 * b_tmp + 5];
      Fi_data[9] = -i;
      s = vB_data[6 * b_tmp + 4];
      Fi_data[15] = s;
      Fi_data[4] = i;
      Fi_data[10] = 0.0;
      i = vB_data[6 * b_tmp + 3];
      Fi_data[16] = -i;
      Fi_data[5] = -s;
      Fi_data[11] = i;
      Fi_data[17] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        i = R[3 * i_0];
        Fi_data[6 * i_0] = i;
        b_0 = (i_0 + 3) * 6;
        Fi_data[b_0] = 0.0;
        Fi_data[b_0 + 3] = i;
        i = R[3 * i_0 + 1];
        Fi_data[6 * i_0 + 1] = i;
        Fi_data[b_0 + 1] = 0.0;
        Fi_data[b_0 + 4] = i;
        i = R[3 * i_0 + 2];
        Fi_data[6 * i_0 + 2] = i;
        Fi_data[b_0 + 2] = 0.0;
        Fi_data[b_0 + 5] = i;
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        s = 0.0;
        i = 0.0;
        for (coffset = 0; coffset < 6; coffset++) {
          loop_ub = 6 * coffset + i_0;
          _mm_storeu_pd(&tmp_5[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
            (Fi_data[loop_ub], ID_B.X_data_m[b_tmp].f1[loop_ub]), _mm_set_pd
            (vJ_data[coffset + 6 * b_tmp], aB_data[coffset + 6 * ((int32_T)pid -
            1)])), _mm_set_pd(i, s)));
          s = tmp_5[0];
          i = tmp_5[1];
        }

        R_1[i_0] = i;
        X[i_0] = s + y[i_0];
      }

      for (i_0 = 0; i_0 <= 4; i_0 += 2) {
        tmp_4 = _mm_loadu_pd(&X[i_0]);
        tmp_3 = _mm_loadu_pd(&R_1[i_0]);
        _mm_storeu_pd(&aB_data[i_0 + 6 * b_tmp], _mm_add_pd(tmp_4, tmp_3));
      }

      tmp_0[0] = 0.0;
      tmp_0[3] = -T[14];
      tmp_0[6] = T[13];
      tmp_0[1] = T[14];
      tmp_0[4] = 0.0;
      tmp_0[7] = -T[12];
      tmp_0[2] = -T[13];
      tmp_0[5] = T[12];
      tmp_0[8] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        s = tmp_0[i_0 + 3];
        b_idx_0 = tmp_0[i_0];
        b_idx_1 = tmp_0[i_0 + 6];
        for (coffset = 0; coffset < 3; coffset++) {
          loop_ub = coffset << 2;
          R[i_0 + 3 * coffset] = (T[loop_ub + 1] * s + T[loop_ub] * b_idx_0) +
            T[loop_ub + 2] * b_idx_1;
          XDHOffset[coffset + 6 * i_0] = T[(i_0 << 2) + coffset];
          XDHOffset[coffset + 6 * (i_0 + 3)] = 0.0;
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        XDHOffset[6 * i_0 + 3] = R[3 * i_0];
        b_0 = i_0 << 2;
        c_tmp = (i_0 + 3) * 6;
        XDHOffset[c_tmp + 3] = T[b_0];
        XDHOffset[6 * i_0 + 4] = R[3 * i_0 + 1];
        XDHOffset[c_tmp + 4] = T[b_0 + 1];
        XDHOffset[6 * i_0 + 5] = R[3 * i_0 + 2];
        XDHOffset[c_tmp + 5] = T[b_0 + 2];
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        for (coffset = 0; coffset < 6; coffset++) {
          i = 0.0;
          for (loop_ub = 0; loop_ub < 6; loop_ub++) {
            i += ID_B.Xtree_data[(int32_T)pid - 1].f1[6 * loop_ub + i_0] *
              XDHOffset[6 * coffset + loop_ub];
          }

          Fi_data[i_0 + 6 * coffset] = i;
        }
      }

      memcpy(&ID_B.Xtree_data[b_tmp].f1[0], &Fi_data[0], 36U * sizeof(real_T));
    } else {
      ID_mtimes(XDHOffset, S_data, S_size, Si_data, Si_size);
      for (b_0 = 0; b_0 < 6; b_0++) {
        i_0 = 6 * b_tmp + b_0;
        vB_data[i_0] = vJ_data[i_0];
        y[b_0] = 0.0;
      }

      c = Si_size[1];
      for (c_tmp = 0; c_tmp < c; c_tmp++) {
        coffset = c_tmp * 6 - 1;
        for (b_0 = 0; b_0 <= 4; b_0 += 2) {
          tmp_4 = _mm_loadu_pd(&Si_data[(coffset + b_0) + 1]);
          tmp_3 = _mm_loadu_pd(&y[b_0]);
          _mm_storeu_pd(&y[b_0], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(0.0)),
            tmp_3));
        }
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        s = 0.0;
        for (coffset = 0; coffset < 6; coffset++) {
          s += ID_B.X_data_m[b_tmp].f1[6 * coffset + i_0] * a0[coffset];
        }

        aB_data[i_0 + 6 * b_tmp] = s + y[i_0];
      }

      tmp_0[0] = 0.0;
      tmp_0[3] = -T[14];
      tmp_0[6] = T[13];
      tmp_0[1] = T[14];
      tmp_0[4] = 0.0;
      tmp_0[7] = -T[12];
      tmp_0[2] = -T[13];
      tmp_0[5] = T[12];
      tmp_0[8] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        s = tmp_0[i_0 + 3];
        b_idx_0 = tmp_0[i_0];
        b_idx_1 = tmp_0[i_0 + 6];
        for (coffset = 0; coffset < 3; coffset++) {
          loop_ub = coffset << 2;
          R[i_0 + 3 * coffset] = (T[loop_ub + 1] * s + T[loop_ub] * b_idx_0) +
            T[loop_ub + 2] * b_idx_1;
          ID_B.Xtree_data[b_tmp].f1[coffset + 6 * i_0] = T[(i_0 << 2) + coffset];
          ID_B.Xtree_data[b_tmp].f1[coffset + 6 * (i_0 + 3)] = 0.0;
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        ID_B.Xtree_data[b_tmp].f1[6 * i_0 + 3] = R[3 * i_0];
        coffset = i_0 << 2;
        loop_ub = (i_0 + 3) * 6;
        ID_B.Xtree_data[b_tmp].f1[loop_ub + 3] = T[coffset];
        ID_B.Xtree_data[b_tmp].f1[6 * i_0 + 4] = R[3 * i_0 + 1];
        ID_B.Xtree_data[b_tmp].f1[loop_ub + 4] = T[coffset + 1];
        ID_B.Xtree_data[b_tmp].f1[6 * i_0 + 5] = R[3 * i_0 + 2];
        ID_B.Xtree_data[b_tmp].f1[loop_ub + 5] = T[coffset + 2];
      }
    }

    for (i_0 = 0; i_0 < 36; i_0++) {
      XDHOffset[i_0] = ID_DW.obj_h.TreeInternal.Bodies[b_tmp]->
        SpatialInertia[i_0];
    }

    R[0] = 0.0;
    i = vB_data[6 * b_tmp + 2];
    R[3] = -i;
    s = vB_data[6 * b_tmp + 1];
    R[6] = s;
    R[1] = i;
    R[4] = 0.0;
    i = vB_data[6 * b_tmp];
    R[7] = -i;
    R[2] = -s;
    R[5] = i;
    R[8] = 0.0;
    Fi_data[18] = 0.0;
    i = vB_data[6 * b_tmp + 5];
    Fi_data[24] = -i;
    s = vB_data[6 * b_tmp + 4];
    Fi_data[30] = s;
    Fi_data[19] = i;
    Fi_data[25] = 0.0;
    i = vB_data[6 * b_tmp + 3];
    Fi_data[31] = -i;
    Fi_data[20] = -s;
    Fi_data[26] = i;
    Fi_data[32] = 0.0;
    for (i_0 = 0; i_0 < 3; i_0++) {
      i = R[3 * i_0];
      Fi_data[6 * i_0] = i;
      Fi_data[6 * i_0 + 3] = 0.0;
      b_0 = (i_0 + 3) * 6;
      Fi_data[b_0 + 3] = i;
      i = R[3 * i_0 + 1];
      Fi_data[6 * i_0 + 1] = i;
      Fi_data[6 * i_0 + 4] = 0.0;
      Fi_data[b_0 + 4] = i;
      i = R[3 * i_0 + 2];
      Fi_data[6 * i_0 + 2] = i;
      Fi_data[6 * i_0 + 5] = 0.0;
      Fi_data[b_0 + 5] = i;
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      i = 0.0;
      pid = 0.0;
      for (coffset = 0; coffset < 6; coffset++) {
        b_0 = 6 * b_tmp + coffset;
        _mm_storeu_pd(&tmp_5[0], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(XDHOffset[6 *
          coffset + i_0]), _mm_set_pd(aB_data[b_0], vB_data[b_0])), _mm_set_pd
          (pid, i)));
        i = tmp_5[0];
        pid = tmp_5[1];
      }

      X[i_0] = pid;
      y[i_0] = i;
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      s = 0.0;
      i = 0.0;
      for (coffset = 0; coffset < 6; coffset++) {
        _mm_storeu_pd(&tmp_5[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
          (ID_B.Xtree_data[b_tmp].f1[6 * i_0 + coffset], Fi_data[6 * coffset +
           i_0]), _mm_set_pd(0.0, y[coffset])), _mm_set_pd(s, i)));
        i = tmp_5[0];
        s = tmp_5[1];
      }

      f_data[i_0 + 6 * b_tmp] = (X[i_0] + i) - s;
    }
  }

  f = (int32_T)-((-1.0 - nb) + 1.0);
  for (b_0 = 0; b_0 < f; b_0++) {
    i = nb - (real_T)b_0;
    obj = ID_DW.obj_h.TreeInternal.Bodies[(int32_T)i - 1];
    pid = obj->JointInternal.TypeInternal.Length;
    for (i_0 = 0; i_0 < 200; i_0++) {
      obj_Vector[i_0] = obj->JointInternal.TypeInternal.Vector[i_0];
    }

    b_bool = false;
    if (pid < 1.0) {
      i_0 = 0;
    } else {
      i_0 = (int32_T)pid;
    }

    if (i_0 == 5) {
      b_tmp = 1;
      do {
        exitg1 = 0;
        if (b_tmp - 1 < 5) {
          if (obj_Vector[b_tmp - 1] != tmp_6[b_tmp - 1]) {
            exitg1 = 1;
          } else {
            b_tmp++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!b_bool) {
      obj = ID_DW.obj_h.TreeInternal.Bodies[(int32_T)i - 1];
      for (i_0 = 0; i_0 < 16; i_0++) {
        Tinv[i_0] = obj->JointInternal.ChildToJointTransform[i_0];
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        R[3 * i_0] = Tinv[i_0];
        R[3 * i_0 + 1] = Tinv[i_0 + 4];
        R[3 * i_0 + 2] = Tinv[i_0 + 8];
      }

      for (i_0 = 0; i_0 <= 6; i_0 += 2) {
        tmp_4 = _mm_loadu_pd(&R[i_0]);
        _mm_storeu_pd(&R_0[i_0], _mm_mul_pd(tmp_4, _mm_set1_pd(-1.0)));
      }

      for (i_0 = 8; i_0 < 9; i_0++) {
        R_0[i_0] = -R[i_0];
      }

      pid = Tinv[13];
      s = Tinv[12];
      b_idx_0 = Tinv[14];
      for (i_0 = 0; i_0 < 3; i_0++) {
        loop_ub = i_0 << 2;
        Tinv[loop_ub] = R[3 * i_0];
        Tinv[loop_ub + 1] = R[3 * i_0 + 1];
        Tinv[loop_ub + 2] = R[3 * i_0 + 2];
        Tinv[i_0 + 12] = (R_0[i_0 + 3] * pid + R_0[i_0] * s) + R_0[i_0 + 6] *
          b_idx_0;
      }

      Tinv[3] = 0.0;
      Tinv[7] = 0.0;
      Tinv[11] = 0.0;
      Tinv[15] = 1.0;
      obj = ID_DW.obj_h.TreeInternal.Bodies[(int32_T)i - 1];
      rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, S_data, S_size);
      b_idx_0 = ID_DW.obj_h.TreeInternal.VelocityDoFMap[(int32_T)i - 1];
      b_idx_1 = ID_DW.obj_h.TreeInternal.VelocityDoFMap[(int32_T)i + 16];
      if (b_idx_0 > b_idx_1) {
        b_tmp = 0;
        c_tmp = 0;
      } else {
        b_tmp = (int32_T)b_idx_0 - 1;
        c_tmp = (int32_T)b_idx_1;
      }

      tmp_0[0] = 0.0;
      tmp_0[3] = -Tinv[14];
      tmp_0[6] = Tinv[13];
      tmp_0[1] = Tinv[14];
      tmp_0[4] = 0.0;
      tmp_0[7] = -Tinv[12];
      tmp_0[2] = -Tinv[13];
      tmp_0[5] = Tinv[12];
      tmp_0[8] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        s = tmp_0[i_0 + 3];
        b_idx_0 = tmp_0[i_0];
        b_idx_1 = tmp_0[i_0 + 6];
        for (coffset = 0; coffset < 3; coffset++) {
          loop_ub = coffset << 2;
          R[i_0 + 3 * coffset] = (Tinv[loop_ub + 1] * s + Tinv[loop_ub] *
            b_idx_0) + Tinv[loop_ub + 2] * b_idx_1;
          XDHOffset[coffset + 6 * i_0] = Tinv[(i_0 << 2) + coffset];
          XDHOffset[coffset + 6 * (i_0 + 3)] = 0.0;
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        XDHOffset[6 * i_0 + 3] = R[3 * i_0];
        loop_ub = i_0 << 2;
        c = (i_0 + 3) * 6;
        XDHOffset[c + 3] = Tinv[loop_ub];
        XDHOffset[6 * i_0 + 4] = R[3 * i_0 + 1];
        XDHOffset[c + 4] = Tinv[loop_ub + 1];
        XDHOffset[6 * i_0 + 5] = R[3 * i_0 + 2];
        XDHOffset[c + 5] = Tinv[loop_ub + 2];
      }

      ID_mtimes(XDHOffset, S_data, S_size, Si_data, Si_size);
      ID_mtimes_hynmtam(Si_data, Si_size, &f_data[6 * ((int32_T)i - 1)], a0,
                        &loop_ub);
      c_tmp -= b_tmp;
      for (i_0 = 0; i_0 < c_tmp; i_0++) {
        tmp[b_tmp + i_0] = a0[i_0];
      }
    }

    pid = ID_DW.obj_h.TreeInternal.Bodies[(int32_T)i - 1]->ParentIndex;
    if (pid > 0.0) {
      for (i_0 = 0; i_0 < 6; i_0++) {
        s = 0.0;
        for (coffset = 0; coffset < 6; coffset++) {
          s += f_data[((int32_T)i - 1) * 6 + coffset] * ID_B.X_data_m[(int32_T)i
            - 1].f1[6 * i_0 + coffset];
        }

        a0[i_0] = f_data[((int32_T)pid - 1) * 6 + i_0] + s;
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        f_data[i_0 + 6 * ((int32_T)pid - 1)] = a0[i_0];
      }
    }
  }

  RigidBodyTreeDynamics_inverseDy(&ID_DW.obj_h.TreeInternal,
    rtb_TmpSignalConversionAtMATLAB, jointTorqWithoutVel);

  /* Outport: '<Root>/c' incorporates:
   *  MATLABSystem: '<S5>/MATLAB System'
   */
  for (i_0 = 0; i_0 <= 10; i_0 += 2) {
    /* MATLABSystem: '<S5>/MATLAB System' */
    tmp_4 = _mm_loadu_pd(&tmp[i_0]);
    tmp_3 = _mm_loadu_pd(&jointTorqWithoutVel[i_0]);
    _mm_storeu_pd(&ID_Y.c[i_0], _mm_sub_pd(tmp_4, tmp_3));
  }

  /* End of Outport: '<Root>/c' */

  /* MATLABSystem: '<S3>/MATLAB System' */
  a0[0] = 0.0;
  a0[1] = 0.0;
  a0[2] = 0.0;
  a0[3] = -ID_DW.obj.TreeInternal.Gravity[0];
  a0[4] = -ID_DW.obj.TreeInternal.Gravity[1];
  a0[5] = -ID_DW.obj.TreeInternal.Gravity[2];
  nb = ID_DW.obj.TreeInternal.NumBodies;
  f = (int32_T)nb;
  b_tmp = 6 * (int32_T)nb;
  if (b_tmp - 1 >= 0) {
    memset(&vJ_data[0], 0, (uint32_T)b_tmp * sizeof(real_T));
  }

  if (b_tmp - 1 >= 0) {
    memset(&vB_data[0], 0, (uint32_T)b_tmp * sizeof(real_T));
  }

  if (b_tmp - 1 >= 0) {
    memset(&aB_data[0], 0, (uint32_T)b_tmp * sizeof(real_T));
  }

  /* Outport: '<Root>/g' incorporates:
   *  MATLABSystem: '<S3>/MATLAB System'
   */
  memset(&ID_Y.g[0], 0, 12U * sizeof(real_T));

  /* MATLABSystem: '<S3>/MATLAB System' incorporates:
   *  Outport: '<Root>/g'
   */
  for (c_tmp = 0; c_tmp < f; c_tmp++) {
    memset(&XDHOffset[0], 0, 36U * sizeof(real_T));
    for (b_0 = 0; b_0 < 6; b_0++) {
      XDHOffset[b_0 + 6 * b_0] = 1.0;
    }

    for (i_0 = 0; i_0 < 36; i_0++) {
      ID_B.Ic_data[c_tmp].f1[i_0] = XDHOffset[i_0];
      XDHOffset[i_0] = 0.0;
    }

    for (b_0 = 0; b_0 < 6; b_0++) {
      XDHOffset[b_0 + 6 * b_0] = 1.0;
    }

    memcpy(&ID_B.X_data[c_tmp].f1[0], &XDHOffset[0], 36U * sizeof(real_T));
  }

  if ((int32_T)nb - 1 >= 0) {
    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    tmp_2[0] = 0.0;
    tmp_2[4] = 0.0;
    tmp_2[8] = 0.0;
  }

  for (b_tmp = 0; b_tmp < f; b_tmp++) {
    obj = ID_DW.obj.TreeInternal.Bodies[b_tmp];
    rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, S_data, S_size);
    pid = ID_DW.obj.TreeInternal.PositionDoFMap[b_tmp];
    s = ID_DW.obj.TreeInternal.PositionDoFMap[b_tmp + 17];
    memset(&XDHOffset[0], 0, 36U * sizeof(real_T));
    for (c_tmp = 0; c_tmp < 6; c_tmp++) {
      XDHOffset[c_tmp + 6 * c_tmp] = 1.0;
    }

    if (s < pid) {
      obj = ID_DW.obj.TreeInternal.Bodies[b_tmp];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal, T);
      for (i_0 = 0; i_0 < 6; i_0++) {
        vJ_data[i_0 + 6 * b_tmp] = 0.0;
      }
    } else {
      if (pid > s) {
        b_0 = 0;
        c_tmp = 0;
      } else {
        b_0 = (int32_T)pid - 1;
        c_tmp = (int32_T)s;
      }

      obj = ID_DW.obj.TreeInternal.Bodies[b_tmp];
      loop_ub = c_tmp - b_0;
      for (i_0 = 0; i_0 < loop_ub; i_0++) {
        jointTorqWithoutVel[i_0] = rtb_TmpSignalConversionAtMATLAB[b_0 + i_0];
      }

      rigidBodyJoint_transformBodyT_h(&obj->JointInternal, jointTorqWithoutVel,
        &loop_ub, T);
      obj = ID_DW.obj.TreeInternal.Bodies[b_tmp];
      for (i_0 = 0; i_0 < 16; i_0++) {
        Tinv[i_0] = obj->JointInternal.ChildToJointTransform[i_0];
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        R[3 * i_0] = Tinv[i_0];
        R[3 * i_0 + 1] = Tinv[i_0 + 4];
        R[3 * i_0 + 2] = Tinv[i_0 + 8];
      }

      for (i_0 = 0; i_0 <= 6; i_0 += 2) {
        tmp_4 = _mm_loadu_pd(&R[i_0]);
        _mm_storeu_pd(&R_0[i_0], _mm_mul_pd(tmp_4, _mm_set1_pd(-1.0)));
      }

      for (i_0 = 8; i_0 < 9; i_0++) {
        R_0[i_0] = -R[i_0];
      }

      pid = Tinv[13];
      s = Tinv[12];
      b_idx_0 = Tinv[14];
      for (i_0 = 0; i_0 < 3; i_0++) {
        loop_ub = i_0 << 2;
        Tinv[loop_ub] = R[3 * i_0];
        Tinv[loop_ub + 1] = R[3 * i_0 + 1];
        Tinv[loop_ub + 2] = R[3 * i_0 + 2];
        Tinv[i_0 + 12] = (R_0[i_0 + 3] * pid + R_0[i_0] * s) + R_0[i_0 + 6] *
          b_idx_0;
      }

      Tinv[3] = 0.0;
      Tinv[7] = 0.0;
      Tinv[11] = 0.0;
      Tinv[15] = 1.0;
      tmp_0[0] = 0.0;
      tmp_0[3] = -Tinv[14];
      tmp_0[6] = Tinv[13];
      tmp_0[1] = Tinv[14];
      tmp_0[4] = 0.0;
      tmp_0[7] = -Tinv[12];
      tmp_0[2] = -Tinv[13];
      tmp_0[5] = Tinv[12];
      tmp_0[8] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        s = tmp_0[i_0 + 3];
        b_idx_0 = tmp_0[i_0];
        b_idx_1 = tmp_0[i_0 + 6];
        for (coffset = 0; coffset < 3; coffset++) {
          loop_ub = coffset << 2;
          R[i_0 + 3 * coffset] = (Tinv[loop_ub + 1] * s + Tinv[loop_ub] *
            b_idx_0) + Tinv[loop_ub + 2] * b_idx_1;
          XDHOffset[coffset + 6 * i_0] = Tinv[(i_0 << 2) + coffset];
          XDHOffset[coffset + 6 * (i_0 + 3)] = 0.0;
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        XDHOffset[6 * i_0 + 3] = R[3 * i_0];
        b_0 = i_0 << 2;
        c_tmp = (i_0 + 3) * 6;
        XDHOffset[c_tmp + 3] = Tinv[b_0];
        XDHOffset[6 * i_0 + 4] = R[3 * i_0 + 1];
        XDHOffset[c_tmp + 4] = Tinv[b_0 + 1];
        XDHOffset[6 * i_0 + 5] = R[3 * i_0 + 2];
        XDHOffset[c_tmp + 5] = Tinv[b_0 + 2];
      }

      ID_mtimes(XDHOffset, S_data, S_size, Si_data, Si_size);
      for (b_0 = 0; b_0 < 6; b_0++) {
        y[b_0] = 0.0;
      }

      c = Si_size[1];
      for (c_tmp = 0; c_tmp < c; c_tmp++) {
        coffset = c_tmp * 6 - 1;
        for (b_0 = 0; b_0 <= 4; b_0 += 2) {
          tmp_4 = _mm_loadu_pd(&Si_data[(coffset + b_0) + 1]);
          tmp_3 = _mm_loadu_pd(&y[b_0]);
          _mm_storeu_pd(&y[b_0], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(0.0)),
            tmp_3));
        }
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        vJ_data[i_0 + 6 * b_tmp] = y[i_0];
      }
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      R[3 * i_0] = T[i_0];
      R[3 * i_0 + 1] = T[i_0 + 4];
      R[3 * i_0 + 2] = T[i_0 + 8];
    }

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    for (i_0 = 0; i_0 <= 6; i_0 += 2) {
      tmp_4 = _mm_loadu_pd(&R[i_0]);
      _mm_storeu_pd(&R_0[i_0], _mm_mul_pd(tmp_4, _mm_set1_pd(-1.0)));
    }

    for (i_0 = 8; i_0 < 9; i_0++) {
      R_0[i_0] = -R[i_0];
    }

    s = T[13];
    b_idx_0 = T[12];
    b_idx_1 = T[14];
    for (i_0 = 0; i_0 < 3; i_0++) {
      loop_ub = i_0 << 2;
      Tinv[loop_ub] = R[3 * i_0];
      Tinv[loop_ub + 1] = R[3 * i_0 + 1];
      Tinv[loop_ub + 2] = R[3 * i_0 + 2];
      Tinv[i_0 + 12] = (R_0[i_0 + 3] * s + R_0[i_0] * b_idx_0) + R_0[i_0 + 6] *
        b_idx_1;
    }

    Tinv[3] = 0.0;
    Tinv[7] = 0.0;
    Tinv[11] = 0.0;
    Tinv[15] = 1.0;

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    tmp_2[3] = -Tinv[14];
    tmp_2[6] = Tinv[13];
    tmp_2[1] = Tinv[14];
    tmp_2[7] = -Tinv[12];
    tmp_2[2] = -Tinv[13];
    tmp_2[5] = Tinv[12];
    for (i_0 = 0; i_0 < 3; i_0++) {
      s = tmp_2[i_0 + 3];
      b_idx_0 = tmp_2[i_0];
      b_idx_1 = tmp_2[i_0 + 6];
      for (coffset = 0; coffset < 3; coffset++) {
        loop_ub = coffset << 2;
        tmp_0[i_0 + 3 * coffset] = (Tinv[loop_ub + 1] * s + Tinv[loop_ub] *
          b_idx_0) + Tinv[loop_ub + 2] * b_idx_1;
        ID_B.X_data[b_tmp].f1[coffset + 6 * i_0] = Tinv[(i_0 << 2) + coffset];
        ID_B.X_data[b_tmp].f1[coffset + 6 * (i_0 + 3)] = 0.0;
      }
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      ID_B.X_data[b_tmp].f1[6 * i_0 + 3] = tmp_0[3 * i_0];
      coffset = i_0 << 2;
      loop_ub = (i_0 + 3) * 6;
      ID_B.X_data[b_tmp].f1[loop_ub + 3] = Tinv[coffset];
      ID_B.X_data[b_tmp].f1[6 * i_0 + 4] = tmp_0[3 * i_0 + 1];
      ID_B.X_data[b_tmp].f1[loop_ub + 4] = Tinv[coffset + 1];
      ID_B.X_data[b_tmp].f1[6 * i_0 + 5] = tmp_0[3 * i_0 + 2];
      ID_B.X_data[b_tmp].f1[loop_ub + 5] = Tinv[coffset + 2];
    }

    pid = ID_DW.obj.TreeInternal.Bodies[b_tmp]->ParentIndex;
    if (pid > 0.0) {
      for (i_0 = 0; i_0 < 6; i_0++) {
        s = 0.0;
        for (coffset = 0; coffset < 6; coffset++) {
          s += vB_data[((int32_T)pid - 1) * 6 + coffset] * ID_B.X_data[b_tmp]
            .f1[6 * coffset + i_0];
        }

        y[i_0] = vJ_data[6 * b_tmp + i_0] + s;
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        vB_data[i_0 + 6 * b_tmp] = y[i_0];
      }

      ID_mtimes(XDHOffset, S_data, S_size, Si_data, Si_size);
      for (b_0 = 0; b_0 < 6; b_0++) {
        y[b_0] = 0.0;
      }

      c = Si_size[1];
      for (c_tmp = 0; c_tmp < c; c_tmp++) {
        coffset = c_tmp * 6 - 1;
        for (b_0 = 0; b_0 <= 4; b_0 += 2) {
          tmp_4 = _mm_loadu_pd(&Si_data[(coffset + b_0) + 1]);
          tmp_3 = _mm_loadu_pd(&y[b_0]);
          _mm_storeu_pd(&y[b_0], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(0.0)),
            tmp_3));
        }
      }

      R[0] = 0.0;
      i = vB_data[6 * b_tmp + 2];
      R[3] = -i;
      s = vB_data[6 * b_tmp + 1];
      R[6] = s;
      R[1] = i;
      R[4] = 0.0;
      i = vB_data[6 * b_tmp];
      R[7] = -i;
      R[2] = -s;
      R[5] = i;
      R[8] = 0.0;
      Fi_data[3] = 0.0;
      i = vB_data[6 * b_tmp + 5];
      Fi_data[9] = -i;
      s = vB_data[6 * b_tmp + 4];
      Fi_data[15] = s;
      Fi_data[4] = i;
      Fi_data[10] = 0.0;
      i = vB_data[6 * b_tmp + 3];
      Fi_data[16] = -i;
      Fi_data[5] = -s;
      Fi_data[11] = i;
      Fi_data[17] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        i = R[3 * i_0];
        Fi_data[6 * i_0] = i;
        b_0 = (i_0 + 3) * 6;
        Fi_data[b_0] = 0.0;
        Fi_data[b_0 + 3] = i;
        i = R[3 * i_0 + 1];
        Fi_data[6 * i_0 + 1] = i;
        Fi_data[b_0 + 1] = 0.0;
        Fi_data[b_0 + 4] = i;
        i = R[3 * i_0 + 2];
        Fi_data[6 * i_0 + 2] = i;
        Fi_data[b_0 + 2] = 0.0;
        Fi_data[b_0 + 5] = i;
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        s = 0.0;
        i = 0.0;
        for (coffset = 0; coffset < 6; coffset++) {
          loop_ub = 6 * coffset + i_0;
          _mm_storeu_pd(&tmp_5[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
            (Fi_data[loop_ub], ID_B.X_data[b_tmp].f1[loop_ub]), _mm_set_pd
            (vJ_data[coffset + 6 * b_tmp], aB_data[coffset + 6 * ((int32_T)pid -
            1)])), _mm_set_pd(i, s)));
          s = tmp_5[0];
          i = tmp_5[1];
        }

        R_1[i_0] = i;
        X[i_0] = s + y[i_0];
      }

      for (i_0 = 0; i_0 <= 4; i_0 += 2) {
        tmp_4 = _mm_loadu_pd(&X[i_0]);
        tmp_3 = _mm_loadu_pd(&R_1[i_0]);
        _mm_storeu_pd(&aB_data[i_0 + 6 * b_tmp], _mm_add_pd(tmp_4, tmp_3));
      }

      tmp_0[0] = 0.0;
      tmp_0[3] = -T[14];
      tmp_0[6] = T[13];
      tmp_0[1] = T[14];
      tmp_0[4] = 0.0;
      tmp_0[7] = -T[12];
      tmp_0[2] = -T[13];
      tmp_0[5] = T[12];
      tmp_0[8] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        s = tmp_0[i_0 + 3];
        b_idx_0 = tmp_0[i_0];
        b_idx_1 = tmp_0[i_0 + 6];
        for (coffset = 0; coffset < 3; coffset++) {
          loop_ub = coffset << 2;
          R[i_0 + 3 * coffset] = (T[loop_ub + 1] * s + T[loop_ub] * b_idx_0) +
            T[loop_ub + 2] * b_idx_1;
          XDHOffset[coffset + 6 * i_0] = T[(i_0 << 2) + coffset];
          XDHOffset[coffset + 6 * (i_0 + 3)] = 0.0;
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        XDHOffset[6 * i_0 + 3] = R[3 * i_0];
        b_0 = i_0 << 2;
        c_tmp = (i_0 + 3) * 6;
        XDHOffset[c_tmp + 3] = T[b_0];
        XDHOffset[6 * i_0 + 4] = R[3 * i_0 + 1];
        XDHOffset[c_tmp + 4] = T[b_0 + 1];
        XDHOffset[6 * i_0 + 5] = R[3 * i_0 + 2];
        XDHOffset[c_tmp + 5] = T[b_0 + 2];
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        for (coffset = 0; coffset < 6; coffset++) {
          i = 0.0;
          for (loop_ub = 0; loop_ub < 6; loop_ub++) {
            i += ID_B.Ic_data[(int32_T)pid - 1].f1[6 * loop_ub + i_0] *
              XDHOffset[6 * coffset + loop_ub];
          }

          Fi_data[i_0 + 6 * coffset] = i;
        }
      }

      memcpy(&ID_B.Ic_data[b_tmp].f1[0], &Fi_data[0], 36U * sizeof(real_T));
    } else {
      ID_mtimes(XDHOffset, S_data, S_size, Si_data, Si_size);
      for (b_0 = 0; b_0 < 6; b_0++) {
        i_0 = 6 * b_tmp + b_0;
        vB_data[i_0] = vJ_data[i_0];
        y[b_0] = 0.0;
      }

      c = Si_size[1];
      for (c_tmp = 0; c_tmp < c; c_tmp++) {
        coffset = c_tmp * 6 - 1;
        for (b_0 = 0; b_0 <= 4; b_0 += 2) {
          tmp_4 = _mm_loadu_pd(&Si_data[(coffset + b_0) + 1]);
          tmp_3 = _mm_loadu_pd(&y[b_0]);
          _mm_storeu_pd(&y[b_0], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(0.0)),
            tmp_3));
        }
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        s = 0.0;
        for (coffset = 0; coffset < 6; coffset++) {
          s += ID_B.X_data[b_tmp].f1[6 * coffset + i_0] * a0[coffset];
        }

        aB_data[i_0 + 6 * b_tmp] = s + y[i_0];
      }

      tmp_0[0] = 0.0;
      tmp_0[3] = -T[14];
      tmp_0[6] = T[13];
      tmp_0[1] = T[14];
      tmp_0[4] = 0.0;
      tmp_0[7] = -T[12];
      tmp_0[2] = -T[13];
      tmp_0[5] = T[12];
      tmp_0[8] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        s = tmp_0[i_0 + 3];
        b_idx_0 = tmp_0[i_0];
        b_idx_1 = tmp_0[i_0 + 6];
        for (coffset = 0; coffset < 3; coffset++) {
          loop_ub = coffset << 2;
          R[i_0 + 3 * coffset] = (T[loop_ub + 1] * s + T[loop_ub] * b_idx_0) +
            T[loop_ub + 2] * b_idx_1;
          ID_B.Ic_data[b_tmp].f1[coffset + 6 * i_0] = T[(i_0 << 2) + coffset];
          ID_B.Ic_data[b_tmp].f1[coffset + 6 * (i_0 + 3)] = 0.0;
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        ID_B.Ic_data[b_tmp].f1[6 * i_0 + 3] = R[3 * i_0];
        coffset = i_0 << 2;
        loop_ub = (i_0 + 3) * 6;
        ID_B.Ic_data[b_tmp].f1[loop_ub + 3] = T[coffset];
        ID_B.Ic_data[b_tmp].f1[6 * i_0 + 4] = R[3 * i_0 + 1];
        ID_B.Ic_data[b_tmp].f1[loop_ub + 4] = T[coffset + 1];
        ID_B.Ic_data[b_tmp].f1[6 * i_0 + 5] = R[3 * i_0 + 2];
        ID_B.Ic_data[b_tmp].f1[loop_ub + 5] = T[coffset + 2];
      }
    }

    for (i_0 = 0; i_0 < 36; i_0++) {
      XDHOffset[i_0] = ID_DW.obj.TreeInternal.Bodies[b_tmp]->SpatialInertia[i_0];
    }

    R[0] = 0.0;

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    i = vB_data[6 * b_tmp + 2];
    R[3] = -i;

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    s = vB_data[6 * b_tmp + 1];
    R[6] = s;
    R[1] = i;
    R[4] = 0.0;

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    i = vB_data[6 * b_tmp];
    R[7] = -i;
    R[2] = -s;
    R[5] = i;
    R[8] = 0.0;

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    Fi_data[18] = 0.0;
    i = vB_data[6 * b_tmp + 5];
    Fi_data[24] = -i;
    s = vB_data[6 * b_tmp + 4];
    Fi_data[30] = s;
    Fi_data[19] = i;
    Fi_data[25] = 0.0;
    i = vB_data[6 * b_tmp + 3];
    Fi_data[31] = -i;
    Fi_data[20] = -s;
    Fi_data[26] = i;
    Fi_data[32] = 0.0;
    for (i_0 = 0; i_0 < 3; i_0++) {
      i = R[3 * i_0];
      Fi_data[6 * i_0] = i;
      Fi_data[6 * i_0 + 3] = 0.0;
      b_0 = (i_0 + 3) * 6;
      Fi_data[b_0 + 3] = i;
      i = R[3 * i_0 + 1];
      Fi_data[6 * i_0 + 1] = i;
      Fi_data[6 * i_0 + 4] = 0.0;
      Fi_data[b_0 + 4] = i;
      i = R[3 * i_0 + 2];
      Fi_data[6 * i_0 + 2] = i;
      Fi_data[6 * i_0 + 5] = 0.0;
      Fi_data[b_0 + 5] = i;
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      i = 0.0;
      pid = 0.0;
      for (coffset = 0; coffset < 6; coffset++) {
        b_0 = 6 * b_tmp + coffset;
        _mm_storeu_pd(&tmp_5[0], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(XDHOffset[6 *
          coffset + i_0]), _mm_set_pd(aB_data[b_0], vB_data[b_0])), _mm_set_pd
          (pid, i)));
        i = tmp_5[0];
        pid = tmp_5[1];
      }

      X[i_0] = pid;
      y[i_0] = i;
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      s = 0.0;
      i = 0.0;
      for (coffset = 0; coffset < 6; coffset++) {
        _mm_storeu_pd(&tmp_5[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
          (ID_B.Ic_data[b_tmp].f1[6 * i_0 + coffset], Fi_data[6 * coffset + i_0]),
          _mm_set_pd(0.0, y[coffset])), _mm_set_pd(s, i)));
        i = tmp_5[0];
        s = tmp_5[1];
      }

      f_data[i_0 + 6 * b_tmp] = (X[i_0] + i) - s;
    }
  }

  f = (int32_T)-((-1.0 - nb) + 1.0);
  for (b_0 = 0; b_0 < f; b_0++) {
    i = nb - (real_T)b_0;
    obj = ID_DW.obj.TreeInternal.Bodies[(int32_T)i - 1];
    pid = obj->JointInternal.TypeInternal.Length;
    for (i_0 = 0; i_0 < 200; i_0++) {
      obj_Vector[i_0] = obj->JointInternal.TypeInternal.Vector[i_0];
    }

    b_bool = false;
    if (pid < 1.0) {
      i_0 = 0;
    } else {
      i_0 = (int32_T)pid;
    }

    if (i_0 == 5) {
      b_tmp = 1;
      do {
        exitg1 = 0;
        if (b_tmp - 1 < 5) {
          if (obj_Vector[b_tmp - 1] != tmp_6[b_tmp - 1]) {
            exitg1 = 1;
          } else {
            b_tmp++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!b_bool) {
      for (i_0 = 0; i_0 < 16; i_0++) {
        Tinv[i_0] = obj->JointInternal.ChildToJointTransform[i_0];
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        R[3 * i_0] = Tinv[i_0];
        R[3 * i_0 + 1] = Tinv[i_0 + 4];
        R[3 * i_0 + 2] = Tinv[i_0 + 8];
      }

      for (i_0 = 0; i_0 <= 6; i_0 += 2) {
        tmp_4 = _mm_loadu_pd(&R[i_0]);
        _mm_storeu_pd(&R_0[i_0], _mm_mul_pd(tmp_4, _mm_set1_pd(-1.0)));
      }

      for (i_0 = 8; i_0 < 9; i_0++) {
        R_0[i_0] = -R[i_0];
      }

      pid = Tinv[13];
      s = Tinv[12];
      b_idx_0 = Tinv[14];
      for (i_0 = 0; i_0 < 3; i_0++) {
        loop_ub = i_0 << 2;
        Tinv[loop_ub] = R[3 * i_0];
        Tinv[loop_ub + 1] = R[3 * i_0 + 1];
        Tinv[loop_ub + 2] = R[3 * i_0 + 2];
        Tinv[i_0 + 12] = (R_0[i_0 + 3] * pid + R_0[i_0] * s) + R_0[i_0 + 6] *
          b_idx_0;
      }

      Tinv[3] = 0.0;
      Tinv[7] = 0.0;
      Tinv[11] = 0.0;
      Tinv[15] = 1.0;
      obj = ID_DW.obj.TreeInternal.Bodies[(int32_T)i - 1];
      rigidBodyJoint_get_MotionSubspa(&obj->JointInternal, Si_data, Si_size);
      tmp_0[0] = 0.0;
      tmp_0[3] = -Tinv[14];
      tmp_0[6] = Tinv[13];
      tmp_0[1] = Tinv[14];
      tmp_0[4] = 0.0;
      tmp_0[7] = -Tinv[12];
      tmp_0[2] = -Tinv[13];
      tmp_0[5] = Tinv[12];
      tmp_0[8] = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        s = tmp_0[i_0 + 3];
        b_idx_0 = tmp_0[i_0];
        b_idx_1 = tmp_0[i_0 + 6];
        for (coffset = 0; coffset < 3; coffset++) {
          loop_ub = coffset << 2;
          R[i_0 + 3 * coffset] = (Tinv[loop_ub + 1] * s + Tinv[loop_ub] *
            b_idx_0) + Tinv[loop_ub + 2] * b_idx_1;
          XDHOffset[coffset + 6 * i_0] = Tinv[(i_0 << 2) + coffset];
          XDHOffset[coffset + 6 * (i_0 + 3)] = 0.0;
        }
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        XDHOffset[6 * i_0 + 3] = R[3 * i_0];
        loop_ub = i_0 << 2;
        c = (i_0 + 3) * 6;
        XDHOffset[c + 3] = Tinv[loop_ub];
        XDHOffset[6 * i_0 + 4] = R[3 * i_0 + 1];
        XDHOffset[c + 4] = Tinv[loop_ub + 1];
        XDHOffset[6 * i_0 + 5] = R[3 * i_0 + 2];
        XDHOffset[c + 5] = Tinv[loop_ub + 2];
      }

      ID_mtimes(XDHOffset, Si_data, Si_size, S_data, S_size);
      c = S_size[1];
      for (b_tmp = 0; b_tmp < c; b_tmp++) {
        coffset = b_tmp * 6 - 1;
        s = 0.0;
        for (c_tmp = 0; c_tmp < 6; c_tmp++) {
          s += f_data[((int32_T)i - 1) * 6 + c_tmp] * S_data[(coffset + c_tmp) +
            1];
        }

        a0[b_tmp] = s;
      }

      pid = ID_DW.obj.TreeInternal.VelocityDoFMap[(int32_T)i - 1];
      s = ID_DW.obj.TreeInternal.VelocityDoFMap[(int32_T)i + 16];
      if (pid > s) {
        b_tmp = 0;
        c_tmp = 0;
      } else {
        b_tmp = (int32_T)pid - 1;
        c_tmp = (int32_T)s;
      }

      c_tmp -= b_tmp;
      for (i_0 = 0; i_0 < c_tmp; i_0++) {
        ID_Y.g[b_tmp + i_0] = a0[i_0];
      }
    }

    pid = ID_DW.obj.TreeInternal.Bodies[(int32_T)i - 1]->ParentIndex;
    if (pid > 0.0) {
      for (i_0 = 0; i_0 < 6; i_0++) {
        s = 0.0;
        for (coffset = 0; coffset < 6; coffset++) {
          s += f_data[((int32_T)i - 1) * 6 + coffset] * ID_B.X_data[(int32_T)i -
            1].f1[6 * i_0 + coffset];
        }

        a0[i_0] = f_data[((int32_T)pid - 1) * 6 + i_0] + s;
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        f_data[i_0 + 6 * ((int32_T)pid - 1)] = a0[i_0];
      }
    }
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(ID_M->rtwLogInfo, (&ID_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.001s, 0.0s] */
    if ((rtmGetTFinal(ID_M)!=-1) &&
        !((rtmGetTFinal(ID_M)-ID_M->Timing.taskTime0) > ID_M->Timing.taskTime0 *
          (DBL_EPSILON))) {
      rtmSetErrorStatus(ID_M, "Simulation finished");
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
  if (!(++ID_M->Timing.clockTick0)) {
    ++ID_M->Timing.clockTickH0;
  }

  ID_M->Timing.taskTime0 = ID_M->Timing.clockTick0 * ID_M->Timing.stepSize0 +
    ID_M->Timing.clockTickH0 * ID_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void ID_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)ID_M, 0,
                sizeof(RT_MODEL_ID_T));
  rtmSetTFinal(ID_M, -1);
  ID_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    ID_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(ID_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(ID_M->rtwLogInfo, (NULL));
    rtliSetLogT(ID_M->rtwLogInfo, "tout");
    rtliSetLogX(ID_M->rtwLogInfo, "");
    rtliSetLogXFinal(ID_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(ID_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(ID_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(ID_M->rtwLogInfo, 0);
    rtliSetLogDecimation(ID_M->rtwLogInfo, 1);
    rtliSetLogY(ID_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(ID_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(ID_M->rtwLogInfo, (NULL));
  }

  /* states (dwork) */
  (void) memset((void *)&ID_DW, 0,
                sizeof(DW_ID_T));

  /* external inputs */
  (void)memset(&ID_U, 0, sizeof(ExtU_ID_T));

  /* external outputs */
  (void)memset(&ID_Y, 0, sizeof(ExtY_ID_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(ID_M->rtwLogInfo, 0.0, rtmGetTFinal(ID_M),
    ID_M->Timing.stepSize0, (&rtmGetErrorStatus(ID_M)));

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

    emxInitStruct_robotics_slmanip_(&ID_DW.obj_g);

    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    for (i = 0; i < 34; i++) {
      ID_DW.obj_g.TreeInternal._pobj0[i].
        CollisionsInternal.matlabCodegenIsDeleted = true;
    }

    ID_DW.obj_g.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    for (i = 0; i < 34; i++) {
      ID_DW.obj_g.TreeInternal._pobj0[i].matlabCodegenIsDeleted = true;
    }

    ID_DW.obj_g.TreeInternal.Base.matlabCodegenIsDeleted = true;
    ID_DW.obj_g.TreeInternal.matlabCodegenIsDeleted = true;
    ID_DW.method_k = 7U;
    ID_DW.method_not_empty_j = true;
    ID_DW.state_k = 1144108930U;
    ID_DW.state_not_empty_g = true;
    ID_DW.state_o[0] = 362436069U;
    ID_DW.state_o[1] = 521288629U;
    ID_DW.state_not_empty_d = true;
    memcpy(&ID_DW.state_l[0], &tmp[0], 625U * sizeof(uint32_T));
    ID_DW.state_not_empty_p = true;
    ID_DW.obj_g.matlabCodegenIsDeleted = false;
    ID_DW.objisempty_h = true;
    ID_DW.obj_g.isInitialized = 1;
    ID_MassMatrixBlock_setupImpl(&ID_DW.obj_g);

    /* End of Start for MATLABSystem: '<S4>/MATLAB System' */
    emxInitStruct_robotics_slmani_h(&ID_DW.obj_h);

    /* Start for MATLABSystem: '<S5>/MATLAB System' */
    for (i = 0; i < 34; i++) {
      ID_DW.obj_h.TreeInternal._pobj0[i].
        CollisionsInternal.matlabCodegenIsDeleted = true;
    }

    ID_DW.obj_h.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    for (i = 0; i < 34; i++) {
      ID_DW.obj_h.TreeInternal._pobj0[i].matlabCodegenIsDeleted = true;
    }

    ID_DW.obj_h.TreeInternal.Base.matlabCodegenIsDeleted = true;
    ID_DW.obj_h.TreeInternal.matlabCodegenIsDeleted = true;
    ID_DW.method = 7U;
    ID_DW.method_not_empty = true;
    ID_DW.state = 1144108930U;
    ID_DW.state_not_empty_i = true;
    ID_DW.state_p[0] = 362436069U;
    ID_DW.state_p[1] = 521288629U;
    ID_DW.state_not_empty_l = true;
    memcpy(&ID_DW.state_m[0], &tmp[0], 625U * sizeof(uint32_T));
    ID_DW.state_not_empty = true;
    ID_DW.obj_h.matlabCodegenIsDeleted = false;
    ID_DW.objisempty = true;
    ID_DW.obj_h.isInitialized = 1;
    VelocityProductTorqueBlock_setu(&ID_DW.obj_h);

    /* End of Start for MATLABSystem: '<S5>/MATLAB System' */
    emxInitStruct_robotics_slman_hy(&ID_DW.obj);

    /* Start for MATLABSystem: '<S3>/MATLAB System' */
    for (i = 0; i < 34; i++) {
      ID_DW.obj.TreeInternal._pobj0[i].CollisionsInternal.matlabCodegenIsDeleted
        = true;
    }

    ID_DW.obj.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
    for (i = 0; i < 34; i++) {
      ID_DW.obj.TreeInternal._pobj0[i].matlabCodegenIsDeleted = true;
    }

    ID_DW.obj.TreeInternal.Base.matlabCodegenIsDeleted = true;
    ID_DW.obj.TreeInternal.matlabCodegenIsDeleted = true;
    ID_DW.method_h = 7U;
    ID_DW.method_not_empty_o = true;
    ID_DW.state_b = 1144108930U;
    ID_DW.state_not_empty_d1 = true;
    ID_DW.state_n[0] = 362436069U;
    ID_DW.state_n[1] = 521288629U;
    ID_DW.state_not_empty_k = true;
    memcpy(&ID_DW.state_d[0], &tmp[0], 625U * sizeof(uint32_T));
    ID_DW.state_not_empty_gu = true;
    ID_DW.obj.matlabCodegenIsDeleted = false;
    ID_DW.objisempty_n = true;
    ID_DW.obj.isInitialized = 1;
    ID_GravityTorqueBlock_setupImpl(&ID_DW.obj);

    /* End of Start for MATLABSystem: '<S3>/MATLAB System' */
  }
}

/* Model terminate function */
void ID_terminate(void)
{
  e_robotics_manip_internal_Rig_T *obj_0;
  f_robotics_manip_internal_Col_T obj;
  g_robotics_manip_internal_Col_T *obj_1;
  real_T b_0;
  int32_T b;
  int32_T b_i;
  int32_T c;

  /* Terminate for MATLABSystem: '<S4>/MATLAB System' */
  if (!ID_DW.obj_g.matlabCodegenIsDeleted) {
    ID_DW.obj_g.matlabCodegenIsDeleted = true;
  }

  if (!ID_DW.obj_g.TreeInternal.matlabCodegenIsDeleted) {
    ID_DW.obj_g.TreeInternal.matlabCodegenIsDeleted = true;
  }

  if (!ID_DW.obj_g.TreeInternal.Base.matlabCodegenIsDeleted) {
    ID_DW.obj_g.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }

  for (b = 0; b < 34; b++) {
    obj_0 = &ID_DW.obj_g.TreeInternal._pobj0[b];
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
    }
  }

  if (!ID_DW.obj_g.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted)
  {
    ID_DW.obj_g.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    b_0 = ID_DW.obj_g.TreeInternal.Base.CollisionsInternal.Size;
    b = (int32_T)b_0;
    for (b_i = 0; b_i < b; b_i++) {
      obj =
        ID_DW.obj_g.TreeInternal.Base.CollisionsInternal.CollisionGeometries->data
        [b_i];
      collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
      ID_DW.obj_g.TreeInternal.Base.CollisionsInternal.CollisionGeometries->
        data[b_i] = obj;
    }
  }

  for (b = 0; b < 34; b++) {
    obj_1 = &ID_DW.obj_g.TreeInternal._pobj0[b].CollisionsInternal;
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
  emxFreeStruct_robotics_slmanip_(&ID_DW.obj_g);

  /* Terminate for MATLABSystem: '<S5>/MATLAB System' */
  if (!ID_DW.obj_h.matlabCodegenIsDeleted) {
    ID_DW.obj_h.matlabCodegenIsDeleted = true;
  }

  if (!ID_DW.obj_h.TreeInternal.matlabCodegenIsDeleted) {
    ID_DW.obj_h.TreeInternal.matlabCodegenIsDeleted = true;
  }

  if (!ID_DW.obj_h.TreeInternal.Base.matlabCodegenIsDeleted) {
    ID_DW.obj_h.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }

  for (b = 0; b < 34; b++) {
    obj_0 = &ID_DW.obj_h.TreeInternal._pobj0[b];
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
    }
  }

  if (!ID_DW.obj_h.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted)
  {
    ID_DW.obj_h.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted =
      true;
    b_0 = ID_DW.obj_h.TreeInternal.Base.CollisionsInternal.Size;
    b = (int32_T)b_0;
    for (b_i = 0; b_i < b; b_i++) {
      obj =
        ID_DW.obj_h.TreeInternal.Base.CollisionsInternal.CollisionGeometries->data
        [b_i];
      collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
      ID_DW.obj_h.TreeInternal.Base.CollisionsInternal.CollisionGeometries->
        data[b_i] = obj;
    }
  }

  for (b = 0; b < 34; b++) {
    obj_1 = &ID_DW.obj_h.TreeInternal._pobj0[b].CollisionsInternal;
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
  emxFreeStruct_robotics_slmani_h(&ID_DW.obj_h);

  /* Terminate for MATLABSystem: '<S3>/MATLAB System' */
  if (!ID_DW.obj.matlabCodegenIsDeleted) {
    ID_DW.obj.matlabCodegenIsDeleted = true;
  }

  if (!ID_DW.obj.TreeInternal.matlabCodegenIsDeleted) {
    ID_DW.obj.TreeInternal.matlabCodegenIsDeleted = true;
  }

  if (!ID_DW.obj.TreeInternal.Base.matlabCodegenIsDeleted) {
    ID_DW.obj.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }

  for (b = 0; b < 34; b++) {
    obj_0 = &ID_DW.obj.TreeInternal._pobj0[b];
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
    }
  }

  if (!ID_DW.obj.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted) {
    ID_DW.obj.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
    b_0 = ID_DW.obj.TreeInternal.Base.CollisionsInternal.Size;
    b = (int32_T)b_0;
    for (b_i = 0; b_i < b; b_i++) {
      obj =
        ID_DW.obj.TreeInternal.Base.CollisionsInternal.CollisionGeometries->
        data[b_i];
      collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
      ID_DW.obj.TreeInternal.Base.CollisionsInternal.CollisionGeometries->
        data[b_i] = obj;
    }
  }

  for (b = 0; b < 34; b++) {
    obj_1 = &ID_DW.obj.TreeInternal._pobj0[b].CollisionsInternal;
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
  emxFreeStruct_robotics_slman_hy(&ID_DW.obj);
}
