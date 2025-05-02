/*
 * JSTraj_types.h
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

#ifndef JSTraj_types_h_
#define JSTraj_types_h_
#include "rtwtypes.h"
#ifndef struct_tag_sLAdmPYwCtLCgYiuKLhpEMH
#define struct_tag_sLAdmPYwCtLCgYiuKLhpEMH

struct tag_sLAdmPYwCtLCgYiuKLhpEMH
{
  real_T breaks[4];
  real_T coefs[108];
};

#endif                                 /* struct_tag_sLAdmPYwCtLCgYiuKLhpEMH */

#ifndef typedef_sLAdmPYwCtLCgYiuKLhpEMH_JSTra_T
#define typedef_sLAdmPYwCtLCgYiuKLhpEMH_JSTra_T

typedef struct tag_sLAdmPYwCtLCgYiuKLhpEMH sLAdmPYwCtLCgYiuKLhpEMH_JSTra_T;

#endif                             /* typedef_sLAdmPYwCtLCgYiuKLhpEMH_JSTra_T */

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap_JSTraj_T
#define typedef_cell_wrap_JSTraj_T

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap_JSTraj_T;

#endif                                 /* typedef_cell_wrap_JSTraj_T */

#ifndef struct_tag_s0zqNb4bmef035xQw2UioF
#define struct_tag_s0zqNb4bmef035xQw2UioF

struct tag_s0zqNb4bmef035xQw2UioF
{
  real_T f1[12];
  real_T f2[2];
  real_T f3[12];
  real_T f4[12];
};

#endif                                 /* struct_tag_s0zqNb4bmef035xQw2UioF */

#ifndef typedef_cell_JSTraj_T
#define typedef_cell_JSTraj_T

typedef struct tag_s0zqNb4bmef035xQw2UioF cell_JSTraj_T;

#endif                                 /* typedef_cell_JSTraj_T */

#ifndef struct_tag_8PvMgPLhVvZBPeKCSAdTe
#define struct_tag_8PvMgPLhVvZBPeKCSAdTe

struct tag_8PvMgPLhVvZBPeKCSAdTe
{
  real_T f1[12];
};

#endif                                 /* struct_tag_8PvMgPLhVvZBPeKCSAdTe */

#ifndef typedef_e_cell_wrap_JSTraj_T
#define typedef_e_cell_wrap_JSTraj_T

typedef struct tag_8PvMgPLhVvZBPeKCSAdTe e_cell_wrap_JSTraj_T;

#endif                                 /* typedef_e_cell_wrap_JSTraj_T */

#ifndef struct_tag_WmaIKEbNO2sryoA1ikz3KF
#define struct_tag_WmaIKEbNO2sryoA1ikz3KF

struct tag_WmaIKEbNO2sryoA1ikz3KF
{
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  cell_wrap_JSTraj_T inputVarSize[5];
  sLAdmPYwCtLCgYiuKLhpEMH_JSTra_T PPStruct;
  sLAdmPYwCtLCgYiuKLhpEMH_JSTra_T PPDStruct;
  sLAdmPYwCtLCgYiuKLhpEMH_JSTra_T PPDDStruct;
  cell_JSTraj_T PrevOptInputs;
  boolean_T PPFormUpdatedNeeded;
};

#endif                                 /* struct_tag_WmaIKEbNO2sryoA1ikz3KF */

#ifndef typedef_robotics_slcore_internal_bloc_T
#define typedef_robotics_slcore_internal_bloc_T

typedef struct tag_WmaIKEbNO2sryoA1ikz3KF robotics_slcore_internal_bloc_T;

#endif                             /* typedef_robotics_slcore_internal_bloc_T */

#ifndef SS_UINT64
#define SS_UINT64                      18
#endif

#ifndef SS_INT64
#define SS_INT64                       19
#endif

/* Parameters (default storage) */
typedef struct P_JSTraj_T_ P_JSTraj_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_JSTraj_T RT_MODEL_JSTraj_T;

#endif                                 /* JSTraj_types_h_ */
