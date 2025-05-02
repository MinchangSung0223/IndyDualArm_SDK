/*
 * TSTraj_types.h
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

#ifndef TSTraj_types_h_
#define TSTraj_types_h_
#include "rtwtypes.h"
#ifndef struct_tag_siswYcTR8LLamuD4YWmtXHC
#define struct_tag_siswYcTR8LLamuD4YWmtXHC

struct tag_siswYcTR8LLamuD4YWmtXHC
{
  real_T breaks[6];
  real_T coefs[15];
};

#endif                                 /* struct_tag_siswYcTR8LLamuD4YWmtXHC */

#ifndef typedef_siswYcTR8LLamuD4YWmtXHC_TSTra_T
#define typedef_siswYcTR8LLamuD4YWmtXHC_TSTra_T

typedef struct tag_siswYcTR8LLamuD4YWmtXHC siswYcTR8LLamuD4YWmtXHC_TSTra_T;

#endif                             /* typedef_siswYcTR8LLamuD4YWmtXHC_TSTra_T */

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap_TSTraj_T
#define typedef_cell_wrap_TSTraj_T

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap_TSTraj_T;

#endif                                 /* typedef_cell_wrap_TSTraj_T */

#ifndef struct_tag_87XkqHTEbe5d4GkRtgoGPE
#define struct_tag_87XkqHTEbe5d4GkRtgoGPE

struct tag_87XkqHTEbe5d4GkRtgoGPE
{
  real_T f1[6];
  real_T f2;
};

#endif                                 /* struct_tag_87XkqHTEbe5d4GkRtgoGPE */

#ifndef typedef_cell_TSTraj_T
#define typedef_cell_TSTraj_T

typedef struct tag_87XkqHTEbe5d4GkRtgoGPE cell_TSTraj_T;

#endif                                 /* typedef_cell_TSTraj_T */

#ifndef struct_tag_lM6NXw4p4yHfSTt2CytjfF
#define struct_tag_lM6NXw4p4yHfSTt2CytjfF

struct tag_lM6NXw4p4yHfSTt2CytjfF
{
  real_T f1[4];
};

#endif                                 /* struct_tag_lM6NXw4p4yHfSTt2CytjfF */

#ifndef typedef_g_cell_wrap_TSTraj_T
#define typedef_g_cell_wrap_TSTraj_T

typedef struct tag_lM6NXw4p4yHfSTt2CytjfF g_cell_wrap_TSTraj_T;

#endif                                 /* typedef_g_cell_wrap_TSTraj_T */

#ifndef struct_tag_UhgjFElPoFSai6WN2VDfeH
#define struct_tag_UhgjFElPoFSai6WN2VDfeH

struct tag_UhgjFElPoFSai6WN2VDfeH
{
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  cell_wrap_TSTraj_T inputVarSize[3];
  real_T EndTime[2];
  siswYcTR8LLamuD4YWmtXHC_TSTra_T PPCell[3];
  siswYcTR8LLamuD4YWmtXHC_TSTra_T PPDCell[3];
  siswYcTR8LLamuD4YWmtXHC_TSTra_T PPDDCell[3];
  cell_TSTraj_T PrevOptInputs;
  boolean_T PPFormUpdatedNeeded;
};

#endif                                 /* struct_tag_UhgjFElPoFSai6WN2VDfeH */

#ifndef typedef_robotics_slcore_internal_bloc_T
#define typedef_robotics_slcore_internal_bloc_T

typedef struct tag_UhgjFElPoFSai6WN2VDfeH robotics_slcore_internal_bloc_T;

#endif                             /* typedef_robotics_slcore_internal_bloc_T */

#ifndef struct_emxArray_real_T_3x5x3
#define struct_emxArray_real_T_3x5x3

struct emxArray_real_T_3x5x3
{
  real_T data[45];
  int32_T size[3];
};

#endif                                 /* struct_emxArray_real_T_3x5x3 */

#ifndef typedef_emxArray_real_T_3x5x3_TSTraj_T
#define typedef_emxArray_real_T_3x5x3_TSTraj_T

typedef struct emxArray_real_T_3x5x3 emxArray_real_T_3x5x3_TSTraj_T;

#endif                              /* typedef_emxArray_real_T_3x5x3_TSTraj_T */

#ifndef struct_tag_vjEZ2dxatR8VOmLd9oOqoD
#define struct_tag_vjEZ2dxatR8VOmLd9oOqoD

struct tag_vjEZ2dxatR8VOmLd9oOqoD
{
  real_T breaks[6];
  emxArray_real_T_3x5x3_TSTraj_T coefs;
};

#endif                                 /* struct_tag_vjEZ2dxatR8VOmLd9oOqoD */

#ifndef typedef_s_vjEZ2dxatR8VOmLd9oOqoD_TSTr_T
#define typedef_s_vjEZ2dxatR8VOmLd9oOqoD_TSTr_T

typedef struct tag_vjEZ2dxatR8VOmLd9oOqoD s_vjEZ2dxatR8VOmLd9oOqoD_TSTr_T;

#endif                             /* typedef_s_vjEZ2dxatR8VOmLd9oOqoD_TSTr_T */

#ifndef struct_emxArray_real_T_9x3
#define struct_emxArray_real_T_9x3

struct emxArray_real_T_9x3
{
  real_T data[27];
  int32_T size[2];
};

#endif                                 /* struct_emxArray_real_T_9x3 */

#ifndef typedef_emxArray_real_T_9x3_TSTraj_T
#define typedef_emxArray_real_T_9x3_TSTraj_T

typedef struct emxArray_real_T_9x3 emxArray_real_T_9x3_TSTraj_T;

#endif                                /* typedef_emxArray_real_T_9x3_TSTraj_T */

#ifndef struct_tag_tWnNAbYywbfFPUcqC0QPPE
#define struct_tag_tWnNAbYywbfFPUcqC0QPPE

struct tag_tWnNAbYywbfFPUcqC0QPPE
{
  emxArray_real_T_9x3_TSTraj_T f1;
};

#endif                                 /* struct_tag_tWnNAbYywbfFPUcqC0QPPE */

#ifndef typedef_i_cell_wrap_TSTraj_T
#define typedef_i_cell_wrap_TSTraj_T

typedef struct tag_tWnNAbYywbfFPUcqC0QPPE i_cell_wrap_TSTraj_T;

#endif                                 /* typedef_i_cell_wrap_TSTraj_T */

#ifndef struct_emxArray_tag_vjEZ2dxatR8VOmLd9o
#define struct_emxArray_tag_vjEZ2dxatR8VOmLd9o

struct emxArray_tag_vjEZ2dxatR8VOmLd9o
{
  s_vjEZ2dxatR8VOmLd9oOqoD_TSTr_T data[3];
  int32_T size;
};

#endif                              /* struct_emxArray_tag_vjEZ2dxatR8VOmLd9o */

#ifndef typedef_emxArray_s_vjEZ2dxatR8VOmLd9o_T
#define typedef_emxArray_s_vjEZ2dxatR8VOmLd9o_T

typedef struct emxArray_tag_vjEZ2dxatR8VOmLd9o emxArray_s_vjEZ2dxatR8VOmLd9o_T;

#endif                             /* typedef_emxArray_s_vjEZ2dxatR8VOmLd9o_T */

#ifndef SS_UINT64
#define SS_UINT64                      18
#endif

#ifndef SS_INT64
#define SS_INT64                       19
#endif

/* Parameters (default storage) */
typedef struct P_TSTraj_T_ P_TSTraj_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_TSTraj_T RT_MODEL_TSTraj_T;

#endif                                 /* TSTraj_types_h_ */
