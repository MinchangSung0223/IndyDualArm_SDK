/*
 * TaskSpaceController_types.h
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

#ifndef TaskSpaceController_types_h_
#define TaskSpaceController_types_h_
#include "rtwtypes.h"

/* Custom Type definition for MATLAB Function: '<S1>/SPDControllerWithQP_Right1' */
#ifndef struct_tag_smNINkioqq1a7FyOE4CETSB
#define struct_tag_smNINkioqq1a7FyOE4CETSB

struct tag_smNINkioqq1a7FyOE4CETSB
{
  real_T xstar[7];
  real_T fstar;
  real_T firstorderopt;
  real_T lambda[25];
  int32_T state;
  real_T maxConstr;
  int32_T iterations;
  real_T searchDir[7];
};

#endif                                 /* struct_tag_smNINkioqq1a7FyOE4CETSB */

#ifndef typedef_smNINkioqq1a7FyOE4CETSB_TaskS_T
#define typedef_smNINkioqq1a7FyOE4CETSB_TaskS_T

typedef struct tag_smNINkioqq1a7FyOE4CETSB smNINkioqq1a7FyOE4CETSB_TaskS_T;

#endif                             /* typedef_smNINkioqq1a7FyOE4CETSB_TaskS_T */

#ifndef struct_tag_slzZ8M58FXlZqTD433BZJUH
#define struct_tag_slzZ8M58FXlZqTD433BZJUH

struct tag_slzZ8M58FXlZqTD433BZJUH
{
  real_T grad[7];
  real_T Hx[6];
  boolean_T hasLinear;
  int32_T nvar;
  int32_T maxVar;
  real_T beta;
  real_T rho;
  int32_T objtype;
  int32_T prev_objtype;
  int32_T prev_nvar;
  boolean_T prev_hasLinear;
  real_T gammaScalar;
};

#endif                                 /* struct_tag_slzZ8M58FXlZqTD433BZJUH */

#ifndef typedef_slzZ8M58FXlZqTD433BZJUH_TaskS_T
#define typedef_slzZ8M58FXlZqTD433BZJUH_TaskS_T

typedef struct tag_slzZ8M58FXlZqTD433BZJUH slzZ8M58FXlZqTD433BZJUH_TaskS_T;

#endif                             /* typedef_slzZ8M58FXlZqTD433BZJUH_TaskS_T */

#ifndef struct_tag_s8VdrbiRqBTaOPdh3e5fO1B
#define struct_tag_s8VdrbiRqBTaOPdh3e5fO1B

struct tag_s8VdrbiRqBTaOPdh3e5fO1B
{
  real_T FMat[49];
  int32_T ldm;
  int32_T ndims;
  int32_T info;
  real_T scaleFactor;
  boolean_T ConvexCheck;
  real_T regTol_;
  real_T workspace_[336];
  real_T workspace2_[336];
};

#endif                                 /* struct_tag_s8VdrbiRqBTaOPdh3e5fO1B */

#ifndef typedef_s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T
#define typedef_s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T

typedef struct tag_s8VdrbiRqBTaOPdh3e5fO1B s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T;

#endif                             /* typedef_s8VdrbiRqBTaOPdh3e5fO1B_TaskS_T */

#ifndef struct_tag_sIOJhD9KwAkF5sEguPjYquC
#define struct_tag_sIOJhD9KwAkF5sEguPjYquC

struct tag_sIOJhD9KwAkF5sEguPjYquC
{
  boolean_T RemainFeasible;
  int32_T MaxIterations;
  real_T ConstrRelTolFactor;
  real_T ProbRelTolFactor;
};

#endif                                 /* struct_tag_sIOJhD9KwAkF5sEguPjYquC */

#ifndef typedef_sIOJhD9KwAkF5sEguPjYquC_TaskS_T
#define typedef_sIOJhD9KwAkF5sEguPjYquC_TaskS_T

typedef struct tag_sIOJhD9KwAkF5sEguPjYquC sIOJhD9KwAkF5sEguPjYquC_TaskS_T;

#endif                             /* typedef_sIOJhD9KwAkF5sEguPjYquC_TaskS_T */

#ifndef struct_tag_skbvZoOR3lvVJ6HLoaviTXC
#define struct_tag_skbvZoOR3lvVJ6HLoaviTXC

struct tag_skbvZoOR3lvVJ6HLoaviTXC
{
  int32_T ldq;
  real_T QR[175];
  real_T Q[49];
  int32_T jpvt[25];
  int32_T mrows;
  int32_T ncols;
  real_T tau[7];
  int32_T minRowCol;
  boolean_T usedPivoting;
};

#endif                                 /* struct_tag_skbvZoOR3lvVJ6HLoaviTXC */

#ifndef typedef_skbvZoOR3lvVJ6HLoaviTXC_TaskS_T
#define typedef_skbvZoOR3lvVJ6HLoaviTXC_TaskS_T

typedef struct tag_skbvZoOR3lvVJ6HLoaviTXC skbvZoOR3lvVJ6HLoaviTXC_TaskS_T;

#endif                             /* typedef_skbvZoOR3lvVJ6HLoaviTXC_TaskS_T */

#ifndef struct_tag_sfFz4TA1WVIyRsxqhsR6OaD
#define struct_tag_sfFz4TA1WVIyRsxqhsR6OaD

struct tag_sfFz4TA1WVIyRsxqhsR6OaD
{
  real_T workspace_float[175];
  int32_T workspace_int[25];
  int32_T workspace_sort[25];
};

#endif                                 /* struct_tag_sfFz4TA1WVIyRsxqhsR6OaD */

#ifndef typedef_sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T
#define typedef_sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T

typedef struct tag_sfFz4TA1WVIyRsxqhsR6OaD sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T;

#endif                             /* typedef_sfFz4TA1WVIyRsxqhsR6OaD_TaskS_T */

#ifndef struct_tag_sVMzcbzaOHlwbaxNrLGVFM
#define struct_tag_sVMzcbzaOHlwbaxNrLGVFM

struct tag_sVMzcbzaOHlwbaxNrLGVFM
{
  real_T ineqlin[24];
  real_T lower[6];
  real_T upper[6];
};

#endif                                 /* struct_tag_sVMzcbzaOHlwbaxNrLGVFM */

#ifndef typedef_sVMzcbzaOHlwbaxNrLGVFM_TaskSp_T
#define typedef_sVMzcbzaOHlwbaxNrLGVFM_TaskSp_T

typedef struct tag_sVMzcbzaOHlwbaxNrLGVFM sVMzcbzaOHlwbaxNrLGVFM_TaskSp_T;

#endif                             /* typedef_sVMzcbzaOHlwbaxNrLGVFM_TaskSp_T */

#ifndef struct_tag_sAElXDmDj36R7Z42SImJxmG
#define struct_tag_sAElXDmDj36R7Z42SImJxmG

struct tag_sAElXDmDj36R7Z42SImJxmG
{
  int32_T mConstr;
  int32_T mConstrOrig;
  int32_T mConstrMax;
  int32_T nVar;
  int32_T nVarOrig;
  int32_T nVarMax;
  int32_T ldA;
  real_T Aineq[168];
  real_T bineq[24];
  real_T lb[7];
  real_T ub[7];
  int32_T indexLB[7];
  int32_T indexUB[7];
  int32_T indexFixed[7];
  int32_T mEqRemoved;
  real_T ATwset[175];
  real_T bwset[25];
  int32_T nActiveConstr;
  real_T maxConstrWorkspace[25];
  int32_T sizes[5];
  int32_T sizesNormal[5];
  int32_T sizesPhaseOne[5];
  int32_T sizesRegularized[5];
  int32_T sizesRegPhaseOne[5];
  int32_T isActiveIdx[6];
  int32_T isActiveIdxNormal[6];
  int32_T isActiveIdxPhaseOne[6];
  int32_T isActiveIdxRegularized[6];
  int32_T isActiveIdxRegPhaseOne[6];
  boolean_T isActiveConstr[25];
  int32_T Wid[25];
  int32_T Wlocalidx[25];
  int32_T nWConstr[5];
  int32_T probType;
  real_T SLACK0;
};

#endif                                 /* struct_tag_sAElXDmDj36R7Z42SImJxmG */

#ifndef typedef_sAElXDmDj36R7Z42SImJxmG_TaskS_T
#define typedef_sAElXDmDj36R7Z42SImJxmG_TaskS_T

typedef struct tag_sAElXDmDj36R7Z42SImJxmG sAElXDmDj36R7Z42SImJxmG_TaskS_T;

#endif                             /* typedef_sAElXDmDj36R7Z42SImJxmG_TaskS_T */

#ifndef SS_UINT64
#define SS_UINT64                      17
#endif

#ifndef SS_INT64
#define SS_INT64                       18
#endif

/* Parameters (default storage) */
typedef struct P_TaskSpaceController_T_ P_TaskSpaceController_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_TaskSpaceController_T RT_MODEL_TaskSpaceController_T;

#endif                                 /* TaskSpaceController_types_h_ */
