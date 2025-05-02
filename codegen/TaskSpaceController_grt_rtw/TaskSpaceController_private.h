/*
 * TaskSpaceController_private.h
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

#ifndef TaskSpaceController_private_h_
#define TaskSpaceController_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "TaskSpaceController_types.h"
#include "TaskSpaceController.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetTFinal
#define rtmSetTFinal(rtm, val)         ((rtm)->Timing.tFinal = (val))
#endif

extern real_T rt_hypotd_snf(real_T u0, real_T u1);

/* Exported functions */
extern void TaskSpaceController_dlog6(const real_T lambda[6], real_T dlog[36]);
extern int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);

#endif                                 /* TaskSpaceController_private_h_ */
