/*
 * TSTraj_private.h
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

#ifndef TSTraj_private_h_
#define TSTraj_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "TSTraj.h"
#include "TSTraj_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetTFinal
#define rtmSetTFinal(rtm, val)         ((rtm)->Timing.tFinal = (val))
#endif

extern real_T rt_powd_snf(real_T u0, real_T u1);
extern void TSTraj_TransToRp(const real_T rtu_T[16], real_T rty_p[3],
  B_TransToRp_TSTraj_T *localB);
extern void TrapezoidalVelocityProfile_Init(DW_TrapezoidalVelocityProfile_T
  *localDW);
extern void TrapezoidalVelocityProfileTraje(real_T rtu_0, const real_T rtu_1[6],
  real_T rtu_2, B_TrapezoidalVelocityProfileT_T *localB,
  DW_TrapezoidalVelocityProfile_T *localDW);

#endif                                 /* TSTraj_private_h_ */
