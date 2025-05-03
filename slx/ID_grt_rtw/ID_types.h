/*
 * ID_types.h
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

#ifndef ID_types_h_
#define ID_types_h_
#include "rtwtypes.h"
#ifndef struct_tag_3BSkcDv2LER5BCV49FILBD
#define struct_tag_3BSkcDv2LER5BCV49FILBD

struct tag_3BSkcDv2LER5BCV49FILBD
{
  void *CollisionPrimitive;
};

#endif                                 /* struct_tag_3BSkcDv2LER5BCV49FILBD */

#ifndef typedef_f_robotics_manip_internal_Col_T
#define typedef_f_robotics_manip_internal_Col_T

typedef struct tag_3BSkcDv2LER5BCV49FILBD f_robotics_manip_internal_Col_T;

#endif                             /* typedef_f_robotics_manip_internal_Col_T */

#ifndef struct_tag_xcgxZ5He9ABYYFkMb0gw9F
#define struct_tag_xcgxZ5He9ABYYFkMb0gw9F

struct tag_xcgxZ5He9ABYYFkMb0gw9F
{
  real_T f1[36];
};

#endif                                 /* struct_tag_xcgxZ5He9ABYYFkMb0gw9F */

#ifndef typedef_h_cell_wrap_ID_T
#define typedef_h_cell_wrap_ID_T

typedef struct tag_xcgxZ5He9ABYYFkMb0gw9F h_cell_wrap_ID_T;

#endif                                 /* typedef_h_cell_wrap_ID_T */

#ifndef struct_tag_4JsrFHKaAew6w7N5GrPwAE
#define struct_tag_4JsrFHKaAew6w7N5GrPwAE

struct tag_4JsrFHKaAew6w7N5GrPwAE
{
  real_T Length;
  char_T Vector[200];
};

#endif                                 /* struct_tag_4JsrFHKaAew6w7N5GrPwAE */

#ifndef typedef_e_robotics_manip_internal_Cha_T
#define typedef_e_robotics_manip_internal_Cha_T

typedef struct tag_4JsrFHKaAew6w7N5GrPwAE e_robotics_manip_internal_Cha_T;

#endif                             /* typedef_e_robotics_manip_internal_Cha_T */

#ifndef struct_tag_0wAA6tLoej0zPuTCXXY0HG
#define struct_tag_0wAA6tLoej0zPuTCXXY0HG

struct tag_0wAA6tLoej0zPuTCXXY0HG
{
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  e_robotics_manip_internal_Cha_T NameInternal;
  real_T JointAxisInternal[3];
  real_T MotionSubspaceInternal[36];
  e_robotics_manip_internal_Cha_T TypeInternal;
};

#endif                                 /* struct_tag_0wAA6tLoej0zPuTCXXY0HG */

#ifndef typedef_b_rigidBodyJoint_ID_T
#define typedef_b_rigidBodyJoint_ID_T

typedef struct tag_0wAA6tLoej0zPuTCXXY0HG b_rigidBodyJoint_ID_T;

#endif                                 /* typedef_b_rigidBodyJoint_ID_T */

#ifndef struct_tag_xcgxZ5He9ABYYFkMb0gw9F
#define struct_tag_xcgxZ5He9ABYYFkMb0gw9F

struct tag_xcgxZ5He9ABYYFkMb0gw9F
{
  real_T f1[36];
};

#endif                                 /* struct_tag_xcgxZ5He9ABYYFkMb0gw9F */

#ifndef typedef_g_cell_wrap_ID_T
#define typedef_g_cell_wrap_ID_T

typedef struct tag_xcgxZ5He9ABYYFkMb0gw9F g_cell_wrap_ID_T;

#endif                                 /* typedef_g_cell_wrap_ID_T */

#ifndef struct_emxArray_tag_3BSkcDv2LER5BCV49F
#define struct_emxArray_tag_3BSkcDv2LER5BCV49F

struct emxArray_tag_3BSkcDv2LER5BCV49F
{
  f_robotics_manip_internal_Col_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_tag_3BSkcDv2LER5BCV49F */

#ifndef typedef_emxArray_f_robotics_manip_int_T
#define typedef_emxArray_f_robotics_manip_int_T

typedef struct emxArray_tag_3BSkcDv2LER5BCV49F emxArray_f_robotics_manip_int_T;

#endif                             /* typedef_emxArray_f_robotics_manip_int_T */

#ifndef struct_tag_u8YYD3p7O6nMC3eMQHUgpF
#define struct_tag_u8YYD3p7O6nMC3eMQHUgpF

struct tag_u8YYD3p7O6nMC3eMQHUgpF
{
  boolean_T matlabCodegenIsDeleted;
  emxArray_f_robotics_manip_int_T *CollisionGeometries;
  real_T MaxElements;
  real_T Size;
};

#endif                                 /* struct_tag_u8YYD3p7O6nMC3eMQHUgpF */

#ifndef typedef_g_robotics_manip_internal_Col_T
#define typedef_g_robotics_manip_internal_Col_T

typedef struct tag_u8YYD3p7O6nMC3eMQHUgpF g_robotics_manip_internal_Col_T;

#endif                             /* typedef_g_robotics_manip_internal_Col_T */

#ifndef struct_tag_Uq7zxijFmQhcuf59duCKAE
#define struct_tag_Uq7zxijFmQhcuf59duCKAE

struct tag_Uq7zxijFmQhcuf59duCKAE
{
  boolean_T matlabCodegenIsDeleted;
  e_robotics_manip_internal_Cha_T NameInternal;
  real_T Index;
  b_rigidBodyJoint_ID_T JointInternal;
  real_T ParentIndex;
  real_T SpatialInertia[36];
  g_robotics_manip_internal_Col_T CollisionsInternal;
};

#endif                                 /* struct_tag_Uq7zxijFmQhcuf59duCKAE */

#ifndef typedef_e_robotics_manip_internal_Rig_T
#define typedef_e_robotics_manip_internal_Rig_T

typedef struct tag_Uq7zxijFmQhcuf59duCKAE e_robotics_manip_internal_Rig_T;

#endif                             /* typedef_e_robotics_manip_internal_Rig_T */

#ifndef struct_tag_rhHPGZD6ZaGJWYQyRTnWzC
#define struct_tag_rhHPGZD6ZaGJWYQyRTnWzC

struct tag_rhHPGZD6ZaGJWYQyRTnWzC
{
  boolean_T matlabCodegenIsDeleted;
  real_T NumBodies;
  e_robotics_manip_internal_Rig_T Base;
  real_T Gravity[3];
  e_robotics_manip_internal_Rig_T *Bodies[17];
  real_T PositionDoFMap[34];
  real_T VelocityDoFMap[34];
  e_robotics_manip_internal_Rig_T _pobj0[34];
};

#endif                                 /* struct_tag_rhHPGZD6ZaGJWYQyRTnWzC */

#ifndef typedef_f_robotics_manip_internal_Rig_T
#define typedef_f_robotics_manip_internal_Rig_T

typedef struct tag_rhHPGZD6ZaGJWYQyRTnWzC f_robotics_manip_internal_Rig_T;

#endif                             /* typedef_f_robotics_manip_internal_Rig_T */

#ifndef struct_tag_wE54Rb3mXCGTSMJ5xORubC
#define struct_tag_wE54Rb3mXCGTSMJ5xORubC

struct tag_wE54Rb3mXCGTSMJ5xORubC
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  f_robotics_manip_internal_Rig_T TreeInternal;
};

#endif                                 /* struct_tag_wE54Rb3mXCGTSMJ5xORubC */

#ifndef typedef_robotics_slmanip_internal_blo_T
#define typedef_robotics_slmanip_internal_blo_T

typedef struct tag_wE54Rb3mXCGTSMJ5xORubC robotics_slmanip_internal_blo_T;

#endif                             /* typedef_robotics_slmanip_internal_blo_T */

#ifndef struct_tag_bkWCCCuV0NgMFyRNsVzifC
#define struct_tag_bkWCCCuV0NgMFyRNsVzifC

struct tag_bkWCCCuV0NgMFyRNsVzifC
{
  boolean_T matlabCodegenIsDeleted;
  real_T NumBodies;
  e_robotics_manip_internal_Rig_T Base;
  e_robotics_manip_internal_Rig_T *Bodies[17];
  real_T VelocityNumber;
  real_T PositionDoFMap[34];
  real_T VelocityDoFMap[34];
  e_robotics_manip_internal_Rig_T _pobj0[34];
};

#endif                                 /* struct_tag_bkWCCCuV0NgMFyRNsVzifC */

#ifndef typedef_f_robotics_manip_internal_R_h_T
#define typedef_f_robotics_manip_internal_R_h_T

typedef struct tag_bkWCCCuV0NgMFyRNsVzifC f_robotics_manip_internal_R_h_T;

#endif                             /* typedef_f_robotics_manip_internal_R_h_T */

#ifndef struct_tag_Gghd2cQxHm3F8cWuMy0zxE
#define struct_tag_Gghd2cQxHm3F8cWuMy0zxE

struct tag_Gghd2cQxHm3F8cWuMy0zxE
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  f_robotics_manip_internal_R_h_T TreeInternal;
};

#endif                                 /* struct_tag_Gghd2cQxHm3F8cWuMy0zxE */

#ifndef typedef_robotics_slmanip_internal_b_h_T
#define typedef_robotics_slmanip_internal_b_h_T

typedef struct tag_Gghd2cQxHm3F8cWuMy0zxE robotics_slmanip_internal_b_h_T;

#endif                             /* typedef_robotics_slmanip_internal_b_h_T */

#ifndef struct_tag_mlepuvkvPcxlj5FLTOS07E
#define struct_tag_mlepuvkvPcxlj5FLTOS07E

struct tag_mlepuvkvPcxlj5FLTOS07E
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  f_robotics_manip_internal_Rig_T TreeInternal;
};

#endif                                 /* struct_tag_mlepuvkvPcxlj5FLTOS07E */

#ifndef typedef_robotics_slmanip_internal__hy_T
#define typedef_robotics_slmanip_internal__hy_T

typedef struct tag_mlepuvkvPcxlj5FLTOS07E robotics_slmanip_internal__hy_T;

#endif                             /* typedef_robotics_slmanip_internal__hy_T */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_real_T */

#ifndef typedef_emxArray_real_T_ID_T
#define typedef_emxArray_real_T_ID_T

typedef struct emxArray_real_T emxArray_real_T_ID_T;

#endif                                 /* typedef_emxArray_real_T_ID_T */

#ifndef SS_UINT64
#define SS_UINT64                      20
#endif

#ifndef SS_INT64
#define SS_INT64                       21
#endif

/* Parameters (default storage) */
typedef struct P_ID_T_ P_ID_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_ID_T RT_MODEL_ID_T;

#endif                                 /* ID_types_h_ */
