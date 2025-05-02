/*
 * FK_types.h
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

#ifndef FK_types_h_
#define FK_types_h_
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

#ifndef struct_tag_I7lxy6BEal0s7MBxygd9JE
#define struct_tag_I7lxy6BEal0s7MBxygd9JE

struct tag_I7lxy6BEal0s7MBxygd9JE
{
  real_T f1[16];
};

#endif                                 /* struct_tag_I7lxy6BEal0s7MBxygd9JE */

#ifndef typedef_h_cell_wrap_FK_T
#define typedef_h_cell_wrap_FK_T

typedef struct tag_I7lxy6BEal0s7MBxygd9JE h_cell_wrap_FK_T;

#endif                                 /* typedef_h_cell_wrap_FK_T */

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

#ifndef typedef_b_rigidBodyJoint_FK_T
#define typedef_b_rigidBodyJoint_FK_T

typedef struct tag_0wAA6tLoej0zPuTCXXY0HG b_rigidBodyJoint_FK_T;

#endif                                 /* typedef_b_rigidBodyJoint_FK_T */

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

#ifndef struct_tag_VOkVPjfhR8byCI7PpQxV9E
#define struct_tag_VOkVPjfhR8byCI7PpQxV9E

struct tag_VOkVPjfhR8byCI7PpQxV9E
{
  boolean_T matlabCodegenIsDeleted;
  e_robotics_manip_internal_Cha_T NameInternal;
  real_T Index;
  b_rigidBodyJoint_FK_T JointInternal;
  real_T ParentIndex;
  g_robotics_manip_internal_Col_T CollisionsInternal;
};

#endif                                 /* struct_tag_VOkVPjfhR8byCI7PpQxV9E */

#ifndef typedef_e_robotics_manip_internal_Rig_T
#define typedef_e_robotics_manip_internal_Rig_T

typedef struct tag_VOkVPjfhR8byCI7PpQxV9E e_robotics_manip_internal_Rig_T;

#endif                             /* typedef_e_robotics_manip_internal_Rig_T */

#ifndef struct_tag_L2MDPNoTamdRXKDDHP0xaH
#define struct_tag_L2MDPNoTamdRXKDDHP0xaH

struct tag_L2MDPNoTamdRXKDDHP0xaH
{
  boolean_T matlabCodegenIsDeleted;
  real_T NumBodies;
  e_robotics_manip_internal_Rig_T Base;
  e_robotics_manip_internal_Rig_T *Bodies[17];
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T VelocityDoFMap[34];
  e_robotics_manip_internal_Rig_T _pobj0[34];
};

#endif                                 /* struct_tag_L2MDPNoTamdRXKDDHP0xaH */

#ifndef typedef_f_robotics_manip_internal_Rig_T
#define typedef_f_robotics_manip_internal_Rig_T

typedef struct tag_L2MDPNoTamdRXKDDHP0xaH f_robotics_manip_internal_Rig_T;

#endif                             /* typedef_f_robotics_manip_internal_Rig_T */

#ifndef struct_tag_3LUMkUSxq0NL5mVHw19yqB
#define struct_tag_3LUMkUSxq0NL5mVHw19yqB

struct tag_3LUMkUSxq0NL5mVHw19yqB
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  f_robotics_manip_internal_Rig_T TreeInternal;
};

#endif                                 /* struct_tag_3LUMkUSxq0NL5mVHw19yqB */

#ifndef typedef_robotics_slmanip_internal_blo_T
#define typedef_robotics_slmanip_internal_blo_T

typedef struct tag_3LUMkUSxq0NL5mVHw19yqB robotics_slmanip_internal_blo_T;

#endif                             /* typedef_robotics_slmanip_internal_blo_T */

#ifndef struct_tag_SmqPnyswO7ChBsixKhsQpC
#define struct_tag_SmqPnyswO7ChBsixKhsQpC

struct tag_SmqPnyswO7ChBsixKhsQpC
{
  boolean_T matlabCodegenIsDeleted;
  e_robotics_manip_internal_Cha_T NameInternal;
  b_rigidBodyJoint_FK_T JointInternal;
  real_T ParentIndex;
  g_robotics_manip_internal_Col_T CollisionsInternal;
};

#endif                                 /* struct_tag_SmqPnyswO7ChBsixKhsQpC */

#ifndef typedef_e_robotics_manip_internal_R_m_T
#define typedef_e_robotics_manip_internal_R_m_T

typedef struct tag_SmqPnyswO7ChBsixKhsQpC e_robotics_manip_internal_R_m_T;

#endif                             /* typedef_e_robotics_manip_internal_R_m_T */

#ifndef struct_tag_bcy3WD8sKa68JdFp5dTEuH
#define struct_tag_bcy3WD8sKa68JdFp5dTEuH

struct tag_bcy3WD8sKa68JdFp5dTEuH
{
  boolean_T matlabCodegenIsDeleted;
  real_T NumBodies;
  e_robotics_manip_internal_R_m_T Base;
  e_robotics_manip_internal_R_m_T *Bodies[17];
  real_T PositionNumber;
  e_robotics_manip_internal_R_m_T _pobj0[34];
};

#endif                                 /* struct_tag_bcy3WD8sKa68JdFp5dTEuH */

#ifndef typedef_f_robotics_manip_internal_R_m_T
#define typedef_f_robotics_manip_internal_R_m_T

typedef struct tag_bcy3WD8sKa68JdFp5dTEuH f_robotics_manip_internal_R_m_T;

#endif                             /* typedef_f_robotics_manip_internal_R_m_T */

#ifndef struct_tag_zRS0uzD2zyzpgGW9N5OMG
#define struct_tag_zRS0uzD2zyzpgGW9N5OMG

struct tag_zRS0uzD2zyzpgGW9N5OMG
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  f_robotics_manip_internal_R_m_T TreeInternal;
};

#endif                                 /* struct_tag_zRS0uzD2zyzpgGW9N5OMG */

#ifndef typedef_robotics_slmanip_internal_b_m_T
#define typedef_robotics_slmanip_internal_b_m_T

typedef struct tag_zRS0uzD2zyzpgGW9N5OMG robotics_slmanip_internal_b_m_T;

#endif                             /* typedef_robotics_slmanip_internal_b_m_T */

#ifndef struct_emxArray_tag_I7lxy6BEal0s7MBxyg
#define struct_emxArray_tag_I7lxy6BEal0s7MBxyg

struct emxArray_tag_I7lxy6BEal0s7MBxyg
{
  h_cell_wrap_FK_T data[17];
  int32_T size[2];
};

#endif                              /* struct_emxArray_tag_I7lxy6BEal0s7MBxyg */

#ifndef typedef_emxArray_h_cell_wrap_1x17_FK_T
#define typedef_emxArray_h_cell_wrap_1x17_FK_T

typedef struct emxArray_tag_I7lxy6BEal0s7MBxyg emxArray_h_cell_wrap_1x17_FK_T;

#endif                              /* typedef_emxArray_h_cell_wrap_1x17_FK_T */

#ifndef SS_UINT64
#define SS_UINT64                      19
#endif

#ifndef SS_INT64
#define SS_INT64                       20
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_FK_T RT_MODEL_FK_T;

#endif                                 /* FK_types_h_ */
