#ifndef __BJyqJeM9MN8kuyMGX65V0G_h__
#define __BJyqJeM9MN8kuyMGX65V0G_h__

/* Include files */
#include "simstruc.h"
#include "rtwtypes.h"
#include "multiword_types.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_simstruct_bridge.h"
#include "sl_sfcn_cov/sl_sfcn_cov_bridge.h"

/* Type Definitions */
#ifndef struct_tag_sGaAmWJmK5HvPUvd2k3PkCG
#define struct_tag_sGaAmWJmK5HvPUvd2k3PkCG

struct tag_sGaAmWJmK5HvPUvd2k3PkCG
{
  uint32_T nanflag;
  uint32_T ComparisonMethod;
};

#endif                                 /* struct_tag_sGaAmWJmK5HvPUvd2k3PkCG */

#ifndef typedef_sGaAmWJmK5HvPUvd2k3PkCG
#define typedef_sGaAmWJmK5HvPUvd2k3PkCG

typedef struct tag_sGaAmWJmK5HvPUvd2k3PkCG sGaAmWJmK5HvPUvd2k3PkCG;

#endif                                 /* typedef_sGaAmWJmK5HvPUvd2k3PkCG */

#ifndef struct_tag_sJfZvjZdYbIqz8zWAkASFBH
#define struct_tag_sJfZvjZdYbIqz8zWAkASFBH

struct tag_sJfZvjZdYbIqz8zWAkASFBH
{
  void *GeometryInternal;
};

#endif                                 /* struct_tag_sJfZvjZdYbIqz8zWAkASFBH */

#ifndef typedef_sJfZvjZdYbIqz8zWAkASFBH
#define typedef_sJfZvjZdYbIqz8zWAkASFBH

typedef struct tag_sJfZvjZdYbIqz8zWAkASFBH sJfZvjZdYbIqz8zWAkASFBH;

#endif                                 /* typedef_sJfZvjZdYbIqz8zWAkASFBH */

#ifndef struct_tag_yXNlxAMtkozv4BiPFKvBVC
#define struct_tag_yXNlxAMtkozv4BiPFKvBVC

struct tag_yXNlxAMtkozv4BiPFKvBVC
{
  boolean_T f1;
};

#endif                                 /* struct_tag_yXNlxAMtkozv4BiPFKvBVC */

#ifndef typedef_s_yXNlxAMtkozv4BiPFKvBVC
#define typedef_s_yXNlxAMtkozv4BiPFKvBVC

typedef struct tag_yXNlxAMtkozv4BiPFKvBVC s_yXNlxAMtkozv4BiPFKvBVC;

#endif                                 /* typedef_s_yXNlxAMtkozv4BiPFKvBVC */

#ifndef struct_tag_IQ1YOeTj6FAOoyzcDr5CJD
#define struct_tag_IQ1YOeTj6FAOoyzcDr5CJD

struct tag_IQ1YOeTj6FAOoyzcDr5CJD
{
  int32_T f1;
  int32_T f2;
  int32_T f3;
};

#endif                                 /* struct_tag_IQ1YOeTj6FAOoyzcDr5CJD */

#ifndef typedef_s_IQ1YOeTj6FAOoyzcDr5CJD
#define typedef_s_IQ1YOeTj6FAOoyzcDr5CJD

typedef struct tag_IQ1YOeTj6FAOoyzcDr5CJD s_IQ1YOeTj6FAOoyzcDr5CJD;

#endif                                 /* typedef_s_IQ1YOeTj6FAOoyzcDr5CJD */

#ifndef struct_tag_IXZbk4aPjQFR6fO0q1hmvH
#define struct_tag_IXZbk4aPjQFR6fO0q1hmvH

struct tag_IXZbk4aPjQFR6fO0q1hmvH
{
  int32_T __dummy;
};

#endif                                 /* struct_tag_IXZbk4aPjQFR6fO0q1hmvH */

#ifndef typedef_rtString_3
#define typedef_rtString_3

typedef struct tag_IXZbk4aPjQFR6fO0q1hmvH rtString_3;

#endif                                 /* typedef_rtString_3 */

#ifndef struct_tag_3BSkcDv2LER5BCV49FILBD
#define struct_tag_3BSkcDv2LER5BCV49FILBD

struct tag_3BSkcDv2LER5BCV49FILBD
{
  void *CollisionPrimitive;
};

#endif                                 /* struct_tag_3BSkcDv2LER5BCV49FILBD */

#ifndef typedef_robotics_manip_internal_CollisionGeometry
#define typedef_robotics_manip_internal_CollisionGeometry

typedef struct tag_3BSkcDv2LER5BCV49FILBD
  robotics_manip_internal_CollisionGeometry;

#endif                                 /* typedef_robotics_manip_internal_CollisionGeometry */

#ifndef struct_coder_array_tag_3BSkcDv2LER5BCV
#define struct_coder_array_tag_3BSkcDv2LER5BCV

struct coder_array_tag_3BSkcDv2LER5BCV
{
  struct {
    robotics_manip_internal_CollisionGeometry *data;
    int32_T numel;
    int32_T allocated;
    boolean_T owner;
  } vector;

  int32_T size[2];
};

#endif                                 /* struct_coder_array_tag_3BSkcDv2LER5BCV */

#ifndef typedef_coder_array_robotics_manip_inte
#define typedef_coder_array_robotics_manip_inte

typedef struct coder_array_tag_3BSkcDv2LER5BCV coder_array_robotics_manip_inte;

#endif                                 /* typedef_coder_array_robotics_manip_inte */

#ifndef struct_tag_u8YYD3p7O6nMC3eMQHUgpF
#define struct_tag_u8YYD3p7O6nMC3eMQHUgpF

struct tag_u8YYD3p7O6nMC3eMQHUgpF
{
  boolean_T matlabCodegenIsDeleted;
  coder_array_robotics_manip_inte CollisionGeometries;
  real_T MaxElements;
  real_T Size;
};

#endif                                 /* struct_tag_u8YYD3p7O6nMC3eMQHUgpF */

#ifndef typedef_robotics_manip_internal_CollisionSet
#define typedef_robotics_manip_internal_CollisionSet

typedef struct tag_u8YYD3p7O6nMC3eMQHUgpF robotics_manip_internal_CollisionSet;

#endif                                 /* typedef_robotics_manip_internal_CollisionSet */

#ifndef struct_coder_array_real_T_2D
#define struct_coder_array_real_T_2D

struct coder_array_real_T_2D
{
  struct {
    real_T *data;
    int32_T numel;
    int32_T allocated;
    boolean_T owner;
  } vector;

  int32_T size[2];
};

#endif                                 /* struct_coder_array_real_T_2D */

#ifndef typedef_coder_array_real_T_2D
#define typedef_coder_array_real_T_2D

typedef struct coder_array_real_T_2D coder_array_real_T_2D;

#endif                                 /* typedef_coder_array_real_T_2D */

#ifndef struct_tag_DuJTIxHyPH0sh2f3wcGFk
#define struct_tag_DuJTIxHyPH0sh2f3wcGFk

struct tag_DuJTIxHyPH0sh2f3wcGFk
{
  s_yXNlxAMtkozv4BiPFKvBVC _data;
};

#endif                                 /* struct_tag_DuJTIxHyPH0sh2f3wcGFk */

#ifndef typedef_s_DuJTIxHyPH0sh2f3wcGFk
#define typedef_s_DuJTIxHyPH0sh2f3wcGFk

typedef struct tag_DuJTIxHyPH0sh2f3wcGFk s_DuJTIxHyPH0sh2f3wcGFk;

#endif                                 /* typedef_s_DuJTIxHyPH0sh2f3wcGFk */

#ifndef struct_tag_Dt6t2l6MSLYBenrysWZNFC
#define struct_tag_Dt6t2l6MSLYBenrysWZNFC

struct tag_Dt6t2l6MSLYBenrysWZNFC
{
  s_IQ1YOeTj6FAOoyzcDr5CJD _data;
};

#endif                                 /* struct_tag_Dt6t2l6MSLYBenrysWZNFC */

#ifndef typedef_s_Dt6t2l6MSLYBenrysWZNFC
#define typedef_s_Dt6t2l6MSLYBenrysWZNFC

typedef struct tag_Dt6t2l6MSLYBenrysWZNFC s_Dt6t2l6MSLYBenrysWZNFC;

#endif                                 /* typedef_s_Dt6t2l6MSLYBenrysWZNFC */

#ifndef struct_tag_4JsrFHKaAew6w7N5GrPwAE
#define struct_tag_4JsrFHKaAew6w7N5GrPwAE

struct tag_4JsrFHKaAew6w7N5GrPwAE
{
  real_T Length;
  char_T Vector[200];
};

#endif                                 /* struct_tag_4JsrFHKaAew6w7N5GrPwAE */

#ifndef typedef_robotics_manip_internal_CharacterVector
#define typedef_robotics_manip_internal_CharacterVector

typedef struct tag_4JsrFHKaAew6w7N5GrPwAE
  robotics_manip_internal_CharacterVector;

#endif                                 /* typedef_robotics_manip_internal_CharacterVector */

#ifndef struct_tag_0wAA6tLoej0zPuTCXXY0HG
#define struct_tag_0wAA6tLoej0zPuTCXXY0HG

struct tag_0wAA6tLoej0zPuTCXXY0HG
{
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  robotics_manip_internal_CharacterVector NameInternal;
  real_T JointAxisInternal[3];
  real_T MotionSubspaceInternal[36];
  robotics_manip_internal_CharacterVector TypeInternal;
};

#endif                                 /* struct_tag_0wAA6tLoej0zPuTCXXY0HG */

#ifndef typedef_rigidBodyJoint
#define typedef_rigidBodyJoint

typedef struct tag_0wAA6tLoej0zPuTCXXY0HG rigidBodyJoint;

#endif                                 /* typedef_rigidBodyJoint */

#ifndef struct_tag_Uq7zxijFmQhcuf59duCKAE
#define struct_tag_Uq7zxijFmQhcuf59duCKAE

struct tag_Uq7zxijFmQhcuf59duCKAE
{
  boolean_T matlabCodegenIsDeleted;
  robotics_manip_internal_CharacterVector NameInternal;
  real_T Index;
  rigidBodyJoint JointInternal;
  real_T ParentIndex;
  real_T SpatialInertia[36];
  robotics_manip_internal_CollisionSet CollisionsInternal;
};

#endif                                 /* struct_tag_Uq7zxijFmQhcuf59duCKAE */

#ifndef typedef_robotics_manip_internal_RigidBody
#define typedef_robotics_manip_internal_RigidBody

typedef struct tag_Uq7zxijFmQhcuf59duCKAE robotics_manip_internal_RigidBody;

#endif                                 /* typedef_robotics_manip_internal_RigidBody */

#ifndef struct_tag_bkWCCCuV0NgMFyRNsVzifC
#define struct_tag_bkWCCCuV0NgMFyRNsVzifC

struct tag_bkWCCCuV0NgMFyRNsVzifC
{
  boolean_T matlabCodegenIsDeleted;
  real_T NumBodies;
  robotics_manip_internal_RigidBody Base;
  robotics_manip_internal_RigidBody *Bodies[17];
  real_T VelocityNumber;
  real_T PositionDoFMap[34];
  real_T VelocityDoFMap[34];
  robotics_manip_internal_RigidBody _pobj0[34];
};

#endif                                 /* struct_tag_bkWCCCuV0NgMFyRNsVzifC */

#ifndef typedef_robotics_manip_internal_RigidBodyTree
#define typedef_robotics_manip_internal_RigidBodyTree

typedef struct tag_bkWCCCuV0NgMFyRNsVzifC robotics_manip_internal_RigidBodyTree;

#endif                                 /* typedef_robotics_manip_internal_RigidBodyTree */

#ifndef struct_tag_Gghd2cQxHm3F8cWuMy0zxE
#define struct_tag_Gghd2cQxHm3F8cWuMy0zxE

struct tag_Gghd2cQxHm3F8cWuMy0zxE
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  robotics_manip_internal_RigidBodyTree TreeInternal;
};

#endif                                 /* struct_tag_Gghd2cQxHm3F8cWuMy0zxE */

#ifndef typedef_robotics_slmanip_internal_block_MassMatrixBlock
#define typedef_robotics_slmanip_internal_block_MassMatrixBlock

typedef struct tag_Gghd2cQxHm3F8cWuMy0zxE
  robotics_slmanip_internal_block_MassMatrixBlock;

#endif                                 /* typedef_robotics_slmanip_internal_block_MassMatrixBlock */

#ifndef struct_tag_sCGDRIJsTfVBdZWrIuhZvwG
#define struct_tag_sCGDRIJsTfVBdZWrIuhZvwG

struct tag_sCGDRIJsTfVBdZWrIuhZvwG
{
  uint8_T Type;
  real_T NameLength;
  uint8_T Name[12];
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T MotionSubspace[36];
  real_T JointAxis[3];
  real_T PositionLimits[14];
  real_T HomePosition[7];
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
};

#endif                                 /* struct_tag_sCGDRIJsTfVBdZWrIuhZvwG */

#ifndef typedef_sCGDRIJsTfVBdZWrIuhZvwG
#define typedef_sCGDRIJsTfVBdZWrIuhZvwG

typedef struct tag_sCGDRIJsTfVBdZWrIuhZvwG sCGDRIJsTfVBdZWrIuhZvwG;

#endif                                 /* typedef_sCGDRIJsTfVBdZWrIuhZvwG */

#ifndef struct_tag_smzGQHcQ1fZcSCW5rtLpn4F
#define struct_tag_smzGQHcQ1fZcSCW5rtLpn4F

struct tag_smzGQHcQ1fZcSCW5rtLpn4F
{
  boolean_T CaseSensitivity;
  char_T PartialMatching[6];
  boolean_T StructExpand;
  boolean_T IgnoreNulls;
  boolean_T SupportOverrides;
};

#endif                                 /* struct_tag_smzGQHcQ1fZcSCW5rtLpn4F */

#ifndef typedef_smzGQHcQ1fZcSCW5rtLpn4F
#define typedef_smzGQHcQ1fZcSCW5rtLpn4F

typedef struct tag_smzGQHcQ1fZcSCW5rtLpn4F smzGQHcQ1fZcSCW5rtLpn4F;

#endif                                 /* typedef_smzGQHcQ1fZcSCW5rtLpn4F */

#ifndef struct_tag_xcgxZ5He9ABYYFkMb0gw9F
#define struct_tag_xcgxZ5He9ABYYFkMb0gw9F

struct tag_xcgxZ5He9ABYYFkMb0gw9F
{
  real_T f1[36];
};

#endif                                 /* struct_tag_xcgxZ5He9ABYYFkMb0gw9F */

#ifndef typedef_cell_wrap
#define typedef_cell_wrap

typedef struct tag_xcgxZ5He9ABYYFkMb0gw9F cell_wrap;

#endif                                 /* typedef_cell_wrap */

#ifndef struct_tag_Mq55LRTpx3F49SLl4MgJ3G
#define struct_tag_Mq55LRTpx3F49SLl4MgJ3G

struct tag_Mq55LRTpx3F49SLl4MgJ3G
{
  real_T f1[2];
};

#endif                                 /* struct_tag_Mq55LRTpx3F49SLl4MgJ3G */

#ifndef typedef_b_cell_wrap
#define typedef_b_cell_wrap

typedef struct tag_Mq55LRTpx3F49SLl4MgJ3G b_cell_wrap;

#endif                                 /* typedef_b_cell_wrap */

#ifndef struct_tag_Hm6EPE5ncyWd4RGCxxctzF
#define struct_tag_Hm6EPE5ncyWd4RGCxxctzF

struct tag_Hm6EPE5ncyWd4RGCxxctzF
{
  char_T f1[30];
  char_T f2[29];
  char_T f3[30];
  char_T f4[30];
  char_T f5[39];
};

#endif                                 /* struct_tag_Hm6EPE5ncyWd4RGCxxctzF */

#ifndef typedef_cell
#define typedef_cell

typedef struct tag_Hm6EPE5ncyWd4RGCxxctzF cell;

#endif                                 /* typedef_cell */

#ifndef struct_tag_ge1UD3YqHcNerzgtJ4AjXF
#define struct_tag_ge1UD3YqHcNerzgtJ4AjXF

struct tag_ge1UD3YqHcNerzgtJ4AjXF
{
  char_T f1[6];
};

#endif                                 /* struct_tag_ge1UD3YqHcNerzgtJ4AjXF */

#ifndef typedef_c_cell_wrap
#define typedef_c_cell_wrap

typedef struct tag_ge1UD3YqHcNerzgtJ4AjXF c_cell_wrap;

#endif                                 /* typedef_c_cell_wrap */

#ifndef struct_tag_n5V6NPhGd1s3l2q7z9eOR
#define struct_tag_n5V6NPhGd1s3l2q7z9eOR

struct tag_n5V6NPhGd1s3l2q7z9eOR
{
  char_T f1[27];
  char_T f2[21];
  char_T f3[26];
};

#endif                                 /* struct_tag_n5V6NPhGd1s3l2q7z9eOR */

#ifndef typedef_b_cell
#define typedef_b_cell

typedef struct tag_n5V6NPhGd1s3l2q7z9eOR b_cell;

#endif                                 /* typedef_b_cell */

#ifndef struct_tag_VqK0uXiUXuliBvHmvcLR0F
#define struct_tag_VqK0uXiUXuliBvHmvcLR0F

struct tag_VqK0uXiUXuliBvHmvcLR0F
{
  char_T f1[28];
  char_T f2[22];
  char_T f3[35];
  char_T f4[26];
  char_T f5[36];
  char_T f6[39];
};

#endif                                 /* struct_tag_VqK0uXiUXuliBvHmvcLR0F */

#ifndef typedef_c_cell
#define typedef_c_cell

typedef struct tag_VqK0uXiUXuliBvHmvcLR0F c_cell;

#endif                                 /* typedef_c_cell */

#ifndef struct_tag_L5JvjW1A13FyCQi5N783sB
#define struct_tag_L5JvjW1A13FyCQi5N783sB

struct tag_L5JvjW1A13FyCQi5N783sB
{
  char_T f1[7];
};

#endif                                 /* struct_tag_L5JvjW1A13FyCQi5N783sB */

#ifndef typedef_d_cell_wrap
#define typedef_d_cell_wrap

typedef struct tag_L5JvjW1A13FyCQi5N783sB d_cell_wrap;

#endif                                 /* typedef_d_cell_wrap */

#ifndef struct_tag_6jR4RtbHdjyG00WYqgD5nF
#define struct_tag_6jR4RtbHdjyG00WYqgD5nF

struct tag_6jR4RtbHdjyG00WYqgD5nF
{
  char_T f1[16];
};

#endif                                 /* struct_tag_6jR4RtbHdjyG00WYqgD5nF */

#ifndef typedef_e_cell_wrap
#define typedef_e_cell_wrap

typedef struct tag_6jR4RtbHdjyG00WYqgD5nF e_cell_wrap;

#endif                                 /* typedef_e_cell_wrap */

#ifndef struct_tag_njgfiHhWBCqqqpWsKZxr7F
#define struct_tag_njgfiHhWBCqqqpWsKZxr7F

struct tag_njgfiHhWBCqqqpWsKZxr7F
{
  char_T f1[15];
  char_T f2[15];
  char_T f3[12];
  char_T f4[11];
  char_T f5[16];
};

#endif                                 /* struct_tag_njgfiHhWBCqqqpWsKZxr7F */

#ifndef typedef_d_cell
#define typedef_d_cell

typedef struct tag_njgfiHhWBCqqqpWsKZxr7F d_cell;

#endif                                 /* typedef_d_cell */

#ifndef struct_tag_FDrX8kOEjZXLXru8nW4swE
#define struct_tag_FDrX8kOEjZXLXru8nW4swE

struct tag_FDrX8kOEjZXLXru8nW4swE
{
  char_T f1[6];
  char_T f2[6];
};

#endif                                 /* struct_tag_FDrX8kOEjZXLXru8nW4swE */

#ifndef typedef_s_FDrX8kOEjZXLXru8nW4swE
#define typedef_s_FDrX8kOEjZXLXru8nW4swE

typedef struct tag_FDrX8kOEjZXLXru8nW4swE s_FDrX8kOEjZXLXru8nW4swE;

#endif                                 /* typedef_s_FDrX8kOEjZXLXru8nW4swE */

#ifndef struct_tag_cf27YhgaPej9AKdSy0N78E
#define struct_tag_cf27YhgaPej9AKdSy0N78E

struct tag_cf27YhgaPej9AKdSy0N78E
{
  char_T f1[3];
  char_T f2[6];
  char_T f3[6];
};

#endif                                 /* struct_tag_cf27YhgaPej9AKdSy0N78E */

#ifndef typedef_e_cell
#define typedef_e_cell

typedef struct tag_cf27YhgaPej9AKdSy0N78E e_cell;

#endif                                 /* typedef_e_cell */

#ifndef struct_tag_hYQNMGF5jxYpBJJQpnmQ1
#define struct_tag_hYQNMGF5jxYpBJJQpnmQ1

struct tag_hYQNMGF5jxYpBJJQpnmQ1
{
  char_T f1[6];
  char_T f2[6];
  char_T f3[3];
};

#endif                                 /* struct_tag_hYQNMGF5jxYpBJJQpnmQ1 */

#ifndef typedef_f_cell
#define typedef_f_cell

typedef struct tag_hYQNMGF5jxYpBJJQpnmQ1 f_cell;

#endif                                 /* typedef_f_cell */

#ifndef struct_tag_2KJQP2Gjk1Awf4eK8vqQsF
#define struct_tag_2KJQP2Gjk1Awf4eK8vqQsF

struct tag_2KJQP2Gjk1Awf4eK8vqQsF
{
  real_T Length;
  char_T Vector[200];
};

#endif                                 /* struct_tag_2KJQP2Gjk1Awf4eK8vqQsF */

#ifndef typedef_b_robotics_manip_internal_CharacterVector
#define typedef_b_robotics_manip_internal_CharacterVector

typedef struct tag_2KJQP2Gjk1Awf4eK8vqQsF
  b_robotics_manip_internal_CharacterVector;

#endif                                 /* typedef_b_robotics_manip_internal_CharacterVector */

#ifndef struct_tag_uwJsGEKtvfiUxcdf0z0AYH
#define struct_tag_uwJsGEKtvfiUxcdf0z0AYH

struct tag_uwJsGEKtvfiUxcdf0z0AYH
{
  char_T f1[4];
};

#endif                                 /* struct_tag_uwJsGEKtvfiUxcdf0z0AYH */

#ifndef typedef_f_cell_wrap
#define typedef_f_cell_wrap

typedef struct tag_uwJsGEKtvfiUxcdf0z0AYH f_cell_wrap;

#endif                                 /* typedef_f_cell_wrap */

#ifndef struct_tag_GBcoDtdkRpt9uNeOkOvtsD
#define struct_tag_GBcoDtdkRpt9uNeOkOvtsD

struct tag_GBcoDtdkRpt9uNeOkOvtsD
{
  char_T f1[8];
  char_T f2[3];
};

#endif                                 /* struct_tag_GBcoDtdkRpt9uNeOkOvtsD */

#ifndef typedef_g_cell
#define typedef_g_cell

typedef struct tag_GBcoDtdkRpt9uNeOkOvtsD g_cell;

#endif                                 /* typedef_g_cell */

#ifndef struct_tag_7rClNlx2n452lpFDKKJsJH
#define struct_tag_7rClNlx2n452lpFDKKJsJH

struct tag_7rClNlx2n452lpFDKKJsJH
{
  char_T f1[4];
  char_T f2[6];
};

#endif                                 /* struct_tag_7rClNlx2n452lpFDKKJsJH */

#ifndef typedef_h_cell
#define typedef_h_cell

typedef struct tag_7rClNlx2n452lpFDKKJsJH h_cell;

#endif                                 /* typedef_h_cell */

#ifndef struct_tag_tz3atwCHJZXtFLBnil52jC
#define struct_tag_tz3atwCHJZXtFLBnil52jC

struct tag_tz3atwCHJZXtFLBnil52jC
{
  char_T f1[8];
  char_T f2[9];
  char_T f3[5];
  char_T f4[8];
};

#endif                                 /* struct_tag_tz3atwCHJZXtFLBnil52jC */

#ifndef typedef_i_cell
#define typedef_i_cell

typedef struct tag_tz3atwCHJZXtFLBnil52jC i_cell;

#endif                                 /* typedef_i_cell */

#ifndef struct_tag_SvAQu5Z41uhzWczF5Op4iF
#define struct_tag_SvAQu5Z41uhzWczF5Op4iF

struct tag_SvAQu5Z41uhzWczF5Op4iF
{
  char_T Value[6];
};

#endif                                 /* struct_tag_SvAQu5Z41uhzWczF5Op4iF */

#ifndef typedef_s_SvAQu5Z41uhzWczF5Op4iF
#define typedef_s_SvAQu5Z41uhzWczF5Op4iF

typedef struct tag_SvAQu5Z41uhzWczF5Op4iF s_SvAQu5Z41uhzWczF5Op4iF;

#endif                                 /* typedef_s_SvAQu5Z41uhzWczF5Op4iF */

#ifndef struct_tag_UxnduFshK9s0lv0uvl2XIG
#define struct_tag_UxnduFshK9s0lv0uvl2XIG

struct tag_UxnduFshK9s0lv0uvl2XIG
{
  char_T Value[20];
};

#endif                                 /* struct_tag_UxnduFshK9s0lv0uvl2XIG */

#ifndef typedef_s_UxnduFshK9s0lv0uvl2XIG
#define typedef_s_UxnduFshK9s0lv0uvl2XIG

typedef struct tag_UxnduFshK9s0lv0uvl2XIG s_UxnduFshK9s0lv0uvl2XIG;

#endif                                 /* typedef_s_UxnduFshK9s0lv0uvl2XIG */

#ifndef struct_tag_i72XuJPFazZPdRJImOKQYH
#define struct_tag_i72XuJPFazZPdRJImOKQYH

struct tag_i72XuJPFazZPdRJImOKQYH
{
  char_T Value[16];
};

#endif                                 /* struct_tag_i72XuJPFazZPdRJImOKQYH */

#ifndef typedef_s_i72XuJPFazZPdRJImOKQYH
#define typedef_s_i72XuJPFazZPdRJImOKQYH

typedef struct tag_i72XuJPFazZPdRJImOKQYH s_i72XuJPFazZPdRJImOKQYH;

#endif                                 /* typedef_s_i72XuJPFazZPdRJImOKQYH */

#ifndef struct_tag_bCBh7Pi2jvxDsMOAcKyLAE
#define struct_tag_bCBh7Pi2jvxDsMOAcKyLAE

struct tag_bCBh7Pi2jvxDsMOAcKyLAE
{
  char_T Value[20];
};

#endif                                 /* struct_tag_bCBh7Pi2jvxDsMOAcKyLAE */

#ifndef typedef_s_bCBh7Pi2jvxDsMOAcKyLAE
#define typedef_s_bCBh7Pi2jvxDsMOAcKyLAE

typedef struct tag_bCBh7Pi2jvxDsMOAcKyLAE s_bCBh7Pi2jvxDsMOAcKyLAE;

#endif                                 /* typedef_s_bCBh7Pi2jvxDsMOAcKyLAE */

#ifndef struct_tag_4lnt01oyo4YFEL1rRRafIE
#define struct_tag_4lnt01oyo4YFEL1rRRafIE

struct tag_4lnt01oyo4YFEL1rRRafIE
{
  char_T f1[8];
  char_T f2[6];
  char_T f3[6];
  char_T f4[6];
  char_T f5[4];
  char_T f6[5];
  real_T f7;
};

#endif                                 /* struct_tag_4lnt01oyo4YFEL1rRRafIE */

#ifndef typedef_j_cell
#define typedef_j_cell

typedef struct tag_4lnt01oyo4YFEL1rRRafIE j_cell;

#endif                                 /* typedef_j_cell */

#ifndef struct_tag_u9RUDmUMX6fTaoUoq3jMnE
#define struct_tag_u9RUDmUMX6fTaoUoq3jMnE

struct tag_u9RUDmUMX6fTaoUoq3jMnE
{
  char_T f1[8];
  char_T f2[4];
  char_T f3[2];
  char_T f4[5];
  real_T f5;
};

#endif                                 /* struct_tag_u9RUDmUMX6fTaoUoq3jMnE */

#ifndef typedef_k_cell
#define typedef_k_cell

typedef struct tag_u9RUDmUMX6fTaoUoq3jMnE k_cell;

#endif                                 /* typedef_k_cell */

#ifndef struct_tag_vbYOEh1BLLxuv31ookWdiG
#define struct_tag_vbYOEh1BLLxuv31ookWdiG

struct tag_vbYOEh1BLLxuv31ookWdiG
{
  char_T Value[4];
};

#endif                                 /* struct_tag_vbYOEh1BLLxuv31ookWdiG */

#ifndef typedef_s_vbYOEh1BLLxuv31ookWdiG
#define typedef_s_vbYOEh1BLLxuv31ookWdiG

typedef struct tag_vbYOEh1BLLxuv31ookWdiG s_vbYOEh1BLLxuv31ookWdiG;

#endif                                 /* typedef_s_vbYOEh1BLLxuv31ookWdiG */

#ifndef struct_tag_sK6gPHXwWOj7fc4UEsWvt6F
#define struct_tag_sK6gPHXwWOj7fc4UEsWvt6F

struct tag_sK6gPHXwWOj7fc4UEsWvt6F
{
  real_T NameLength;
  uint8_T Name[5];
  real_T ParentIndex;
  real_T NumChildren;
  real_T ChildrenIndices[17];
  real_T Mass;
  real_T CenterOfMass[3];
  real_T Inertia[9];
  real_T SpatialInertia[36];
};

#endif                                 /* struct_tag_sK6gPHXwWOj7fc4UEsWvt6F */

#ifndef typedef_sK6gPHXwWOj7fc4UEsWvt6F
#define typedef_sK6gPHXwWOj7fc4UEsWvt6F

typedef struct tag_sK6gPHXwWOj7fc4UEsWvt6F sK6gPHXwWOj7fc4UEsWvt6F;

#endif                                 /* typedef_sK6gPHXwWOj7fc4UEsWvt6F */

#ifndef struct_tag_AR5UYEk3npbUtCrhVuTEEH
#define struct_tag_AR5UYEk3npbUtCrhVuTEEH

struct tag_AR5UYEk3npbUtCrhVuTEEH
{
  sK6gPHXwWOj7fc4UEsWvt6F f1;
  sK6gPHXwWOj7fc4UEsWvt6F f2;
  sK6gPHXwWOj7fc4UEsWvt6F f3;
  sK6gPHXwWOj7fc4UEsWvt6F f4;
  sK6gPHXwWOj7fc4UEsWvt6F f5;
  sK6gPHXwWOj7fc4UEsWvt6F f6;
  sK6gPHXwWOj7fc4UEsWvt6F f7;
  sK6gPHXwWOj7fc4UEsWvt6F f8;
  sK6gPHXwWOj7fc4UEsWvt6F f9;
  sK6gPHXwWOj7fc4UEsWvt6F f10;
  sK6gPHXwWOj7fc4UEsWvt6F f11;
  sK6gPHXwWOj7fc4UEsWvt6F f12;
  sK6gPHXwWOj7fc4UEsWvt6F f13;
  sK6gPHXwWOj7fc4UEsWvt6F f14;
  sK6gPHXwWOj7fc4UEsWvt6F f15;
  sK6gPHXwWOj7fc4UEsWvt6F f16;
  sK6gPHXwWOj7fc4UEsWvt6F f17;
  sK6gPHXwWOj7fc4UEsWvt6F f18;
};

#endif                                 /* struct_tag_AR5UYEk3npbUtCrhVuTEEH */

#ifndef typedef_s_AR5UYEk3npbUtCrhVuTEEH
#define typedef_s_AR5UYEk3npbUtCrhVuTEEH

typedef struct tag_AR5UYEk3npbUtCrhVuTEEH s_AR5UYEk3npbUtCrhVuTEEH;

#endif                                 /* typedef_s_AR5UYEk3npbUtCrhVuTEEH */

#ifndef struct_tag_FUbIbCbxuHhTMQLSXcpQvC
#define struct_tag_FUbIbCbxuHhTMQLSXcpQvC

struct tag_FUbIbCbxuHhTMQLSXcpQvC
{
  s_AR5UYEk3npbUtCrhVuTEEH _data;
};

#endif                                 /* struct_tag_FUbIbCbxuHhTMQLSXcpQvC */

#ifndef typedef_s_FUbIbCbxuHhTMQLSXcpQvC
#define typedef_s_FUbIbCbxuHhTMQLSXcpQvC

typedef struct tag_FUbIbCbxuHhTMQLSXcpQvC s_FUbIbCbxuHhTMQLSXcpQvC;

#endif                                 /* typedef_s_FUbIbCbxuHhTMQLSXcpQvC */

#ifndef struct_tag_eQy1MNoRyTPM4exI8pz5EF
#define struct_tag_eQy1MNoRyTPM4exI8pz5EF

struct tag_eQy1MNoRyTPM4exI8pz5EF
{
  real_T NumBodies;
  real_T MaxNumBodies;
  real_T Gravity[3];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[34];
  real_T VelocityDoFMap[34];
  real_T MaxNameLength;
  real_T MaxJointPositionNumber;
  uint8_T DataFormat;
  real_T JointPositionLimits[24];
  s_FUbIbCbxuHhTMQLSXcpQvC Bodies;
  sCGDRIJsTfVBdZWrIuhZvwG Joints[18];
};

#endif                                 /* struct_tag_eQy1MNoRyTPM4exI8pz5EF */

#ifndef typedef_s_eQy1MNoRyTPM4exI8pz5EF
#define typedef_s_eQy1MNoRyTPM4exI8pz5EF

typedef struct tag_eQy1MNoRyTPM4exI8pz5EF s_eQy1MNoRyTPM4exI8pz5EF;

#endif                                 /* typedef_s_eQy1MNoRyTPM4exI8pz5EF */

#ifndef struct_tag_oqflo8tPmhJ6sxPFLZiJGH
#define struct_tag_oqflo8tPmhJ6sxPFLZiJGH

struct tag_oqflo8tPmhJ6sxPFLZiJGH
{
  b_cell_wrap _data;
};

#endif                                 /* struct_tag_oqflo8tPmhJ6sxPFLZiJGH */

#ifndef typedef_s_oqflo8tPmhJ6sxPFLZiJGH
#define typedef_s_oqflo8tPmhJ6sxPFLZiJGH

typedef struct tag_oqflo8tPmhJ6sxPFLZiJGH s_oqflo8tPmhJ6sxPFLZiJGH;

#endif                                 /* typedef_s_oqflo8tPmhJ6sxPFLZiJGH */

#ifndef struct_tag_TnEayzHWuHj1hMPKnEr7h
#define struct_tag_TnEayzHWuHj1hMPKnEr7h

struct tag_TnEayzHWuHj1hMPKnEr7h
{
  cell _data;
};

#endif                                 /* struct_tag_TnEayzHWuHj1hMPKnEr7h */

#ifndef typedef_s_TnEayzHWuHj1hMPKnEr7h
#define typedef_s_TnEayzHWuHj1hMPKnEr7h

typedef struct tag_TnEayzHWuHj1hMPKnEr7h s_TnEayzHWuHj1hMPKnEr7h;

#endif                                 /* typedef_s_TnEayzHWuHj1hMPKnEr7h */

#ifndef struct_tag_lnEOVMt12CNg5nSw1iwvNF
#define struct_tag_lnEOVMt12CNg5nSw1iwvNF

struct tag_lnEOVMt12CNg5nSw1iwvNF
{
  c_cell_wrap _data;
};

#endif                                 /* struct_tag_lnEOVMt12CNg5nSw1iwvNF */

#ifndef typedef_s_lnEOVMt12CNg5nSw1iwvNF
#define typedef_s_lnEOVMt12CNg5nSw1iwvNF

typedef struct tag_lnEOVMt12CNg5nSw1iwvNF s_lnEOVMt12CNg5nSw1iwvNF;

#endif                                 /* typedef_s_lnEOVMt12CNg5nSw1iwvNF */

#ifndef struct_tag_rLqvsRnn1YgNlP4UZHEHjG
#define struct_tag_rLqvsRnn1YgNlP4UZHEHjG

struct tag_rLqvsRnn1YgNlP4UZHEHjG
{
  b_cell _data;
};

#endif                                 /* struct_tag_rLqvsRnn1YgNlP4UZHEHjG */

#ifndef typedef_s_rLqvsRnn1YgNlP4UZHEHjG
#define typedef_s_rLqvsRnn1YgNlP4UZHEHjG

typedef struct tag_rLqvsRnn1YgNlP4UZHEHjG s_rLqvsRnn1YgNlP4UZHEHjG;

#endif                                 /* typedef_s_rLqvsRnn1YgNlP4UZHEHjG */

#ifndef struct_tag_HNEgOhGwwuu4IndkThqyM
#define struct_tag_HNEgOhGwwuu4IndkThqyM

struct tag_HNEgOhGwwuu4IndkThqyM
{
  c_cell _data;
};

#endif                                 /* struct_tag_HNEgOhGwwuu4IndkThqyM */

#ifndef typedef_s_HNEgOhGwwuu4IndkThqyM
#define typedef_s_HNEgOhGwwuu4IndkThqyM

typedef struct tag_HNEgOhGwwuu4IndkThqyM s_HNEgOhGwwuu4IndkThqyM;

#endif                                 /* typedef_s_HNEgOhGwwuu4IndkThqyM */

#ifndef struct_tag_HOps0FrfA6RiWumqewPwZD
#define struct_tag_HOps0FrfA6RiWumqewPwZD

struct tag_HOps0FrfA6RiWumqewPwZD
{
  d_cell_wrap _data;
};

#endif                                 /* struct_tag_HOps0FrfA6RiWumqewPwZD */

#ifndef typedef_s_HOps0FrfA6RiWumqewPwZD
#define typedef_s_HOps0FrfA6RiWumqewPwZD

typedef struct tag_HOps0FrfA6RiWumqewPwZD s_HOps0FrfA6RiWumqewPwZD;

#endif                                 /* typedef_s_HOps0FrfA6RiWumqewPwZD */

#ifndef struct_tag_1nlLkVeIuST25DF6il3ApD
#define struct_tag_1nlLkVeIuST25DF6il3ApD

struct tag_1nlLkVeIuST25DF6il3ApD
{
  e_cell_wrap _data;
};

#endif                                 /* struct_tag_1nlLkVeIuST25DF6il3ApD */

#ifndef typedef_s_1nlLkVeIuST25DF6il3ApD
#define typedef_s_1nlLkVeIuST25DF6il3ApD

typedef struct tag_1nlLkVeIuST25DF6il3ApD s_1nlLkVeIuST25DF6il3ApD;

#endif                                 /* typedef_s_1nlLkVeIuST25DF6il3ApD */

#ifndef struct_tag_uzuPWHtc1cM7ZRTfbsKeiF
#define struct_tag_uzuPWHtc1cM7ZRTfbsKeiF

struct tag_uzuPWHtc1cM7ZRTfbsKeiF
{
  d_cell _data;
};

#endif                                 /* struct_tag_uzuPWHtc1cM7ZRTfbsKeiF */

#ifndef typedef_s_uzuPWHtc1cM7ZRTfbsKeiF
#define typedef_s_uzuPWHtc1cM7ZRTfbsKeiF

typedef struct tag_uzuPWHtc1cM7ZRTfbsKeiF s_uzuPWHtc1cM7ZRTfbsKeiF;

#endif                                 /* typedef_s_uzuPWHtc1cM7ZRTfbsKeiF */

#ifndef struct_tag_w3m1Q26ivrDTAtgc0mcqVE
#define struct_tag_w3m1Q26ivrDTAtgc0mcqVE

struct tag_w3m1Q26ivrDTAtgc0mcqVE
{
  s_FDrX8kOEjZXLXru8nW4swE _data;
};

#endif                                 /* struct_tag_w3m1Q26ivrDTAtgc0mcqVE */

#ifndef typedef_s_w3m1Q26ivrDTAtgc0mcqVE
#define typedef_s_w3m1Q26ivrDTAtgc0mcqVE

typedef struct tag_w3m1Q26ivrDTAtgc0mcqVE s_w3m1Q26ivrDTAtgc0mcqVE;

#endif                                 /* typedef_s_w3m1Q26ivrDTAtgc0mcqVE */

#ifndef struct_tag_mNpfPkpGE2ymZ9hNiT782E
#define struct_tag_mNpfPkpGE2ymZ9hNiT782E

struct tag_mNpfPkpGE2ymZ9hNiT782E
{
  e_cell _data;
};

#endif                                 /* struct_tag_mNpfPkpGE2ymZ9hNiT782E */

#ifndef typedef_s_mNpfPkpGE2ymZ9hNiT782E
#define typedef_s_mNpfPkpGE2ymZ9hNiT782E

typedef struct tag_mNpfPkpGE2ymZ9hNiT782E s_mNpfPkpGE2ymZ9hNiT782E;

#endif                                 /* typedef_s_mNpfPkpGE2ymZ9hNiT782E */

#ifndef struct_tag_RXzHB0cSwVmz9QtsDQbPBG
#define struct_tag_RXzHB0cSwVmz9QtsDQbPBG

struct tag_RXzHB0cSwVmz9QtsDQbPBG
{
  f_cell _data;
};

#endif                                 /* struct_tag_RXzHB0cSwVmz9QtsDQbPBG */

#ifndef typedef_s_RXzHB0cSwVmz9QtsDQbPBG
#define typedef_s_RXzHB0cSwVmz9QtsDQbPBG

typedef struct tag_RXzHB0cSwVmz9QtsDQbPBG s_RXzHB0cSwVmz9QtsDQbPBG;

#endif                                 /* typedef_s_RXzHB0cSwVmz9QtsDQbPBG */

#ifndef struct_tag_MY3jsqmREaTzOC09vCGedD
#define struct_tag_MY3jsqmREaTzOC09vCGedD

struct tag_MY3jsqmREaTzOC09vCGedD
{
  f_cell_wrap _data;
};

#endif                                 /* struct_tag_MY3jsqmREaTzOC09vCGedD */

#ifndef typedef_s_MY3jsqmREaTzOC09vCGedD
#define typedef_s_MY3jsqmREaTzOC09vCGedD

typedef struct tag_MY3jsqmREaTzOC09vCGedD s_MY3jsqmREaTzOC09vCGedD;

#endif                                 /* typedef_s_MY3jsqmREaTzOC09vCGedD */

#ifndef struct_tag_qH9OLvRzrE6jZf6Q0KWeS
#define struct_tag_qH9OLvRzrE6jZf6Q0KWeS

struct tag_qH9OLvRzrE6jZf6Q0KWeS
{
  g_cell _data;
};

#endif                                 /* struct_tag_qH9OLvRzrE6jZf6Q0KWeS */

#ifndef typedef_s_qH9OLvRzrE6jZf6Q0KWeS
#define typedef_s_qH9OLvRzrE6jZf6Q0KWeS

typedef struct tag_qH9OLvRzrE6jZf6Q0KWeS s_qH9OLvRzrE6jZf6Q0KWeS;

#endif                                 /* typedef_s_qH9OLvRzrE6jZf6Q0KWeS */

#ifndef struct_tag_kIjER3TOnaUUvjksbP9imB
#define struct_tag_kIjER3TOnaUUvjksbP9imB

struct tag_kIjER3TOnaUUvjksbP9imB
{
  h_cell _data;
};

#endif                                 /* struct_tag_kIjER3TOnaUUvjksbP9imB */

#ifndef typedef_s_kIjER3TOnaUUvjksbP9imB
#define typedef_s_kIjER3TOnaUUvjksbP9imB

typedef struct tag_kIjER3TOnaUUvjksbP9imB s_kIjER3TOnaUUvjksbP9imB;

#endif                                 /* typedef_s_kIjER3TOnaUUvjksbP9imB */

#ifndef struct_tag_sNUBrrCthrFQOdBEPtSPXB
#define struct_tag_sNUBrrCthrFQOdBEPtSPXB

struct tag_sNUBrrCthrFQOdBEPtSPXB
{
  i_cell _data;
};

#endif                                 /* struct_tag_sNUBrrCthrFQOdBEPtSPXB */

#ifndef typedef_s_sNUBrrCthrFQOdBEPtSPXB
#define typedef_s_sNUBrrCthrFQOdBEPtSPXB

typedef struct tag_sNUBrrCthrFQOdBEPtSPXB s_sNUBrrCthrFQOdBEPtSPXB;

#endif                                 /* typedef_s_sNUBrrCthrFQOdBEPtSPXB */

#ifndef struct_tag_mcoCTymCR3qfYJPxx4rUaD
#define struct_tag_mcoCTymCR3qfYJPxx4rUaD

struct tag_mcoCTymCR3qfYJPxx4rUaD
{
  j_cell _data;
};

#endif                                 /* struct_tag_mcoCTymCR3qfYJPxx4rUaD */

#ifndef typedef_s_mcoCTymCR3qfYJPxx4rUaD
#define typedef_s_mcoCTymCR3qfYJPxx4rUaD

typedef struct tag_mcoCTymCR3qfYJPxx4rUaD s_mcoCTymCR3qfYJPxx4rUaD;

#endif                                 /* typedef_s_mcoCTymCR3qfYJPxx4rUaD */

#ifndef struct_tag_64qU89zzi5moV4oLaqjD3D
#define struct_tag_64qU89zzi5moV4oLaqjD3D

struct tag_64qU89zzi5moV4oLaqjD3D
{
  k_cell _data;
};

#endif                                 /* struct_tag_64qU89zzi5moV4oLaqjD3D */

#ifndef typedef_s_64qU89zzi5moV4oLaqjD3D
#define typedef_s_64qU89zzi5moV4oLaqjD3D

typedef struct tag_64qU89zzi5moV4oLaqjD3D s_64qU89zzi5moV4oLaqjD3D;

#endif                                 /* typedef_s_64qU89zzi5moV4oLaqjD3D */

#ifndef typedef_InstanceStruct_BJyqJeM9MN8kuyMGX65V0G
#define typedef_InstanceStruct_BJyqJeM9MN8kuyMGX65V0G

typedef struct {
  SimStruct *S;
  robotics_slmanip_internal_block_MassMatrixBlock sysobj;
  boolean_T sysobj_not_empty;
  uint32_T method;
  boolean_T method_not_empty;
  uint32_T state;
  boolean_T state_not_empty;
  uint32_T b_state[2];
  boolean_T b_state_not_empty;
  uint32_T c_state[625];
  boolean_T c_state_not_empty;
  void *emlrtRootTLSGlobal;
  real_T (*u0)[12];
  real_T (*b_y0)[144];
} InstanceStruct_BJyqJeM9MN8kuyMGX65V0G;

#endif                                 /* typedef_InstanceStruct_BJyqJeM9MN8kuyMGX65V0G */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
extern void method_dispatcher_BJyqJeM9MN8kuyMGX65V0G(SimStruct *S, int_T method,
  void* data);

#endif
