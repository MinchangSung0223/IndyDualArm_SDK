/* Include files */

#include "indy7_dualArm_cgxe.h"
#include "m_YWWm119ySh4edf8lUl8ZrE.h"
#include "m_5i5a5OWd9sepnydcE8sQ3F.h"
#include "m_JrnXt4Rd0mM79GD2fCQ2oF.h"

unsigned int cgxe_indy7_dualArm_method_dispatcher(SimStruct* S, int_T method,
  void* data)
{
  if (ssGetChecksum0(S) == 369533013 &&
      ssGetChecksum1(S) == 138022595 &&
      ssGetChecksum2(S) == 1104676813 &&
      ssGetChecksum3(S) == 1279414184) {
    method_dispatcher_YWWm119ySh4edf8lUl8ZrE(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 781240784 &&
      ssGetChecksum1(S) == 196370177 &&
      ssGetChecksum2(S) == 1209679744 &&
      ssGetChecksum3(S) == 2676266124) {
    method_dispatcher_5i5a5OWd9sepnydcE8sQ3F(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 1081421631 &&
      ssGetChecksum1(S) == 1041298756 &&
      ssGetChecksum2(S) == 1626634560 &&
      ssGetChecksum3(S) == 3022584337) {
    method_dispatcher_JrnXt4Rd0mM79GD2fCQ2oF(S, method, data);
    return 1;
  }

  return 0;
}
