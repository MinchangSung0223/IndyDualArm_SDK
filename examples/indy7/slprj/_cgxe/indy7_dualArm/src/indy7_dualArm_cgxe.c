/* Include files */

#include "indy7_dualArm_cgxe.h"
#include "m_T6MwjqiGx6loI7k5o1vVhG.h"
#include "m_TCGSFIrmgTrdrXjldiS8GC.h"
#include "m_s5IVGQHtlmgQqKlp1mGyWG.h"

unsigned int cgxe_indy7_dualArm_method_dispatcher(SimStruct* S, int_T method,
  void* data)
{
  if (ssGetChecksum0(S) == 1114460459 &&
      ssGetChecksum1(S) == 1701144773 &&
      ssGetChecksum2(S) == 1766980214 &&
      ssGetChecksum3(S) == 2101927784) {
    method_dispatcher_T6MwjqiGx6loI7k5o1vVhG(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 2122025888 &&
      ssGetChecksum1(S) == 3622931021 &&
      ssGetChecksum2(S) == 2709846304 &&
      ssGetChecksum3(S) == 3490596982) {
    method_dispatcher_TCGSFIrmgTrdrXjldiS8GC(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 3530623186 &&
      ssGetChecksum1(S) == 3648337709 &&
      ssGetChecksum2(S) == 1100901509 &&
      ssGetChecksum3(S) == 2499504597) {
    method_dispatcher_s5IVGQHtlmgQqKlp1mGyWG(S, method, data);
    return 1;
  }

  return 0;
}
