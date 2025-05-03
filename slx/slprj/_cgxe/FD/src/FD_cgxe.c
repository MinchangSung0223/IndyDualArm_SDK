/* Include files */

#include "FD_cgxe.h"
#include "m_W21pBFWeaWEpX2dLoBMwEF.h"
#include "m_arKSGChzXtd1ydiJSmzxIF.h"
#include "m_4064OUrZvmKL9d2RhI4NJF.h"

unsigned int cgxe_FD_method_dispatcher(SimStruct* S, int_T method, void* data)
{
  if (ssGetChecksum0(S) == 954660155 &&
      ssGetChecksum1(S) == 498346491 &&
      ssGetChecksum2(S) == 2207130181 &&
      ssGetChecksum3(S) == 64013202) {
    method_dispatcher_W21pBFWeaWEpX2dLoBMwEF(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 2594165045 &&
      ssGetChecksum1(S) == 3670876354 &&
      ssGetChecksum2(S) == 709618361 &&
      ssGetChecksum3(S) == 2929162124) {
    method_dispatcher_arKSGChzXtd1ydiJSmzxIF(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 2612486248 &&
      ssGetChecksum1(S) == 3338854649 &&
      ssGetChecksum2(S) == 4084235547 &&
      ssGetChecksum3(S) == 2743526478) {
    method_dispatcher_4064OUrZvmKL9d2RhI4NJF(S, method, data);
    return 1;
  }

  return 0;
}
