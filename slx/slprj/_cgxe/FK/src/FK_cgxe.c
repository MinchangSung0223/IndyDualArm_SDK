/* Include files */

#include "FK_cgxe.h"
#include "m_LJRV8xaUaAupvDb6yXgyhD.h"
#include "m_4dIWKdXw9f42uXOYIbe9fE.h"
#include "m_lxQ3kncuZwyT2eLDpVVMcD.h"
#include "m_VuWzy0sCJJluCsE5Tx8UrF.h"
#include "m_IBRnOsQNtYL7X0HdVTMLIH.h"

unsigned int cgxe_FK_method_dispatcher(SimStruct* S, int_T method, void* data)
{
  if (ssGetChecksum0(S) == 173167807 &&
      ssGetChecksum1(S) == 605056185 &&
      ssGetChecksum2(S) == 2322253468 &&
      ssGetChecksum3(S) == 2221260243) {
    method_dispatcher_LJRV8xaUaAupvDb6yXgyhD(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 2035886445 &&
      ssGetChecksum1(S) == 177704183 &&
      ssGetChecksum2(S) == 3437916696 &&
      ssGetChecksum3(S) == 1220199086) {
    method_dispatcher_4dIWKdXw9f42uXOYIbe9fE(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 2705541145 &&
      ssGetChecksum1(S) == 3741194739 &&
      ssGetChecksum2(S) == 1351966242 &&
      ssGetChecksum3(S) == 1728866603) {
    method_dispatcher_lxQ3kncuZwyT2eLDpVVMcD(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 3384544997 &&
      ssGetChecksum1(S) == 1857972302 &&
      ssGetChecksum2(S) == 473354016 &&
      ssGetChecksum3(S) == 416662537) {
    method_dispatcher_VuWzy0sCJJluCsE5Tx8UrF(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 3973810701 &&
      ssGetChecksum1(S) == 1371350547 &&
      ssGetChecksum2(S) == 290273119 &&
      ssGetChecksum3(S) == 1585343220) {
    method_dispatcher_IBRnOsQNtYL7X0HdVTMLIH(S, method, data);
    return 1;
  }

  return 0;
}
