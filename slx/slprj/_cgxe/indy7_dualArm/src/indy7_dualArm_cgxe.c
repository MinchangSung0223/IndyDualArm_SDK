/* Include files */

#include "indy7_dualArm_cgxe.h"
#include "m_qq0HSn4MnMIEEMjNzXu2qD.h"
#include "m_T6MwjqiGx6loI7k5o1vVhG.h"
#include "m_TCGSFIrmgTrdrXjldiS8GC.h"
#include "m_87mWOnq5ANQkjTWghyXDKD.h"
#include "m_ixuv55C8EtZmU1WzAsjQDD.h"
#include "m_imQiRskwoVIlJDtKF8Ie9C.h"
#include "m_s5IVGQHtlmgQqKlp1mGyWG.h"
#include "m_cGalwQDtj5O4ZBPIMXFzfD.h"

unsigned int cgxe_indy7_dualArm_method_dispatcher(SimStruct* S, int_T method,
  void* data)
{
  if (ssGetChecksum0(S) == 178817427 &&
      ssGetChecksum1(S) == 4072434611 &&
      ssGetChecksum2(S) == 4068674943 &&
      ssGetChecksum3(S) == 1436499906) {
    method_dispatcher_qq0HSn4MnMIEEMjNzXu2qD(S, method, data);
    return 1;
  }

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

  if (ssGetChecksum0(S) == 2414102413 &&
      ssGetChecksum1(S) == 814913094 &&
      ssGetChecksum2(S) == 2581825787 &&
      ssGetChecksum3(S) == 1628411881) {
    method_dispatcher_87mWOnq5ANQkjTWghyXDKD(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 3148610900 &&
      ssGetChecksum1(S) == 2301114800 &&
      ssGetChecksum2(S) == 3139047242 &&
      ssGetChecksum3(S) == 212001703) {
    method_dispatcher_ixuv55C8EtZmU1WzAsjQDD(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 3521690577 &&
      ssGetChecksum1(S) == 1375613089 &&
      ssGetChecksum2(S) == 3985637424 &&
      ssGetChecksum3(S) == 4155627802) {
    method_dispatcher_imQiRskwoVIlJDtKF8Ie9C(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 3530623186 &&
      ssGetChecksum1(S) == 3648337709 &&
      ssGetChecksum2(S) == 1100901509 &&
      ssGetChecksum3(S) == 2499504597) {
    method_dispatcher_s5IVGQHtlmgQqKlp1mGyWG(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 3547873016 &&
      ssGetChecksum1(S) == 3695362885 &&
      ssGetChecksum2(S) == 502855996 &&
      ssGetChecksum3(S) == 1209443866) {
    method_dispatcher_cGalwQDtj5O4ZBPIMXFzfD(S, method, data);
    return 1;
  }

  return 0;
}
