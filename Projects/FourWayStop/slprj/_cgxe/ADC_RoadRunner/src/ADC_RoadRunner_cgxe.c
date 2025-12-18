/* Include files */

#include "ADC_RoadRunner_cgxe.h"
#include "m_8UAZqaK2BMBoeLrgU0riW.h"
#include "m_rujfpslIDEtqsgp0nmdDZD.h"

unsigned int cgxe_ADC_RoadRunner_method_dispatcher(SimStruct* S, int_T method,
  void* data)
{
  if (ssGetChecksum0(S) == 1977862738 &&
      ssGetChecksum1(S) == 1665585472 &&
      ssGetChecksum2(S) == 2991457856 &&
      ssGetChecksum3(S) == 3307896376) {
    method_dispatcher_8UAZqaK2BMBoeLrgU0riW(S, method, data);
    return 1;
  }

  if (ssGetChecksum0(S) == 3177056306 &&
      ssGetChecksum1(S) == 1098077188 &&
      ssGetChecksum2(S) == 1215249704 &&
      ssGetChecksum3(S) == 1210128957) {
    method_dispatcher_rujfpslIDEtqsgp0nmdDZD(S, method, data);
    return 1;
  }

  return 0;
}
