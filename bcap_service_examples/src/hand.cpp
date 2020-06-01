/*
 *
 */

#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <vector>
#include "bcap_core/bcap_funcid.h"
#include "bcap_core/bCAPClient/bcap_client.h"
#include "bcap_core/RACString/rac_string.h"
#include "denso_robot_core/denso_robot_rc8.h"

#define SERVER_ADDR	"192.168.0.1"
#define SERVER_PORT	5007


int main(int argc, char **argv)
{
  int m_fd;
  u_int32_t handler;
  HRESULT hr;
  const char ss []= "tcp:192.168.0.1:5007";
  const char m_type[]="tcp";
  const char m_addr[]=SERVER_ADDR;
  int m_port = SERVER_PORT;
  int m_timeout = 3000;
  int m_retry = 5;
  int m_wait = 0;
  int m_wdt = 400;
  int m_invoke = 180000;
  std::stringstream  ss1;
  std::wstringstream ss2;

  ss1 << m_type << ":" << m_addr << ":" << m_port;
  hr = bCap_Open_Client(ss1.str().c_str(), m_timeout, m_retry, &m_fd);

  if(!SUCCEEDED(hr)){
    std::cerr << "Error:Fail to connect bCapServer" << std::endl;
    exit(1);
  }

  ss2  << L",WDT=" << m_wdt << L",InvokeTimeout=" << m_invoke;
  BSTR bstrOption = SysAllocString(ss2.str().c_str());
  hr = bCap_ServiceStart(m_fd, bstrOption);
  SysFreeString(bstrOption);

  //hr = bCap_ControllerConnect(m_fd, L"b-Cap", L"caoProv.DENSO.VRC", L"192.168.0.1", L"", &handler);
  hr = bCap_ControllerConnect(m_fd, L"", L"CaoProv.DENSO.VRC", L"localhost", L"", &handler);

    std::cerr << hr << " handler: " << handler << std::endl;
  /*******************************************/
  uint32_t lVar;
  uint32_t lhRobot;
  VARIANT lResult;
  VARIANT Args;

  hr = bCap_ControllerGetRobot(m_fd, handler, L"Arm", L"", &lhRobot);

    std::cerr << hr << " lhRobot: " << lhRobot << std::endl;
  
  Args.vt = VT_BSTR;
  Args.bstrVal = L"";
  hr = bCap_RobotExecute(m_fd, lhRobot, L"Takearm", Args, &lResult);
    std::cerr << hr << " Result: " << lResult.vt << std::endl;

  Args.vt = VT_BSTR;
  Args.bstrVal = L"1";
  hr = bCap_RobotExecute(m_fd, lhRobot, L"Motor", Args, &lResult);
    std::cerr << hr << " Result: " << lResult.vt << std::endl;
  

  Args.vt = VT_EMPTY;
  hr = bCap_ControllerExecute(m_fd, handler, L"HandCurPos", Args, &lResult);
  std::cerr << "Result(HandPos): " << hr << ";" << lResult.vt << std::endl;
  std::cerr << lResult.dblVal << std::endl;

  if (lResult.dblVal < 21){
#if 1
  Args.vt = (VT_ARRAY | VT_VARIANT);
  Args.parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);
  VARIANT *pvnt;

  SafeArrayAccessData(Args.parray, (void**)&pvnt);

  pvnt[0].vt = VT_R8;
  if( lResult.dblVal < 10){
    pvnt[0].dblVal = 30.0;
  }else{
    pvnt[0].dblVal = 5.0;
  }

  pvnt[1].vt = VT_UI1;
  pvnt[1].bVal = 100;
  SafeArrayUnaccessData(Args.parray);

  hr = bCap_ControllerExecute(m_fd, handler, L"HandMoveA", Args, &lResult);
  std::cerr << "Result(HandMoveA): " << hr << ";" << lResult.vt << std::endl;
#endif
  }else{
  Args.vt = (VT_ARRAY | VT_VARIANT);
  Args.parray = SafeArrayCreateVector(VT_VARIANT, 0, 4);
  VARIANT *pvnt;

  SafeArrayAccessData(Args.parray, (void**)&pvnt);

  pvnt[0].vt = VT_R8;
  pvnt[0].dblVal = 10.0;

  pvnt[1].vt = VT_UI1;
  pvnt[1].bVal = 100;

  pvnt[2].vt = VT_R8;
  pvnt[2].dblVal = 20.0;

  pvnt[3].vt = VT_BSTR;
  pvnt[3].bstrVal = L"Next";

  SafeArrayUnaccessData(Args.parray);

  hr = bCap_ControllerExecute(m_fd, handler, L"HandMoveAH", Args, &lResult);
  std::cerr << "Result(HandMoveAH): " << hr << ";" << lResult.vt << std::endl;
  }

  /**********************************/
  Args.vt = VT_BSTR;
  Args.bstrVal = L"";
  hr = bCap_RobotExecute(m_fd, lhRobot, L"CurJnt", Args, &lResult);

  if (lResult.vt == 8197){
    std::cerr << "Result(cDims): " << lResult.parray->cDims << std::endl;
    std::cerr << "Result(vt): " << lResult.parray->vt << std::endl;
    std::cerr << "Result(cbElements): " << lResult.parray->cbElements << std::endl;
    double *data;
    hr = SafeArrayAccessData(lResult.parray, (void **)&data);
    for(int i=0;i< lResult.parray->cbElements ; i++){
     std::cerr << i<< ": " << data[i] << std::endl;
    }
  }else{
    std::cerr << "Result: " << lResult.vt << std::endl;

  }

  /*
  Args.vt = VT_EMPTY;
  hr = bCap_ControllerExecute(m_fd, handler, L"ClearError", Args, &lResult);
    std::cerr << "hr: " << hr << std::endl;
    std::cerr << "Result: " << lResult.vt << std::endl;

  */
   /*
  Args.vt = VT_I4;
  Args.lVal = 0x202;
  hr = bCap_RobotExecute(m_fd, lhRobot, L"slvChangeMode", Args, &lResult);
    std::cerr << "Change Mode hr: " << hr << std::endl;
    std::cerr << "Change Mode Result: " << lResult.vt << std::endl;
  */

  Args.vt = VT_BSTR;
  Args.bstrVal = L"0";
  hr = bCap_RobotExecute(m_fd, lhRobot, L"Motor", Args, &lResult);
    std::cerr << hr << " Result: " << lResult.vt << std::endl;

  Args.vt = VT_BSTR;
  Args.bstrVal = L"";
  hr = bCap_RobotExecute(m_fd, lhRobot, L"Givearm", Args, &lResult);
    std::cerr << hr << " Result: " << lResult.vt << std::endl;
  bCap_RobotRelease(m_fd, &lhRobot);


  /*********************************/
  bCap_ControllerDisconnect(m_fd, &handler);
  bCap_ServiceStop(m_fd);
  bCap_Close_Client(&m_fd);

  exit(0);
}
