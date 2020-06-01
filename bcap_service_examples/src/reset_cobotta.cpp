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

  hr = bCap_ControllerConnect(m_fd, L"b-Cap", L"caoProv.DENSO.VRC", L"192.168.0.1", L"", &handler);

  /*******************************************/
  uint32_t lVar;
  uint32_t lhRobot;
  VARIANT lResult;
  VARIANT Args;

  hr = bCap_ControllerGetRobot(m_fd, handler, L"Arm", L"", &lhRobot);
  
  Args.vt = VT_BSTR;
  Args.bstrVal = L"";
  hr = bCap_RobotExecute(m_fd, lhRobot, L"Takearm", Args, &lResult);

    /*
  Args.vt = VT_BSTR;
  Args.bstrVal = L"1";
  hr = bCap_RobotExecute(m_fd, lhRobot, L"Motor", Args, &lResult);
    */
  
  Args.vt = VT_EMPTY;
  hr = bCap_ControllerExecute(m_fd, handler, L"ClearError", Args, &lResult);

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

  Args.vt = VT_BSTR;
  Args.bstrVal = L"";
  hr = bCap_RobotExecute(m_fd, lhRobot, L"Givearm", Args, &lResult);
  bCap_RobotRelease(m_fd, &lhRobot);


  /*********************************/
  bCap_ControllerDisconnect(m_fd, &handler);
  bCap_ServiceStop(m_fd);
  bCap_Close_Client(&m_fd);

  exit(0);
}
