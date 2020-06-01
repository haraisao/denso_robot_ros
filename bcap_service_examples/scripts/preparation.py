#
#
import rospy
from bcap_service.srv import *
from bcap_service.msg import *


global bcapService

def ControllerConnect():
    global bcapService
    func_id = 3
    vntArgs=[variant(8, "b-CAP"), variant(8, "CaoProv.DENSO.VRC"), variant(8, "localhost"), variant(8, "")]
    response = bcapService(func_id, vntArgs)
    return response

def ControllerDisconnect(lh):
    global bcapService
    func_id = 4
    vntArgs=[variant(3, lh), variant(8, "")]
    response = bcapService(func_id, vntArgs)
    return response

def ControllerGetRobot(lh):
    global bcapService
    func_id = 7
    vntArgs=[variant(3, lh), variant(8, "Arm"), variant(8, "")]
    response = bcapService(func_id, vntArgs)
    return response

def ControllerReleaseRobot(lh):
    global bcapService
    func_id = 84
    vntArgs=[variant(3, lh), variant(8, "")]
    response = bcapService(func_id, vntArgs)
    return response

def RobotExecute(lh, cmd, arg=""):
    global bcapService
    func_id = 64
    vntArgs=[variant(3, lh), variant(8, cmd), variant(8, arg)]
    response = bcapService(func_id, vntArgs)
    return response

def ControllerExecute(lh, cmd, arg=""):
    global bcapService
    func_id = 17
    vntArgs=[variant(3, lh), variant(8, cmd), variant(8, arg)]
    response = bcapService(func_id, vntArgs)
    return response


def setup():
    global bcapService
    rospy.wait_for_service('bcap_service')
    bcapService = rospy.ServiceProxy('bcap_service', bcap)
    res = ControllerConnect()
    if res.HRESULT != 0:
      print("Error in ControllerConnect")
      return

    h_cntl = res.vntRet.value
    res = ControllerGetRobot(h_cntl)
    if res.HRESULT != 0:
      print("Error in ControllerGetRobot")
      return
    h_robot = res.vntRet.value

    return [h_cntl, h_robot]

def close(handles):
    ControllerReleaseRobot(handles[1])
    ControllerDisconnect(handles[0])
    return

def main():
    handles = setup()
    RobotExecute(handles[1], "ManualResetPreparation")
    RobotExecute(handles[1], "MotionPreparation")
    close(handles)
    return


if __name__ == '__main__':
    main()
