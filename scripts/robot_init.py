#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from xarm_msgs.srv import SetAxis, SetFloat32, SetInt16, Move, MoveAxisAngle, MoveVelo


def call_service(service_name, srv_type, req):
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        service = rospy.ServiceProxy(service_name, srv_type)
        resp = service(req)
        rospy.loginfo(f"{service_name} response: ret={resp.ret}, msg={resp.message}")
        return resp.ret == 0
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False
    except rospy.ROSException:
        rospy.logerr(f"Timeout waiting for service: {service_name}")
        return False


def main():
    rospy.init_node("xarm_initializer")

    rospy.loginfo("Initializing arm...")

    # 1. Enable motion
    motion_req = SetAxis._request_class()
    motion_req.id = 8
    motion_req.data = 1

    if not call_service("/ufactory/motion_ctrl", SetAxis, motion_req):
        rospy.logerr("Failed to enable motion.")
        return

    # 2. Set mode to 7
    mode_req = SetInt16._request_class()
    mode_req.data = 7
    if not call_service("/ufactory/set_mode", SetInt16, mode_req):
        rospy.logerr("Failed to set mode.")
        return

    # 3. Set state to 0
    state_req = SetInt16._request_class()
    state_req.data = 0
    if not call_service("/ufactory/set_state", SetInt16, state_req):
        rospy.logerr("Failed to set state.")
        return

    pose_init = Move._request_class()
    pose_init.pose = [-19.2, 250.6, 400.5, 3.046, 0.11, -1.507]
    pose_init.mvvelo = 100
    pose_init.mvacc = 1000
    pose_init.mvtime = 0
    pose_init.mvradii = 0
    if not call_service("/ufactory/move_line",Move,pose_init):
        rospy.logerr("Failed to set initial pose.")
        return

    rospy.loginfo("Arm initialization complete.")



if __name__ == "__main__":
    main()
