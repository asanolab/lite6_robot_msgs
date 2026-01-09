#!/usr/bin/env python

import rospy
from xarm_msgs.srv import SetAxis

def main():
    rospy.init_node("xarm_disabler")

    rospy.loginfo("Disabling xArm ...")

    try:
        rospy.wait_for_service("/ufactory/motion_ctrl", timeout=5.0)
        disable_srv = rospy.ServiceProxy("/ufactory/motion_ctrl", SetAxis)

        req = SetAxis._request_class()
        req.id = 8
        req.data = 0

        resp = disable_srv(req)
        rospy.loginfo(f"[motion_ctrl] ret={resp.ret}, message='{resp.message}'")

        if resp.ret == 0:
            rospy.loginfo("✅ xArm disabled successfully.")
        else:
            rospy.logwarn("⚠️ xArm disable returned non-zero ret.")
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    main()