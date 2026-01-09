#!/usr/bin/env python

import rospy
from my_robot_msgs.msg import MovePose
from xarm_msgs.srv import Move
import numpy as np
from std_msgs.msg import Bool


class ArmMotionBridge:
    def __init__(self):
        rospy.init_node("robot_motion_caller")

        # 控制参数（可通过 launch 配置）
        self.mvvelo = rospy.get_param("~mvvelo", 200.0)
        self.mvacc = rospy.get_param("~mvacc", 1000.0)
        self.mvtime = rospy.get_param("~mvtime", 0.0)
        self.coord = rospy.get_param("~coord", 0)
        self.relative = rospy.get_param("~relative", False)
        self.move_done_pub = rospy.Publisher("/arm_control/move_done", Bool, queue_size=1)

        # 设置去重误差容忍范围（单位 mm / rad）
        self.pose_tolerance = rospy.get_param("~pose_tolerance", 0.1)  # 一般0.1mm以内算重复

        self.prev_pose = None

        rospy.wait_for_service("/ufactory/move_line")
        self.move_srv = rospy.ServiceProxy("/ufactory/move_line", Move)

        rospy.Subscriber("/arm_control/move_pose", MovePose, self.pose_callback)
        rospy.loginfo("✅ ArmMotionBridge ready.")

    def pose_callback(self, msg):
        current_pose = np.array(msg.pose)

        # 去重：与上次执行的 pose 相比，误差 < tolerance 就不再执行
        if self.prev_pose is not None:
            diff = np.abs(current_pose - self.prev_pose)
            if np.all(diff < self.pose_tolerance):
                rospy.loginfo("⏸️ Received pose is same as previous (within tolerance), ignoring.")
                return

        # 记录当前 pose 为上一次
        self.prev_pose = current_pose

        rospy.loginfo(f"📍 Executing pose: {current_pose.tolist()}")

        target_pose = Move._request_class()
        target_pose.pose = msg.pose
        target_pose.mvvelo = self.mvvelo
        target_pose.mvacc = self.mvacc
        target_pose.mvtime = self.mvtime

        try:
            self.move_done_pub.publish(Bool(data=False))
            response = self.move_srv(target_pose)
            if response.ret == 0:
                rospy.loginfo("[ArmMotionBridge] ✅ move_line executed successfully.")
                self.move_done_pub.publish(Bool(data=True))

            else:
                rospy.logwarn(f"⚠️ move_line failed: ret={response.ret}, message='{response.message}'")
                self.move_done_pub.publish(Bool(data=False))
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ move_line service call failed: {e}")


if __name__ == "__main__":
    ArmMotionBridge()
    rospy.spin()
