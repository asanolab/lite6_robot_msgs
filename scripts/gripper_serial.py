#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float32


class GripperSerialInterface:
    def __init__(self):
        # 串口初始化
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = int(rospy.get_param("~baud", 115200))
        self.ser = serial.Serial(port, baud, timeout=1)
        rospy.loginfo("Gripper serial connected on /dev/ttyACM0")

        # ROS pub / sub
        self.pub = rospy.Publisher('/gripper/current_distance', Float32, queue_size=10)
        rospy.Subscriber('/gripper/target_distance', Float32, self.target_callback)
        self.target_distance = None

    def target_callback(self, msg):
        self.target_distance = msg.data
        send_str = f"{self.target_distance:.2f}\n"
        self.ser.write(send_str.encode('utf-8'))
        rospy.loginfo(f"Sent target distance: {send_str.strip()}")

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    current_distance = float(line)
                    self.pub.publish(current_distance)
                    rospy.loginfo_throttle(1.0, f"Gripper current distance: {current_distance:.2f}")
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"Serial error: {e}")
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ripper_serial_node')
    node = GripperSerialInterface()
    node.spin()