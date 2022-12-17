from intera_interface import gripper as robot_gripper
import rospy

rospy.init_node('gripper_cal')

right_gripper = robot_gripper.Gripper('right_gripper')
right_gripper.calibrate()
rospy.sleep(2.0)

right_gripper.close(0.02)
rospy.sleep(1.0)