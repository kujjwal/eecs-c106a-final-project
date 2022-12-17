#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner

from intera_interface import gripper as robot_gripper

try:
    from controller import Controller
except ImportError:
    pass
    
def main(tooth_coords=None):
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")


    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    cont = Controller(Kp, Kd, Ki, Kw, Limb())
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    # # 
    # # Add the obstacle to the planning scene here
    # #

    # Joint Constraints
    joint_const = JointConstraint()
    #joint_const.header.frame_id = 'base'
    joint_const.joint_name = 'right_s0'
    joint_const.position = 0
    joint_const.tolerance_above = 3.14/2
    joint_const.tolerance_below = 3.14/2
    joint_const.weight = 1.0

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.05;
    orien_const.absolute_y_axis_tolerance = 0.05;
    orien_const.absolute_z_axis_tolerance = 0.05;
    orien_const.weight = 1.0;

    box_const = np.array([0.4, 1.2, 0.05])
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.5
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = -0.2
    planner.add_box_obstacle(box_const, 'desk', box_pose)

    box_const = np.array([0.05, 0.5, 0.4])
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.79
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 0.0
    planner.add_box_obstacle(box_const, 'jaw_top', box_pose)

    box_const = np.array([1.0, 0.01, 1.0])
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = -0.3
    box_pose.pose.position.z = 0.0
    planner.add_box_obstacle(box_const, 'left_wall', box_pose)

    box_const = np.array([1.0, 0.01, 1.0])
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = 0.3
    box_pose.pose.position.z = 0.0
    planner.add_box_obstacle(box_const, 'right_wall', box_pose)

    while not rospy.is_shutdown():
        try:
            goal_1 = PoseStamped()
            goal_1.header.frame_id = "base"

            #x, y, and z position
            goal_1.pose.position.x = 0.529
            goal_1.pose.position.y = 0.055
            goal_1.pose.position.z = -0.09

            #Orientation as a quaternion
            goal_1.pose.orientation.x = 0.0
            goal_1.pose.orientation.y = -1.0
            goal_1.pose.orientation.z = 0.0
            goal_1.pose.orientation.w = 0.0

            # Might have to edit this . . . 
            plan = planner.plan_to_pose(goal_1, [orien_const])
            input("Press <Enter> to move the right arm to goal pose 1: ")
            if not cont.execute_plan(plan[1]): 
                raise Exception("Execution failed")
            right_gripper.close(0.02)
            rospy.sleep(1.0)
        except Exception as e:
            print(e)
            traceback.print_exc()
        else:
            break    

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
