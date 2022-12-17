import numpy as np
import os, sys

from intera_interface import gripper as robot_gripper
import rospy

# 1. Get centroids from camera_test
# 2. Scaling and world frame coordinate math
# 3. Actuate to tooth coordinates
# 4. Close gripper
# 5. Move straight up to extract tooth

def main(tooth_centroid):

    rospy.init_node('gripper_test')
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    centroid_x,centroid_y = tooth_centroid
    print('Centroid: ', tooth_centroid)
    scaling_factor = -0.206/62
    #y_3d = scaling_factor * (centroid_y - 577) + 0.040
    multiplicand = 1
    if centroid_x >= 642:
        multiplicand = -1
    y_3d = multiplicand * ((0.0000133 * (centroid_x - 642)**2) - multiplicand * 0.00105 * (centroid_x - 642)) + 0.048
    x_3d = (((centroid_x-628)*0.05/81)+1) * (0.879-np.sqrt(abs(0.313**2 * (1 - ((y_3d-0.071)**2)/(0.213**2)))))
    z_3d = 0.02
    #point_str = str(x_3d) + ' ' + str(y_3d) + ' ' + str(z_3d)
    point_str = str(0.566) + ' ' + str(0.04) + ' ' + str(z_3d)
    print('String of endpoints: ', point_str)
    os.system('python3 go_to_cartesian_pose.py -p 0.620 -0.118 0.02 -o 0.407 0.913 0.015 -0.026 --linear_speed 0.3 --linear_accel 0.3')
    #os.system('python3 go_to_cartesian_pose.py -p 0.566 0.04 0.02 -o 0.684 0.729 0.0 0.0 --linear_speed 0.3 --linear_accel 0.3')

    print('Closing Gripper')
    right_gripper.close(0.005)
    rospy.sleep(1.0)
    print('Gripper should be closed')

    z_3d_prime = 0.5
    point_str_new = str(x_3d) + ' ' + str(y_3d) + ' ' + str(z_3d_prime)
    #os.system('python3 go_to_cartesian_pose.py -p 0.566 0.04 0.52 -o 0.684 0.729 0.0 0.0 --linear_speed 0.3 --linear_accel 0.3')
    os.system('python3 go_to_cartesian_pose.py -p 0.499 -0.451 0.613 -o 0.0 1.0 0.0 0.0 --linear_speed 0.3 --linear_accel 0.3')

    tray_x = 0.75
    tray_y = -0.412
    tray_z = 0.04
    point_str_tray = str(tray_x) + ' ' + str(tray_y) + ' ' + str(tray_z)
    os.system('python3 go_to_cartesian_pose.py -p ' + point_str_tray + ' -o 0.0 1.0 0.0 0.0 --linear_speed 0.3 --linear_accel 0.3')

    right_gripper.open()
    rospy.sleep(1.0)

    os.system('roslaunch intera_examples sawyer_tuck.launch')

if __name__ == '__main__':
    centroid_x = int(sys.argv[1])
    centroid_y = int(sys.argv[2])
    main((centroid_x, centroid_y))