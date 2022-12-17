python3 go_to_cartesian_pose.py -p 0.566 0.04 0.02 -o 0.703 0.711 0.0 0.0 --linear_speed 0.3 --linear_accel 0.3
python3 calibrate_gripper.py
python3 go_to_cartesian_pose.py -p 0.566 0.04 0.5 -o 0.703 0.711 0.0 0.0 --linear_speed 0.3 --linear_accel 0.3
