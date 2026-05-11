#!/usr/bin/python3
import rospy
import numpy as np
import time
from ExoKinematic import ExoKinematic

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

joint_angles = JointState()
current_joint_angles = np.zeros(7)
prev_joint_angles = np.zeros(7)
joint_vels = np.zeros(7)

def deg_to_rad(deg):
    return np.pi * deg / 180.0
def arm_position_callback(msg:Float32MultiArray):
    global joint_angles, current_joint_angles, prev_joint_angles
    global update_time, prev_time, joint_vels

    update_time = time.time()
    dt = update_time - prev_time

    current_joint_angles[0] = deg_to_rad(msg.data[0])
    current_joint_angles[1] = deg_to_rad(-msg.data[1])
    current_joint_angles[2] = deg_to_rad(0.0)
    current_joint_angles[3] = deg_to_rad(msg.data[2])
    current_joint_angles[4] = deg_to_rad(0.0)
    current_joint_angles[5] = deg_to_rad(msg.data[4] - 270.0)
    current_joint_angles[6] = deg_to_rad(0.0)

    joint_vels = (current_joint_angles - prev_joint_angles)/dt

    prev_joint_angles = current_joint_angles.copy()
    prev_time = update_time

    joint_angles.position = list(current_joint_angles)

update_time = time.time()
prev_time = update_time
if __name__ == "__main__":
    rospy.init_node("motion_tracking_sim")
    rate = rospy.Rate(50)

    rospy.Subscriber('/amt21/positions', Float32MultiArray, arm_position_callback)

    robot_cmd_pub = rospy.Publisher("/set_end_effector_velocity", Float32MultiArray, queue_size=10)
    set_joint_angle_pub = rospy.Publisher('/set_joint_angles', JointState, queue_size=10)

    cmd = Float32MultiArray()

    kin = ExoKinematic()
    vel_thresh = 0.02

    while not rospy.is_shutdown():
        ee_vel = kin.compute_ee_velocities(
            current_pos = current_joint_angles,
            current_ang_vel = np.array(
                [
                    joint_vels[0],
                    joint_vels[1],
                    joint_vels[3],
                    joint_vels[4],
                    joint_vels[5],
                    joint_vels[6],
                ]
            ) 
        )
        
        print(ee_vel)

        cmd.data = [-ee_vel[1][0], ee_vel[0][0], ee_vel[2][0]]

        set_joint_angle_pub.publish(joint_angles)

        if abs(ee_vel[1][0]) < vel_thresh or \
            abs(ee_vel[0][0]) < vel_thresh or \
            abs(ee_vel[2][0]) < vel_thresh: 
            robot_cmd_pub.publish(cmd)
        
        rate.sleep()