import tf
import rospy
import numpy as np

from dataclasses import dataclass
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion

@dataclass
class JointFrame:
    orientation  : list
    translation  : list
    child_frame  : str
    parent_frame : str

def get_joint_pos_base(listener : tf.TransformListener, joint_str:str, base:str='/map'):
    try:
        now_time = rospy.Time.now()
        listener.waitForTransform(
            base, 
            joint_str, 
            now_time,
            rospy.Duration(0.1)
        )
        trans, rot = listener.lookupTransform(
            base, 
            joint_str,
            rospy.Time(0)
        )
        rot = list(euler_from_quaternion(rot))
    except Exception as e:
        print(f'Unable to get tranforamtion from {joint_str} to {base}')
        print( str(e) )
        return False, None, None

    return True, trans, rot

def get_joint_state_base(listener : tf.TransformListener, joint_state:JointState, dt:float) -> JointState:
    joint_state.header.stamp = rospy.Time.now()
    num_joint = len(joint_state.name)

    for num, joint in list(zip(range(num_joint), joint_state.name)):
        ret, cur_trans, cur_orient = get_joint_pos_base(listener, joint, '/map')
        if ret:
            cur_pos  = cur_orient[2]
            
            prev_pos = joint_state.position[num]
            cur_velocity = (cur_pos - prev_pos)/dt
            
            joint_state.position[num] = cur_pos
            joint_state.velocity[num] = cur_velocity

    return joint_state

def get_joint_multi_state_base(listener : tf.TransformListener, joint_state:JointState, dt:float) -> JointState:
    joint_state.header.stamp = rospy.Time.now()
    num_joint = len(joint_state.name)

    for num, joint in list(zip(range(num_joint), joint_state.name)):
        ret, cur_trans, cur_orient = get_joint_pos_base(listener, joint, '/map')
        if ret:
            cur_pos  = cur_orient[2]
            
            prev_pos = joint_state.position[num]
            cur_velocity = (cur_pos - prev_pos)/dt
            
            joint_state.position[num] = cur_pos
            joint_state.velocity[num] = cur_velocity

    return joint_state
    