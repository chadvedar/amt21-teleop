#!/usr/bin/python3
import rospy
import tf

from dynamic_reconfigure.server import Server
from typing import List, Dict
from numpy import pi
from sensor_msgs.msg import JointState
from copy import deepcopy

from JointState import JointFrame
from JointState import get_joint_state_base
from JointState import get_joint_pos_base
from teleop_exo_suit.cfg import joint_framesConfig

def update_jointangle(joint_frames:Dict, num_joint:int, angle:float):
    # joint_frames[num_joint].orientation[2] = angle * pi / 180.0
    joint_frames[num_joint].orientation[2] = angle 

def update_jointframe_callback(config, joint_frames:Dict):
    th_s = [config['theta1'], 
            config['theta2'], 
            config['theta3'], 
            config['theta4'],
            config['theta5'],
            config['theta6'],
            config['theta7']]
    
    for i in range(7):
        update_jointangle(joint_frames, i+1, th_s[i])

    return config

def set_joint_angles(msg:JointState, joint_frames:Dict):
    print(msg.position)
    for num, pos in enumerate(msg.position):
        update_jointangle(joint_frames, num+1, pos)

def publish_joint_transformation(
            bd:           tf.TransformBroadcaster,
            translation:  list,
            orientation:  list,
            now_time:     rospy.Time,
            child_frame:  str,
            parent_frame: str):
    
    assert len(orientation) == 3
    assert len(translation) == 3

    orient = tf.transformations.quaternion_from_euler(*orientation, axes='sxyz')
    bd.sendTransform(translation = translation,
                     rotation    = orient,
                     time        = now_time,
                     child       = child_frame,
                     parent      = parent_frame)

def publish_joint_state(
            publisher         : rospy.Publisher,
            joint_frames      : Dict,
            joint_frames_prev : Dict,
            dt                : float,
            parent_frame      : str = '/map'):
    
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.header.frame_id = parent_frame
    
    num_joints = len(list(filter( lambda x:x>=0, joint_frames.keys() )))
    for i in range(1, num_joints+1):
        joint_state.name.append( joint_frames[i].child_frame )
        joint_state.position.append( joint_frames[i].orientation[2] )
        joint_state.velocity.append( 
            (joint_frames[i].orientation[2] - joint_frames_prev[i].orientation[2])/dt 
        )

    publisher.publish( joint_state )

def init_robot_joint_frames(name:str = '') -> Dict:
    joint_frames : dict = dict()

    J_ANGLE  = [0.0 for _ in range(7)]
    J_OFFSET = [0.1, 0.20, 0.20, 0.07]
    J_TABLE  = [ [ 0.0,         0.0,    0.0,          1.0,   '/map',                f'/{name}/orgin',      -1 ],
                 [ 0.0,         pi/2,   0.0,  J_OFFSET[0],   f'/{name}/orgin',      f'/{name}/fix1',      -14 ],
                 [ J_ANGLE[0],  0.0,    0.0,          0.0,   f'/{name}/fix1',       f'/{name}/shoulder_y',  1 ],
                 [ pi/2,        0.0,    0.0,          0.0,   f'/{name}/shoulder_y', f'/{name}/fix2',       -2 ],
                 [ 0.0,         pi/2,   0.0,          0.0,   f'/{name}/fix2',       f'/{name}/fix3',       -3 ],
                 [ J_ANGLE[1],  0.0,    0.0,          0.0,   f'/{name}/fix3',       f'/{name}/shoulder_x',  2 ],
                 [ pi/2,        0.0,    0.0,          0.0,   f'/{name}/shoulder_x', f'/{name}/fix4',       -4 ],
                 [ 0.0,        -pi/2,   0.0,          0.0,   f'/{name}/fix4',       f'/{name}/fix5',       -5 ],
                 [ J_ANGLE[2],  0.0,    0.0,          0.0,   f'/{name}/fix5',       f'/{name}/shoulder_z',  3 ],
                 [ 0.0,         pi/2,   J_OFFSET[1],  0.0,   f'/{name}/shoulder_z', f'/{name}/fix6',       -6 ],
                 [ J_ANGLE[3],  0.0,    0.0,          0.0,   f'/{name}/fix6',       f'/{name}/elbow',       4 ],
                 [ pi/2,        0.0,    0.0,  J_OFFSET[2],   f'/{name}/elbow',      f'/{name}/fix7',       -7 ],
                 [ 0.0,         pi/2,   0.0,          0.0,   f'/{name}/fix7',       f'/{name}/fix8',       -8 ],
                 [ J_ANGLE[4],  0.0,    0.0,          0.0,   f'/{name}/fix8',       f'/{name}/wrist_y',     5 ],
                 [ pi/2,        0.0,    0.0,          0.0,   f'/{name}/wrist_y',    f'/{name}/fix9',       -9 ],
                 [ 0.0,         pi/2,   0.0,          0.0,   f'/{name}/fix9',       f'/{name}/fix10',     -10 ],
                 [ J_ANGLE[5],  0.0,    0.0,          0.0,   f'/{name}/fix10',      f'/{name}/wrist_z',     6 ],
                 [ pi/2,        0.0,    0.0,          0.0,   f'/{name}/wrist_z',    f'/{name}/fix11',     -11 ],
                 [ 0.0,         pi/2,   0.0,          0.0,   f'/{name}/fix11',      f'/{name}/fix12',     -12 ],
                 [ J_ANGLE[6],  0.0,    0.0,          0.0,   f'/{name}/fix12',      f'/{name}/wrist_x',     7 ],
                 [ 0.0,         0.0,    0.0,  J_OFFSET[3],   f'/{name}/wrist_x',    f'/{name}/hand',      -13 ],               
                 ]

    for joint in J_TABLE:
        joint_frame : JointFrame = JointFrame(
            orientation  = [ joint[1], 0.0, joint[0] ],
            translation  = [ joint[3], 0.0, joint[2] ],
            child_frame  = joint[5],
            parent_frame = joint[4]  
        )
        joint_frames[ joint[6] ] = joint_frame

    return joint_frames

if __name__ == "__main__":
    rospy.init_node('robot_bringup')
    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    tf_prefix : str = ''
    if rospy.has_param('/exo_suit/name'):
        tf_prefix = rospy.get_param('/exo_suit/name')

    rate = rospy.Rate(100.0)

    joint_frames : Dict = init_robot_joint_frames(name = tf_prefix)
    bds = [tf.TransformBroadcaster() for _ in range(len(joint_frames))]

    jointframes_server = Server(joint_framesConfig, 
        lambda config, level: update_jointframe_callback(config, joint_frames)
    )

    rospy.Subscriber('/set_joint_angles', JointState, lambda msg : set_joint_angles(msg, joint_frames) )

    now_time = rospy.Time.now()
    prev_time = now_time

    joint_frames_prev = deepcopy(joint_frames)

    while not rospy.is_shutdown():
        now_time = rospy.Time.now()
        dt = (now_time - prev_time).to_sec()

        # for i in range(1,5):
        #     joint_frames[i].orientation[2] += 0.25*dt

        for joint_number, joint_frame, bd in list( zip(joint_frames.keys(), joint_frames.values(), bds) ):
            publish_joint_transformation(
                bd           = bd,
                translation  = joint_frame.translation,
                orientation  = joint_frame.orientation,
                now_time     = now_time,
                child_frame  = joint_frame.child_frame,
                parent_frame = joint_frame.parent_frame
            )

        publish_joint_state(
            publisher         = joint_state_pub,
            joint_frames      = joint_frames,
            joint_frames_prev = joint_frames_prev,
            dt                = dt,
            parent_frame      = f'/{tf_prefix}/orgin'
        )

        joint_frames_prev = deepcopy(joint_frames)
        prev_time = now_time

        rate.sleep()