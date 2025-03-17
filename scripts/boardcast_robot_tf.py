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
    joint_frames[num_joint].orientation[2] = angle * pi / 180.0

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

def init_robot_joint_frames() -> Dict:
    joint_frames : dict = dict()

    J_ANGLE  = [0.0 for _ in range(7)]
    J_OFFSET = [0.1, 0.20, 0.20, 0.07]
    J_TABLE  = [ [ 0.0,         0.0,    0.0,         0.0,   '/map',        '/fix1',       -1 ],
                 [ 0.0,         pi/2,   0.0,         0.0,   '/fix1',       '/fix1.1',   -1.1 ],
                 [ J_ANGLE[0],  0.0,    0.0, J_OFFSET[0],   '/fix1.1',     '/shoulder_x',  1 ],
                 [ pi/2,        0.0,    0.0,         0.0,   '/shoulder_x', '/fix2',       -2 ],
                 [ 0.0,         pi/2,   0.0,         0.0,   '/fix2',       '/fix2.1',   -2.1 ],
                 [ J_ANGLE[1],  0.0,    0.0,         0.0,   '/fix2.1',     '/shoulder_y',  2 ],
                 [ pi/2,        0.0,    0.0,         0.0,   '/shoulder_y', '/fix3',       -3 ],
                 [ 0.0,         pi/2,   0.0,         0.0,   '/fix3',       '/fix3.1',   -3.1 ],
                 [ J_ANGLE[2],  0.0,    0.0,         0.0,   '/fix3.1',     '/shoulder_z',  3 ],
                 [ 0.0,         pi/2,   0.0,         0.0,   '/shoulder_z', '/fix3.2',   -3.2 ],
                 [ J_ANGLE[3],  0.0,  -J_OFFSET[1],  0.0,   '/fix3.2',     '/elbow',       4 ],
                 [ 0.0,         pi/2,   0.0,         0.0,   '/elbow',      '/fix4',       -5 ],
                 [ J_ANGLE[4],  0.0,   J_OFFSET[2],  0.0,   '/fix4',       '/wrist_z',     5 ],
                 [ 0.0,         pi/2,   0.0,         0.0,   '/wrist_z',    '/fix5.1',   -5.1 ],
                 [ J_ANGLE[5],  0.0,    0.0,         0.0,   '/fix5.1',     '/wrist_x',     6 ], 
                 [ pi/2,        0.0,    0.0,         0.0,   '/wrist_x',    '/fix5',       -6 ],
                 [ 0.0,         pi/2,   0.0,         0.0,   '/fix5',       '/fix6',     -6.1 ],
                 [ J_ANGLE[6],  0.0,    0.0,         0.0,   '/fix6',       '/wrist_y',     7 ],
                 [ 0.0,         0.0,    0.0, J_OFFSET[3],   '/wrist_y',    '/hand',       -7 ]]

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

    rate = rospy.Rate(1000.0)

    joint_frames : Dict = init_robot_joint_frames()
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
            dt                = dt
        )

        joint_frames_prev = deepcopy(joint_frames)
        prev_time = now_time

        rate.sleep()