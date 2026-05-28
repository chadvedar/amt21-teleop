#!/usr/bin/python3
import numpy as np
import sympy as sp

class ExoKinematic:
    def __init__(self):
        
        print("ready")
        
    def compute_fk(self, current_pos: np.array):
        th1 = current_pos[0]
        th2 = current_pos[1]
        th3 = current_pos[2]
        th4 = current_pos[3]
        th5 = current_pos[4]
        th6 = current_pos[5]
        th7 = current_pos[6]

        J00 = 0.07*(-(-(-(np.sin(th1)*np.sin(th3) + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.sin(th4) + np.cos(th1)*np.cos(th2)*np.cos(th4))*np.sin(th5) \
                + (-np.sin(th1)*np.cos(th3) + np.sin(th2)*np.sin(th3)*np.cos(th1))*np.cos(th5))*np.sin(th6) + ((np.sin(th1)*np.sin(th3) + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.cos(th4) \
                + np.sin(th4)*np.cos(th1)*np.cos(th2))*np.cos(th6))*np.cos(th7) + 0.07*((-(np.sin(th1)*np.sin(th3) + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.sin(th4) \
                + np.cos(th1)*np.cos(th2)*np.cos(th4))*np.cos(th5) + (-np.sin(th1)*np.cos(th3) + np.sin(th2)*np.sin(th3)*np.cos(th1))*np.sin(th5))*np.sin(th7) \
                + 0.2*(np.sin(th1)*np.sin(th3) + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.cos(th4) + 0.2*np.sin(th4)*np.cos(th1)*np.cos(th2) + 0.2*np.cos(th1)*np.cos(th2)
        
        J01 = 0.07*((-np.sin(th2)*np.sin(th4) + np.cos(th2)*np.cos(th3)*np.cos(th4))*np.sin(th1)*np.cos(th6) - (np.sin(th2)*np.sin(th5)*np.cos(th4) + np.sin(th3)*np.cos(th2)*np.cos(th5) \
                + np.sin(th4)*np.sin(th5)*np.cos(th2)*np.cos(th3))*np.sin(th1)*np.sin(th6))*np.cos(th7) + 0.07*(-np.sin(th2)*np.cos(th4)*np.cos(th5) + np.sin(th3)*np.sin(th5)*np.cos(th2) \
                - np.sin(th4)*np.cos(th2)*np.cos(th3)*np.cos(th5))*np.sin(th1)*np.sin(th7) - 0.2*np.sin(th1)*np.sin(th2)*np.sin(th4) - 0.2*np.sin(th1)*np.sin(th2) \
                + 0.2*np.sin(th1)*np.cos(th2)*np.cos(th3)*np.cos(th4)
        
        J02 = 0.07*(-(-(np.sin(th1)*np.sin(th2)*np.sin(th3) + np.cos(th1)*np.cos(th3))*np.sin(th4)*np.sin(th5) + (np.sin(th1)*np.sin(th2)*np.cos(th3) \
                - np.sin(th3)*np.cos(th1))*np.cos(th5))*np.sin(th6) - (np.sin(th1)*np.sin(th2)*np.sin(th3) + np.cos(th1)*np.cos(th3))*np.cos(th4)*np.cos(th6))*np.cos(th7) \
                + 0.07*((np.sin(th1)*np.sin(th2)*np.sin(th3) + np.cos(th1)*np.cos(th3))*np.sin(th4)*np.cos(th5) + (np.sin(th1)*np.sin(th2)*np.cos(th3) \
                - np.sin(th3)*np.cos(th1))*np.sin(th5))*np.sin(th7) - 0.2*(np.sin(th1)*np.sin(th2)*np.sin(th3) + np.cos(th1)*np.cos(th3))*np.cos(th4)
    
        J03 = 0.07*((-(np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.sin(th4) + np.sin(th1)*np.cos(th2)*np.cos(th4))*np.cos(th6) \
                + (-(np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.cos(th4) - np.sin(th1)*np.sin(th4)*np.cos(th2))*np.sin(th5)*np.sin(th6))*np.cos(th7) \
                + 0.07*(-(np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.cos(th4) - np.sin(th1)*np.sin(th4)*np.cos(th2))*np.sin(th7)*np.cos(th5) \
                - 0.2*(np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.sin(th4) + 0.2*np.sin(th1)*np.cos(th2)*np.cos(th4)

        J04 = 0.07*(-(-(np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.sin(th4) + np.sin(th1)*np.cos(th2)*np.cos(th4))*np.sin(th5) \
                + (np.sin(th1)*np.sin(th2)*np.sin(th3) + np.cos(th1)*np.cos(th3))*np.cos(th5))*np.sin(th7) - 0.07*(-(-(np.sin(th1)*np.sin(th2)*np.cos(th3) \
                - np.sin(th3)*np.cos(th1))*np.sin(th4) + np.sin(th1)*np.cos(th2)*np.cos(th4))*np.cos(th5) - (np.sin(th1)*np.sin(th2)*np.sin(th3) \
                + np.cos(th1)*np.cos(th3))*np.sin(th5))*np.sin(th6)*np.cos(th7)
        
        J05 = 0.07*(-(-(-(np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.sin(th4) + np.sin(th1)*np.cos(th2)*np.cos(th4))*np.sin(th5) + (np.sin(th1)*np.sin(th2)*np.sin(th3) \
                + np.cos(th1)*np.cos(th3))*np.cos(th5))*np.cos(th6) - ((np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.cos(th4) \
                + np.sin(th1)*np.sin(th4)*np.cos(th2))*np.sin(th6))*np.cos(th7)
                       
        J06 = -0.07*(-(-(-(np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.sin(th4) + np.sin(th1)*np.cos(th2)*np.cos(th4))*np.sin(th5) + (np.sin(th1)*np.sin(th2)*np.sin(th3) \
                + np.cos(th1)*np.cos(th3))*np.cos(th5))*np.sin(th6) + ((np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.cos(th4) \
                + np.sin(th1)*np.sin(th4)*np.cos(th2))*np.cos(th6))*np.sin(th7) + 0.07*((-(np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.sin(th4) \
                + np.sin(th1)*np.cos(th2)*np.cos(th4))*np.cos(th5) + (np.sin(th1)*np.sin(th2)*np.sin(th3) + np.cos(th1)*np.cos(th3))*np.sin(th5))*np.cos(th7)
        
        J10 = 0.0

        J11 = 0.07*(-(-(-np.sin(th2)*np.sin(th4)*np.cos(th3) + np.cos(th2)*np.cos(th4))*np.sin(th5) + np.sin(th2)*np.sin(th3)*np.cos(th5))*np.sin(th6) + (np.sin(th2)*np.cos(th3)*np.cos(th4) \
                + np.sin(th4)*np.cos(th2))*np.cos(th6))*np.cos(th7) + 0.07*((-np.sin(th2)*np.sin(th4)*np.cos(th3) + np.cos(th2)*np.cos(th4))*np.cos(th5) \
                + np.sin(th2)*np.sin(th3)*np.sin(th5))*np.sin(th7) + 0.2*np.sin(th2)*np.cos(th3)*np.cos(th4) + 0.2*np.sin(th4)*np.cos(th2) + 0.2*np.cos(th2)

        J12 = -0.07*(np.sin(th3)*np.sin(th4)*np.cos(th5) + np.sin(th5)*np.cos(th3))*np.sin(th7)*np.cos(th2) + 0.07*(-np.sin(th3)*np.sin(th4)*np.sin(th5)*np.sin(th6) \
                + np.sin(th3)*np.cos(th4)*np.cos(th6) + np.sin(th6)*np.cos(th3)*np.cos(th5))*np.cos(th2)*np.cos(th7) + 0.2*np.sin(th3)*np.cos(th2)*np.cos(th4)
        
        J13 = 0.07*(-np.sin(th2)*np.sin(th4) + np.cos(th2)*np.cos(th3)*np.cos(th4))*np.sin(th7)*np.cos(th5) + 0.07*((-np.sin(th2)*np.sin(th4) \
                + np.cos(th2)*np.cos(th3)*np.cos(th4))*np.sin(th5)*np.sin(th6) + (np.sin(th2)*np.cos(th4) + np.sin(th4)*np.cos(th2)*np.cos(th3))*np.cos(th6))*np.cos(th7) \
                + 0.2*np.sin(th2)*np.cos(th4) + 0.2*np.sin(th4)*np.cos(th2)*np.cos(th3)

        J14 = 0.07*(-(np.sin(th2)*np.cos(th4) + np.sin(th4)*np.cos(th2)*np.cos(th3))*np.sin(th5) - np.sin(th3)*np.cos(th2)*np.cos(th5))*np.sin(th7) - 0.07*(-(np.sin(th2)*np.cos(th4) \
                + np.sin(th4)*np.cos(th2)*np.cos(th3))*np.cos(th5) + np.sin(th3)*np.sin(th5)*np.cos(th2))*np.sin(th6)*np.cos(th7)

        J15 = 0.07*(-(-(np.sin(th2)*np.cos(th4) + np.sin(th4)*np.cos(th2)*np.cos(th3))*np.sin(th5) - np.sin(th3)*np.cos(th2)*np.cos(th5))*np.cos(th6) - (np.sin(th2)*np.sin(th4) \
                - np.cos(th2)*np.cos(th3)*np.cos(th4))*np.sin(th6))*np.cos(th7)
                                                                                                                                       
        J16 = -0.07*(-(-(np.sin(th2)*np.cos(th4) + np.sin(th4)*np.cos(th2)*np.cos(th3))*np.sin(th5) - np.sin(th3)*np.cos(th2)*np.cos(th5))*np.sin(th6) + (np.sin(th2)*np.sin(th4) \
                - np.cos(th2)*np.cos(th3)*np.cos(th4))*np.cos(th6))*np.sin(th7) + 0.07*((np.sin(th2)*np.cos(th4) + np.sin(th4)*np.cos(th2)*np.cos(th3))*np.cos(th5) \
                - np.sin(th3)*np.sin(th5)*np.cos(th2))*np.cos(th7)
        
        J20 = 0.07*(-(-(-(np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.sin(th4) + np.sin(th1)*np.cos(th2)*np.cos(th4))*np.sin(th5) \
                + (np.sin(th1)*np.sin(th2)*np.sin(th3) + np.cos(th1)*np.cos(th3))*np.cos(th5))*np.sin(th6) + ((np.sin(th1)*np.sin(th2)*np.cos(th3) \
                - np.sin(th3)*np.cos(th1))*np.cos(th4) + np.sin(th1)*np.sin(th4)*np.cos(th2))*np.cos(th6))*np.cos(th7) + 0.07*((-(np.sin(th1)*np.sin(th2)*np.cos(th3) \
                - np.sin(th3)*np.cos(th1))*np.sin(th4) + np.sin(th1)*np.cos(th2)*np.cos(th4))*np.cos(th5) + (np.sin(th1)*np.sin(th2)*np.sin(th3) + np.cos(th1)*np.cos(th3))*np.sin(th5))*np.sin(th7) \
                + 0.2*(np.sin(th1)*np.sin(th2)*np.cos(th3) - np.sin(th3)*np.cos(th1))*np.cos(th4) + 0.2*np.sin(th1)*np.sin(th4)*np.cos(th2) + 0.2*np.sin(th1)*np.cos(th2)

        J21 = 0.07*((np.sin(th2)*np.sin(th4) - np.cos(th2)*np.cos(th3)*np.cos(th4))*np.cos(th1)*np.cos(th6) + (np.sin(th2)*np.sin(th5)*np.cos(th4) + np.sin(th3)*np.cos(th2)*np.cos(th5) \
                + np.sin(th4)*np.sin(th5)*np.cos(th2)*np.cos(th3))*np.sin(th6)*np.cos(th1))*np.cos(th7) + 0.07*(np.sin(th2)*np.cos(th4)*np.cos(th5) \
                - np.sin(th3)*np.sin(th5)*np.cos(th2) + np.sin(th4)*np.cos(th2)*np.cos(th3)*np.cos(th5))*np.sin(th7)*np.cos(th1) + 0.2*np.sin(th2)*np.sin(th4)*np.cos(th1) \
                + 0.2*np.sin(th2)*np.cos(th1) - 0.2*np.cos(th1)*np.cos(th2)*np.cos(th3)*np.cos(th4)

        J22 = 0.07*(-(-(np.sin(th1)*np.sin(th3) + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.cos(th5) + (-np.sin(th1)*np.cos(th3) + np.sin(th2)*np.sin(th3)*np.cos(th1))*np.sin(th4)*np.sin(th5))*np.sin(th6) \
                + (-np.sin(th1)*np.cos(th3) + np.sin(th2)*np.sin(th3)*np.cos(th1))*np.cos(th4)*np.cos(th6))*np.cos(th7) \
                + 0.07*(-(np.sin(th1)*np.sin(th3) + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.sin(th5) - (-np.sin(th1)*np.cos(th3) + np.sin(th2)*np.sin(th3)*np.cos(th1))*np.sin(th4)*np.cos(th5))*np.sin(th7) \
                + 0.2*(-np.sin(th1)*np.cos(th3) + np.sin(th2)*np.sin(th3)*np.cos(th1))*np.cos(th4)

        J23 = 0.07*(((np.sin(th1)*np.sin(th3) + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.sin(th4) - np.cos(th1)*np.cos(th2)*np.cos(th4))*np.cos(th6) + ((np.sin(th1)*np.sin(th3) \
                + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.cos(th4) + np.sin(th4)*np.cos(th1)*np.cos(th2))*np.sin(th5)*np.sin(th6))*np.cos(th7) + 0.07*((np.sin(th1)*np.sin(th3) \
                + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.cos(th4) + np.sin(th4)*np.cos(th1)*np.cos(th2))*np.sin(th7)*np.cos(th5) + 0.2*(np.sin(th1)*np.sin(th3) \
                + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.sin(th4) - 0.2*np.cos(th1)*np.cos(th2)*np.cos(th4)
        
        J24 = 0.07*(-((np.sin(th1)*np.sin(th3) + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.sin(th4) - np.cos(th1)*np.cos(th2)*np.cos(th4))*np.sin(th5) + (np.sin(th1)*np.cos(th3) \
                - np.sin(th2)*np.sin(th3)*np.cos(th1))*np.cos(th5))*np.sin(th7) - 0.07*(-((np.sin(th1)*np.sin(th3) + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.sin(th4) \
                - np.cos(th1)*np.cos(th2)*np.cos(th4))*np.cos(th5) - (np.sin(th1)*np.cos(th3) - np.sin(th2)*np.sin(th3)*np.cos(th1))*np.sin(th5))*np.sin(th6)*np.cos(th7)
        
        J25 = 0.07*(-(-((np.sin(th1)*np.sin(th3) + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.sin(th4) - np.cos(th1)*np.cos(th2)*np.cos(th4))*np.sin(th5) \
                + (np.sin(th1)*np.cos(th3) - np.sin(th2)*np.sin(th3)*np.cos(th1))*np.cos(th5))*np.cos(th6) - (-(np.sin(th1)*np.sin(th3) \
                + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.cos(th4) - np.sin(th4)*np.cos(th1)*np.cos(th2))*np.sin(th6))*np.cos(th7)
        
        J26 = -0.07*(-(-((np.sin(th1)*np.sin(th3) + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.sin(th4) - np.cos(th1)*np.cos(th2)*np.cos(th4))*np.sin(th5) \
                + (np.sin(th1)*np.cos(th3) - np.sin(th2)*np.sin(th3)*np.cos(th1))*np.cos(th5))*np.sin(th6) + (-(np.sin(th1)*np.sin(th3) \
                + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.cos(th4) - np.sin(th4)*np.cos(th1)*np.cos(th2))*np.cos(th6))*np.sin(th7) + 0.07*(((np.sin(th1)*np.sin(th3) \
                + np.sin(th2)*np.cos(th1)*np.cos(th3))*np.sin(th4) - np.cos(th1)*np.cos(th2)*np.cos(th4))*np.cos(th5) + (np.sin(th1)*np.cos(th3) - np.sin(th2)*np.sin(th3)*np.cos(th1))*np.sin(th5))*np.cos(th7)

        J = np.array([
            [J00, J01, J02, J03, J04, J05, J06],
            [J10, J11, J12, J13, J14, J15, J16],
            [J20, J21, J22, J23, J24, J25, J26]
        ])
        return J

    def compute_ee_velocities(self, current_pos: np.ndarray, current_ang_vel: np.ndarray):
        current_ang_vel = np.reshape(current_ang_vel, (7,1))
        # print(current_ang_vel)
        J = self.compute_fk(current_pos)
        ee_vels = np.dot(J, current_ang_vel)
   
        return ee_vels
