#!/usr/bin/env python3
from xmlrpc.client import boolean
import rospy
import time
from geometry_msgs.msg import Twist 
import termios, sys, os
from dynamixel_workbench_msgs.srv import DynamixelCommand
from spatialmath import *
from spatialmath.base import *
import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb

TERMIOS = termios
# definir q1, definir el punto actual. 
def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c

def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))


def mapfun(value,fromLow,fromHigh,toLow,toHigh):
    output = int((value - fromLow)* (toHigh - toLow) / (fromHigh - fromLow) + toLow)
    return output

def inv_kci(T):
    l = np.array([14.5, 10.7, 10.7, 9])
    Tw = T-(l[3]*T[0:4,2]).reshape(4,1)
    q1 = np.arctan2(Tw[1,3],Tw[0,3])
    # Solucion 2R
    h = Tw[2,3] - l[0]
    r = np.sqrt(Tw[0,3]**2 + Tw[1,3]**2)
    # Codo abajo
    the3 = np.arccos((r**2+h**2-l[1]**2-l[2]**2)/(2*l[1]*l[2]))
    the2 = np.arctan2(h,r) - np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))
    q2d = -(np.pi/2-the2)
    q3d = the3

    # Codo arriba
    the2 = np.arctan2(h,r) + np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))
    q2u = -(np.pi/2-the2)
    q3u = -the3

    # Solucion q4
    Rp = (rotz(q1).T).dot(T[0:3,0:3])
    pitch = np.arctan2(Rp[2,0],Rp[0,0])
    q4d = pitch - q2d - q3d
    q4u = pitch - q2u - q3u
    if q4u > (7/6)*np.pi:
        q4u = q4u-2*np.pi
    qinv = np.empty((1,4))
    qinv[:] =np.NaN
    qinv[0,:] = np.array([q1*180/3.1416,q2u*180/3.1416,q3u*180/3.1416, q4u*180/3.1416])
    return qinv

def give_Traj(initia_pos, axe_movement, q1, MLD, MLA, n_points):
    print(initia_pos)
    initial_pos_matrix = SE3(initia_pos[0],initia_pos[1], initia_pos[2])*SE3.Rz(initia_pos[3], unit='deg')*SE3.Ry(initia_pos[4] ,unit='deg')
    if axe_movement == 1:
        future_pos = SE3(initia_pos[0]+ MLD,initia_pos[1], initia_pos[2])*SE3.Rz(initia_pos[3], unit='deg')*SE3.Ry(initia_pos[4], unit='deg')
        new_position = initia_pos
        new_position[0] = initia_pos[0]+ MLD
    elif axe_movement == 2:
        future_pos = SE3(initia_pos[0],initia_pos[1]+ MLD, initia_pos[2])*SE3.Rz(initia_pos[3], unit='deg')*SE3.Ry(initia_pos[4], unit='deg')
        new_position = initia_pos
        new_position[1] = initia_pos[1]+ MLD
    elif axe_movement == 3:
        future_pos = SE3(initia_pos[0],initia_pos[1], initia_pos[2]+MLD)*SE3.Rz(initia_pos[3], unit='deg')*SE3.Ry(initia_pos[4], unit='deg')
        new_position = initia_pos
        new_position[2] = initia_pos[2]+ MLD
    elif axe_movement == 4:
        future_pos = SE3(initia_pos[0],initia_pos[1], initia_pos[2])*SE3.Rz(initia_pos[3], unit='deg')*SE3.Ry(initia_pos[4]+MLA ,unit='deg')
        new_position = initia_pos
        new_position[4] = initia_pos[4]+ MLA
    print('Posicion Final')
    print(new_position)
    Ts = rtb.tools.trajectory.ctraj(initial_pos_matrix, future_pos, n_points)
    #print(initial_pos_matrix)
    #print(future_pos)
    Traj = np.zeros((n_points,4))
    for i in range(0,n_points):
        Traj[i,:] = inv_kci(Ts[i].A)
    
    return Traj, n_points, new_position

def move(Total_Pos, iterations):
    for i in range(0,iterations):
        #print(i)
        jointCommand('', 1, 'Goal_Position', mapfun(Total_Pos[i,0],-150,150,0,1023), 0.5)
        jointCommand('', 2, 'Goal_Position', mapfun(Total_Pos[i,1],-150,150,0,1023), 0.5)
        jointCommand('', 3, 'Goal_Position', mapfun(Total_Pos[i,2],-150,150,0,1023), 0.5)
        jointCommand('', 4, 'Goal_Position', mapfun(Total_Pos[i,3],-150,150,0,1023), 0.5)
        #print(Total_Pos[i,:])
        print(Total_Pos[i])
        



if __name__ == '__main__':
    try:
        # Torque_Limit (0,1023)

        #Postura de la herramienta [x,y,y,q1,rot y], COROOBORAR
        aux_position = [22,0,17,0,140]
        # Codo arriba = 0, codo abajo = 1
        # Trayectoria
        #move([[0,-30,-60,-50]], 0)
        jointCommand('', 1, 'Torque_Limit', 350, 0)
        jointCommand('', 1, 'Goal_Position', mapfun(0,-150,150,0,1023), 0.5)
        #jointCommand('', 2, 'Torque_Limit', 500, 0)
        jointCommand('', 2, 'Goal_Position', mapfun(-30,-150,150,0,1023), 0.5)
        #jointCommand('', 3, 'Torque_Limit', 350, 0)
        jointCommand('', 3, 'Goal_Position', mapfun(-60,-150,150,0,1023), 0.5)
        jointCommand('', 4, 'Goal_Position', mapfun(-50,-150,150,0,1023), 0.5)
        T = SE3(aux_position[0],aux_position[1], aux_position[2])*SE3.Rz(aux_position[3], unit='deg')*SE3.Ry(aux_position[4], unit='deg')
        l = np.array([14.5, 10.7, 10.7, 9])
        Tw = T.A-(l[3]*T.A[0:4,2]).reshape(4,1)
        q1 = np.arctan2(Tw[1,3],Tw[0,3])
        #print(q1)
        i=1
        delta_translation = 5
        delta_rotation = 30
        pasos = 5
        while(1):
            Tec=getkey()
            
            if Tec==b'w':
                i=i+1
                if i == 5:
                    i=1
                if i == 1:
                    print('Está en el movimiento en  translaciónx')
                elif i == 2:
                    print('Está en el movimiento en translación y')
                elif i == 3:
                    print('Está en el movimiento translación en z')
                elif i == 4:
                    print('Está en el movimiento rotación en y respecto q1')
            if Tec==b's':
                i=i-1
                if i == 0:
                    i=4
                if i == 1:
                    print('Está en el movimiento en  translaciónx')
                elif i == 2:
                    print('Está en el movimiento en translación y')
                elif i == 3:
                    print('Está en el movimiento translación en z')
                elif i == 4:
                    print('Está en el movimiento rotación en y respecto q1')

            if Tec==b'a':
                if i == 1:
                    Traj, ponins, aux_position= give_Traj(aux_position, i, q1, -delta_translation, 0, pasos)
        
                elif i == 2:
                    Traj, ponins, aux_position = give_Traj(aux_position, i, q1, -delta_translation, 0, pasos)

                elif i == 3:
                    Traj, ponins, aux_position= give_Traj(aux_position, i, q1, -delta_translation, 0, pasos)
                    
                elif i ==4:
                    Traj, ponins, aux_position = give_Traj(aux_position, i, q1, 0, -delta_rotation, pasos)
                   
                move(Traj, ponins)
                
            if Tec==b'd':
                if i == 1:
                    Traj, ponins, aux_position = give_Traj(aux_position, i, q1, delta_translation, 0, pasos)
                    
                elif i == 2:
                    Traj, ponins, aux_position = give_Traj(aux_position, i, q1, delta_translation, 0, pasos)
                    
                elif i == 3:
                    Traj, ponins, aux_position = give_Traj(aux_position, i, q1, delta_translation, 0, pasos)
                    
                elif i ==4:
                    Traj, ponins, aux_position = give_Traj(aux_position, i, q1, 0, delta_rotation, pasos)
                    
                move(Traj,  ponins)
                
            if Tec==b'\x1b':
                break   
        
    except rospy.ROSInterruptException:
        pass
    #source devel/setup.bash