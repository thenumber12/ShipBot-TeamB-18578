# -*- coding: utf-8 -*-
"""
Created on Tue May 11 10:56:30 2021

@author: siddg
"""

import hebi
import numpy as np
import matplotlib.pyplot as plt
import hebiMath as hb
#import mayavi.mlab as mlab
#import moviepy.editor as mpy
from time import sleep

pi = np.pi
cos = np.cos
sin = np.sin
exp = np.exp

q = 0.2
test_pose = np.array([0, pi/2 + pi/6, -pi/6, -pi/2, 0])
rest_pose = np.array([ 0.03184748, -0.14459251, -2.85362004,  -pi/2, -0.24832714])
#rest_pose = np.array([ 0.03942377,  1.89762294, -2.05142927, -0.75621683, -0.21945286])
upright_pose = np.array([0, pi/2, 0, 0, 0])

def setup():
    lookup = hebi.Lookup()
    group = lookup.get_group_from_names(['ShipBotB'], ['Base', 'Shoulder', 'Elbow', 'Wrist', 'EndEffector'])
    group.feedback_frequency = 100
    group.command_lifetime = 250
    model = hebi.robot_model.import_from_hrdf("HEBI Robot 2021-04-19.hrdf")
    M = group.size
    cmd = hebi.GroupCommand(M)
    return [group, model, cmd, M]

def getFbk(group):
  fbk = group.get_next_feedback()
  return fbk

def getIK(model, target_xyz, initial_angles):
    ee_pos_objective = hebi.robot_model.endeffector_position_objective(target_xyz)
    return model.solve_inverse_kinematics(initial_angles, ee_pos_objective)

def getFKCoords(model, angles):
    return np.array(model.get_forward_kinematics('output', angles))[:, :3, 3]

def setCmd(group, cmd, getFbk, pos_cmd, vel_cmd, acc_cmd):
    position = getFbk(group).position
    effort_cmd = (hb.get_dynamic_comp_efforts(position, pos_cmd, vel_cmd, acc_cmd, model, Ts)) + hb.get_grav_comp_efforts(model, position, [0, 0, 1])    
    cmd.position = pos_cmd
    cmd.velocity = vel_cmd
    cmd.effort = effort_cmd
    group.send_command(cmd)
    

[group, model, cmd, M] = setup()

start_pose = getFbk(group).position

pos_cmd = np.zeros(M)
vel_cmd = np.zeros(M)
acc_cmd = np.zeros(M)

N_states = 10
N_states += 2

pos = np.zeros([M, N_states])
vel = np.zeros([M, N_states])
acc = np.zeros([M, N_states])

vel[:, 1:-1] = np.nan
acc[:, 1:-1] = np.nan

pos[:, 0] = start_pose
pos[:, -1] = rest_pose

#target_xyz = np.array([-0.4042441,  -0.12140763,  0.14616062])

target_xyz = np.array([-0.06928682, -0.13799377,  0.60339546]) + np.array([-0.35, 0.0, -0.25])

#target_xyz = np.array([-0.4, 0, 0.5])

target_pose = getIK(model, target_xyz, upright_pose)

for i in range(1, N_states-1):
    pos[:, i] = upright_pose
    if (2 < i < N_states - 3):
        vel[:, i] = np.zeros(M)


Ts = 0.01
T_step = 1.0
T_init = 1.5
t = np.arange(N_states)*T_step
t[1:] = t[1:] + T_init
t[-1] = t[-1] + T_init
T = np.max(t)
N = np.int(T/Ts)

traj = hebi.trajectory.create_trajectory(t, pos, vel, acc)


X = getFKCoords(model, upright_pose)

Y = getFKCoords(model, target_pose)

j = 0

flag = False

#while(not flag):
#     t = j*Ts
#     pos_cmd, vel_cmd, acc_cmd = traj.get_state(t)
#     if (t < T/3):
#          j += 1
#     else:
#          pos_cmd[3] = -pi/2
#     setCmd(group, cmd, getFbk, pos_cmd, vel_cmd, acc_cmd)

for j in range(N):
     t = j*Ts
     pos_cmd, vel_cmd, acc_cmd = traj.get_state(t)
     setCmd(group, cmd, getFbk, pos_cmd, vel_cmd, acc_cmd)
     #print(getFKCoords(model, getFbk(group).position)[0, :])		
     #print(getFKCoords(model, getFbk(group).position)[-1, :])
     sleep(Ts)
