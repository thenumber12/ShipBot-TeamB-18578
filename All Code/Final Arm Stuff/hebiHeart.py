# -*- coding: utf-8 -*-
"""
Created on Tue May 11 10:56:30 2021

@author: siddg
"""

import hebi
import numpy as np
import matplotlib.pyplot as plt
import hebiMath as hb
from time import sleep

pi = np.pi
cos = np.cos
sin = np.sin
exp = np.exp

f_x = lambda x: sin(x)**3

f_y = lambda x: (13*cos(x) - 5*cos(2*x) - 2*cos(3*x) - cos(4*x))/16

test_pose = np.array([0, pi/2 + pi/6, -pi/6, -pi/2, 0])
rest_pose = np.array([ 0.03184748, -0.14459251, -2.85362004,  -pi/2, -0.24832714])
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
    return np.array(model.get_forward_kinematics('output', angles))[-1, :3, 3]

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

N_states = 20
N_states += 2

pos = np.zeros([M, N_states])
vel = np.zeros([M, N_states])
acc = np.zeros([M, N_states])

vel[:, 1:-1] = np.nan
acc[:, 1:-1] = np.nan

pos[:, 0] = start_pose
pos[:, -1] = rest_pose

phi = 2*pi*np.arange(N_states-2)/(N_states-2)

target_xyz = np.zeros([N_states-2, 3])
target_pose = np.zeros([N_states-2, 5])
target_pose[0, :] = upright_pose
X = np.zeros([N_states-2, 3])

for i in range(N_states-2):
	target_xyz[i, :] = np.array([-0.05736467, -0.13856637,  0.6064685 ]) + np.array([0.2*cos(phi[i]), 0, 0.2*sin(phi[i]) - 0.10])
	target_pose[i, :] = getIK(model, target_xyz[i, :], target_pose[i, :])
	X[i, :] = getFKCoords(model, target_pose[i, :])

#target_xyz = np.array([-0.06928682, -0.13799377,  0.60339546]) + np.array([-0.35, 0.0, -0.25])


plt.plot(target_xyz[:, 0], target_xyz[:, 2], c = 'b')

plt.plot(X[:, 0], X[:, 2], c = 'r')

plt.xlim([-0.7, 0.7])

plt.ylim([-0.7, 0.7])

plt.show()


for i in range(1, N_states-1):
    pos[:, i] = np.array([pi/2*(i-1)/(N_states-2), pi/2, 0, 0, 0])


Ts = 0.01
T_step = 0.5
T_init = 2.5
t = np.arange(N_states)*T_step
t[1:] = t[1:] + T_init
t[-1] = t[-1] + T_init
T = np.max(t)
N = np.int(T/Ts)

traj = hebi.trajectory.create_trajectory(t, pos, vel, acc)




#Y = getFKCoords(model, target_pose)


for j in range(N):
     t = j*Ts
     pos_cmd, vel_cmd, acc_cmd = traj.get_state(t)
     setCmd(group, cmd, getFbk, pos_cmd, vel_cmd, acc_cmd)
     #print(getFKCoords(model, getFbk(group).position))
     sleep(Ts)


