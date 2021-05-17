# -*- coding: utf-8 -*-
"""
Created on Sat May  8 16:35:11 2021

@author: siddg
"""

import numpy as np
import matplotlib.pyplot as plt
import hebi
from mpl_toolkits.mplot3d import Axes3D

pi = np.pi
cos = np.cos
sin = np.sin

f_x = lambda x: sin(x)**3

f_y = lambda x: (13*cos(x) - 5*cos(2*x) - 2*cos(3*x) - cos(4*x))/16

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

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_aspect('auto')

l = 0.6

angles = getFbk(group).position

initial_angles = angles

model = hebi.robot_model.import_from_hrdf("HEBI Robot 2021-04-19.hrdf")

X = np.array(model.get_forward_kinematics('output', angles))[:, :3, 3]

base_xyz = X[0, :]

target_xyz = base_xyz + np.array([0.1, 0.1, 0.3])

N = 1000

theta = 2*pi*np.arange(N)/N

xyz = np.zeros([N, 3])

xyz[:, 0] = target_xyz[0] + np.zeros(N)

xyz[:, 1] = target_xyz[1] + f_x(theta)/8

xyz[:, 2] = target_xyz[2] + f_y(theta)/8

k = 100

ee_pos_objective = hebi.robot_model.endeffector_position_objective(xyz[k, :])

ik_result_angles = model.solve_inverse_kinematics(initial_angles, ee_pos_objective)

Y = np.array(model.get_forward_kinematics('output', ik_result_angles))[:, :3, 3]

ax.plot(X[:, 0], X[:, 1], X[:, 2], c = 'tab:blue', marker = 'o')

#ax.plot(Y[:, 0], Y[:, 1], Y[:, 2], c = 'tab:red', marker = 'o')

#ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], c = 'tab:orange')

#ax.scatter(xyz[k, 0], xyz[k, 1], xyz[k, 2], c = 'tab:green')

ax.plot([0, l], [0, 0], [0, 0], c = 'r')
ax.plot([0, 0], [0, l], [0, 0], c = 'g')
ax.plot([0, 0], [0, 0], [0, l], c = 'b')
ax.set_xlim3d(-l, l)
ax.set_ylim3d(-l, l)
ax.set_zlim3d(-l, l)
plt.grid()
plt.show()
