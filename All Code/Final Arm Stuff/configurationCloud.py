# -*- coding: utf-8 -*-
"""
Created on Sat May  8 15:50:57 2021

@author: siddg
"""

import numpy as np
import matplotlib.pyplot as plt
import hebi
from mpl_toolkits.mplot3d import Axes3D

pi = np.pi
cos = np.cos
sin = np.sin

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_aspect('auto')

l = 0.75

angles = np.array([0, pi/2, 0, 0, 0])

target_xyz = np.array([0.4, 0.2, 0.5])

initial_angles = np.array([0, pi/2, 0, 0, 0])

model = hebi.robot_model.import_from_hrdf("HEBI Robot 2021-04-19.hrdf")

X = np.array(model.get_forward_kinematics('output', angles))[:, :3, 3]

ee_pos_objective = hebi.robot_model.endeffector_position_objective(target_xyz)

ik_result_joint_angle = model.solve_inverse_kinematics(initial_angles, ee_pos_objective)

N = 7

pos = np.zeros([N**5, 3])

for i1 in range(N):
    for i2 in range(N):
        for i3 in range(N):
            for i4 in range(N):
                for i5 in range(N):
                    angles = np.array([2*pi/N*i1, 2*pi/N*i2, 2*pi/N*i3, 2*pi/N*i4, 2*pi/N*i5])
                    pos[i5 + i4*N + i3*(N**2) + i2*(N**3) + i1*(N**4), :] = np.array(model.get_forward_kinematics('output', angles))[:, :3, 3][-1, :]

#ax.plot(X[:, 0], X[:, 1], X[:, 2], marker = 'o', c = 'b')
                    
ax.scatter(pos[:, 0], pos[:, 1], pos[:, 2], c = 'b', alpha = 0.5)

ax.plot([-l, l], [0, 0], [0, 0], c = 'k')
ax.plot([0, 0], [-l, l], [0, 0], c = 'k')
ax.plot([0, 0], [0, 0], [-l, l], c = 'k')
ax.set_xlim3d(-l, l)
ax.set_ylim3d(-l, l)
ax.set_zlim3d(-l, l)
plt.grid()