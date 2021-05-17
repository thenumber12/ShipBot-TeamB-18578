# -*- coding: utf-8 -*-
"""
Created on Fri May  7 01:13:15 2021

@author: siddg
"""

import hebi
import numpy as np
import matplotlib.pyplot as plt
from time import sleep
import advancedTests as at

pi = np.pi
cos = np.cos
sin = np.sin
exp = np.exp


def setup():
    lookup = hebi.Lookup()
    group = lookup.get_group_from_names(['ShipBotB'], ['Base', 'Shoulder', 'Elbow', 'Wrist', 'EndEffector'])
    group.feedback_frequency = 200
    group.command_lifetime = 100
    model = hebi.robot_model.RobotModel()
    model.add_actuator('X5-4')
    model.add_bracket('X5-HeavyBracket','Right-Outside')
    model.add_actuator('X8-9')
    model.add_link('X5', 0.265, pi)
    model.add_actuator('X5-4')
    model.add_link('X5', 0.325, pi)
    model.add_actuator('X5-1')
    model.add_bracket('X5-HeavyBracket','Left-Outside')
    model.add_actuator('X5-1')

    return [group, model]

def getFbk(group):
  fbk = group.get_next_feedback()
  if fbk is None:
    print('Could not get feedback')
    raise RuntimeError('Could not get feedback')
  return fbk

def setCommand(model, fbk, group, t):
	cmd = hebi.GroupCommand(5)
	cmd.position = np.array([pi/2 + pi/6*sin(2*pi*0.05*t), pi/2, 0, pi/2, 0])

	group.send_command(cmd)

[group, model] = setup()

cmd = hebi.GroupCommand(5)

#cmd.control_strategy = 'Strategy3'

#group_info = group.request_info()

#group_info.write_gains("saved_gains.xml")

#cmd.read_gains("saved_gains.xml")

#group.send_command_with_acknowledgement(cmd)

N_states = 6

pos = np.zeros([5, N_states])
vel = np.zeros([5, N_states])
acc = np.zeros([5, N_states])

vel[:, 1:-1] = np.nan
acc[:, 1:-1] = np.nan

start_pos = getFbk(group).position

#pos[:, 0] = start_pos
#pos[:, 1] = np.array([pi/2, pi/2, 0, pi/2, 0])
#pos[:, 2] = np.array([pi/6, pi/2 + pi/6, -pi/6, pi/2, 0])
#pos[:, 3] = np.array([pi/2, pi/2, 0,  pi/2, 0])
#pos[:, 4] = np.array([pi/6, pi/2 + pi/6, -pi/6, pi/2, 0])
#pos[:, 5] = start_pos

pos[:, 0] = start_pos
pos[:, 1] = np.array([0, pi/2, 0, pi/2, 0])
pos[:, 2] = np.array([0, pi/2, 0, pi/2, 0])
pos[:, 3] = np.array([0, pi/2, 0, pi/2, 0])
pos[:, 4] = np.array([0, pi/2, 0, pi/2, 0])
pos[:, 5] = start_pos

Ts = 0.005

T_step = 5.0

T = (N_states-1)*T_step

N = np.int(T/Ts)

t = np.arange(N_states)*T_step

traj = hebi.trajectory.create_trajectory(t, pos, vel, acc)

current_pos = np.zeros([5, N])

gyro = np.zeros([5, 3, N])

pos_cmd = np.zeros(5)
vel_cmd = np.zeros(5)
acc_cmd = np.zeros(5)

for i in range(N):
	t = i*Ts
	pos_cmd, vel_cmd, acc_cmd = traj.get_state(t)
	cmd.position = pos_cmd
	cmd.velocity = vel_cmd
	group.send_command(cmd)
	current_pos[:, i] = getFbk(group).position
	sleep(Ts)

t = np.arange(N)*Ts

plt.figure()

plt.plot(t, current_pos.T)

plt.grid()