# -*- coding: utf-8 -*-
"""
Created on Mon Apr 19 21:42:01 2021

@author: sgangadh
"""


import hebi

import numpy

from time import sleep


pi = numpy.pi

cos = numpy.cos

sin = numpy.sin

def get_grav_comp_efforts(robot_model, positions, gravityVec):

    # Normalize gravity vector (to 1g, or 9.8 m/s^2)

    normed_gravity = gravityVec / numpy.linalg.norm(gravityVec) * 9.81



    jacobians = robot_model.get_jacobians('CoM', positions)

    # Get torque for each module

    # comp_torque = J' * wrench_vector

    # (for each frame, sum this quantity)

    comp_torque = numpy.zeros((robot_model.dof_count, 1))



    # Wrench vector

    wrench_vec = numpy.zeros(6)  # For a single frame; this is (Fx/y/z, tau x/y/z)

    num_frames = robot_model.get_frame_count('CoM')



    for i in range(num_frames):

        # Add the torques for each joint to support the mass at this frame

        wrench_vec[0:3] = normed_gravity * robot_model.masses[i]

        comp_torque += numpy.matmul(jacobians[i].transpose(), numpy.reshape(wrench_vec, (6, 1)))



    return numpy.squeeze(comp_torque)




def setup():
    lookup = hebi.Lookup()
    
    group = lookup.get_group_from_names(['ShipBotB'], ['Base', 'Shoulder', 'Elbow', 'Wrist', 'EndEffector'])
    
    model = hebi.robot_model.import_from_hrdf("HEBI Robot 2021-04-19.hrdf")
    
    return [group, model]


def get_fbk(group):

  fbk = group.get_next_feedback()

  if fbk is None:

    print('Could not get feedback')

    raise RuntimeError('Could not get feedback')

  return fbk



def setCommand(model, fbk, group, t):
    
    cmd = hebi.GroupCommand(5)
    
    cmd.position = numpy.array([pi/2 + pi/4*sin(2*pi*0.1*t), pi/2 + pi/4*sin(2*pi*0.1*t), pi/4*sin(2*pi*0.1*t), pi/2, 0])
    
    #cmd.position = numpy.array([0.5, 0, -2.7, pi/2, 0])
    
    #cmd.effort = get_grav_comp_efforts(model, fbk.position, [0.0, 0.0, 1.0])
    
    group.send_command(cmd)
    
    

Ts = 1e-1

[group, model] = setup()
    
    
for i in range(2000):
    t = i*0.05
    fbk = get_fbk(group)
    setCommand(model, fbk, group, t)
    sleep(0.05)
    
for i in range(10000):
    fbk = get_fbk(group)    
    cmd = hebi.GroupCommand(5)
    cmd.position = numpy.array([0.5, 0, -2.7, pi/2, 0])
    group.send_command(cmd)
    
    
    
    