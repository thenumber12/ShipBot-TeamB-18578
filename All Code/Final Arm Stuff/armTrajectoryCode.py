# -*- coding: utf-8 -*-
"""
Created on Tue Apr 20 00:46:54 2021

@author: siddg
"""

import numpy as np
import hebi as h
from time import sleep

pi = np.pi; sin = np.sin; cos = np.cos; exp = np.exp

def gravityCompensation(model, pos, g):
    g_norm = 9.81*g / np.linalg.norm(g, 2)
    J = model.get_jacobians('CoM', positions)
    torque = np.zeros(6)
    frames = model.get_frame_count('CoM')
    return 0