import hebi
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

pi = np.pi
cos = np.cos
sin = np.sin
exp = np.exp


def setup():
	lookup = hebi.Lookup()
	group = lookup.get_group_from_names(['ShipBotB'], ['Base', 'Shoulder', 'Elbow', 'Wrist', 'EndEffector'])
	group.feedback_frequency = 100
	group.command_lifetime = 250
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

cmd.read_gains('gains.xml')

group.send_command(cmd)

group_info = group.request_info()

group_info.write_gains("saved_gains.xml")
