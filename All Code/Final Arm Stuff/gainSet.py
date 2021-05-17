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
	model = hebi.robot_model.import_from_hrdf("HEBI Robot 2021-04-19.hrdf")

	return [group, model]


[group, model] = setup()

cmd = hebi.GroupCommand(group.size)

cmd.position_limit_min = -1*np.array([np.inf, np.inf, np.inf, np.inf, np.inf])
cmd.position_limit_max = +1*np.array([np.inf, np.inf, np.inf, np.inf, np.inf])

cmd.read_gains('gains.xml')

group.send_command(cmd)

group_info = group.request_info()

group_info.write_gains("saved_gains.xml")
