import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import hebi
import hebiMath as hb
from mpl_toolkits.mplot3d import Axes3D
from time import sleep
print("Environment Ready")

pi = np.pi
cos = np.cos
sin = np.sin

theta = 18*pi/180

R_rot = np.array([[1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)]])

f_x = lambda x: sin(x)**3

f_y = lambda x: (13*cos(x) - 5*cos(2*x) - 2*cos(3*x) - cos(4*x))/16

A = np.matmul(np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]]), R_rot)

r_cam = np.array([-0.1023, -0.1792, 0.0400])

rest_pose = np.array([ 0.03184748, -0.14459251, -2.85362004,  -pi/2, -0.24832714])

test_pose = np.array([0, pi/2 + pi/4, -pi/4, -pi/2, 0])

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

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, rs.format.z16, 30)
config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** 0)
colorizer = rs.colorizer()


    # Grab camera data
if not False:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    depth_frame = decimate.process(depth_frame)

        # Grab new intrinsics (may be changed by decimation)
    depth_intrinsics = rs.video_stream_profile(
        depth_frame.profile).get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    depth_colormap = np.asanyarray(
        colorizer.colorize(depth_frame).get_data())

    if True:
        mapped_frame, color_source = color_frame, color_image
    else:
        mapped_frame, color_source = depth_frame, depth_colormap

    points = pc.calculate(depth_frame)
    pc.map_to(mapped_frame)

    # Pointcloud data to arrays
    v, t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
    texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
    r_pix = verts.reshape(480, 848, 3)[261, 589, :]
    r_arm = np.matmul(A, r_pix) + r_cam
    #print(np.amin(verts.reshape(480, 848, 3)[:, :, 2]))
    print(r_arm)
    plt.figure()
    plt.subplot(131)
    plt.imshow(verts.reshape(480, 848, 3)[:, :, 0])
    plt.subplot(132)
    plt.imshow(verts.reshape(480, 848, 3)[:, :, 1])
    plt.subplot(133)
    plt.imshow(verts.reshape(480, 848, 3)[:, :, 2])
    plt.show()

[group, model, cmd, M] = setup()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_aspect('auto')

l = 0.6

start_pose = getFbk(group).position

pos_cmd = np.zeros(M)
vel_cmd = np.zeros(M)
acc_cmd = np.zeros(M)

N_states = 8
N_states += 2

pos = np.zeros([M, N_states])
vel = np.zeros([M, N_states])
acc = np.zeros([M, N_states])

vel[:, 1:-1] = np.nan
acc[:, 1:-1] = np.nan

pos[:, 0] = start_pose
pos[:, -1] = rest_pose



target_xyz = r_arm



target_pose = getIK(model, target_xyz, test_pose)

for i in range(1, N_states-1):
    pos[:, i] = target_pose
    if (2 < i < N_states - 3):
        vel[:, i] = np.zeros(M)

Ts = 0.01
T_step = 1.00
T_init = 3.00
t = np.arange(N_states)*T_step
t[1:] = t[1:] + T_init
t[-1] = t[-1] + T_init
T = np.max(t)
N = np.int(T/Ts)

traj = hebi.trajectory.create_trajectory(t, pos, vel, acc)

#for j in range(N):
#     t = j*Ts
#     pos_cmd, vel_cmd, acc_cmd = traj.get_state(t)
#     setCmd(group, cmd, getFbk, pos_cmd, vel_cmd, acc_cmd)
     #print(getFKCoords(model, getFbk(group).position)[0, :])		
     #print(getFKCoords(model, getFbk(group).position)[-1, :])
#     sleep(Ts)

angles = getFbk(group).position

X = getFKCoords(model, angles)

ax.plot(X[:, 0], X[:, 1], X[:, 2], c = 'tab:blue', marker = 'o')
ax.scatter(r_arm[0], r_arm[1], r_arm[2], c = 'tab:green')
ax.plot([0, l], [0, 0], [0, 0], c = 'r')
ax.plot([0, 0], [0, l], [0, 0], c = 'g')
ax.plot([0, 0], [0, 0], [0, l], c = 'b')
ax.set_xlim3d(-l, l)
ax.set_ylim3d(-l, l)
ax.set_zlim3d(-l, l)
plt.grid()
plt.show()
