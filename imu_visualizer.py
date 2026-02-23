import matplotlib
matplotlib.use('TkAgg')

import serial
import numpy as np
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

PORT = 'COM7'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.01)
time.sleep(2)

latest_q = np.array([0,0,0,1])

def quat_to_rot(q):
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ])

def rot_to_euler(R):
    roll  = np.arctan2(R[2,1], R[2,2])
    pitch = -np.arcsin(R[2,0])
    yaw   = np.arctan2(R[1,0], R[0,0])
    return np.degrees([roll,pitch,yaw])

# Create cube vertices
cube_size = 0.5
vertices = np.array([
    [-cube_size, -cube_size, -cube_size],
    [ cube_size, -cube_size, -cube_size],
    [ cube_size,  cube_size, -cube_size],
    [-cube_size,  cube_size, -cube_size],
    [-cube_size, -cube_size,  cube_size],
    [ cube_size, -cube_size,  cube_size],
    [ cube_size,  cube_size,  cube_size],
    [-cube_size,  cube_size,  cube_size]
])

faces = [
    [0,1,2,3],
    [4,5,6,7],
    [0,1,5,4],
    [2,3,7,6],
    [1,2,6,5],
    [4,7,3,0]
]

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Fixed world axes
ax.quiver(0,0,0,1,0,0,color='gray',linestyle='dashed')
ax.quiver(0,0,0,0,1,0,color='gray',linestyle='dashed')
ax.quiver(0,0,0,0,0,1,color='gray',linestyle='dashed')

while True:

    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        parts = line.split(',')
        if len(parts)==4:
            try:
                latest_q = np.array([float(p) for p in parts])
            except:
                pass

    R = quat_to_rot(latest_q)
    euler = rot_to_euler(R)

    ax.cla()

    # Keep fixed view
    ax.set_xlim([-1.5,1.5])
    ax.set_ylim([-1.5,1.5])
    ax.set_zlim([-1.5,1.5])
    ax.view_init(elev=25, azim=40)

    # Draw world frame
    ax.quiver(0,0,0,1,0,0,color='gray',linestyle='dashed')
    ax.quiver(0,0,0,0,1,0,color='gray',linestyle='dashed')
    ax.quiver(0,0,0,0,0,1,color='gray',linestyle='dashed')

    # Rotate cube
    rotated_vertices = vertices @ R.T

    cube_faces = [[rotated_vertices[idx] for idx in face] for face in faces]
    cube = Poly3DCollection(cube_faces, alpha=0.3, facecolor='cyan')
    ax.add_collection3d(cube)

    # Body axes
    ax.quiver(0,0,0, R[0,0], R[1,0], R[2,0], color='r', linewidth=3)
    ax.quiver(0,0,0, R[0,1], R[1,1], R[2,1], color='g', linewidth=3)
    ax.quiver(0,0,0, R[0,2], R[1,2], R[2,2], color='b', linewidth=3)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Display roll pitch yaw
    ax.text2D(0.05,0.95,
        f"Roll:  {euler[0]:.1f}°\nPitch: {euler[1]:.1f}°\nYaw:   {euler[2]:.1f}°",
        transform=ax.transAxes)

    plt.draw()
    plt.pause(0.02)
