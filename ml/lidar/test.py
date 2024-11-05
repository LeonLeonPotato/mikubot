import subprocess
import json
import numpy as np
import matplotlib.pyplot as plt
import time

process = subprocess.Popen([
    "/Users/leon.zhu/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-cli-macos/pros",
    "terminal"
], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def quaternion_multiply(q1, q2):
    """Multiplies two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

def quaternion_conjugate(q):
    """Returns the conjugate of a quaternion."""
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def rotate_point_by_quaternion(point, quaternion):
    """Rotates a point in 3D space using a quaternion."""
    # Represent the point as a quaternion with 0 as the scalar part
    point_quaternion = np.array([0, *point])
    
    # Perform the rotation: q * p * q^-1
    q_conjugate = quaternion_conjugate(quaternion)
    rotated_point = quaternion_multiply(quaternion_multiply(quaternion, point_quaternion), q_conjugate)
    
    # Return the vector part of the rotated quaternion
    return rotated_point[1:]

def project_point(distance, direction, quaternion):
    """Projects a point from the origin given a distance, direction, and rotation quaternion."""
    # Normalize the direction vector
    direction = np.array(direction)
    direction = direction / np.linalg.norm(direction)
    
    # Define the point at the specified distance along the direction
    point = distance * direction
    
    # Rotate the point using the quaternion
    projected_point = rotate_point_by_quaternion(point, quaternion)
    
    return projected_point

xs, ys, zs = [], [], []

plt.ion()
plt.show()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

scatter = ax.scatter([], [], [], c='r')
ax.set_xlim(-500, 500)
ax.set_ylim(-500, 500)
ax.set_zlim(-500, 500)

while True:
    line = process.stdout.readline().decode("utf-8").strip()

    if "LIDAR" in line:
        data = line.split("LIDAR: ")[1]
        data = json.loads(data)
        conf = data["confidence"]
        distance = data["distance"]
        pitch = data["pitch"]
        yaw = data["yaw"]

        if conf > 10:

            point = np.array([
                distance * np.cos(pitch) * np.cos(yaw),
                distance * np.cos(pitch) * np.sin(yaw),
                distance * np.sin(pitch)
            ])

            xs.append(point[0])
            ys.append(point[1])
            zs.append(point[2])

            print(point)

            if len(xs) > 200:
                xs.pop(0)
                ys.pop(0)
                zs.pop(0)

            # ax.cla()
            scatter._offsets3d = (xs, ys, zs)

        plt.draw()
        plt.pause(0.02)