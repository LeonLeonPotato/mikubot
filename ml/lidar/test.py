import glfw
import moderngl
import numpy as np
import time
import json
import subprocess
import threading
from pyrr import Matrix44, Vector3
import os
import sys

prospath = {
    "macos": "/Users/leon.zhu/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-cli-macos/pros",
    "win32": "C:/Users/leon/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-cli-windows/pros"
}

proc = subprocess.Popen([
    prospath[sys.platform],
    "terminal"
], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

if not glfw.init():
    raise Exception("GLFW cannot be initialized!")

glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 4)
glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 1)
glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, glfw.TRUE)

window = glfw.create_window(800, 600, "Hello World", None, None)
if not window:
    glfw.terminate()
    raise Exception("GLFW window cannot be created!")

glfw.make_context_current(window)

ctx = moderngl.create_context(require=410)

with open("ml/lidar/vert.glsl") as f:
    with open("ml/lidar/frag.glsl") as g:
        program = ctx.program(
            vertex_shader=f.read(),
            fragment_shader=g.read()
        )

projection_matrix = Matrix44.perspective_projection(
    fovy=60.0, 
    aspect=1, 
    near=0.1, 
    far=10000.0
)

model_matrix = Matrix44.from_translation([0, 0, 0])
rotation_y = Matrix44.from_y_rotation(0)
scaling = Matrix44.from_scale([0.5, 0.5, 0.5])
model_matrix = scaling * rotation_y * model_matrix

def project_quaternion(quaternion, dist):
    x = dist * (1 - 2 * (quaternion[1]**2 + quaternion[2]**2))
    y = dist * 2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3])
    z = -dist * 2 * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3])

    return x, y, z

def project_pitch_yaw_dist(pitch, yaw, dist):
    x = dist * np.cos(pitch) * np.cos(yaw)
    y = dist * np.sin(pitch)
    z = dist * np.cos(pitch) * np.sin(yaw)

    return x, y, z

program['model'].write(model_matrix.astype('f4').tobytes())
program['projection'].write(projection_matrix.astype('f4').tobytes())

vbo = ctx.buffer(reserve=2**24)
vao = ctx.simple_vertex_array(program, vbo, 'in_position')

listlock = threading.Lock()
lidarpoints = []
def lidar():
    global lidarpoints
    while True:
        line = proc.stdout.readline().decode("utf-8").strip()
        if "LIDAR" in line:
            line = line.split("LIDAR: ")[1]
            line = json.loads(line)
            if line['confidence'] >= 10:
                relp = np.array(project_quaternion(
                    line['quaternion'],
                    line["distance"]
                ))
                globp = relp

                listlock.acquire()
                lidarpoints.append(globp)
                listlock.release()

lidarthread = threading.Thread(target=lidar, daemon=True)
lidarthread.start()

while not glfw.window_should_close(window):
    if glfw.get_key(window, glfw.KEY_P) == glfw.PRESS:
        lidarpoints.clear()

    ctx.clear(0.0, 0.0, 0.0, 1.0)

    ctx.enable(moderngl.PROGRAM_POINT_SIZE)
    ctx.enable(moderngl.BLEND)
    ctx.blend_func = moderngl.SRC_ALPHA, moderngl.ONE_MINUS_DST_ALPHA

    camera_pos = [0, 0, 0]
    view_matrix = Matrix44.look_at(
        eye=camera_pos,
        target=project_pitch_yaw_dist(0, 0, 1),
        up=[0, 0, 1]
    )

    program['view'].write(view_matrix.astype('f4').tobytes())

    listlock.acquire()
    new_vertices = np.array(lidarpoints, dtype='f4').flatten()
    listlock.release()

    vbo.clear()
    vbo.write(new_vertices.tobytes())
    vao.render(moderngl.POINTS)

    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
