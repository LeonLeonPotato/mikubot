import glfw
import moderngl
import numpy as np
import time
import json
import subprocess
import threading
from pyrr import Matrix44, Vector3

proc = subprocess.Popen([
    "/Users/leon.zhu/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-cli-macos/pros",
    "terminal"
], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

if not glfw.init():
    raise Exception("GLFW cannot be initialized!")

glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, glfw.TRUE)

window = glfw.create_window(800, 600, "Hello World", None, None)
if not window:
    glfw.terminate()
    raise Exception("GLFW window cannot be created!")

glfw.make_context_current(window)

ctx = moderngl.create_context()

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


def project_pitch_yaw_dist(pitch, yaw, dist):
    x = dist * np.cos(pitch) * np.cos(yaw)
    y = dist * np.sin(pitch)
    z = dist * np.cos(pitch) * np.sin(yaw)

    return x, y, z

camera_pos = Vector3(project_pitch_yaw_dist(0.707, 0.707, 100.0))
camera_target = Vector3([0.0, 0.0, 0.0])
camera_up = Vector3([0.0, 1.0, 0.0])

view_matrix = Matrix44.look_at(
    eye=camera_pos,
    target=camera_target,
    up=camera_up
)

program['model'].write(model_matrix.astype('f4').tobytes())
program['view'].write(view_matrix.astype('f4').tobytes())
program['projection'].write(projection_matrix.astype('f4').tobytes())

vbo = ctx.buffer(reserve=2**16)
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
                listlock.acquire()
                lidarpoints.append(
                    project_pitch_yaw_dist(
                        line["pitch"],
                        line["yaw"],
                        line["distance"]
                    )
                )
                listlock.release()

lidarthread = threading.Thread(target=lidar, daemon=True)
lidarthread.start()

while not glfw.window_should_close(window):
    if glfw.get_key(window, glfw.KEY_P) == glfw.PRESS:
        lidarpoints.clear()

    ctx.clear(0.0, 0.0, 0.0, 1.0)

    ctx.point_size = 5.0

    camera_pos = Vector3(project_pitch_yaw_dist(45, time.time() * 0.5, 750.0))
    view_matrix = Matrix44.look_at(
        eye=camera_pos,
        target=camera_target,
        up=camera_up
    )

    program['view'].write(view_matrix.astype('f4').tobytes())

    listlock.acquire()
    new_vertices = np.array(lidarpoints + [(0, 0, 0)], dtype='f4').flatten()
    listlock.release()

    vbo.clear()
    vbo.write(new_vertices.tobytes())
    vao.render(moderngl.POINTS)

    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
