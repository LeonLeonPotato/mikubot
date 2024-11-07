import glfw
import moderngl
import numpy as np
import time
from pyrr import Matrix44, Vector3
import bridge

def make_window():
    if not glfw.init():
        raise Exception("GLFW cannot be initialized!")

    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 4)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 1)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, glfw.TRUE)
    glfw.window_hint(glfw.SAMPLES, 4)

    window = glfw.create_window(800, 600, "LiDAR Visualizer", None, None)
    if not window:
        glfw.terminate()
        raise Exception("GLFW window cannot be created!")

    glfw.make_context_current(window)

    return window

def project_pitch_yaw_dist(pitch, yaw, dist):
    x = dist * np.cos(pitch) * np.cos(yaw)
    y = dist * np.cos(pitch) * np.sin(yaw)
    z = dist * np.sin(pitch)

    return Vector3([x, y, z])

last_x = 400
last_y = 300
yaw = 0.0
pitch = 0.0

def mouse_callback(window, xpos, ypos, sensitivity=0.3):
    global last_x, last_y, yaw, pitch
    x_offset = (last_x - xpos) * sensitivity
    y_offset = (last_y- ypos) * sensitivity

    last_x = xpos
    last_y = ypos

    yaw += x_offset
    pitch += y_offset

if __name__ == "__main__":
    window = make_window()
    ctx = moderngl.create_context(require=410)
    ctx.enable(moderngl.DEPTH_TEST)
    ctx.enable(moderngl.PROGRAM_POINT_SIZE)
    ctx.enable(moderngl.BLEND)
    ctx.blend_func = moderngl.SRC_ALPHA, moderngl.ONE_MINUS_DST_ALPHA

    projection_matrix = Matrix44.perspective_projection(
        fovy=60.0, 
        aspect=1, 
        near=0.1, 
        far=10000.0
    )

    with open("ml/lidar/vert.glsl") as f:
        with open("ml/lidar/frag.glsl") as g:
            program = ctx.program(
                vertex_shader=f.read(),
                fragment_shader=g.read()
            )

    program['model'].write(Matrix44.from_translation([0, 0, 0]).astype('f4').tobytes())
    program['projection'].write(projection_matrix.astype('f4').tobytes())

    camera_pos = Vector3([0, 0, 0])
    camera_front = project_pitch_yaw_dist(np.radians(pitch), np.radians(yaw), 1)
    camera_speed = 10
    camera_up = Vector3([0, 0, 1])
    glfw.set_cursor_pos_callback(window, mouse_callback)

    bridge.run()

    while not glfw.window_should_close(window):
        if glfw.get_key(window, glfw.KEY_P) == glfw.PRESS:
            bridge.lidarpoints.clear()
        
        if glfw.get_key(window, glfw.KEY_W) == glfw.PRESS:
            camera_pos += camera_front * camera_speed
        if glfw.get_key(window, glfw.KEY_S) == glfw.PRESS:
            camera_pos -= camera_front * camera_speed
        if glfw.get_key(window, glfw.KEY_A) == glfw.PRESS:
            camera_pos -= camera_front.cross(camera_up).normalized * camera_speed
        if glfw.get_key(window, glfw.KEY_D) == glfw.PRESS:
            camera_pos += camera_front.cross(camera_up).normalized * camera_speed

        ctx.clear(0.0, 0.0, 0.0, 1.0)

        camera_front = project_pitch_yaw_dist(np.radians(pitch), np.radians(yaw), 1)
        view_matrix = Matrix44.look_at(
            eye=camera_pos,
            target=camera_pos + camera_front,
            up=camera_up
        )

        program['view'].write(view_matrix.astype('f4').tobytes())

        bridge.listlock.acquire()
        new_vertices = np.array(bridge.lidarpoints, dtype='f4').flatten()
        bridge.listlock.release()

        if len(new_vertices) > 0:
            vbo = ctx.buffer(reserve=2**24)
            vao = ctx.simple_vertex_array(program, vbo, 'in_position')
            vbo.write(new_vertices.tobytes())
            vao.render(moderngl.POINTS)

        glfw.swap_buffers(window)
        glfw.poll_events()

    glfw.terminate()
