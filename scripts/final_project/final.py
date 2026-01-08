###
# Final project with MuJoCo
# SI100B Robotics Programming
# This code is modified based on the MuJoCo template code at https://github.com/pab47/pab47.github.io/tree/master.
# Date: Dec., 2025
###

import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

xml_path = '../../models/universal_robots_ur5e/scene.xml' #xml file (assumes this is in the same folder as this file)
#################################
## USER CODE: Set simulation parameters here
#################################
simend = 180 #simulation time (second)
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)
#################################

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# Helper function
def IK_controller(model, data, X_ref, q_pos):
    # Compute Jacobian
    position_Q = data.site_xpos[0]

    jacp = np.zeros((3, 6))
    mj.mj_jac(model, data, jacp, None, position_Q, 7)

    J = jacp.copy()
    Jinv = np.linalg.pinv(J)

    # Reference point
    X = position_Q.copy()
    dX = X_ref - X

    # Compute control input
    dq = Jinv @ dX

    return q_pos + dq

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

# Get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1920, 1080, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)


########################################
## USER CODE: Set camera view here
########################################
# Example on how to set camera configuration
cam.azimuth =  85.66333333333347 
cam.elevation =  -35.33333333333329
cam.distance =  2.22
cam.lookat = np.array([ -0.09343103051557476 , 0.31359595076587915 , 0.22170312166086661 ])
########################################

# Initialize the controller
init_controller(model,data)

# Set the controller
mj.set_mjcb_control(controller)

# Initialize joint configuration
init_qpos = np.array([-1.6353559, -1.28588984, 2.14838487, -2.61087434, -1.5903009, -0.06818645])
data.qpos[:] = init_qpos
cur_q_pos = init_qpos.copy()

traj_points = []
MAX_TRAJ = 5e5  # Maximum number of trajectory points to store
LINE_RGBA = np.array([1.0, 0.0, 0.0, 1.0])

######################################
## USER CODE STARTS HERE
######################################

######################################
## USER CODE ENDS HERE
######################################

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        # Store trajectory
        mj_end_eff_pos = data.site_xpos[0]
        if (mj_end_eff_pos[2] < 0.1):
            traj_points.append(mj_end_eff_pos.copy())
        if len(traj_points) > MAX_TRAJ:
            traj_points.pop(0)
            
        # Get current joint configuration
        cur_q_pos = data.qpos.copy()
        
        ######################################
        ## USER CODE STARTS HERE
        ######################################
        X_ref = np.array([0.0, 0.1, 0.1])  # Desired end-effector position, this line is just an example
        
        ######################################
        ## USER CODE ENDS HERE
        ######################################

        # Compute control input using IK
        cur_ctrl = IK_controller(model, data, X_ref, cur_q_pos)
        
        # Apply control input
        data.ctrl[:] = cur_ctrl
        mj.mj_step(model, data)
        data.time += 0.02

    if (data.time>=simend):
        break

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth = ',cam.azimuth,'\n','cam.elevation = ',cam.elevation,'\n','cam.distance = ',cam.distance)
        print('cam.lookat = np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    # Add trajectory as spheres
    for j in range(1, len(traj_points)):
        if scene.ngeom >= scene.maxgeom:
            break  # avoid overflow

        geom = scene.geoms[scene.ngeom]
        scene.ngeom += 1
        
        p1 = traj_points[j-1]
        p2 = traj_points[j]
        direction = p2 - p1
        midpoint = (p1 + p2) / 2.0
        
        # Configure this geom as a line
        geom.type = mj.mjtGeom.mjGEOM_SPHERE  # Use sphere for endpoints
        geom.rgba[:] = LINE_RGBA
        geom.size[:] = np.array([0.002, 0.002, 0.002])
        geom.pos[:] = midpoint
        geom.mat[:] = np.eye(3)  # no rotation
        geom.dataid = -1
        geom.segid = -1
        geom.objtype = 0
        geom.objid = 0
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()

######################################
## USER CODE STARTS HERE
######################################

# 定义书写区域参数
WRITING_HEIGHT = 0.1  # 书写高度
LIFT_HEIGHT = 0.15   # 抬笔高度
WRITING_AREA = {
    'x_min': -0.5, 'x_max': 0.5,
    'y_min': 0.1, 'y_max': 0.6
}

# 定义"刘子轩"的笔画坐标（简化版，实际需要更精细的笔画数据）
liu_character = {
    'name': '刘',
    'strokes': [
        # 第一笔
        [np.array([-0.3, 0.5, WRITING_HEIGHT]), np.array([-0.2, 0.5, WRITING_HEIGHT])],
        # 第二笔  
        [np.array([-0.25, 0.55, WRITING_HEIGHT]), np.array([-0.25, 0.45, WRITING_HEIGHT])],
        # 更多笔画...
    ]
}

zi_character = {
    'name': '子', 
    'strokes': [
        [np.array([-0.1, 0.5, WRITING_HEIGHT]), np.array([0.0, 0.5, WRITING_HEIGHT])],
        [np.array([-0.05, 0.55, WRITING_HEIGHT]), np.array([-0.05, 0.45, WRITING_HEIGHT])],
        # 更多笔画...
    ]
}

xuan_character = {
    'name': '轩',
    'strokes': [
        [np.array([0.1, 0.5, WRITING_HEIGHT]), np.array([0.2, 0.5, WRITING_HEIGHT])],
        [np.array([0.15, 0.55, WRITING_HEIGHT]), np.array([0.15, 0.45, WRITING_HEIGHT])],
        # 更多笔画...
    ]
}

# 组合所有字符
writing_content = [liu_character, zi_character, xuan_character]

# 轨迹状态机
class WritingTrajectory:
    def __init__(self):
        self.current_stroke = 0
        self.current_character = 0
        self.current_point = 0
        self.is_lifted = True
        self.trajectory_points = []
        self.generate_full_trajectory()
    
    def generate_full_trajectory(self):
        """生成完整的书写轨迹"""
        # 从起始位置移动到第一个字符上方
        start_pos = np.array([-0.4, 0.3, LIFT_HEIGHT])
        self.trajectory_points.append(start_pos)
        
        for char_idx, character in enumerate(writing_content):
            # 移动到字符起始位置上方
            first_stroke_start = character['strokes'][0][0].copy()
            first_stroke_start[2] = LIFT_HEIGHT
            self.trajectory_points.append(first_stroke_start)
            
            for stroke_idx, stroke in enumerate(character['strokes']):
                # 落笔
                stroke_start = stroke[0].copy()
                stroke_start[2] = WRITING_HEIGHT
                self.trajectory_points.append(stroke_start)
                
                # 书写笔画
                for point in stroke[1:]:
                    self.trajectory_points.append(point.copy())
                
                # 抬笔（如果不是最后一个笔画）
                if stroke_idx < len(character['strokes']) - 1:
                    stroke_end = stroke[-1].copy()
                    stroke_end[2] = LIFT_HEIGHT
                    self.trajectory_points.append(stroke_end)
            
            # 字符完成，抬笔
            if char_idx < len(writing_content) - 1:
                last_stroke_end = character['strokes'][-1][-1].copy()
                last_stroke_end[2] = LIFT_HEIGHT
                self.trajectory_points.append(last_stroke_end)
        
        # 移动到终止位置
        end_pos = np.array([0.0, 0.35, LIFT_HEIGHT])
        self.trajectory_points.append(end_pos)

# 创建轨迹实例
trajectory = WritingTrajectory()
current_target_index = 0
interpolation_step = 0
interpolation_points = []

######################################
## USER CODE ENDS HERE
######################################

def linear_interpolate(start, end, num_points):
    """线性插值"""
    return [start + (end - start) * i / (num_points - 1) for i in range(num_points)]

def circular_interpolate(center, radius, start_angle, end_angle, num_points):
    """圆弧插值（用于曲线笔画）"""
    points = []
    for i in range(num_points):
        angle = start_angle + (end_angle - start_angle) * i / (num_points - 1)
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        points.append(np.array([x, y, WRITING_HEIGHT]))
    return points

def generate_interpolation_points(current_pos, target_pos, method='linear'):
    """生成插值点"""
    distance = np.linalg.norm(target_pos - current_pos)
    num_points = max(int(distance / 0.01), 5)  # 根据距离决定插值点数
    
    if method == 'linear':
        return linear_interpolate(current_pos, target_pos, num_points)
    elif method == 'circular':
        # 简化处理，实际需要更复杂的圆弧计算
        return linear_interpolate(current_pos, target_pos, num_points)
    else:
        return linear_interpolate(current_pos, target_pos, num_points)

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        # Store trajectory
        mj_end_eff_pos = data.site_xpos[0]
        if (mj_end_eff_pos[2] < 0.1):
            traj_points.append(mj_end_eff_pos.copy())
        if len(traj_points) > MAX_TRAJ:
            traj_points.pop(0)
            
        # Get current joint configuration
        cur_q_pos = data.qpos.copy()
        
        ######################################
        ## USER CODE STARTS HERE
        ######################################
        
        # 检查是否完成所有轨迹点
        if current_target_index >= len(trajectory.trajectory_points):
            # 所有书写完成，移动到终止姿态
            target_joints = np.array([0.0, -2.32, -1.38, -2.45, 1.57, 0.0])
            joint_error = np.linalg.norm(cur_q_pos - target_joints)
            
            if joint_error > 0.01:
                # 使用关节空间控制逐渐接近目标关节角
                alpha = 0.02  # 控制速度
                cur_ctrl = cur_q_pos + alpha * (target_joints - cur_q_pos)
            else:
                # 已经到达目标位置，保持静止
                cur_ctrl = target_joints
                
            # 直接设置控制量，跳过IK计算
            data.ctrl[:] = cur_ctrl
            mj.mj_step(model, data)
            data.time += 0.02
            continue
        
        # 获取当前目标点
        if not interpolation_points:
            # 需要生成新的插值路径
            current_target = trajectory.trajectory_points[current_target_index]
            interpolation_points = generate_interpolation_points(
                mj_end_eff_pos, current_target
            )
            interpolation_step = 0
        
        # 设置参考位置
        if interpolation_step < len(interpolation_points):
            X_ref = interpolation_points[interpolation_step]
            interpolation_step += 1
        else:
            # 当前目标点完成，移动到下一个
            current_target_index += 1
            interpolation_points = []
            continue
        
        ######################################
        ## USER CODE ENDS HERE
        ######################################

        # Compute control input using IK
        cur_ctrl = IK_controller(model, data, X_ref, cur_q_pos)
        
        # Apply control input
        data.ctrl[:] = cur_ctrl
        mj.mj_step(model, data)
        data.time += 0.02

    if (data.time>=simend):
        break
        
    # 其余渲染代码保持不变...    
    