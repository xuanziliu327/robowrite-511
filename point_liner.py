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
cam.azimuth =180
#-78.5是正上方
cam.elevation = 0
cam.distance =   1.4079120659102675 
cam.lookat = np.array( [0.3,0.35, 0.11])
########################################

# Initialize the controller
init_controller(model,data)

# Set the controller
mj.set_mjcb_control(controller)

# Initialize joint configuration
init_qpos = np.array([-1.54378411, -1.47409264  ,2.64825884 ,-2.41641367 ,-1.64235967 ,-0.06818644])
data.qpos[:] = init_qpos
cur_q_pos = init_qpos.copy()

traj_points = []
MAX_TRAJ = 5e5  # Maximum number of trajectory points to store
LINE_RGBA = np.array([1.0, 0.0, 0.0, 1.0])

######################################
### BAISIC INTERPOLATION FUNCTIONS ###
def LinearInterpolate(q0, q1, t, t_total):
    if t_total<=0:
        return q1
    alpha=t/t_total
    alpha=np.clip(alpha,0,1)
    return (1-alpha)*q0+alpha*q1


######################################

############################################
### BONUS: BEZIER INTERPOLATION FUNCTION ###
def QuadBezierInterpolate(q0, q1, q2, t, t_total):
    if t_total<=0:
        return q2
    u=t/t_total
    u=np.clip(u,0,1)
    return ((1-u)**2*q0+2*u*(1-u)*q1+u**2*q2)
############################################

######################################
## USER CODE STARTS HERE
######################################


q0=np.array([0.25375,0.13375,0.1])
q1=np.array([0.305,0.13375,0.1])
q2=np.array([0.305,0.165,0.1])
q3=np.array([0.279375,0.17125,0.11])
q4=np.array([0.25375,0.1775,0.1])
q5=np.array([0.305,0.1775,0.1])
q6=np.array([0.279375,0.1875,0.11])
q7=np.array([0.25375,0.1975,0.1])
q8=np.array([0.29375,0.1975,0.1])
q9=np.array([0.305,0.1975,0.1])
q10=np.array([0.305,0.21625,0.1])
q11=np.array([0.305,0.23500000000000001,0.1])
q12=np.array([0.29375,0.23500000000000001,0.1])
q13=np.array([0.25375,0.23500000000000001,0.1])
q14=np.array([0.25375,0.2425,0.11])
q15=np.array([0.25375,0.25,0.1])
q16=np.array([0.25375,0.2875,0.1])
q17=np.array([0.305,0.25,0.1])
q18=np.array([0.305,0.2875,0.1])
q19=np.array([0.279375,0.294375,0.11])
q20=np.array([0.25375,0.30125,0.1])
q21=np.array([0.305,0.30125,0.1])
q22=np.array([0.279375,0.31125,0.11])
q23=np.array([0.25375,0.32125000000000004,0.1])
q24=np.array([0.305,0.35875,0.1])
q25=np.array([0.279375,0.35750000000000004,0.11])
q26=np.array([0.25375,0.35624999999999996,0.1])
q27=np.array([0.305,0.31875,0.1])
q28=np.array([0.279375,0.345625,0.11])
#U
q29=np.array([0.25375,0.37250000000000005,0.1])
q30=np.array([0.29375,0.37250000000000005,0.1])
q31=np.array([0.305,0.37250000000000005,0.1])
q32=np.array([0.305,0.39125,0.1])
q33=np.array([0.305,0.41000000000000003,0.1])
q34=np.array([0.29375,0.41000000000000003,0.1])
q35=np.array([0.25375,0.41000000000000003,0.1])
q36=np.array([0.279375,0.416875,0.11])

q37=np.array([0.305,0.42374999999999996,0.1])
q38=np.array([0.25375,0.44375,0.1])
q39=np.array([0.305,0.46625000000000005,0.1])
q40=np.array([0.296875,0.44937499999999997,0.11])
q41=np.array([0.28875,0.4325,0.1])
q42=np.array([0.28875,0.4575,0.1])
q43=np.array([0.296875,0.46875,0.11])

q44=np.array([0.305,0.48,0.1])
q45=np.array([0.25375,0.48,0.1])
q46=np.array([0.305,0.515,0.1])
q47=np.array([0.25375,0.515,0.1])
q48=np.array([0.29875,0.36812500000000004,0.15])

a1=np.array([0.34125,0.22125,0.1])
a2=np.array([0.3525,0.2275,0.1])
a3=np.array([0.354375,0.2175,0.1])
a4=np.array([0.35625,0.20750000000000002,0.1])
a5=np.array([0.35625,0.245,0.1])
a6=np.array([0.35625,0.240625,0.1])
a7=np.array([0.35625,0.23625000000000002,0.1])
a8=np.array([0.385,0.23375,0.1])
a9=np.array([0.40375,0.20500000000000002,0.1])
a10=np.array([0.38625,0.209375,0.1])
a11=np.array([0.36875,0.21375,0.1])
a12=np.array([0.38375000000000004,0.23750000000000002,0.1])
a13=np.array([0.39625,0.24125,0.1])
a14=np.array([0.373125,0.248125,0.1])
a15=np.array([0.35,0.255,0.1])
a16=np.array([0.39125,0.255,0.1])
a17=np.array([0.36625,0.26187499999999997,0.1])
a18=np.array([0.34125,0.26875000000000004,0.1])
a19=np.array([0.4075,0.26875000000000004,0.1])
a20=np.array([0.4,0.25875000000000004,0.1])
a21=np.array([0.3725,0.275625,0.1])
a22=np.array([0.345,0.2925,0.1])
a23=np.array([0.345,0.33875,0.1])
a24=np.array([0.3625,0.3175,0.1])
a25=np.array([0.40625,0.3175,0.1])
a26=np.array([0.4,0.30625,0.1])
a27=np.array([0.385625,0.29500000000000004,0.1])
a28=np.array([0.37124999999999997,0.28375,0.1])
a29=np.array([0.37124999999999997,0.34750000000000003,0.1])
a30=np.array([0.36875,0.355,0.1])
a31=np.array([0.35125,0.36250000000000004,0.1])
a32=np.array([0.35125,0.38749999999999996,0.1])
a33=np.array([0.345625,0.38187499999999996,0.1])
a34=np.array([0.33999999999999997,0.37625,0.1])
a35=np.array([0.3725,0.36375,0.1])
a36=np.array([0.3725,0.38749999999999996,0.1])
a37=np.array([0.365625,0.38249999999999995,0.1])
a38=np.array([0.35875,0.37750000000000006,0.1])
a39=np.array([0.4075,0.37750000000000006,0.1])
a40=np.array([0.400625,0.37,0.1])
a41=np.array([0.39375,0.36250000000000004,0.1])
a42=np.array([0.38375000000000004,0.39,0.1])
a43=np.array([0.365625,0.39249999999999996,0.1])
a44=np.array([0.34750000000000003,0.395,0.1])
a45=np.array([0.34750000000000003,0.42125,0.1])
a46=np.array([0.359375,0.406875,0.1])
a47=np.array([0.37124999999999997,0.39249999999999996,0.1])
a48=np.array([0.37124999999999997,0.42625,0.1])
a49=np.array([0.359375,0.416875,0.1])
a50=np.array([0.34750000000000003,0.4075,0.1])
a51=np.array([0.4075,0.4075,0.1])
a52=np.array([0.4325,0.27,0.1])
b1=np.array([0.4575,0.1325,0.1])
b2=np.array([0.4425,0.1325,0.1])
b3=np.array([0.44375,0.14625,0.1])
b4=np.array([0.44875,0.16625,0.1])
b5=np.array([0.4575,0.15875,0.1])
b6=np.array([0.49124999999999996,0.12875,0.1])
b7=np.array([0.49124999999999996,0.16375,0.1])
b8=np.array([0.48062499999999997,0.16875,0.1])
b9=np.array([0.47,0.17375000000000002,0.1])
b10=np.array([0.445,0.17375000000000002,0.1])
b11=np.array([0.445,0.18875,0.1])
b12=np.array([0.445,0.2025,0.1])
b13=np.array([0.47,0.2025,0.1])
b14=np.array([0.4925,0.2025,0.1])
b15=np.array([0.4925,0.18875,0.1])
b16=np.array([0.4925,0.17375000000000002,0.1])
b17=np.array([0.47,0.17375000000000002,0.1])
b18=np.array([0.46375,0.195625,0.1])
b19=np.array([0.4575,0.2175,0.1])
b20=np.array([0.4425,0.2175,0.1])
b21=np.array([0.4425,0.23125,0.1])
b22=np.array([0.44875,0.25125,0.1])
b23=np.array([0.4575,0.24375,0.1])
b24=np.array([0.49124999999999996,0.21375,0.1])
b25=np.array([0.49124999999999996,0.24875,0.1])
b26=np.array([0.468125,0.2675,0.1])
b27=np.array([0.445,0.28625,0.1])
b28=np.array([0.445,0.26625,0.1])
b29=np.array([0.46875,0.26,0.1])
b30=np.array([0.4625,0.26,0.1])
b31=np.array([0.4625,0.27375,0.1])
b32=np.array([0.4625,0.2875,0.1])
b33=np.array([0.47750000000000004,0.2875,0.1])
b34=np.array([0.4925,0.2875,0.1])
b35=np.array([0.4925,0.27249999999999996,0.1])
b36=np.array([0.4925,0.26,0.1])
b37=np.array([0.48125,0.26,0.1])
b38=np.array([0.463125,0.295625,0.1])
b39=np.array([0.445,0.33125000000000004,0.1])
b40=np.array([0.445,0.31125,0.1])
b41=np.array([0.46875,0.305,0.1])
b42=np.array([0.4625,0.305,0.1])
b43=np.array([0.4625,0.31875,0.1])
b44=np.array([0.4625,0.3325,0.1])
b45=np.array([0.47750000000000004,0.3325,0.1])
b46=np.array([0.4925,0.3325,0.1])
b47=np.array([0.4925,0.3175,0.1])
b48=np.array([0.4925,0.305,0.1])
b49=np.array([0.48125,0.305,0.1])
b50=np.array([0.468125,0.325625,0.1])
b51=np.array([0.45499999999999996,0.34625,0.1])
b52=np.array([0.4425,0.34625,0.1])
b53=np.array([0.4425,0.35875,0.1])
b54=np.array([0.4425,0.37124999999999997,0.1])
b55=np.array([0.45625,0.37124999999999997,0.1])
b56=np.array([0.46625,0.37124999999999997,0.1])
b57=np.array([0.46625,0.35750000000000004,0.1])
b58=np.array([0.46625,0.375,0.1])
b59=np.array([0.48125,0.375,0.1])
b60=np.array([0.49375,0.375,0.1])
b61=np.array([0.49375,0.36,0.1])
b62=np.array([0.49375,0.345,0.1])
b63=np.array([0.48125,0.345,0.1])
b64=np.array([0.468125,0.36812500000000004,0.1])
b65=np.array([0.45499999999999996,0.39125,0.1])
b66=np.array([0.4425,0.39125,0.1])
b67=np.array([0.4425,0.40375000000000005,0.1])
b68=np.array([0.4425,0.41625,0.1])
b69=np.array([0.45625,0.41625,0.1])
b70=np.array([0.46625,0.41625,0.1])
b71=np.array([0.46625,0.40249999999999997,0.1])
b72=np.array([0.46625,0.42000000000000004,0.1])
b73=np.array([0.48125,0.42000000000000004,0.1])
b74=np.array([0.49375,0.42000000000000004,0.1])
b75=np.array([0.49375,0.405,0.1])
b76=np.array([0.49375,0.39,0.1])
b77=np.array([0.48125,0.39,0.1])
b78=np.array([0.47,0.41125,0.1])
b79=np.array([0.45875,0.4325,0.1])
b80=np.array([0.445,0.44999999999999996,0.1])
b81=np.array([0.4925,0.44999999999999996,0.1])
b82=np.array([0.46875,0.47562499999999996,0.1])
b83=np.array([0.445,0.50125,0.1])
b84=np.array([0.445,0.48124999999999996,0.1])
b85=np.array([0.46875,0.475,0.1])
b86=np.array([0.4625,0.475,0.1])
b87=np.array([0.4625,0.48875,0.1])
b88=np.array([0.4625,0.5025000000000001,0.1])
b89=np.array([0.47750000000000004,0.5025000000000001,0.1])
b90=np.array([0.4925,0.5025000000000001,0.1])
b91=np.array([0.4925,0.48750000000000004,0.1])
b92=np.array([0.4925,0.475,0.1])
b93=np.array([0.48125,0.475,0.1])
b94=np.array([0.463125,0.515625,0.1])
b95=np.array([0.445,0.54375,0.1])
b96=np.array([0.445,0.52375,0.1])
b97=np.array([0.46875,0.5175,0.1])
b98=np.array([0.4625,0.5175,0.1])
b99=np.array([0.4625,0.53125,0.1])
b100=np.array([0.4625,0.545,0.1])
b101=np.array([0.47750000000000004,0.545,0.1])
b102=np.array([0.4925,0.545,0.1])
b103=np.array([0.4925,0.53,0.1])
b104=np.array([0.4925,0.5175,0.1])
b105=np.array([0.48125,0.5175,0.1])
# q0= np.array([0.25, 0.1,  0.1 ])
# q1= np.array( [0.25, 0.6 , 0.1 ])
# q2= np.array( [0.5 , 0.6 , 0.1 ])
# q3= np.array([0.5,  0.1,  0.1 ])
# q4= np.array([0.25, 0.1  ,0.1 ])
#边框数据测试


t_total = 177

######################################
## USER CODE ENDS HERE
######################################
n=127
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
        # Desired end-effector position, this line is just an example
        #L
        if (data.time < 0.6*t_total/n):
            X_ref = LinearInterpolate(q0, q1, data.time, 0.6*t_total/n)
        elif (data.time <1.3* t_total/n):
            X_ref = LinearInterpolate(q1, q1, data.time - 0.6*t_total/n, 0.7*t_total/n)
        
        elif (data.time <2* t_total/n):
            X_ref = LinearInterpolate(q1, q2, data.time - 1.3*t_total/n, 0.7*t_total/n)
        elif (data.time <2.3* t_total/n):
            X_ref = LinearInterpolate(q2, q2, data.time - 2*t_total/n, 0.3*t_total/n)
        elif (data.time <2.5* t_total/n):
            X_ref = QuadBezierInterpolate(q2, q3, q4, data.time - 2.3*t_total/n, 0.2*t_total/n)
        
        #i
        elif (data.time <3* t_total/n):
            X_ref = LinearInterpolate(q4, q4, data.time - 2.5*t_total/n, 0.5*t_total/n)
        elif (data.time <3.8* t_total/n):
            X_ref = LinearInterpolate(q4, q5, data.time - 3*t_total/n, 0.8*t_total/n)
        elif (data.time <4.2* t_total/n):
            X_ref = LinearInterpolate(q5, q5, data.time - 3.8*t_total/n, 0.4*t_total/n)
        elif (data.time <5* t_total/n):
            X_ref = QuadBezierInterpolate(q5, q6, q7, data.time - 4*t_total/n, t_total/n)
        
        
        elif (data.time <6* t_total/n):
            X_ref = LinearInterpolate(q7, q8, data.time - 5*t_total/n, t_total/n)
        elif (data.time <7* t_total/n):
            X_ref = QuadBezierInterpolate(q8, q9, q10, data.time - 6*t_total/n, t_total/n)
        elif (data.time <8* t_total/n):
            X_ref = QuadBezierInterpolate(q10, q11, q12, data.time - 7*t_total/n, t_total/n)
        elif (data.time <9* t_total/n):
            X_ref = LinearInterpolate(q12, q13, data.time - 8*t_total/n, t_total/n)
        elif (data.time <10* t_total/n):
            X_ref = QuadBezierInterpolate(q13, q14, q15, data.time - 9*t_total/n, t_total/n)
        elif (data.time <11* t_total/n):
            X_ref = LinearInterpolate(q15, q16, data.time - 10*t_total/n, t_total/n)
        elif (data.time <12* t_total/n):
            X_ref = LinearInterpolate(q16, q17, data.time - 11*t_total/n, t_total/n)
        elif (data.time <13* t_total/n):
            X_ref = LinearInterpolate(q17, q18, data.time - 12*t_total/n, t_total/n)
        elif (data.time <14* t_total/n):
            X_ref = QuadBezierInterpolate(q18, q19, q20, data.time - 13*t_total/n, t_total/n)
        elif (data.time <15* t_total/n):
            X_ref = LinearInterpolate(q20, q21, data.time - 14*t_total/n, t_total/n)
        elif (data.time <16* t_total/n):
            X_ref = QuadBezierInterpolate(q21, q22, q23, data.time - 15*t_total/n, t_total/n)
        elif (data.time <16.8* t_total/n):
            X_ref = LinearInterpolate(q23, q24, data.time - 16*t_total/n, 0.8*t_total/n)
        elif (data.time <17*t_total/n):
            X_ref = QuadBezierInterpolate(q24, q25, q26, data.time - 16.8*t_total/n, 0.2*t_total/n)
        
  ################################################################################################################3
        elif (data.time <18* t_total/n):
            X_ref = LinearInterpolate(q26, q27, data.time - 17*t_total/n, t_total/n)
        elif (data.time <19* t_total/n):
            X_ref = QuadBezierInterpolate(q27, q28, q29, data.time - 18*t_total/n, t_total/n)
        elif (data.time <20* t_total/n):
            X_ref = LinearInterpolate(q29, q30, data.time - 19*t_total/n, t_total/n)
        elif (data.time <21* t_total/n):
            X_ref = QuadBezierInterpolate(q30, q31, q32, data.time - 20*t_total/n, t_total/n)
        elif (data.time <22* t_total/n):
            X_ref = QuadBezierInterpolate(q32, q33, q34, data.time - 21*t_total/n, t_total/n)
        elif (data.time <23* t_total/n):
            X_ref = LinearInterpolate(q34, q35, data.time - 22*t_total/n, t_total/n)
        elif (data.time <24* t_total/n):
            X_ref = QuadBezierInterpolate(q35, q36, q37, data.time - 23*t_total/n, t_total/n)
        elif (data.time <25* t_total/n):
            X_ref = LinearInterpolate(q37, q38, data.time - 24*t_total/n, t_total/n)
        elif (data.time <26* t_total/n):
            X_ref = LinearInterpolate(q38, q39, data.time - 25*t_total/n, t_total/n)
        elif (data.time <27* t_total/n):
            X_ref = QuadBezierInterpolate(q39, q40, q41, data.time - 26*t_total/n, t_total/n)
        elif (data.time <28* t_total/n):
            X_ref = LinearInterpolate(q41, q42, data.time - 27*t_total/n, t_total/n)
        elif (data.time <29* t_total/n):
            X_ref = QuadBezierInterpolate(q42, q43, q44, data.time - 28*t_total/n, t_total/n)
        elif (data.time <30* t_total/n):
            X_ref = LinearInterpolate(q44, q45, data.time - 29*t_total/n, t_total/n)
        elif (data.time <31* t_total/n):
            X_ref = LinearInterpolate(q45, q46, data.time -30* t_total/n, t_total/n)
        elif (data.time <32* t_total/n):
            X_ref = LinearInterpolate(q46, q47, data.time - 31*t_total/n, t_total/n)




        elif (data.time <33* t_total/n):
            X_ref = QuadBezierInterpolate(q47, q48, a1, data.time - 32*t_total/n, t_total/n)
        elif (data.time <34* t_total/n):
            X_ref = LinearInterpolate(a1, a2, data.time - 33*t_total/n, t_total/n)
        elif (data.time <35* t_total/n):
            X_ref = QuadBezierInterpolate(a2, a3, a4, data.time - 34*t_total/n, t_total/n)
        elif (data.time <36* t_total/n):
            X_ref = LinearInterpolate(a4, a5, data.time -35* t_total/n, t_total/n)
        elif (data.time <37* t_total/n):
            X_ref = QuadBezierInterpolate(a5, a6, a7, data.time - 36*t_total/n, t_total/n)
        elif (data.time <38* t_total/n):
            X_ref = QuadBezierInterpolate(a7, a8, a9, data.time - 37*t_total/n, t_total/n)
        elif (data.time <39* t_total/n):
            X_ref = QuadBezierInterpolate(a9, a10, a11, data.time - 38*t_total/n, t_total/n)
        elif (data.time <40* t_total/n):
            X_ref = QuadBezierInterpolate(a11, a12, a13, data.time - 39*t_total/n, t_total/n)
        elif (data.time <41* t_total/n):
            X_ref = QuadBezierInterpolate(a13, a14, a15, data.time - 40*t_total/n, t_total/n)
        elif (data.time <42* t_total/n):
            X_ref = LinearInterpolate(a15, a16, data.time - 41*t_total/n, t_total/n)
        elif (data.time <43* t_total/n):
            X_ref = QuadBezierInterpolate(a16, a17, a18, data.time - 42*t_total/n, t_total/n)
        elif (data.time <44* t_total/n):
            X_ref = LinearInterpolate(a18, a19, data.time -43* t_total/n, t_total/n)
        elif (data.time <45* t_total/n):
            X_ref = LinearInterpolate(a19, a20, data.time - 44*t_total/n, t_total/n)
        elif (data.time <46* t_total/n):
            X_ref = QuadBezierInterpolate(a20, a21, a22, data.time - 45*t_total/n, t_total/n)
#子
        elif (data.time <47* t_total/n):
            X_ref = LinearInterpolate(a22, a23, data.time - 46*t_total/n, t_total/n)
        elif (data.time <48* t_total/n):
            X_ref = LinearInterpolate(a23, a24, data.time - 47*t_total/n, t_total/n)
        elif (data.time <49* t_total/n):
            X_ref = LinearInterpolate(a24, a25, data.time - 48*t_total/n, t_total/n)
        elif (data.time <50* t_total/n):
            X_ref = LinearInterpolate(a25, a26, data.time - 49*t_total/n, t_total/n)
        elif (data.time <51* t_total/n):
            X_ref = QuadBezierInterpolate(a26, a27, a28, data.time - 50*t_total/n, t_total/n)
        elif (data.time <52* t_total/n):
            X_ref = LinearInterpolate(a28, a29, data.time - 51*t_total/n, t_total/n)
        elif (data.time <53* t_total/n):
            X_ref = QuadBezierInterpolate(a29, a30, a31, data.time - 52*t_total/n, t_total/n)
#轩
        elif (data.time <54* t_total/n):
            X_ref = LinearInterpolate(a31, a32, data.time - 53*t_total/n, t_total/n)
        elif (data.time <55* t_total/n):
            X_ref = QuadBezierInterpolate(a32, a33, a34, data.time - 54*t_total/n, t_total/n)
        elif (data.time <56* t_total/n):
            X_ref = LinearInterpolate(a34, a35, data.time - 55*t_total/n, t_total/n)
        elif (data.time <57* t_total/n):
            X_ref = LinearInterpolate(a35, a36, data.time -56*t_total/n, t_total/n)
        elif (data.time <58* t_total/n):
            X_ref = QuadBezierInterpolate(a36, a37, a38, data.time - 57*t_total/n, t_total/n)
        elif (data.time <59* t_total/n):
            X_ref = LinearInterpolate(a38, a39, data.time - 58*t_total/n, t_total/n)
        elif (data.time <60* t_total/n):
            X_ref = QuadBezierInterpolate(a39, a40, a41, data.time - 59*t_total/n, t_total/n)
        elif (data.time <61* t_total/n):
            X_ref = LinearInterpolate(a41, a42, data.time - 60*t_total/n, t_total/n)
        elif (data.time <62* t_total/n):
            X_ref = QuadBezierInterpolate(a42, a43, a44, data.time - 51*t_total/n, t_total/n)
        elif (data.time <63* t_total/n):
            X_ref = LinearInterpolate(a44, a45, data.time -62* t_total/n, t_total/n)
        elif (data.time <64* t_total/n):
            X_ref = QuadBezierInterpolate(a45, a46, a47, data.time - 63*t_total/n, t_total/n)
        elif (data.time <65* t_total/n):
            X_ref = LinearInterpolate(a47, a48, data.time -64*t_total/n, t_total/n)
        elif (data.time <66* t_total/n):
            X_ref = QuadBezierInterpolate(a48, a49, a50, data.time - 65*t_total/n, t_total/n)
        elif (data.time <67* t_total/n):
            X_ref = LinearInterpolate(a50, a51, data.time - 66*t_total/n, t_total/n)
        elif (data.time <68* t_total/n):
            X_ref = QuadBezierInterpolate(a51, a52, b1, data.time - 67*t_total/n, t_total/n)

    #2    
        
        elif (data.time <69* t_total/n):
            X_ref = QuadBezierInterpolate(b1, b2, b3, data.time - 68*t_total/n, t_total/n)
        elif (data.time <70* t_total/n):
            X_ref = QuadBezierInterpolate(b3, b4, b5, data.time - 69*t_total/n, t_total/n)
        elif (data.time <71* t_total/n):
            X_ref = LinearInterpolate(b5, b6, data.time -70* t_total/n, t_total/n)
        elif (data.time <72* t_total/n):
            X_ref = LinearInterpolate(b6, b7, data.time - 71*t_total/n, t_total/n)
        elif (data.time <73* t_total/n):
            X_ref = QuadBezierInterpolate(b7, b8, b9, data.time - 72*t_total/n, t_total/n)
       #0 
        elif (data.time <74* t_total/n):
            X_ref = QuadBezierInterpolate(b9, b10, b11, data.time - 73*t_total/n, t_total/n)
        elif (data.time <75* t_total/n):
            X_ref = QuadBezierInterpolate(b11, b12, b13, data.time - 74*t_total/n, t_total/n)
        elif (data.time <76* t_total/n):
            X_ref = QuadBezierInterpolate(b13, b14, b15, data.time - 75*t_total/n, t_total/n)
        elif (data.time <77* t_total/n):
            X_ref = QuadBezierInterpolate(b15, b16, b17, data.time - 76*t_total/n, t_total/n)
        elif (data.time <78* t_total/n):
            X_ref = QuadBezierInterpolate(b17, b18, b19, data.time - 77*t_total/n, t_total/n)
        #2
        elif (data.time <79* t_total/n):
            X_ref = QuadBezierInterpolate(b19, b20, b21, data.time - 78*t_total/n, t_total/n)
        elif (data.time <80* t_total/n):
            X_ref = QuadBezierInterpolate(b21, b22, b23, data.time - 79*t_total/n, t_total/n)
        elif (data.time <81* t_total/n):
            X_ref = LinearInterpolate(b23, b24, data.time - 80*t_total/n, t_total/n)
        elif (data.time <82* t_total/n):
            X_ref = LinearInterpolate(b24, b25, data.time -81* t_total/n, t_total/n)
        elif (data.time <83* t_total/n):
            X_ref = QuadBezierInterpolate(b25,b26, b27, data.time - 82*t_total/n, t_total/n)
#5
        elif (data.time <84* t_total/n):
            X_ref = LinearInterpolate(b27, b28, data.time - 83*t_total/n, t_total/n)
        elif (data.time <85* t_total/n):
            X_ref = LinearInterpolate(b28, b29, data.time -84* t_total/n, t_total/n)
        elif (data.time <86* t_total/n):
            X_ref = QuadBezierInterpolate(b29, b30, b31, data.time - 85*t_total/n, t_total/n)
        elif (data.time <87* t_total/n):
            X_ref = QuadBezierInterpolate(b31, b32, b33, data.time - 86*t_total/n, t_total/n)
        elif (data.time <88* t_total/n):
            X_ref = QuadBezierInterpolate(b33, b34, b35, data.time - 87*t_total/n, t_total/n)
        elif (data.time <89* t_total/n):
            X_ref = QuadBezierInterpolate(b35, b36, b37, data.time - 88*t_total/n, t_total/n)
        elif (data.time <90* t_total/n):
            X_ref = QuadBezierInterpolate(b37, b38, b39, data.time - 89*t_total/n, t_total/n)
#5

        elif (data.time <91* t_total/n):
            X_ref = LinearInterpolate(b39, b40, data.time -90.* t_total/n, t_total/n)
        elif (data.time <92* t_total/n):
            X_ref = LinearInterpolate(b40, b41, data.time - 91*t_total/n, t_total/n)
        elif (data.time <93* t_total/n):
            X_ref = QuadBezierInterpolate(b41, b42, b43, data.time - 92*t_total/n, t_total/n)
        elif (data.time <94* t_total/n):
            X_ref = QuadBezierInterpolate(b43, b44, b45, data.time - 93*t_total/n, t_total/n)
        elif (data.time <95* t_total/n):
            X_ref = QuadBezierInterpolate(b45, b46, b47, data.time - 94*t_total/n, t_total/n)
        elif (data.time <96* t_total/n):
            X_ref = QuadBezierInterpolate(b47, b48, b49, data.time - 95*t_total/n, t_total/n)
        elif (data.time <97* t_total/n):
            X_ref = QuadBezierInterpolate(b49, b50, b51, data.time - 96*t_total/n, t_total/n)


    #3
        elif (data.time <98* t_total/n):
            X_ref = QuadBezierInterpolate(b51, b52, b53, data.time - 97*t_total/n, t_total/n)
        elif (data.time <99* t_total/n):
            X_ref = QuadBezierInterpolate(b53, b54, b55, data.time - 98*t_total/n, t_total/n)
        elif (data.time <100* t_total/n):
            X_ref = QuadBezierInterpolate(b55, b56, b57, data.time - 99*t_total/n, t_total/n)
        elif (data.time <101* t_total/n):
            X_ref = QuadBezierInterpolate(b57, b58, b59, data.time - 100*t_total/n, t_total/n)
        elif (data.time <102* t_total/n):
            X_ref = QuadBezierInterpolate(b59, b60, b61, data.time -101*t_total/n, t_total/n)
        elif (data.time <103* t_total/n):
            X_ref = QuadBezierInterpolate(b61, b62, b63, data.time - 102*t_total/n, t_total/n)
        elif (data.time <104* t_total/n):
            X_ref = QuadBezierInterpolate(b63, b64, b65, data.time - 103*t_total/n, t_total/n)
        elif (data.time <105* t_total/n):
            X_ref = QuadBezierInterpolate(b65, b66, b67, data.time - 104*t_total/n, t_total/n)
        elif (data.time <106* t_total/n):
            X_ref = QuadBezierInterpolate(b67, b68, b69, data.time - 105*t_total/n, t_total/n)
        elif (data.time <107* t_total/n):
            X_ref = QuadBezierInterpolate(b69, b70, b71, data.time - 106*t_total/n, t_total/n)
        elif (data.time <108* t_total/n):
            X_ref = QuadBezierInterpolate(b71, b72, b73, data.time - 107*t_total/n, t_total/n)
        elif (data.time <109* t_total/n):
            X_ref = QuadBezierInterpolate(b73, b74, b75, data.time - 108*t_total/n, t_total/n)
        elif (data.time <110* t_total/n):
            X_ref = QuadBezierInterpolate(b75, b76, b77, data.time - 109*t_total/n, t_total/n)
        elif (data.time <111* t_total/n):
            X_ref = QuadBezierInterpolate(b77, b78, b79, data.time - 110*t_total/n, t_total/n)


        elif (data.time <112* t_total/n):
            X_ref = LinearInterpolate(b79, b80, data.time -111* t_total/n, t_total/n)
        elif (data.time <113* t_total/n):
            X_ref = LinearInterpolate(b80, b81, data.time - 112*t_total/n, t_total/n)
        elif (data.time <114* t_total/n):
            X_ref = QuadBezierInterpolate(b81, b82, b83, data.time - 113*t_total/n, t_total/n) 

        elif (data.time <115* t_total/n):
            X_ref = LinearInterpolate(b83, b84, data.time - 114*t_total/n, t_total/n)
        elif (data.time <2* t_total/n):
            X_ref = LinearInterpolate(b84, b85, data.time - 115*t_total/n, t_total/n)
        elif (data.time <116* t_total/n):
            X_ref = QuadBezierInterpolate(b85, b86, b87, data.time - 116*t_total/n, t_total/n)
        elif (data.time <117* t_total/n):
            X_ref = QuadBezierInterpolate(b87, b88, b89, data.time - 117*t_total/n, t_total/n)
        elif (data.time <118* t_total/n):
            X_ref = QuadBezierInterpolate(b89, b90, b91, data.time - 118*t_total/n, t_total/n)
        elif (data.time <119* t_total/n):
            X_ref = QuadBezierInterpolate(b91, b92, b93, data.time - 119*t_total/n, t_total/n)
        elif (data.time <120* t_total/n):
            X_ref = QuadBezierInterpolate(b93, b94, b95, data.time - 120*t_total/n, t_total/n)

        elif (data.time <121* t_total/n):
            X_ref = LinearInterpolate(b95, b96, data.time - 121*t_total/n, t_total/n)
        elif (data.time <122* t_total/n):
            X_ref = LinearInterpolate(b96, b97, data.time - 122*t_total/n, t_total/n)
        elif (data.time <123* t_total/n):
            X_ref = QuadBezierInterpolate(b97, b98, b99, data.time - 123*t_total/n, t_total/n)
        elif (data.time <124* t_total/n):
            X_ref = QuadBezierInterpolate(b99, b100, b101, data.time - 124*t_total/n, t_total/n)
        elif (data.time <125* t_total/n):
            X_ref = QuadBezierInterpolate(b101, b102, b103, data.time - 125*t_total/n, t_total/n)
        elif (data.time <126* t_total/n):
            X_ref = QuadBezierInterpolate(b103, b104, b105, data.time - 126*t_total/n, t_total/n)
        
        
        
        
        


        
        
        
        
        
        
        

          
        elif (data.time <simend):
            X_ref = LinearInterpolate(b105,b105, data.time - 127*t_total/n, simend-t_total)
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            #(起点，终点，划线的datatime运行时间记录,运行总时长)
        # Compute control input using IK
        cur_ctrl = IK_controller(model, data, X_ref, cur_q_pos)


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
print(cam.azimuth,cam.elevation,cam.distance,cam.lookat)
print(data.qpos)