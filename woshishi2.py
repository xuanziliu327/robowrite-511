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
import sys

xml_path = '../../models/universal_robots_ur5e/scene.xml' #xml file (assumes this is in the same folder as this file)
#################################
## USER CODE: Set simulation parameters here
#################################
simend = 180  # 仿真时间设为180秒(3分钟)，符合要求
print_camera_config = 0  # 设置为0，不打印相机配置

# 字体和字符处理相关参数
import math

# 简单字符模式
def get_simple_char_strokes(char):
    """简单字符笔画生成"""
    strokes = []
    
    # 如果是空格或标点，返回空列表
    if char == ' ' or char == ',' or char == '.':
        return strokes
    
    # 转换为小写以简化处理
    char_lower = char.lower()
    
    # 基本字母的简单表示
    if char_lower == 'l':
        strokes.append([[0.00, 0.00], [0.00, 0.10], [0.08, 0.10]])  # L形
    elif char_lower == 'i':
        strokes.append([[0.04, 0.00], [0.04, 0.08]])  # 竖线
        strokes.append([[0.04, 0.09], [0.04, 0.095]])  # 点
    elif char_lower == 'u':
        strokes.append([[0.00, 0.10], [0.00, 0.02], [0.04, 0.00], [0.08, 0.02], [0.08, 0.10]])  # U形
    elif char_lower == 'a':
        strokes.append([[0.00, 0.00], [0.04, 0.10], [0.08, 0.00]])  # 三角形顶部
        strokes.append([[0.02, 0.05], [0.06, 0.05]])  # 横线
    elif char_lower == 'b':
        strokes.append([[0.00, 0.00], [0.00, 0.10]])  # 竖线
        strokes.append([[0.00, 0.10], [0.04, 0.08], [0.04, 0.02], [0.00, 0.00]])  # 半圆
    elif char_lower == 'c':
        strokes.append([[0.08, 0.02], [0.04, 0.00], [0.00, 0.02], [0.00, 0.08], [0.04, 0.10], [0.08, 0.08]])
    elif char_lower == 'd':
        strokes.append([[0.08, 0.00], [0.08, 0.10]])  # 竖线
        strokes.append([[0.08, 0.10], [0.04, 0.08], [0.04, 0.02], [0.08, 0.00]])  # 半圆
    elif char_lower == 'e':
        strokes.append([[0.08, 0.00], [0.00, 0.00], [0.00, 0.10], [0.08, 0.10]])  # 左边和上下边
        strokes.append([[0.00, 0.05], [0.06, 0.05]])  # 横线
    elif char_lower == 'f':
        strokes.append([[0.00, 0.00], [0.00, 0.10]])  # 竖线
        strokes.append([[0.00, 0.10], [0.06, 0.10]])  # 上横线
        strokes.append([[0.00, 0.05], [0.04, 0.05]])  # 中横线
    elif char_lower == 'g':
        strokes.append([[0.04, 0.00], [0.08, 0.02], [0.08, 0.08], [0.04, 0.10], 
                       [0.00, 0.08], [0.00, 0.02], [0.04, 0.00]])  # 圆圈
        strokes.append([[0.04, 0.05], [0.04, 0.12]])  # 下竖线
    elif char_lower == 'h':
        strokes.append([[0.00, 0.00], [0.00, 0.10]])  # 左竖线
        strokes.append([[0.08, 0.00], [0.08, 0.10]])  # 右竖线
        strokes.append([[0.00, 0.05], [0.08, 0.05]])  # 横线
    elif char_lower == 'j':
        strokes.append([[0.06, 0.00], [0.06, 0.08]])  # 竖线
        strokes.append([[0.06, 0.08], [0.04, 0.10], [0.02, 0.10], [0.00, 0.08]])  # 钩
        strokes.append([[0.06, 0.09], [0.06, 0.095]])  # 点
    elif char_lower == 'k':
        strokes.append([[0.00, 0.00], [0.00, 0.10]])  # 竖线
        strokes.append([[0.00, 0.05], [0.08, 0.00]])  # 左上斜线
        strokes.append([[0.00, 0.05], [0.08, 0.10]])  # 左下斜线
    elif char_lower == 'm':
        strokes.append([[0.00, 0.00], [0.00, 0.10]])  # 左竖线
        strokes.append([[0.04, 0.00], [0.04, 0.10]])  # 中竖线
        strokes.append([[0.08, 0.00], [0.08, 0.10]])  # 右竖线
        strokes.append([[0.00, 0.10], [0.04, 0.05], [0.08, 0.10]])  # 顶部
    elif char_lower == 'n':
        strokes.append([[0.00, 0.00], [0.00, 0.10]])  # 左竖线
        strokes.append([[0.08, 0.00], [0.08, 0.10]])  # 右竖线
        strokes.append([[0.00, 0.10], [0.08, 0.00]])  # 斜线
    elif char_lower == 'o':
        strokes.append([[0.04, 0.00], [0.08, 0.02], [0.08, 0.08], [0.04, 0.10], 
                       [0.00, 0.08], [0.00, 0.02], [0.04, 0.00]])
    elif char_lower == 'p':
        strokes.append([[0.00, 0.00], [0.00, 0.10]])  # 竖线
        strokes.append([[0.00, 0.05], [0.04, 0.05], [0.04, 0.10], [0.00, 0.10]])  # 上半圆
    elif char_lower == 'q':
        strokes.append([[0.04, 0.00], [0.08, 0.02], [0.08, 0.08], [0.04, 0.10], 
                       [0.00, 0.08], [0.00, 0.02], [0.04, 0.00]])  # 圆圈
        strokes.append([[0.06, 0.05], [0.10, 0.00]])  # 斜线
    elif char_lower == 'r':
        strokes.append([[0.00, 0.00], [0.00, 0.10]])  # 竖线
        strokes.append([[0.00, 0.10], [0.04, 0.08], [0.04, 0.02], [0.00, 0.00]])  # 上半部分
        strokes.append([[0.00, 0.05], [0.04, 0.00]])  # 右下斜线
    elif char_lower == 's':
        strokes.append([[0.08, 0.02], [0.04, 0.00], [0.00, 0.02], [0.00, 0.08], [0.04, 0.10], [0.08, 0.08]])
    elif char_lower == 't':
        strokes.append([[0.04, 0.00], [0.04, 0.10]])  # 竖线
        strokes.append([[0.00, 0.10], [0.08, 0.10]])  # 顶横线
    elif char_lower == 'v':
        strokes.append([[0.00, 0.10], [0.04, 0.00], [0.08, 0.10]])
    elif char_lower == 'w':
        strokes.append([[0.00, 0.10], [0.02, 0.00], [0.04, 0.05], [0.06, 0.00], [0.08, 0.10]])
    elif char_lower == 'x':
        strokes.append([[0.00, 0.00], [0.08, 0.10]])  # 左上到右下
        strokes.append([[0.00, 0.10], [0.08, 0.00]])  # 左下到右上
    elif char_lower == 'y':
        strokes.append([[0.00, 0.10], [0.04, 0.05], [0.08, 0.10]])  # V形
        strokes.append([[0.04, 0.05], [0.04, 0.00]])  # 竖线
    elif char_lower == 'z':
        strokes.append([[0.00, 0.10], [0.08, 0.10]])  # 顶横线
        strokes.append([[0.08, 0.10], [0.00, 0.00]])  # 斜线
        strokes.append([[0.00, 0.00], [0.08, 0.00]])  # 底横线
    elif char.isdigit():
        # 数字的简单表示
        if char == '0':
            strokes.append([[0.04, 0.00], [0.08, 0.02], [0.08, 0.08], [0.04, 0.10], 
                           [0.00, 0.08], [0.00, 0.02], [0.04, 0.00]])
        elif char == '1':
            strokes.append([[0.04, 0.00], [0.04, 0.10]])
            strokes.append([[0.02, 0.08], [0.06, 0.08]])  # 底部横线
        elif char == '2':
            strokes.append([[0.00, 0.08], [0.04, 0.10], [0.08, 0.08], [0.08, 0.02], [0.00, 0.02], [0.00, 0.00], [0.08, 0.00]])
        elif char == '3':
            strokes.append([[0.00, 0.10], [0.08, 0.10], [0.04, 0.05], [0.08, 0.05], [0.08, 0.00], [0.00, 0.00]])
        elif char == '4':
            strokes.append([[0.06, 0.00], [0.06, 0.10]])  # 右竖线
            strokes.append([[0.06, 0.05], [0.00, 0.05]])  # 横线
            strokes.append([[0.00, 0.05], [0.00, 0.10]])  # 左竖线
        elif char == '5':
            strokes.append([[0.08, 0.10], [0.00, 0.10], [0.00, 0.05], [0.08, 0.05], [0.08, 0.00], [0.00, 0.00]])
        elif char == '6':
            strokes.append([[0.08, 0.10], [0.00, 0.10], [0.00, 0.00], [0.08, 0.00], [0.08, 0.05], [0.00, 0.05]])
        elif char == '7':
            strokes.append([[0.00, 0.10], [0.08, 0.10], [0.04, 0.00]])
        elif char == '8':
            strokes.append([[0.04, 0.00], [0.08, 0.02], [0.08, 0.05], [0.04, 0.05], [0.08, 0.05], [0.08, 0.08], [0.04, 0.10], 
                           [0.00, 0.08], [0.00, 0.05], [0.04, 0.05], [0.00, 0.05], [0.00, 0.02], [0.04, 0.00]])
        elif char == '9':
            strokes.append([[0.08, 0.00], [0.08, 0.10], [0.00, 0.10], [0.00, 0.05], [0.08, 0.05]])
    else:
        # 默认画一个方框
        strokes.append([[0.00, 0.00], [0.08, 0.00], [0.08, 0.10], [0.00, 0.10], [0.00, 0.00]])
    
    return strokes

# 书写参数
WRITING_PLANE_Z = 0.1  # 书写平面高度
WRITING_AREA = {
    'min_x': -0.5, 'max_x': 0.5,
    'min_y': 0.1, 'max_y': 0.6
}  # 书写区域

# 球面书写参数
SPHERE_CENTER = np.array([0.0, 0.35, 1.3])  # 球心坐标
SPHERE_RADIUS = 1.3  # 球半径
SPHERE_Z_MIN = 0.0  # z坐标最小值
SPHERE_Z_MAX = 0.1  # z坐标最大值

# 字符大小和间距
CHAR_WIDTH = 0.08  # 字符宽度
CHAR_HEIGHT = 0.1  # 字符高度
CHAR_SPACING = 0.02  # 字符间距
LINE_SPACING = 0.12  # 行间距

# 轨迹规划参数
LIFT_HEIGHT = 0.05  # 抬笔高度
WRITING_SPEED = 0.008  # 书写速度 (m/s)
INTERPOLATION_STEPS = 20  # 插值步数

# 全局变量用于轨迹规划
trajectory_points = []  # 存储所有轨迹点
current_traj_index = 0  # 当前轨迹点索引
is_writing = False  # 是否正在书写
is_sphere_mode = False  # 是否使用球面模式
last_point_time = 0  # 最后点时间
target_X_ref = np.array([0.0, 0.1, 0.1])  # 目标位置
should_draw_trajectory = True  # 是否应该绘制轨迹

# 贝塞尔插值函数
def bezier_interpolation(p0, p1, p2, p3, steps):
    """三次贝塞尔曲线插值"""
    points = []
    for i in range(steps + 1):
        t = i / steps
        # 贝塞尔曲线公式
        point = (1-t)**3 * p0 + 3*(1-t)**2*t * p1 + 3*(1-t)*t**2 * p2 + t**3 * p3
        points.append(point)
    return points

def linear_interpolation(p0, p1, steps):
    """线性插值"""
    points = []
    for i in range(steps + 1):
        t = i / steps
        point = p0 * (1-t) + p1 * t
        points.append(point)
    return points

def plan_character_trajectory(strokes, start_pos):
    """规划单个字符的轨迹"""
    points = []
    
    if not strokes:
        return points, start_pos
    
    for stroke_idx, stroke in enumerate(strokes):
        if len(stroke) < 2:
            continue
            
        # 如果是第一个笔画，移动到起点上方
        if stroke_idx == 0:
            # 抬笔到起点上方
            start_above = np.array([start_pos[0], start_pos[1], WRITING_PLANE_Z + LIFT_HEIGHT])
            points.append(start_above)
            
            # 落笔到起点
            stroke_start_3d = np.array([start_pos[0] + stroke[0][0], 
                                       start_pos[1] + stroke[0][1], 
                                       WRITING_PLANE_Z])
            points.extend(linear_interpolation(start_above, stroke_start_3d, INTERPOLATION_STEPS // 2))
        else:
            # 抬笔到下一个笔画的起点上方
            last_point = points[-1]
            lift_point = np.array([last_point[0], last_point[1], WRITING_PLANE_Z + LIFT_HEIGHT])
            points.append(lift_point)
            
            next_stroke_start_3d = np.array([start_pos[0] + stroke[0][0], 
                                            start_pos[1] + stroke[0][1], 
                                            WRITING_PLANE_Z])
            next_stroke_above = np.array([next_stroke_start_3d[0], 
                                         next_stroke_start_3d[1], 
                                         WRITING_PLANE_Z + LIFT_HEIGHT])
            
            # 移动到下一个笔画起点上方
            points.extend(linear_interpolation(lift_point, next_stroke_above, INTERPOLATION_STEPS))
            
            # 落笔
            points.extend(linear_interpolation(next_stroke_above, next_stroke_start_3d, INTERPOLATION_STEPS // 2))
        
        # 书写笔画
        for i in range(len(stroke) - 1):
            p0_2d = stroke[i]
            p1_2d = stroke[i + 1]
            
            # 转换为三维坐标
            p0 = np.array([start_pos[0] + p0_2d[0], start_pos[1] + p0_2d[1], WRITING_PLANE_Z])
            p1 = np.array([start_pos[0] + p1_2d[0], start_pos[1] + p1_2d[1], WRITING_PLANE_Z])
            
            # 对于直线段使用线性插值，对于曲线段使用贝塞尔插值
            if i < len(stroke) - 2:
                p2_2d = stroke[i + 2]
                p2 = np.array([start_pos[0] + p2_2d[0], start_pos[1] + p2_2d[1], WRITING_PLANE_Z])
                
                # 计算控制点
                cp1 = p0 + (p1 - p0) * 0.3
                cp2 = p1 + (p2 - p1) * 0.3
                
                # 贝塞尔插值
                points.extend(bezier_interpolation(p0, cp1, cp2, p1, INTERPOLATION_STEPS))
            else:
                # 线性插值
                points.extend(linear_interpolation(p0, p1, INTERPOLATION_STEPS))
    
    return points, start_pos

def project_to_sphere(point_2d, row, col, total_rows, total_cols):
    """将平面点投影到球面上（三维正交参考系变换）"""
    # 将二维书写区域映射到球面参数空间
    u = point_2d[0] / (total_cols * (CHAR_WIDTH + CHAR_SPACING)) * 2 - 1
    v = point_2d[1] / (total_rows * (CHAR_HEIGHT + LINE_SPACING)) * 2 - 1
    
    # 限制在[-1, 1]范围内
    u = max(-1, min(1, u))
    v = max(-1, min(1, v))
    
    # 计算球面坐标（参数方程）
    theta = math.pi * 0.5 + u * math.pi * 0.25  # 经度，限制在90°±45°范围内
    phi = math.pi * 0.3 + v * math.pi * 0.2     # 纬度，限制在54°±18°范围内
    
    # 转换为笛卡尔坐标
    x = SPHERE_RADIUS * math.sin(phi) * math.cos(theta)
    y = SPHERE_RADIUS * math.sin(phi) * math.sin(theta) + SPHERE_CENTER[1]
    z = SPHERE_RADIUS * math.cos(phi)
    
    # 调整z坐标以确保在指定范围内
    if z < SPHERE_Z_MIN:
        z = SPHERE_Z_MIN
    elif z > SPHERE_Z_MAX:
        z = SPHERE_Z_MAX
    
    return np.array([x, y, z])

def calculate_writing_time(trajectory_points):
    """计算书写时间"""
    total_distance = 0
    for i in range(1, len(trajectory_points)):
        total_distance += np.linalg.norm(trajectory_points[i] - trajectory_points[i-1])
    
    return total_distance / WRITING_SPEED

#################################

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# 改进的逆运动学控制器
def IK_controller(model, data, X_ref, q_pos, Kp=2.0):
    # 获取当前位置
    current_pos = data.site_xpos[0]
    
    # 计算位置误差
    error = X_ref - current_pos
    
    # 计算雅可比矩阵
    jacp = np.zeros((3, 6))
    mj.mj_jac(model, data, jacp, None, current_pos, 7)
    
    # 计算伪逆
    J = jacp.copy()
    try:
        Jinv = np.linalg.pinv(J)
    except:
        Jinv = np.zeros((6, 3))
    
    # 计算关节速度
    dq = Jinv @ (error * Kp)
    
    # 限制关节速度
    max_dq = 0.5  # 最大关节速度
    dq = np.clip(dq, -max_dq, max_dq)
    
    return q_pos + dq * 0.01

def init_controller(model,data):
    # 初始化控制器
    global trajectory_points, current_traj_index, is_writing, is_sphere_mode, WRITING_SPEED, target_X_ref
    
    # 获取用户输入
    user_input = input("请输入要书写的字符串（仅包含英文字母、数字、空格、逗号和句号）: ")
    
    # 验证输入
    valid_chars = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 ,.")
    for char in user_input:
        if char not in valid_chars:
            print(f"错误：字符 '{char}' 不被支持")
            return
    
    # 询问是否使用球面模式
    sphere_choice = input("是否使用球面书写模式？(y/n): ").lower()
    is_sphere_mode = sphere_choice == 'y'
    
    # 计算行列数
    chars_per_line = int((WRITING_AREA['max_x'] - WRITING_AREA['min_x']) / (CHAR_WIDTH + CHAR_SPACING))
    max_lines = 2  # 最多两行
    
    # 检查内容是否超出书写区域
    non_space_chars = len([c for c in user_input if c not in [' ', ',', '.']])
    lines_needed = (non_space_chars + chars_per_line - 1) // chars_per_line
    if lines_needed > max_lines:
        print("错误：您的书写内容超出书写区域")
        return
    
    print(f"开始规划轨迹...")
    
    # 初始化轨迹点
    trajectory_points = []
    current_traj_index = 0
    
    # 起始位置（抬笔状态）
    start_pos = np.array([WRITING_AREA['min_x'], WRITING_AREA['max_y'], WRITING_PLANE_Z + LIFT_HEIGHT])
    trajectory_points.append(start_pos)
    
    # 当前书写位置
    current_x = WRITING_AREA['min_x']
    current_y = WRITING_AREA['max_y']
    current_line = 0
    chars_in_line = 0
    
    # 处理每个字符
    for char in user_input:
        if char == ' ':
            # 空格：移动到下一个位置
            current_x += CHAR_WIDTH + CHAR_SPACING
            chars_in_line += 1
            continue
        
        if char == ',' or char == '.':
            # 标点符号：使用较小尺寸
            char_width = CHAR_WIDTH * 0.5
        else:
            char_width = CHAR_WIDTH
        
        # 检查是否需要换行
        if current_x + char_width > WRITING_AREA['max_x']:
            current_line += 1
            if current_line >= max_lines:
                print("错误：超出最大行数")
                return
            
            # 换行：抬笔移动到下一行开始
            current_x = WRITING_AREA['min_x']
            current_y -= (CHAR_HEIGHT + LINE_SPACING)
            chars_in_line = 0
            
            # 添加抬笔移动轨迹
            lift_point = trajectory_points[-1].copy()
            lift_point[2] = WRITING_PLANE_Z + LIFT_HEIGHT
            trajectory_points.append(lift_point)
            
            next_pos = np.array([current_x, current_y, WRITING_PLANE_Z + LIFT_HEIGHT])
            trajectory_points.extend(linear_interpolation(lift_point, next_pos, INTERPOLATION_STEPS))
        
        # 获取字符笔画
        strokes = get_simple_char_strokes(char)
        
        if strokes:
            # 字符起始位置
            char_start_pos = np.array([current_x, current_y, WRITING_PLANE_Z])
            
            # 如果是球面模式，投影到球面
            if is_sphere_mode:
                char_start_pos = project_to_sphere(
                    char_start_pos[:2], 
                    current_line, 
                    chars_in_line, 
                    max_lines, 
                    chars_per_line
                )
            
            print(f"字符 '{char}' 起始位置: {char_start_pos}")
            
            # 规划字符轨迹
            char_points, _ = plan_character_trajectory(strokes, char_start_pos)
            
            if len(char_points) > 0:
                trajectory_points.extend(char_points)
        
        # 更新位置
        current_x += char_width + CHAR_SPACING
        chars_in_line += 1
    
    # 添加终止位置
    if trajectory_points:
        # 抬笔
        last_point = trajectory_points[-1].copy()
        last_point[2] = WRITING_PLANE_Z + LIFT_HEIGHT
        trajectory_points.append(last_point)
        
        # 移动到终止位姿上方
        end_pos_above = np.array([0.0, 0.1, WRITING_PLANE_Z + LIFT_HEIGHT])
        trajectory_points.extend(linear_interpolation(last_point, end_pos_above, INTERPOLATION_STEPS))
        
        # 移动到终止位姿
        end_pos = np.array([0.0, 0.1, WRITING_PLANE_Z])
        trajectory_points.extend(linear_interpolation(end_pos_above, end_pos, INTERPOLATION_STEPS // 2))
    
    # 计算总时间
    total_time = calculate_writing_time(trajectory_points)
    print(f"轨迹规划完成，共{len(trajectory_points)}个点，预计时间: {total_time:.2f}秒")
    
    if total_time > simend:
        print(f"警告：预计时间({total_time:.2f}s)超过限制({simend}s)，将调整速度")
        WRITING_SPEED *= (total_time / simend) * 1.1
        print(f"调整后的书写速度: {WRITING_SPEED:.6f} m/s")
    
    # 设置初始目标点
    if trajectory_points:
        target_X_ref = trajectory_points[0].copy()
    
    is_writing = True
    print(f"开始书写，总轨迹点数: {len(trajectory_points)}")

def controller(model, data):
    # 控制器主函数 - 直接控制版本
    global trajectory_points, current_traj_index, is_writing, target_X_ref
    
    if not is_writing or current_traj_index >= len(trajectory_points):
        # 如果书写完成，移动到终止位姿
        if current_traj_index >= len(trajectory_points):
            # 设置终止位姿
            terminal_qpos = np.array([0.0, -2.32, -1.38, -2.45, 1.57, 0.0])
            data.ctrl[:] = terminal_qpos
            return
        
        # 等待开始
        data.ctrl[:] = data.qpos.copy()
        return
    
    # 获取当前位置和目标位置
    current_pos = data.site_xpos[0]
    target_pos = trajectory_points[current_traj_index]
    
    # 计算距离
    distance = np.linalg.norm(current_pos - target_pos)
    
    # 如果距离很小，移动到下一个点
    if distance < 0.01:  # 1cm精度
        current_traj_index += 1
        if current_traj_index < len(trajectory_points):
            target_pos = trajectory_points[current_traj_index]
            target_X_ref = target_pos.copy()
            print(f"移动到轨迹点 {current_traj_index}/{len(trajectory_points)}")
        else:
            print("轨迹完成")
            is_writing = False
            return
    
    # 使用逆运动学计算控制量
    cur_q_pos = data.qpos.copy()
    data.ctrl[:] = IK_controller(model, data, target_pos, cur_q_pos)

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
# 设置相机视角以清晰观察整个书写过程
cam.azimuth = 45.0  # 方位角，提供斜视角
cam.elevation = -25.0  # 仰角，稍微俯视
cam.distance = 2.5  # 距离，确保能看到整个工作区域
cam.lookat = np.array([0.0, 0.35, 0.3])  # 注视点：书写区域中心偏上
########################################

# 初始化全局变量（已在顶部定义）

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
LINE_RGBA = np.array([1.0, 0.0, 0.0, 1.0])  # 红色轨迹

######################################
## USER CODE STARTS HERE
######################################

# 轨迹记录函数
def should_record_trajectory(pos):
    """判断是否应该记录轨迹点"""
    # 如果当前点在抬笔高度以上，不记录
    if pos[2] > WRITING_PLANE_Z + LIFT_HEIGHT * 0.5:
        return False
    
    # 确保在书写区域内
    if pos[0] < WRITING_AREA['min_x'] - 0.1 or pos[0] > WRITING_AREA['max_x'] + 0.1 or \
       pos[1] < WRITING_AREA['min_y'] - 0.1 or pos[1] > WRITING_AREA['max_y'] + 0.1:
        return False
    
    return True

######################################
## USER CODE ENDS HERE
######################################

# 记录轨迹点的列表
recorded_points = []

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        # 获取当前末端执行器位置
        mj_end_eff_pos = data.site_xpos[0]
        
        # 记录轨迹点
        if should_record_trajectory(mj_end_eff_pos):
            recorded_points.append(mj_end_eff_pos.copy())
        
        if len(recorded_points) > MAX_TRAJ:
            recorded_points.pop(0)
            
        # 仿真步进
        mj.mj_step(model, data)
        data.time += 0.002

    if (data.time >= simend):
        print("仿真时间结束")
        break

    # 检查是否完成所有轨迹点
    if current_traj_index >= len(trajectory_points) and is_writing:
        print("书写完成！")
        is_writing = False

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # 打印相机配置（如果需要）
    if (print_camera_config == 1):
        print('cam.azimuth = ', cam.azimuth,'\n','cam.elevation = ', cam.elevation,'\n','cam.distance = ', cam.distance)
        print('cam.lookat = np.array([', cam.lookat[0],',', cam.lookat[1],',', cam.lookat[2],'])')

    # 更新场景和渲染
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    
    # 添加轨迹点作为球体（只在书写时显示）
    if should_draw_trajectory:
        for j in range(1, min(len(recorded_points), 1000)):  # 限制显示的点数
            if scene.ngeom >= scene.maxgeom:
                break  # 避免溢出

            geom = scene.geoms[scene.ngeom]
            scene.ngeom += 1
            
            p1 = recorded_points[j-1]
            p2 = recorded_points[j]
            
            # 计算线段中点
            midpoint = (p1 + p2) / 2.0
            
            # 配置几何体为球体
            geom.type = mj.mjtGeom.mjGEOM_SPHERE
            geom.rgba[:] = LINE_RGBA
            geom.size[:] = np.array([0.003, 0.003, 0.003])  # 小球体表示轨迹点
            geom.pos[:] = midpoint
            geom.mat[:] = np.eye(3)  # 无旋转
            geom.dataid = -1
            geom.segid = -1
            geom.objtype = 0
            geom.objid = 0
    
    mj.mjr_render(viewport, scene, context)

    # 交换OpenGL缓冲区
    glfw.swap_buffers(window)

    # 处理GUI事件
    glfw.poll_events()

glfw.terminate()