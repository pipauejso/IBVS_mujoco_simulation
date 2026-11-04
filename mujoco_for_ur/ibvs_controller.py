"""《具身智能控制基础》任务 B：四点 IBVS 交互矩阵与速度控制律。"""

import numpy as np


def calculate_interaction_matrix(points, depths, intrinsics):
    """计算归一化图像坐标的交互矩阵 L，形状 (8, 6)。

    points: (4, 2)，四点像素坐标，每行 (u, v)。
    depths: (4,)，与各点一一对应的光学 Z 深度，单位 m，必须有限且 >0。
    intrinsics: (fx, fy, cx, cy)，单位 px。
    输出行顺序：[x1, y1, x2, y2, x3, y3, x4, y4] 的时间导数。
    输出列顺序：相机速度 [vx, vy, vz, wx, wy, wz]。
    相机光学坐标轴：向右、向下、向前；线速度 m/s，角速度 rad/s。

    TODO B1：从针孔模型推导并实现逐点交互矩阵；拒绝无效深度。
    注意：输入是像素坐标，输出描述归一化坐标的变化率。
    """
    raise NotImplementedError(
        "任务 B1 未完成：请实现 ibvs_controller.py 的 calculate_interaction_matrix()。"
    )


def compute_joint_velocity(points, desired_points, depths, intrinsics,
                           robot_jacobian, lambda_gain=2.0, damping=1e-4):
    """计算关节速度 qdot，返回形状 (n,) 的有限浮点 ndarray，单位 rad/s。

    points / desired_points: (4, 2)，当前/期望像素坐标，角点身份一致。
    depths: (4,)，当前逐点光学 Z 深度，单位 m。
    intrinsics: (fx, fy, cx, cy)。
    robot_jacobian: (6, n)，把 qdot 映射到相机光学坐标系的 [v, omega]。
    lambda_gain: 正数控制增益，期望误差随时间衰减。
    damping: 阻尼参数，用于处理病态矩阵。

    TODO B2：
        1. 构造归一化图像误差（约定 current - desired）。
        2. 调用 B1，结合机器人雅可比建立图像误差与关节速度的关系。
        3. 设计负反馈，用阻尼最小二乘求解关节速度。
    不读取目标真值位姿，不将误差直接当作关节角/关节位置。
    速度限幅、丢失目标停车和收敛停车由外层框架提供。
    """
    raise NotImplementedError(
        "任务 B2 未完成：请实现 ibvs_controller.py 的 compute_joint_velocity()。"
    )
