"""《具身智能控制基础》任务 A：从 RGB 图像提取四个目标角点。"""

import cv2
import numpy as np


def detect_feature_points(image, previous_points=None):
    """返回 (points, success)，由学生实现。

    输入：
        image: uint8 RGB 图像，形状 (height, width, 3)，不是 BGR。
        previous_points: 上次成功检测的 (4, 2) 像素坐标，首次为 None。
            此信息仅用于匹配角点身份，不允许用旧坐标替代本帧检测。
    输出：
        成功：(浮点 ndarray，形状 (4, 2)，True)，每行为 (u, v)。
        失败：(None, False)，包括无目标、不完整、贴边或被裁切的情况。
    约定：
        初次按左上、右上、右下、左下排列；后续保持同一物理角点的编号。
        不修改输入数组，不读取 MuJoCo 位姿或用几何投影代替图像检测。

    TODO A：颜色分割、轮廓筛选、透视四角点提取、精化与跨帧对应。
    提示：红色色相跨 HSV 两端；倾斜四边形不能用外接矩形代替。
    建议拒绝面积过小（例如 <30 px²）或距边界不足 5 px 的目标。
    """
    raise NotImplementedError(
        "任务 A 未完成：请实现 feature_detection.py 的 detect_feature_points()。"
    )
