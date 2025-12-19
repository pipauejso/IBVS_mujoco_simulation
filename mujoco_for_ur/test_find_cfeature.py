import cv2
import numpy as np
from visual_servo_module import VisualServoController  # 导入视觉伺服模块
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
# desired_points = None
# def set_desired_features(points_star):
#         """
#         设置特征点的期望位置
        
#         参数:
#         - points_star: 4个角点的期望像素坐标 (4x2)
#         """
#     global desired_points
#     desired_points = points_star

def find_red_corners(image_path):
        # 加载Mujoco模型
    model = mujoco.MjModel.from_xml_path('universal_robots_ur5e/scene.xml')
    data = mujoco.MjData(model)
    # 1. 读取图片
    img = cv2.imread(image_path)
    plt.imshow(img)
    if img is None:
        print("错误: 无法找到图片，请检查路径。")
        return
    vs_controller = VisualServoController(model, data, 'end_effector_camera', 'target', (640, 480))
    vs_controller.get_feature_points(img)
    # 2. 预处理：转换到 HSV 颜色空间
    # HSV 更有利于颜色分割，因为红色在 Hue (色相) 的两端 (0-10 和 170-180)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 3. 定义红色的 HSV 范围
    # 红色通常跨越 0 和 180，所以需要定义两个范围并合并
    
    # 范围 1: 0-10 (深红/橙红)
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    
    # 范围 2: 170-180 (紫红)
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    # 创建掩膜 (Mask)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2  # 合并两个掩膜

    # 4. 形态学操作 (去除噪点和填充内部孔洞)
    # 图片中的红色方块里有小蓝点，我们需要用“闭运算”把它们填满，保证方块是实心的
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # 也可以稍微膨胀一点，确保边缘平滑
    mask = cv2.dilate(mask, kernel, iterations=1)

    # 5. 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        print("未检测到红色物体")
        return

    # 找到面积最大的轮廓 (假设红色方块是主要的红色物体)
    largest_contour = max(contours, key=cv2.contourArea)

    # 6. 多边形拟合 (获取 4 个角点)
    # 计算轮廓周长
    epsilon = 0.02 * cv2.arcLength(largest_contour, True)
    # 进行多边形逼近
    approx = cv2.approxPolyDP(largest_contour, epsilon, True)

    # 如果拟合结果正好是 4 个点，直接使用；
    # 如果不是（可能因为边缘不平滑），则使用最小外接矩形 (MinAreaRect)
    corners = []
    
    if len(approx) == 4:
        print("检测到精确的四边形。")
        corners = approx.reshape(-1, 2) # 转换为 (4, 2) 的数组
    else:
        print(f"检测到轮廓（拟合点数: {len(approx)}），使用最小外接矩形修正。")
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        corners = np.int0(box)

    # 7. 绘制结果
    # 画出轮廓
    cv2.drawContours(img, [largest_contour], -1, (0, 255, 0), 2)
    
    # 画出 4 个角点并打印坐标
    print("4 个角点的坐标 (x, y):")
    for i, point in enumerate(corners):
        x, y = point
        print(f"点 {i+1}: ({x}, {y})")
        
        # 在图上画圆圈
        cv2.circle(img, (x, y), 8, (255, 0, 0), -1) # 蓝色实心圆
        # 标上序号
        cv2.putText(img, str(i+1), (x - 20, y - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    # 显示结果
    cv2.imshow("Red Mask", mask) # 显示掩膜（调试用）
    cv2.imshow("Detected Corners", img) # 显示最终结果
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# --- 执行 ---
# 请将此处的文件名替换为你保存的图片文件名
find_red_corners("End-Effector Camera_screenshot_12.12.2025.png")