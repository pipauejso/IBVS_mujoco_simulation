# visual_servo_module.py
# 完整的 VisualServoController 类（已修复深度、符号方向、稳定性问题）

import numpy as np
import cv2
from scipy.spatial.transform import Rotation

class VisualServoController:
    def __init__(self, model, data, camera_name, target_name, resolution, lambda_gain=0.2):
        """
        初始化视觉伺服控制器（推荐 lambda_gain=0.1~0.3，避免振荡）
        """
        self.model = model
        self.data = data
        self.resolution = resolution
        self.lambda_gain = lambda_gain

        # camera / target ids
        self.camera_id = model.camera(camera_name).id
        self.target_body_id = model.body(target_name).id

        # intrinsics
        self.fx, self.fy, self.cx, self.cy = self._calculate_camera_intrinsics()

        # target geom size (half-sizes for box)
        self.target_size = self._get_target_size(target_name)

        self.desired_points = None
        self.desired_z = None  # 新增：期望深度
        self.current_points = None

        print(
            f"视觉伺服控制器: 相机内参 - fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.2f}, cy={self.cy:.2f}"
        )
        print(f"视觉伺服控制器: 目标尺寸 - {self.target_size}")

    def _get_target_size(self, target_name):
        body_id = self.model.body(target_name).id
        for i in range(self.model.ngeom):
            if self.model.geom_bodyid[i] == body_id:
                return self.model.geom_size[i].copy()
        return np.array([0.05, 0.05, 0.02], dtype=np.float64)

    def _calculate_camera_intrinsics(self):
        width, height = self.resolution
        fovy_deg = float(self.model.cam_fovy[self.camera_id])
        fovy = np.deg2rad(fovy_deg)
        fx = fy = height / (2.0 * np.tan(fovy / 2.0))
        cx = width / 2.0
        cy = height / 2.0
        return float(fx), float(fy), float(cx), float(cy)

    def _extract_features_from_image(self, image):
        """
        输出 4 个角点 (4x2, float32)，顺序: [左上, 右上, 右下, 左下]
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print("未检测到红色物体")
            return None, False

        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) < 20:
            print("红色区域太小，忽略")
            return None, False

        rect = cv2.minAreaRect(largest_contour)
        corners = cv2.boxPoints(rect).astype(np.float32)

        # 亚像素精炼
        try:
            gray = mask.astype(np.uint8)
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            corners = cv2.cornerSubPix(gray, corners.reshape(-1, 1, 2), (5, 5), (-1, -1), term).reshape(-1, 2)
        except:
            pass

        # 排序到固定顺序
        center = corners.mean(axis=0)
        angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
        corners = corners[np.argsort(angles)]
        start = np.argmin(corners[:, 0] + corners[:, 1])
        corners = np.roll(corners, -start, axis=0)

        # 修正方向（顺时针）
        area2 = 0.0
        for i in range(4):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % 4]
            area2 += (x1 * y2 - x2 * y1)
        if area2 < 0:
            corners = corners[[0, 3, 2, 1]]

        return corners.astype(np.float32), True

    def _get_depth_for_points(self):
        P_w_tar = self.data.xpos[self.target_body_id].copy()
        R_w_tar = self.data.xmat[self.target_body_id].reshape((3, 3))
        hx, hy, _ = self.target_size
        offsets_local = [
            np.array([+hx, +hy, 0.0]),
            np.array([+hx, -hy, 0.0]),
            np.array([-hx, -hy, 0.0]),
            np.array([-hx, +hy, 0.0]),
        ]
        P_w_corners = [P_w_tar + R_w_tar @ offset for offset in offsets_local]

        R_w_cam = self.data.cam_xmat[self.camera_id].reshape((3, 3))
        P_w_cam = self.data.cam_xpos[self.camera_id].copy()
        R_cam_w = R_w_cam.T

        depths = []
        for P_w_corner in P_w_corners:
            P_cam = R_cam_w @ (P_w_corner - P_w_cam)
            depths.append(float(-P_cam[2]))  # 正深度
        return depths

    def get_feature_points(self, image=None):
        if image is None or not isinstance(image, np.ndarray):
            return None, None

        points, success = self._extract_features_from_image(image)
        if not success:
            return None, None

        Z = self._get_depth_for_points()
        self.current_points = points
        return points, Z

    def set_desired_features(self, points_star):
        self.desired_points = points_star
        self.desired_z = self._get_depth_for_points()  # 保存期望深度

    def calculate_interaction_matrix(self, points, Z):
        Z_array = np.asarray(Z, dtype=np.float64).reshape(-1)
        L = np.zeros((8, 6), dtype=np.float64)
        for i in range(4):
            u, v = float(points[i][0]), float(points[i][1])
            Z_i = max(float(Z_array[i]), 1e-6)
            x = (u - self.cx) / self.fx
            y = (v - self.cy) / self.fy
            L_i = np.array([
                [-1/Z_i, 0, x/Z_i, x*y, -(1 + x*x), y],
                [0, -1/Z_i, y/Z_i, 1 + y*y, -x*y, -x]
            ], dtype=np.float64)
            L[2*i:2*i+2] = L_i
        return L

    def process_visual_servo(self, robot_controller, current_joint_angles, image=None):
        points, Z_current = self.get_feature_points(image)
        if points is None:
            return None

        # 如果未设置期望，使用当前位置
        if self.desired_points is None:
            self.set_desired_features(points)
            print("已自动设置期望特征点为当前位置")
            return None

        # 误差：期望 - 当前（标准 e = s* - s）
        error_norm = np.zeros((8, 1), dtype=np.float64)
        error_pixel = np.zeros((8, 1))
        for i in range(4):
            error_pixel[2*i]   = self.desired_points[i][0] - points[i][0]
            error_pixel[2*i+1] = self.desired_points[i][1] - points[i][1]
            error_norm[2*i]   = error_pixel[2*i] / self.fx
            error_norm[2*i+1] = error_pixel[2*i+1] / self.fy

        err_mag_pixel = np.linalg.norm(error_pixel)
        print(f"\n=== 视觉伺服调试 ===")
        print(f"深度值 Z: {[f'{z:.3f}' for z in Z_current]}")
        print(f"特征误差: {err_mag_pixel:.2f} pixels")

        if err_mag_pixel < 2.0:
            print("误差小于阈值，停止控制")
            return np.zeros(6)

        # 用期望深度计算 L（稳定）
        Z = self.desired_z if self.desired_z is not None else Z_current
        L = self.calculate_interaction_matrix(points, Z)
        print(f"交互矩阵条件数: {np.linalg.cond(L):.2e}")

        # 相机速度 v = -λ * L^+ * e
        L_pinv = np.linalg.pinv(L)
        camera_velocity = -self.lambda_gain * L_pinv @ error_norm
        camera_velocity[2] = -camera_velocity[2]  # 符号修正

        # 雅可比（body Jacobian）
        jacobian = robot_controller.robot.jacobe(current_joint_angles)
        print(f"雅可比条件数: {np.linalg.cond(jacobian):.2e}")

        # DLS 求解关节速度
        damping = 0.05
        J_T = jacobian.T
        joint_velocities = J_T @ np.linalg.inv(jacobian @ J_T + damping**2 * np.eye(6)) @ camera_velocity

        # 限幅
        max_vel = 0.5
        joint_velocities = np.clip(joint_velocities.flatten(), -max_vel, max_vel)
        print(f"关节速度(限幅): {np.round(joint_velocities, 4)}")

        return joint_velocities

    def draw_feature_points(self, image):
        image_with_features = image.copy()
        if self.current_points is not None:
            for i in range(4):
                u = int(round(float(self.current_points[i][0])))
                v = int(round(float(self.current_points[i][1])))
                cv2.circle(image_with_features, (u, v), 8, (255, 0, 0), -1)
                cv2.putText(image_with_features, str(i+1), (u-20, v-20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        if self.desired_points is not None:
            for i in range(4):
                u = int(round(float(self.desired_points[i][0])))
                v = int(round(float(self.desired_points[i][1])))
                cv2.circle(image_with_features, (u, v), 8, (0, 0, 255), 2)
                cv2.putText(image_with_features, str(i+1), (u-20, v-20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        if self.current_points is not None and self.desired_points is not None:
            cv2.polylines(image_with_features, [np.int32(np.round(self.current_points))], True, (255, 0, 0), 2)
            cv2.polylines(image_with_features, [np.int32(np.round(self.desired_points))], True, (0, 0, 255), 2)

        return image_with_features