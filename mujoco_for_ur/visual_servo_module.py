import numpy as np
import cv2

class VisualServoController:
    def __init__(self, model, data, camera_name, target_site_name, resolution, lambda_gain=0.001):
        """
        初始化视觉伺服控制器
        
        参数:
        - model: Mujoco模型对象
        - data: Mujoco数据对象
        - camera_name: 相机名称
        - target_site_name: 目标特征点的site名称
        - resolution: 相机分辨率 (宽度, 高度)
        - lambda_gain: 控制增益
        """
        self.model = model
        self.data = data
        self.resolution = resolution
        self.lambda_gain = lambda_gain
        
        # 获取相机ID
        self.camera_id = model.camera(camera_name).id
        
        # 获取目标site的ID和位置
        self.target_site_id = model.site(target_site_name).id
        
        # 计算相机内参
        self.fx, self.fy, self.cx, self.cy = self._calculate_camera_intrinsics()
        
        # 获取target的尺寸
        self.target_size = model.site_size[self.target_site_id].copy()
        
        # 目标特征点在图像中的期望位置（初始化为当前位置）
        self.desired_points = None

        # 期望深度
        self.desired_z = None
        
        # 当前特征点在图像中的位置
        self.current_points = None

        self._get_depth_for_points()
        
        print(f"视觉伺服控制器: 相机内参 - fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")
        print(f"视觉伺服控制器: 目标尺寸 - {self.target_size}")
    def _extract_features_from_image(self, image):
        """
        从图像中提取红色矩形特征点。
        根据成功脚本优化颜色阈值和形态学操作，确保准确检测红色目标的4个角点。
        """
        # 1. 转换到HSV颜色空间   小心：OpenCV默认是BGR，而Mujoco是RGB
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        
        # 2. 定义红色的HSV范围（使用成功脚本的阈值）
        # 范围1: 0-10 (深红/橙红)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        
        # 范围2: 170-180 (紫红)
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        # 创建掩膜
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2  # 合并两个掩膜

        # 3. 形态学操作
        kernel = np.ones((5, 5), np.uint8)
        # 闭运算：填充内部孔洞（解决红色方块内的小蓝点问题）
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # 膨胀操作：确保边缘平滑
        mask = cv2.dilate(mask, kernel, iterations=1)

        # 4. 寻找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print("未检测到红色物体")
            return None, False

        # 找到面积最大的轮廓（假设红色方块是主要的红色物体）
        largest_contour = max(contours, key=cv2.contourArea)

        # 5. 多边形拟合（获取4个角点）
        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)

        # 处理拟合结果
        corners = []
        
        if len(approx) == 4:
            print("检测到精确的四边形。")
            corners = approx.reshape(-1, 2)  # 转换为 (4, 2) 的数组
        else:
            print(f"检测到轮廓（拟合点数: {len(approx)}），使用最小外接矩形修正。")
            rect = cv2.minAreaRect(largest_contour)
            box = cv2.boxPoints(rect)
            corners = np.int0(box).astype(np.float32)  # 转换为int后再转换为float32

        # 6. 对四个角点进行排序：固定顺序 [左上，右上，右下，左下]
        # 计算中心点
        center = np.mean(corners, axis=0)
        # 根据点与中心的相对位置排序
        def sort_key(point):
            diff = point - center
            # 使用arctan2计算角度，从左上角开始逆时针排序
            return np.arctan2(diff[1], diff[0])
        
        sorted_points = sorted(corners, key=sort_key)
        # 调整起点，确保是左上角开始
        sorted_points = np.roll(sorted_points, -np.argmin([p[0]+p[1] for p in sorted_points]), axis=0)

        return np.array(sorted_points, dtype=np.float32), True

    def _calculate_camera_intrinsics(self):
        """
        根据相机fovy和图像分辨率计算相机内参
        
        返回:
        - fx, fy: 焦距（像素单位）
        - cx, cy: 主点坐标（像素单位）
        """
        width, height = self.resolution
        
        # 从MuJoCo模型获取相机fovy（弧度）
        fovy = self.model.cam_fovy[self.camera_id]
        
        # 计算焦距
        fx = fy = height / (2 * np.tan(fovy / 2))
        
        # 主点坐标（图像中心）
        cx = width / 2
        cy = height / 2
        
        return fx, fy, cx, cy
    def _get_depth_for_points(self):
        """
        为图像中提取的角点提供固定深度值。
        """
        P_w_tar = self.data.site_xpos[self.target_site_id].copy()
        P_w_tar1 = P_w_tar.copy() +[self.target_size[0]/2, self.target_size[1]/2, 0] #左上
        P_w_tar2 = P_w_tar.copy() +[self.target_size[0]/2, -self.target_size[1]/2, 0]#右上
        P_w_tar3 = P_w_tar.copy() +[-self.target_size[0]/2, -self.target_size[1]/2, 0]#右下
        P_w_tar4 = P_w_tar.copy() +[-self.target_size[0]/2, self.target_size[1]/2, 0]#左下

        R_w_tar = self.data.site_xmat[self.target_site_id].reshape((3, 3))

        R_w_cam = self.data.cam_xmat[self.camera_id].reshape((3, 3))
        P_w_cam = self.data.cam_xpos[self.camera_id].copy()

        P_w_cam_tar1 = P_w_tar1- P_w_cam
        P_w_cam_tar2 = P_w_tar2- P_w_cam
        P_w_cam_tar3 = P_w_tar3- P_w_cam
        P_w_cam_tar4 = P_w_tar4- P_w_cam

        ##R_cam_w 等于R_w_cam的转置
        R_cam_w =   R_w_cam.T

        P_cam_tar1 = R_cam_w @ P_w_cam_tar1
        P_cam_tar2 = R_cam_w @ P_w_cam_tar2
        P_cam_tar3 = R_cam_w @ P_w_cam_tar3
        P_cam_tar4 = R_cam_w @ P_w_cam_tar4

        self.desired_z = [-P_cam_tar1[2],-P_cam_tar2[2],-P_cam_tar3[2] ,-P_cam_tar4[2]]
        return [P_cam_tar1[2],P_cam_tar2[2],P_cam_tar3[2] ,P_cam_tar4[2]]  # 固定深度值

    def get_feature_points(self, image=None):
        """
        从图像中检测红色target的四个角点（优先图像处理，失败则回退模型计算）
        参数:
        - image: 输入图像（可选）
        
        返回:
        - points: 检测到的4个角点的像素坐标 (4x2数组)
        - Z: 固定深度值
        """
        # 优先使用图像处理方法
        if image is not None and isinstance(image, np.ndarray):
            points, success = self._extract_features_from_image(image)
            if success:
                # 图像处理成功，使用固定深度
                #Z = 0.2
                Z = self.desired_z
                self.current_points = points
                return points, Z
        # 图像处理失败，打印警告并回退
        print("视觉警告：图像处理未检测到目标，回退至模型计算。")
    
        # 回退到基于模型的方法
        return False


       
    def set_desired_features(self, points_star):
        """
        设置特征点的期望位置
        
        参数:
        - points_star: 4个角点的期望像素坐标 (4x2)
        """
        self.desired_points = points_star
        self._get_depth_for_points()
    
    def calculate_interaction_matrix(self, points, Z):
        """
        计算交互矩阵L(s)
        修改：为每个特征点使用独立的深度Z_i进行计算，提高精度。
        """
        # 如果Z是单个值，则扩展为每个点一个深度（使用平均值）
        if np.isscalar(Z):
            Z_array = np.full(4, Z)
        else:
            Z_array = Z
    
        L = np.zeros((8, 6))
    
        for i in range(4):
            u = points[i][0]
            v = points[i][1]
            Z_i = Z_array[i] if i < len(Z_array) else Z_array[0]  # 安全索引
        
            # 计算归一化坐标（更准确的公式）
            x = (u - self.cx) / self.fx
            y = (v - self.cy) / self.fy
        
            # 单个特征点的交互矩阵 (2x6) - 使用更标准的公式
            L_i = np.array([
                [-1/Z_i, 0,     x/Z_i,     x*y,   -(1 + x*x),   y],
                [0,     -1/Z_i, y/Z_i, 1 + y*y,       -x*y,   -x]
                ])
        
            # 注意：上述公式中的焦距已被归一化坐标吸收
            # 将单个特征点的交互矩阵添加到总交互矩阵中
            L[2*i:2*i+2, :] = L_i
    
        return L
    
    def compute_robot_velocity(self, interaction_matrix, error, robot_jacobian):
        """
        计算机器人关节速度
        
        参数:
        - interaction_matrix: 交互矩阵 (8x6)
        - error: 特征误差 (8x1)
        - robot_jacobian: 机器人雅可比矩阵 (6x6)
        
        返回:
        - joint_velocities: 关节速度指令 (6x1)
        """
        # 计算相机速度
        camera_velocity = -self.lambda_gain * np.linalg.pinv(interaction_matrix) @ error
        
        # 将相机速度转换为关节速度
        joint_velocities = np.linalg.pinv(robot_jacobian) @ camera_velocity
        
        return joint_velocities
    
    def process_visual_servo(self, robot_controller, current_joint_angles, image=None):
        """
        处理视觉伺服控制循环
        
        参数:
        - robot_controller: 机器人控制器对象
        - current_joint_angles: 当前关节角度
        - image: 输入图像（可选）
        
        返回:
        - joint_velocities: 关节速度指令 (6x1)，如果没有期望特征点则返回None
        """
        # 获取当前特征点
        points, Z = self.get_feature_points(image)
        
        # 如果没有设置期望特征点，则使用当前位置作为期望位置
        if self.desired_points is None:
            self.desired_points = points
            return None
        
        # 计算特征误差
        error = np.zeros((8, 1))
        for i in range(4):
            error[2*i] = self.desired_points[i][0] - points[i][0]
            error[2*i+1] = self.desired_points[i][1] - points[i][1]
        
        # 如果误差很小，停止控制
        if np.linalg.norm(error) < 2.0:
            return np.zeros(6)
        
        # 计算交互矩阵
        L = self.calculate_interaction_matrix(points, Z)
        
        # 获取机器人雅可比矩阵
        jacobian = robot_controller.robot.jacobe(current_joint_angles)
        
        # 计算关节速度
        joint_velocities = self.compute_robot_velocity(L, error, jacobian)
        
        return joint_velocities
    
    def draw_feature_points(self, image):
        """
        在图像上绘制特征点
        
        参数:
        - image: 输入图像
        
        返回:
        - image_with_features: 绘制了特征点的图像
        """
        image_with_features = image.copy()
        
        # 绘制当前特征点
        if self.current_points is not None:
            # 打印当前特征点坐标
            print("4 个角点的坐标 (x, y):")
            for i in range(4):
                u = int(self.current_points[i][0])
                v = int(self.current_points[i][1])
                print(f"点 {i+1}: ({u}, {v})")
                
                # 增大圆圈大小，使用蓝色实心圆
                cv2.circle(image_with_features, (u, v), 8, (255, 0, 0), -1)
                # 调整角点编号位置和样式
                cv2.putText(image_with_features, str(i+1), (u-20, v-20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        
        # 绘制期望特征点
        if self.desired_points is not None:
            for i in range(4):
                u = int(self.desired_points[i][0])
                v = int(self.desired_points[i][1])
                cv2.circle(image_with_features, (u, v), 8, (0, 0, 255), 2)
                # 为期望点也添加编号
                cv2.putText(image_with_features, str(i+1), (u-20, v-20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        
        # 绘制连线
        if self.current_points is not None and self.desired_points is not None:
            # 绘制当前特征点的连线
            cv2.polylines(image_with_features, [np.int32(self.current_points)], True, (255, 0, 0), 2)
            # 绘制期望特征点的连线
            cv2.polylines(image_with_features, [np.int32(self.desired_points)], True, (0, 0, 255), 2)
        
        return image_with_features