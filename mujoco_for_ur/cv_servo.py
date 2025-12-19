import mujoco
import mujoco.viewer
import time
import numpy as np
from scipy.spatial.transform import Rotation
import roboticstoolbox as rtb
import cv2
import glfw
import os

# 设置OpenCV使用GTK后端，解决Qt平台插件问题

class IBVSController:
    def __init__(self, camera_matrix):
        self.camera_matrix = camera_matrix  # 相机内参矩阵
        self.Kp = 0.1  # IBVS比例增益
        self.target_features = None  # 目标特征点
    
    def detect_cube_corners(self, image):
        """检测图像中的立方体四个角点"""
        # 转为灰度图
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 应用高斯模糊减少噪声
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 使用Shi-Tomasi角点检测，优化参数以提高可靠性
        corners = cv2.goodFeaturesToTrack(blurred, maxCorners=8, qualityLevel=0.01, minDistance=20)
        
        if corners is None:
            return None
        
        # 修复NumPy弃用警告：将np.int0替换为np.intp
        corners = np.intp(corners)
        
        if len(corners) < 4:
            return None
        
        # 取前4个角点并排序
        corners = corners.reshape(-1, 2)[:4]
        corners = sorted(corners, key=lambda x: (x[0]**2 + x[1]**2))  # 按距离原点排序
        
        return corners
    
    def set_target_features(self, features):
        """设置目标特征点"""
        self.target_features = features
    
    def compute_error(self, current_features):
        """计算特征点误差"""
        if self.target_features is None or current_features is None:
            return None
        
        # 计算每个特征点的误差
        error = []
        for current, target in zip(current_features, self.target_features):
            error.append(current[0] - target[0])
            error.append(current[1] - target[1])
        
        return np.array(error)
    
    def compute_jacobian(self, features, depth=0.5):
        """计算图像雅可比矩阵"""
        if features is None:
            return None
        
        jacobian = []
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        for u, v in features:
            # 图像雅可比矩阵的一行
            row_x = [-fx/depth, 0, (u-cx)/depth, (u-cx)*(v-cy)/fx, -(fx**2 + (u-cx)**2)/fx, (v-cy)]
            row_y = [0, -fy/depth, (v-cy)/depth, (fy**2 + (v-cy)**2)/fy, -(u-cx)*(v-cy)/fy, -(u-cx)]
            jacobian.append(row_x)
            jacobian.append(row_y)
        
        return np.array(jacobian)
    
    def compute_control(self, current_features, depth=0.5):
        """计算控制指令"""
        # 计算误差
        error = self.compute_error(current_features)
        if error is None:
            return None
        
        # 计算雅可比矩阵
        jacobian = self.compute_jacobian(current_features, depth)
        if jacobian is None:
            return None
        
        # 计算控制输入（速度）
        # 使用伪逆求解
        try:
            jacobian_pinv = np.linalg.pinv(jacobian)
            velocity = -self.Kp * np.dot(jacobian_pinv, error)
            return velocity
        except:
            return None

def main():
    model = mujoco.MjModel.from_xml_path('universal_robots_ur5e/scene.xml')
    data = mujoco.MjData(model)
    L1 = rtb.RevoluteMDH(d=0.163, a=0, alpha=0)
    L2 = rtb.RevoluteMDH(d=0, a=0, alpha=-np.pi/2)
    L3 = rtb.RevoluteMDH(d=0.0, a=0.425, alpha=0)
    L4 = rtb.RevoluteMDH(d=0.134, a=0.392, alpha=0)
    L5 = rtb.RevoluteMDH(d=0.1, a=0, alpha=-np.pi/2)
    L6 = rtb.RevoluteMDH(d=0.1, a=0, alpha=np.pi/2)

    # 构建UR5e机器人模型
    ur5e = rtb.DHRobot([L1, L2, L3, L4, L5, L6], name='UR5e')

    # 打印机器人模型信息
    print(ur5e)
    q_init = np.array([0, 0, 0, 0, 0, 0])
    print(ur5e.fkine(q_init))

    # 初始关节配置
    data.actuator('shoulder_pan').ctrl = 0.5
    data.actuator('shoulder_lift').ctrl = -1.6
    data.actuator('elbow').ctrl = 1.5
    data.actuator('wrist_1').ctrl = -np.pi/2
    data.actuator('wrist_2').ctrl = -np.pi/2
    data.actuator('wrist_3').ctrl = 0
    
    # 获取site的id
    site_id = model.site('attachment_site').id
    # 获取相机的id
    camera_id = model.camera('end_effector_camera').id
    
    # ========== 初始化离屏渲染 ==========
    resolution = (640, 480)  # 提高分辨率以便更好地检测特征点
    
    # 初始化GLFW
    if not glfw.init():
        print("Failed to initialize GLFW")
        return
    
    # 创建离屏渲染窗口
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    offscreen_window = glfw.create_window(resolution[0], resolution[1], "Offscreen Camera", None, None)
    
    if not offscreen_window:
        print("Failed to create offscreen window")
        glfw.terminate()
        return
    
    glfw.make_context_current(offscreen_window)
    
    # 创建场景和上下文
    offscreen_scene = mujoco.MjvScene(model, maxgeom=1000)
    offscreen_context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
    
    # 创建相机
    offscreen_cam = mujoco.MjvCamera()
    offscreen_cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    if camera_id != -1:
        offscreen_cam.fixedcamid = camera_id
        print(f"末端相机ID: {camera_id}")
    else:
        print("警告: 未找到 'end_effector_camera' 相机")
    
    # 创建帧缓冲对象
    viewport = mujoco.MjrRect(0, 0, resolution[0], resolution[1])
    
    # 设置离屏缓冲区
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, offscreen_context)
    
    # 创建OpenCV窗口
    cv2.namedWindow('End-Effector Camera', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('End-Effector Camera', resolution[0], resolution[1])
    cv2.namedWindow('Target Features', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Target Features', resolution[0], resolution[1])
    
    # 初始化IBVS控制器
    # 相机内参矩阵（简化版，实际应根据相机参数设置）
    fx = 500
    fy = 500
    cx = resolution[0] / 2
    cy = resolution[1] / 2
    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0, 0, 1]])
    ibvs_controller = IBVSController(camera_matrix)
    
    # 状态变量
    target_features_acquired = False
    servoing_active = False
    reference_image = None
    
    # ========== 主仿真循环 ==========
    with mujoco.viewer.launch_passive(model, data) as viewer:
        step = 0
        
        print("主仿真窗口和末端相机窗口已启动")
        print("主窗口: 显示整个场景")
        print("End-Effector Camera: 显示末端执行器相机视角")
        print("按 'SPACE' 键拍摄参考图像（立方体上方0.1m处）")
        print("按 'S' 键开始IBVS视觉伺服控制")
        print("按 'ESC' 键关闭程序")
        
        # 初始为自由视角
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        
        while viewer.is_running() and glfw.window_should_close(offscreen_window) == 0:
            mujoco.mj_step(model, data)
            
            # ---------- 获取末端位姿 ----------
            end_pos = data.site_xpos[site_id].copy()
            end_rot_mat = data.site_xmat[site_id].reshape(3, 3).copy()

            # 将旋转矩阵转换为四元数
            rot = Rotation.from_matrix(end_rot_mat)
            end_quat = rot.as_quat()

            # 获取关节角度和力矩
            joint_angles = data.qpos[:6].copy()
            actual_joint_torques = data.qfrc_actuator[:6].copy()
            
            # 使用rtb获取当前末端位姿
            end_pose = ur5e.fkine(joint_angles)
            
            # ---------- 离屏渲染相机图像 ----------
            try:
                # 确保使用离屏渲染上下文
                glfw.make_context_current(offscreen_window)
                
                # 更新场景
                mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), 
                                       mujoco.MjvPerturb(), offscreen_cam, 
                                       mujoco.mjtCatBit.mjCAT_ALL, offscreen_scene)
                
                # 渲染
                mujoco.mjr_render(viewport, offscreen_scene, offscreen_context)
                
                # 读取像素
                rgb = np.zeros((resolution[1], resolution[0], 3), dtype=np.uint8)
                mujoco.mjr_readPixels(rgb, None, viewport, offscreen_context)
                
                # 转换颜色空间 (OpenCV使用BGR格式)
                bgr = cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR)
                
                # 检测立方体角点
                current_features = ibvs_controller.detect_cube_corners(bgr)
                
                # 显示特征点
                display_img = bgr.copy()
                if current_features is not None:
                    for i, corner in enumerate(current_features):
                        x, y = corner.ravel()
                        cv2.circle(display_img, (x, y), 5, (0, 255, 0), -1)
                        cv2.putText(display_img, str(i+1), (x+10, y-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # 显示图像
                cv2.imshow('End-Effector Camera', display_img)
                
                # 显示参考图像
                if reference_image is not None:
                    cv2.imshow('Target Features', reference_image)
                
                # 键盘控制
                cv_key = cv2.waitKey(1)
                if cv_key == 27:  # ESC键
                    print("程序退出...")
                    break
                elif cv_key == ord(' '):  # 空格键：拍摄参考图像
                    if current_features is not None and len(current_features) == 4:
                        # 在立方体上方0.1m处拍摄参考图像
                        # 保存当前特征点作为目标
                        ibvs_controller.set_target_features(current_features)
                        target_features_acquired = True
                        
                        # 保存参考图像
                        reference_img = display_img.copy()
                        for i, corner in enumerate(current_features):
                            x, y = corner.ravel()
                            cv2.circle(reference_img, (x, y), 5, (0, 0, 255), -1)
                            cv2.putText(reference_img, f"T{i+1}", (x+10, y-10), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        
                        reference_image = reference_img
                        print("已获取参考图像和目标特征点！")
                elif cv_key == ord('s') or cv_key == ord('S'):  # S键：开始伺服控制
                    if target_features_acquired:
                        servoing_active = True
                        print("IBVS视觉伺服控制已启动！")
                
                # IBVS视觉伺服控制
                if servoing_active and current_features is not None:
                    # 计算控制指令
                    velocity = ibvs_controller.compute_control(current_features)
                    
                    if velocity is not None:
                        # 应用控制指令（关节速度控制）
                        data.actuator('shoulder_pan_vel').ctrl = velocity[0]
                        data.actuator('shoulder_lift_vel').ctrl = velocity[1]
                        data.actuator('elbow_vel').ctrl = velocity[2]
                        data.actuator('wrist_1_vel').ctrl = velocity[3]
                        data.actuator('wrist_2_vel').ctrl = velocity[4]
                        data.actuator('wrist_3_vel').ctrl = velocity[5]
                        
                        # 打印控制信息
                        if step % 50 == 0:
                            error = ibvs_controller.compute_error(current_features)
                            if error is not None:
                                print(f"\n=== IBVS控制信息 ===")
                                print(f"特征点误差: {np.round(error, 4)}")
                                print(f"控制速度: {np.round(velocity, 4)}")
                
            except Exception as e:
                print(f"渲染相机图像时出错: {e}")
                import traceback
                traceback.print_exc()
            
            viewer.sync()
            time.sleep(0.01)
            step += 1
    
    # ---------- 清理资源 ----------
    cv2.destroyAllWindows()
    
    # 清理离屏渲染资源
    try:
        mujoco.mjr_freeContext(offscreen_context)
        glfw.destroy_window(offscreen_window)
        glfw.terminate()
    except:
        pass

    print("程序结束")

if __name__ == "__main__":
    main()