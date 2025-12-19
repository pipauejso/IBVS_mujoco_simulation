import mujoco
import mujoco.viewer
import time
import numpy as np
from scipy.spatial.transform import Rotation
import roboticstoolbox as rtb
import cv2
import glfw
import matplotlib.pyplot as plt
from spatialmath import SE3

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
    #ur5e.tool= SE3.Trans(0,0,0)*SE3.Ry(np.pi)
    # 打印机器人模型信息
    print(ur5e)
    q_init = np.array([0, 0, 0, -0, 0, 0])
    print(ur5e.fkine(q_init))

    # data.actuator('wrist_3_vel').ctrl = 0
    # data.actuator('shoulder_lift').ctrl = -1.6
    # data.actuator('wrist_1').ctrl = -np.pi/2
    # data.actuator('wrist_2').ctrl = -np.pi/2
    # data.actuator('elbow').ctrl = 1.5

    data.actuator('wrist_3_vel_init').ctrl = 0.0
    data.actuator('shoulder_lift_vel_init').ctrl = 0.0
    data.actuator('wrist_1_vel_init').ctrl = 0.0
    data.actuator('wrist_2_vel_init').ctrl = 0.0
    data.actuator('elbow_vel_init').ctrl = 0.0
    data.actuator('shoulder_pan_vel_init').ctrl = 0.0

    # ====== 初始化速度绘图 ======
    plt.ion()  # 开启交互模式
    fig, axs = plt.subplots(3, 2, figsize=(10, 8))
    axs = axs.flatten()
    joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
    target_vels = [0.01] * 6  # 你当前设定的目标速度
    time_buffer = []
    actual_vel_buffers = [[] for _ in range(6)]
    target_vel_buffers = [[] for _ in range(6)]
    max_plot_points = 500  # 最多显示500个点


    
    # 获取site的id
    site_id = model.site('attachment_site').id
    # 获取相机的id
    camera_id = model.camera('end_effector_camera').id

    #获取目标target ID
    target_site_id = model.site('target').id
    
    # ========== 初始化离屏渲染 ==========
    resolution = (320, 240) 
    
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
    
    # ========== 主仿真循环 ==========
    with mujoco.viewer.launch_passive(model, data) as viewer:
        step = 0
        
        print("主仿真窗口和末端相机窗口已启动")
        print("主窗口: 显示整个场景")
        print("小窗口: 显示末端执行器相机视角")
        print("按 'ESC' 键关闭程序")
        
        # 初始为自由视角
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        
        while viewer.is_running() and glfw.window_should_close(offscreen_window) == 0:
            mujoco.mj_step(model, data)

            # ========== 速度记录 ==========
            current_time = data.time
            actual_vels = data.qvel[:6].copy()

            time_buffer.append(current_time)
            for i in range(6):
                actual_vel_buffers[i].append(actual_vels[i])
                target_vel_buffers[i].append(target_vels[i])

            if len(time_buffer) > max_plot_points:
                time_buffer.pop(0)
                for i in range(6):
                    actual_vel_buffers[i].pop(0)
                    target_vel_buffers[i].pop(0)

            # ========== 更新绘图（每10步）==========
            if step % 10 == 0:
                for i in range(6):
                    axs[i].clear()
                    axs[i].plot(time_buffer, actual_vel_buffers[i], 'b-', label='Actual')
                    axs[i].plot(time_buffer, target_vel_buffers[i], 'r--', label='Target')
                    axs[i].set_title(joint_names[i])
                    axs[i].set_ylabel('Vel (rad/s)')
                    axs[i].legend()
                    axs[i].grid(True)
                axs[5].set_xlabel('Time (s)')
                plt.tight_layout()
                plt.pause(0.001)
            # ---------- 获取末端位姿 ----------
            end_pos = data.site_xpos[site_id].copy()
            end_rot_mat = data.site_xmat[site_id].reshape(3, 3).copy()

            #-------获取目标位姿-------
            target_pos = data.site_xpos[target_site_id].copy()
            target_rot_mat = data.site_xmat[target_site_id].reshape(3, 3).copy()

            #------获取相机的位姿------
            camera_pos = data.cam_xpos[camera_id].copy()
            camera_rot_mat = data.cam_xmat[camera_id].reshape(3, 3).copy()

            # 将旋转矩阵转换为四元数
            rot = Rotation.from_matrix(end_rot_mat)
            end_quat = rot.as_quat()

            # 获取关节角度和力矩
            joint_angles = data.qpos[:6].copy()
            actual_joint_torques = data.qfrc_actuator[:6].copy()
            
            # 使用rtb获取当前末端位姿
            end_pose = ur5e.fkine(joint_angles)

            ##计算雅可比
            J = ur5e.jacob0(joint_angles)
            
            # 每隔100步打印一次
            if step % 100 == 0:
                print(f"\n=== 步数 {step} ===")
                print(f"末端位置 (xyz): {np.round(end_pos, 4)}")
                print(f"关节角度 (rad): {np.round(joint_angles, 4)}")
                print(f"末端姿态 (四元数wxyz): {np.round(end_quat, 4)}")
                print(f"关节力矩 (前6个): {np.round(actual_joint_torques[:6], 4)}")
                print(f"末端位姿: {np.round(end_pose, 4)}")
                print(f"目标位置 (xyz): {np.round(target_pos, 4)}")
                print(f"目标姿态: {np.round(target_rot_mat, 4)}")
                print(f"相机位置 (xyz): {np.round(camera_pos, 4)}")
                print(f"相机姿态: {np.round(camera_rot_mat, 4)}")
                print(f"雅可比矩阵: {np.round(J, 4)}")
            
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
                
                # 显示图像
                cv2.imshow('End-Effector Camera', bgr)
                
                # 检查ESC键
                cv_key = cv2.waitKey(1)
                if cv_key == 27:  # ESC键
                    print("程序退出...")
                    break
            except Exception as e:
                print(f"渲染相机图像时出错: {e}")
            
            viewer.sync()
            time.sleep(0.01)
            step += 1
    
    # ---------- 清理资源 ----------
    cv2.destroyAllWindows()
    plt.ioff()
    plt.close(fig)
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
