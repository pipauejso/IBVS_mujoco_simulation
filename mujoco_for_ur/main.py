import mujoco
import mujoco.viewer
import time
import numpy as np
from scipy.spatial.transform import Rotation
from rbt_module import RobotController
from actulator_module import ActuatorController
from mujoco_render_module import MujocoRenderer
from opencv_render_module import OpenCVRenderer
from keyboard_controller import KeyboardController
from visual_servo_module import VisualServoController  # 导入视觉伺服模块

def main():
    # 加载Mujoco模型
    model = mujoco.MjModel.from_xml_path('universal_robots_ur5e/scene.xml')
    data = mujoco.MjData(model)
    
    # 初始化各模块
    robot_controller = RobotController()
    actuator_controller = ActuatorController(model, data)
    
    # 设置渲染分辨率
    resolution = (320, 240)
    
    # 初始化Mujoco渲染器
    mujoco_renderer = MujocoRenderer(model, data, "end_effector_camera", resolution)
    mujoco_renderer.setup_offscreen_rendering()
    
    # 初始化OpenCV渲染器
    opencv_renderer = OpenCVRenderer("End-Effector Camera", resolution)
    
    # 初始化键盘控制器，控制target site
    keyboard_controller = KeyboardController(model, data, "target", step_size=0.01)
    
    # 初始化视觉伺服控制器
    visual_servo_controller = VisualServoController(
        model, data, "end_effector_camera", "target", resolution, lambda_gain=0.1
    )
    
    # 打印机器人信息
    robot_controller.print_robot_info()
    
    # 获取site的id
    site_id = model.site('attachment_site').id
    
    # 主仿真循环
    with mujoco.viewer.launch_passive(model, data) as viewer:
        step = 0
        print_interval = 10000  # 大幅减少打印频率
        vs_enabled = False  # 视觉伺服是否启用
        
        print("主仿真窗口和末端相机窗口已启动")
        print("主窗口: 显示整个场景")
        print("小窗口: 显示末端执行器相机视角")
        print("按 'ESC' 键关闭程序")
        print("按 's' 键保存当前图像")
        print("按 'v' 键启用/禁用视觉伺服")
        print("按 't' 键设置当前位置为目标位置")
        
        # 初始为自由视角
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        
        while viewer.is_running() and mujoco_renderer.is_window_open():
            mujoco.mj_step(model, data)
            
            # 获取关节状态
            state = actuator_controller.get_joint_states(site_id)
            
            # 使用rtb获取当前末端位姿
            end_pose = robot_controller.get_forward_kinematics(state['joint_angles'])
            
            # 每隔100步打印一次状态信息
            if step % print_interval == 0:
                actuator_controller.print_joint_states(state, step)
                # print(f"末端位姿: {np.round(end_pose, 5)}")
            
            # ---------- 渲染流程 ----------
            # 1. 使用Mujoco渲染图像
            rgb_image = mujoco_renderer.render_image()
            
            # 2. 使用视觉伺服控制器处理图像和控制
            if vs_enabled:
                # 计算视觉伺服控制指令
                joint_velocities = visual_servo_controller.process_visual_servo(
                    robot_controller, state['joint_angles'], rgb_image  # 添加图像参数
                )
                
                if joint_velocities is not None:
                    # 将关节速度转换为执行器控制值
                    actuator_names = [
                        'shoulder_pan_vel_init',
                        'shoulder_lift_vel_init',
                        'elbow_vel_init',
                        'wrist_1_vel_init',
                        'wrist_2_vel_init',
                        'wrist_3_vel_init'
                    ]
                    
                    for i, name in enumerate(actuator_names):
                        data.actuator(name).ctrl = joint_velocities[i]
            
            # 3. 在图像上绘制特征点
            image_with_features = visual_servo_controller.draw_feature_points(rgb_image)
            
            # 4. 使用OpenCV显示图像
            key_pressed = opencv_renderer.show_image(image_with_features)
            
            # 5. 处理键盘输入
            if key_pressed == 27:  # ESC键
                print("程序退出...")
                break
            elif key_pressed == ord('s'):  # 's'键保存截图
                opencv_renderer.capture_screenshot(image_with_features, f"screenshot_{step}.png")
            elif key_pressed == ord('v'):  # 'v'键启用/禁用视觉伺服
                vs_enabled = not vs_enabled
                print(f"视觉伺服 {'已启用' if vs_enabled else '已禁用'}")
            elif key_pressed == ord('t'):  # 't'键设置当前位置为目标位置
                points, _ = visual_servo_controller.get_feature_points(rgb_image)  # 传递图像参数
                visual_servo_controller.set_desired_features(points)
                print(f"已设置新的目标特征点")
            else:
                # 让键盘控制器处理其他按键
                keyboard_controller.process_key_input(key_pressed)
            
            viewer.sync()
            time.sleep(0.01)
            step += 1
    
    # 清理资源
    opencv_renderer.cleanup()
    mujoco_renderer.cleanup()
    print("程序结束")

if __name__ == "__main__":
    main()