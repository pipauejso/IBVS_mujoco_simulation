import numpy as np

class ActuatorController:
    def __init__(self, model, data):
        """初始化执行器控制器"""
        self.model = model
        self.data = data
        
        # 获取执行器名称列表
        self.actuator_names = [self.model.actuator(i).name for i in range(self.model.nu)]
        
        # 存储关节设置速度
        self.joint_set_velocities = np.zeros(6)
    
    def set_joint_control(self, joint_name, control_value):
        """设置关节控制值"""
        if joint_name in self.actuator_names:
            self.data.actuator(joint_name).ctrl = control_value
        else:
            print(f"警告: 未找到执行器 '{joint_name}'")
    
    def get_joint_states(self, site_id=None):
        """获取关节状态信息"""
        joint_angles = self.data.qpos[:6].copy()
        actual_joint_torques = self.data.qfrc_actuator[:6].copy()
        joint_velocities = self.data.qvel[:6].copy()
        
        # 获取设置的关节速度
        self.joint_set_velocities = np.array([
            self.data.actuator('shoulder_pan_vel_init').ctrl,
            self.data.actuator('shoulder_lift_vel_init').ctrl,
            self.data.actuator('elbow_vel_init').ctrl,
            self.data.actuator('wrist_1_vel_init').ctrl,
            self.data.actuator('wrist_2_vel_init').ctrl,
            self.data.actuator('wrist_3_vel_init').ctrl
        ])
        
        state = {
            "joint_angles": joint_angles,
            "joint_torques": actual_joint_torques,
            "joint_velocities": joint_velocities,
            "joint_set_velocities": self.joint_set_velocities
        }
        
        # 如果提供了site_id，获取末端执行器位置和姿态
        if site_id is not None:
            end_pos = self.data.site_xpos[site_id].copy()
            end_rot_mat = self.data.site_xmat[site_id].reshape(3, 3).copy()
            state["end_pos"] = end_pos
            state["end_rot_mat"] = end_rot_mat
        
        return state
    
    def print_joint_states(self, state, step):
        """打印关节状态信息"""
        print(f"\n=== 步数 {step} ===")
        print(f"关节角度 (rad): {np.round(state['joint_angles'], 4)}")
        print(f"关节力矩 (前6个): {np.round(state['joint_torques'][:6], 4)}")
        print(f"关节速度: {np.round(state['joint_velocities'], 5)}")
        print(f"关节设置速度: {np.round(state['joint_set_velocities'], 5)}")
        
        if "end_pos" in state:
            print(f"末端位置 (xyz): {np.round(state['end_pos'], 4)}")
