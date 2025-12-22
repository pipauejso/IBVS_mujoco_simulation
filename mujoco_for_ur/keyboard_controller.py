import numpy as np
import cv2
from scipy.spatial.transform import Rotation

class KeyboardController:
    def __init__(self, model, data, target_name="target", step_size=0.001):
        """
        初始化键盘控制器，用于控制指定mocap body的位置和姿态
        """
        self.model = model
        self.data = data
        self.target_name = target_name
        self.step_size = step_size
        
        # 获取mocap body的ID
        try:
            body_id = model.body(target_name).id
            self.mocap_id = model.body_mocapid[body_id]
            if self.mocap_id == -1:
                raise ValueError(f"Body '{target_name}' 不是 mocap body")
            print(f"键盘控制器: 已找到mocap body '{target_name}'，mocap_id为 {self.mocap_id}")
        except Exception as e:
            raise ValueError(f"键盘控制器错误: {e}")
        
        # 存储当前位置和姿态
        self.current_pos = np.zeros(3)
        self.current_euler = np.zeros(3)
        
        self.update_current_state()
        
        # 键位映射
        self.key_mappings = {
            81: np.array([-1, 0, 0]),   # 左箭头 - X减小
            83: np.array([1, 0, 0]),    # 右箭头 - X增大
            82: np.array([0, 1, 0]),    # 上箭头 - Y增大
            84: np.array([0, -1, 0]),   # 下箭头 - Y减小
            ord('1'): np.array([0, 0, 1]),   # Z增大
            ord('2'): np.array([0, 0, -1]),  # Z减小
            ord('3'): np.array([0, 0, 1]),   # 绕Z轴正方向旋转
            ord('4'): np.array([0, 0, -1]),  # 绕Z轴负方向旋转
            ord('5'): np.array([1, 0, 0]),   # 绕X轴正方向旋转
            ord('6'): np.array([-1, 0, 0]),  # 绕X轴负方向旋转
            ord('7'): np.array([0, 1, 0]),   # 绕Y轴正方向旋转
            ord('8'): np.array([0, -1, 0]),  # 绕Y轴负方向旋转
        }
        
        self.rotation_keys = {ord('3'), ord('4'), ord('5'), ord('6'), ord('7'), ord('8')}
        self.print_controls()
    
    def update_current_state(self):
        """更新当前mocap body的位置和姿态"""
        self.current_pos = self.data.mocap_pos[self.mocap_id].copy()
        
        # MuJoCo mocap_quat 顺序是 [w, x, y, z]
        mj_quat = self.data.mocap_quat[self.mocap_id].copy()
        # 转换为 Scipy 顺序 [x, y, z, w]
        scipy_quat = np.array([mj_quat[1], mj_quat[2], mj_quat[3], mj_quat[0]])
        rot = Rotation.from_quat(scipy_quat)
        self.current_euler = rot.as_euler('xyz', degrees=False)
    
    def process_key_input(self, key_pressed):
        """处理键盘输入并更新mocap body位置或姿态"""
        if key_pressed in self.key_mappings:
            direction = self.key_mappings[key_pressed]
            
            if key_pressed not in self.rotation_keys:
                # 位置控制
                new_pos = self.current_pos + direction * self.step_size
                self.data.mocap_pos[self.mocap_id] = new_pos
                self.current_pos = new_pos
            else:
                # 旋转控制
                rotation_step = 0.1  # 弧度
                new_euler = self.current_euler + direction * rotation_step
                # Scipy 返回 [x, y, z, w]
                scipy_quat = Rotation.from_euler('xyz', new_euler).as_quat()
                # 转换为 MuJoCo 顺序 [w, x, y, z]
                mj_quat = np.array([scipy_quat[3], scipy_quat[0], scipy_quat[1], scipy_quat[2]])
                self.data.mocap_quat[self.mocap_id] = mj_quat
                self.current_euler = new_euler
            
            return True
        return False
    
    def print_controls(self):
        """打印控制说明"""
        print("\n=== 键盘控制器操作说明 ===")
        print("位置控制:")
        print("  方向键: X/Y轴移动")
        print("  1/2: Z轴移动")
        print("旋转控制:")
        print("  3/4: 绕Z轴旋转")
        print("  5/6: 绕X轴旋转")
        print("  7/8: 绕Y轴旋转")
        print(f"位置步长: {self.step_size:.3f}m, 旋转步长: 0.1rad")
        print("========================\n")