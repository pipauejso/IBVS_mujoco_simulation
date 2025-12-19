import numpy as np
import cv2  # 导入cv2模块
from scipy.spatial.transform import Rotation

class KeyboardController:
    def __init__(self, model, data, site_name="target", step_size=0.001):
        """
        初始化键盘控制器，用于控制指定site的位置和姿态
        
        参数:
        - model: Mujoco模型对象
        - data: Mujoco数据对象
        - site_name: 要控制的site名称
        - step_size: 每次按键移动的距离
        """
        self.model = model
        self.data = data
        self.site_name = site_name
        self.step_size = step_size
        
        # 获取site的ID
        try:
            self.site_id = model.site(site_name).id
            print(f"键盘控制器: 已找到site '{site_name}'，ID为 {self.site_id}")
        except:
            raise ValueError(f"键盘控制器错误: 未找到site '{site_name}'")
        
        # 存储当前site位置和姿态
        self.current_pos = np.zeros(3)
        self.current_euler = np.zeros(3)
        
        self.update_current_state()
        
        # 键位映射 - 使用上下左右箭头键和数字1-8
        # 使用实际的键值代替cv2常量，确保兼容性
        self.key_mappings = {
            # X轴方向控制
            81: np.array([-1, 0, 0]),  # 左箭头 - X减小
            83: np.array([1, 0, 0]),   # 右箭头 - X增大
            
            # Y轴方向控制
            82: np.array([0, 1, 0]),      # 上箭头 - Y增大
            84: np.array([0, -1, 0]),    # 下箭头 - Y减小
            
            # Z轴方向控制
            ord('1'): np.array([0, 0, 1]),        # 数字1 - Z增大（上移）
            ord('2'): np.array([0, 0, -1]),       # 数字2 - Z减小（下移）
            # 欧拉角控制
            ord('3'): np.array([0, 0, 1]),      # 数字3 - 绕Z轴正方向旋转
            ord('4'): np.array([0, 0, -1]),     # 数字4 - 绕Z轴负方向旋转
            ord('5'): np.array([1, 0, 0]),      # 数字5 - 绕X轴正方向旋转
            ord('6'): np.array([-1, 0, 0]),     # 数字6 - 绕X轴负方向旋转
            ord('7'): np.array([0, 1, 0]),      # 数字7 - 绕Y轴正方向旋转
            ord('8'): np.array([0, -1, 0]),     # 数字8 - 绕Y轴负方向旋转


        }
        
        # 旋转控制键集合
        self.rotation_keys = {ord('3'), ord('4'), ord('5'), ord('6'), ord('7'), ord('8')}
        
        # 打印控制说明
        self.print_controls()
    
    def update_current_state(self):
        """更新当前site的位置和姿态"""
        self.current_pos = self.model.site_pos[self.site_id].copy()
        
        # 获取四元数 (MuJoCo 的四元数顺序是 [x, y, z, w])
        quat = self.model.site_quat[self.site_id].copy()
        
        # 使用 scipy 将四元数转换为欧拉角
        rot = Rotation.from_quat(quat)
        
        # 转换为欧拉角，指定旋转顺序
        self.current_euler = rot.as_euler('xyz', degrees=False)  # 弧度制
    
    def process_key_input(self, key_pressed):
        """
        处理键盘输入并更新site位置或姿态
        
        参数:
        - key_pressed: 按下的键盘按键代码
        
        返回:
        - bool: 如果按键被处理，返回True；否则返回False
        """
        if key_pressed in self.key_mappings:
            direction = self.key_mappings[key_pressed]
            
            # 处理位置控制
            if key_pressed not in self.rotation_keys:
                new_pos = self.current_pos + direction * self.step_size
                self.model.site_pos[self.site_id] = new_pos
                self.current_pos = new_pos
            
            # 处理旋转控制
            else:
                # 旋转步长，单独设置避免受位置步长影响
                rotation_step = 1  # 弧度
                new_euler = self.current_euler + direction * rotation_step
                new_quat = Rotation.from_euler('xyz', new_euler).as_quat()
                self.model.site_quat[self.site_id] = new_quat
                self.current_euler = new_euler
            
            return True
        
        return False
    
    def print_controls(self):
        """打印控制说明"""
        print("\n=== 键盘控制器操作说明 ===")
        print("使用以下按键控制target site的位置:")
        print("  左箭头: X轴负方向移动")
        print("  右箭头: X轴正方向移动")
        print("  上箭头: Y轴正方向移动")
        print("  下箭头: Y轴负方向移动")
        print("  数字1: Z轴正方向移动 (上移)")
        print("  数字2: Z轴负方向移动 (下移)")
        print("\n使用以下按键控制target site的旋转:")
        print("  数字3: 绕Z轴正方向旋转")
        print("  数字4: 绕Z轴负方向旋转")
        print("  数字5: 绕X轴正方向旋转")
        print("  数字6: 绕X轴负方向旋转")
        print("  数字7: 绕Y轴正方向旋转")
        print("  数字8: 绕Y轴负方向旋转")
        print(f"每次位置移动距离: {self.step_size:.3f} 米")
        print("每次旋转角度: 0.1 弧度")
        print("========================\n")