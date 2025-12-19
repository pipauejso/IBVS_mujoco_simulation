import numpy as np
import roboticstoolbox as rtb

class RobotController:
    def __init__(self):
        """初始化UR5e机器人模型"""
        # 创建UR5e的DHRobot模型
        L1 = rtb.RevoluteMDH(d=0.163, a=0, alpha=0)
        L2 = rtb.RevoluteMDH(d=0, a=0, alpha=-np.pi/2)
        L3 = rtb.RevoluteMDH(d=0.0, a=0.425, alpha=0)
        L4 = rtb.RevoluteMDH(d=0.134, a=0.392, alpha=0)
        L5 = rtb.RevoluteMDH(d=0.1, a=0, alpha=-np.pi/2)
        L6 = rtb.RevoluteMDH(d=0.1, a=0, alpha=np.pi/2)
        
        # 构建UR5e机器人模型
        self.robot = rtb.DHRobot([L1, L2, L3, L4, L5, L6], name='UR5e')
        
        # 获取模型信息
        self.num_joints = self.robot.n
        self.joint_names = self.robot.name
        
    def get_forward_kinematics(self, joint_angles):
        """计算正向运动学"""
        return self.robot.fkine(joint_angles)
    
    def get_jacobian(self, joint_angles):
        """计算雅可比矩阵"""
        return self.robot.jacobe(joint_angles)
    
    def get_robot_info(self):
        """返回机器人信息"""
        return {
            "name": self.robot.name,
            "num_joints": self.num_joints,
            "joint_names": self.joint_names
        }
    
    def print_robot_info(self):
        """打印机器人信息"""
        print(self.robot)