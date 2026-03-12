# README.md

# MuJoCo IBVS for UR5e Robot

这个项目使用 **MuJoCo** 仿真 UR5e 机器人臂，并实现 **图像基视觉伺服（Image-Based Visual Servoing, IBVS）**，让末端相机跟踪红色矩形目标。项目包括键盘控制 target site、视觉伺服控制、OpenCV 图像显示和 MuJoCo 渲染。

## 特性
- MuJoCo 仿真 UR5e 机器人臂（带末端相机）
- 实时检测红色矩形目标的 4 个角点
- IBVS 控制：末端相机自动跟踪目标（像素误差 → 0）
- 键盘交互：移动/旋转 target site、设置期望位置、启用/禁用伺服
- 双窗口：MuJoCo 主场景 + OpenCV 末端相机视图

## 依赖
- Python 3.10.x（推荐 3.10.12）
- MuJoCo（pip install mujoco）
- roboticstoolbox-python（用于 UR5e 运动学）
- OpenCV（pip install opencv-python）
- SciPy、NumPy、Matplotlib 等

## 安装
1. **克隆仓库**
   ```bash
   git clone https://github.com/pipauejso/IBVS_mujoco_simulation.git
   ```

2. **创建虚拟环境**
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # Linux/Mac
   # 或 Windows: venv\Scripts\activate
   ```

3. **安装依赖** 
   ```bash
   pip install --upgrade pip
   ```
   查看env_config.txt安装 

   （如果没有 requirements.txt，先运行 `pip freeze > requirements.txt` 生成）

## 运行
```bash
python main.py
```

### 操作说明 操作需要在OpenCV相机视图窗口进行操作
| 按键 | 功能 |
|------|------|
| **ESC** | 退出程序 |
| **s** | 保存当前末端相机图像 |
| **v** | 启用/禁用视觉伺服 |
| **t** | 设置当前特征点为期望位置（自动保存深度） |
| **数字 3/4** | 绕 Z 轴旋转 target |
| **数字 5/6** | 绕 X 轴旋转 target |
| **数字 7/8** | 绕 Y 轴旋转 target |
| **方向键** | 平移 target（步长 0.01m） |

- 启用伺服后：末端相机自动跟踪红色矩形目标。
- 特征点标签：1（左上）、2（右上）、3（右下）、4（左下）。



## 注意事项
- 需要 MuJoCo 许可证（免费学术版）。
- 确保系统有 OpenGL 支持（Ubuntu: `sudo apt install libgl1-mesa-glx`）。
- 如果渲染失败，检查 `mujoco.viewer` 是否正常。
- 跨平台：推荐重建 venv + 安装 requirements.txt，不要直接用他人 venv。

## 贡献
欢迎 PR！如果有 bug 或想加功能（如 PBVS、目标跟踪滤波），提 issue 或 fork。

## 许可证
MIT License

目前该仿真存在一个问题，IBVS收敛后，相机会绕光轴旋转45度，目前还未解决

Happy simulating! 🚀


