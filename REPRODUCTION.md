# UR5e IBVS 本地复现记录

本版本在 https://github.com/pipauejso/IBVS_mujoco_simulation 的基线提交
`f5ee24eda4978631f009b03ef64a3f48af26eb08` 上完成修复与验证。修复已纳入当前代码版本。
首次安装、克隆与启动步骤见 [README.md](README.md)，以下命令均在克隆后的项目根目录执行。

## 环境与启动

实测环境为 Windows、Python 3.10.19、
MuJoCo 3.4.0、NumPy 2.2.6、SciPy 1.15.3、OpenCV 4.13.0.92、Matplotlib 3.10.8。
主实验复用了现有环境，未升级或降级其软件包；`pip check` 通过。另新建了 `ibvs-repro` 环境验证安装步骤。
复现所需的直接依赖已记录在 `requirements.txt`。新环境可按以下方式安装：

```powershell
conda create -n ibvs-repro python=3.10 -y
conda activate ibvs-repro
python -m pip install -r requirements.txt
```

本机现有 `mujoco` 环境可直接使用：

```powershell
conda activate mujoco

# 自动演示：两个窗口，初始保存参考图像，施加平移扰动，自动启用伺服
python -B mujoco_for_ur/main.py --demo

# 持续交互，初始伺服关闭，参考图像已自动保存
python -B mujoco_for_ur/main.py

# 五组自动实验、回归检查，并保存录像
powershell -ExecutionPolicy Bypass -File .\reproduce.ps1 -Record
```

也可以双击 `run_demo.bat` 或 `run_interactive.bat`。启动脚本需要 `conda` 在 PATH 中，通过环境列表定位解释器后直接调用。
本机 Conda 4.10.3 在已激活目标环境时重复使用 `conda run` 会误调用 base Python，已复现并在启动入口中绕过；见 [命令验证记录](COMMAND_VALIDATION.md)。
Python 入口支持从任意工作目录启动。`--headless` 使用真实离屏 OpenGL 渲染，仍需可用的图形驱动。
`--demo` 默认执行 600 个控制周期，即 12 秒仿真；`--steps` 可修改周期数。
单独运行组合扰动并录像：

```powershell
python -B mujoco_for_ur/main.py --headless --demo --scenario combined --record
```

## 交互方式

所有按键请在 `End-Effector Camera` 窗口有焦点时使用。

| 按键 | 操作 |
| --- | --- |
| `v` | 开关伺服，切换时清零速度指令 |
| `t` | 将当前检测到的四个角点保存为新参考；未检测到目标时保留旧参考 |
| `s` | 截图，保存到当前输出目录 |
| 方向键 | 沿世界 X/Y 轴平移目标，每次 0.01 m |
| `1` / `2` | 沿世界 Z 轴正向/负向平移 |
| `3` / `4` | 绕世界 Z 轴正向/负向旋转，每次 0.1 rad |
| `5` / `6` | 绕世界 X 轴旋转 |
| `7` / `8` | 绕世界 Y 轴旋转 |
| `Esc` | 退出 |

绿色为当前角点，黄色为参考角点。建议先按 `v` 启用，再小步移动或旋转目标观察跟踪。
相机初始与目标距离较近，大幅移动可能使目标离开视野。

## 遇到的问题与修复

1. 原程序在本地首次运行直接报 `ModuleNotFoundError: No module named 'roboticstoolbox'`。
   主路径改用 MuJoCo 原生运动学，省去额外依赖，也避免手写 MDH 参数与仿真模型不一致。
   上游的 `cv_servo.py`、`test_rtb.py`、`test_find_cfeature.py` 是保留的历史实验脚本，未纳入新启动入口。
2. 原模型路径依赖当前工作目录。现在根据 `main.py` 的绝对位置定位 XML 和资源。
3. 原控制器把末端 body 雅可比直接当成相机雅可比，并只反转一个平移分量。
   现在用 `mj_jac` 在实际相机光心计算雅可比，包含相机相对腕部的位置偏移。
   将线速度、角速度统一从世界坐标旋转到光学坐标（右、下、前）。
4. 统一误差和控制符号：`e = current - desired`，`qdot = -gain * (L J)^+ e`。
   使用当前逐点深度、阻尼最小二乘和整体速度缩放，关节速度指令不超过 0.5 rad/s。
5. 原检测始终使用最小外接矩形，破坏倾斜目标的透视角点；膨胀还会改变目标尺寸。
   现在直接提取轮廓四边形并做亚像素细化，使用上一帧保持角点对应，避免 45° 附近重新排序跳变。
6. 原角点深度按固定几何顺序生成，与图像排序不对应。
   现在逐像素射线与可见目标表面求交，保证每个角点与自身深度配对。
7. 原程序关掉伺服或丢失特征时可能保留上一条非零速度。
   现在未启用、检测丢失、目标被裁切时都写入零速度指令；无效参考不会覆盖已有参考。
8. 修正 Windows 方向键：使用 `waitKeyEx` 并支持完整键码。
   旋转改用旋转增量复合，消除初始目标姿态的欧拉角万向节锁警告。
9. 渲染资源由 `mujoco.Renderer` 管理，异常和正常退出均关闭资源；不再全局终止 GLFW。
   控制周期固定为 0.02 s，物理步长仍为原模型的 0.002 s。
10. 自动实验发现 RMS 停止阈值 0.5 px 会留下约 3.35° 的倾斜姿态误差，
    收紧为 0.2 px 后该组达到约 1.93°。未改动原机器人、目标或执行器 XML。

相机轴定义参考 [MuJoCo 官方可视化文档](https://mujoco.readthedocs.io/en/latest/programming/visualization.html)。

## 实测结果

每组使用原场景初始化，先以零速度运行 100 个物理步稳定状态，再从 RGB 图像保存期望角点，
然后对目标施加以下扰动并执行 600 个闭环控制周期。控制器反馈使用 OpenCV 检测，未使用投影真值替代检测。
`translation` 和 `depth` 的方向以初始相机光学坐标为准；姿态增量在目标初始局部坐标下施加。

| 实验 | 目标扰动 | 初始 RMS / px | 最终 RMS / px | 最终相对姿态误差 / ° | 光轴旋转残差 / ° |
| --- | --- | ---: | ---: | ---: | ---: |
| translation | 横向 +25 mm、纵向 -18 mm | 23.8792 | 0.1528 | 1.1268 | 0.0874 |
| depth | 沿光轴远离 30 mm | 7.5592 | 0.1196 | 0.6132 | -0.1212 |
| rotation | 绕目标法向旋转 25° | 23.7316 | 0.0000 | 0.9400 | -0.1871 |
| tilt | 局部 X/Y 方向倾斜 +12°/-10° | 3.0878 | 0.1249 | 1.9252 | -0.0052 |
| combined | 上述平移、深度，局部 XYZ 旋转 +8°/-6°/+18° | 27.0614 | 0.1689 | 0.6027 | -0.1434 |

五组均通过：最终 RMS < 1 px、全程未丢失特征、相对姿态误差 < 3°。
10 项回归检查通过，包括完整图像雅可比的有限差分核对、透视四边形、跨 45° 角点身份、
倾斜表面的逐点深度、特征丢失、裁切、无效参考、Windows 方向键和初始奇异姿态下的旋转。
另实测运行了 MuJoCo 与 OpenCV 双窗口，并验证自动结束和资源释放。

误差定义为四点共 8 个像素坐标残差的 RMS，即 `sqrt(mean((current - desired)^2))`。
姿态指标比较目标相对相机的最终姿态与参考姿态；光轴残差是相对旋转向量的光学 Z 分量。
姿态指标仅用于结果核对，不参与控制。像素 0 误差受光栅量化影响，并不表示三维位姿完全相同。

每组产物位于 `results/<scenario>/`：

- `reference.png`、`initial.png`、`final.png`：参考、扰动后和闭环最终画面。
- `camera.mp4`：使用 `--record` 生成的实际渲染录像。
- `convergence.png`：像素误差与六轴速度指令。
- `trajectory.csv`：时间、误差、检测状态、关节角、实际关节速度和指令。
- `summary.json`：软件版本、误差、姿态指标、特征丢失帧数和验收结果。

本次生成的汇总见 `results/comparison.png` 和 `results/summary.csv`。

## 适用范围

深度使用仿真中已知目标表面，属于有深度真值的 IBVS 复现；实物需要额外的深度估计或传感器。
原目标是有厚度的半透明红色方块，本方法以主要可见平面近似深度。
大幅倾斜、严重遮挡和裁切时可能检测不到完整四边形，此时停止发送运动速度。
保持角点身份依赖相邻帧运动连续；无标记正方形存在 90° 对称歧义，目标瞬间大角度跳变或丢失后重现不能保证物理角点身份。
上述实测未出现上游提到的额外 45° 光轴偏转，但这些结果不构成任意初态下的全局收敛保证。
