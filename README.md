# 具身智能控制基础：UR5e 视觉伺服课程设计

本项目面向研一同学，通过 MuJoCo 中的 UR5e 机械臂和末端相机，完成“图像感知—误差计算—运动控制—环境反馈”的闭环。目标是：红色目标发生移动或旋转后，控制机械臂运动，使相机中的目标四角点回到参考图像的位置。

这是**学生填空版**。仿真、渲染、机器人运动学、深度计算、数据记录和停车保护已经提供；同学们需要完成视觉检测和 IBVS（图像基视觉伺服）控制算法。请先阅读接口，再编写代码。

## 一、学习目标与任务边界

完成课程设计后，应能够：

1. 从 RGB 图像中检测目标的四个角点，并保持角点在连续帧中的对应关系。
2. 从针孔相机模型推导图像交互矩阵，理解图像运动与相机运动的关系。
3. 结合机器人雅可比设计关节速度控制律，理解负反馈、阻尼和速度约束。
4. 用误差曲线、运动录像与对照实验分析闭环系统，而不只展示“机械臂动了”。

本实验采用基于模型的视觉反馈控制，不要求训练神经网络。深度由仿真中的已知目标表面计算，属于教学中提供的辅助信息；四角点必须来自实际渲染的 RGB 图像。三维目标位姿只用于框架计算深度与评价结果，不得用位姿真值或投影角点替代学生检测/控制算法。

## 二、代码结构与待完成内容

```text
mujoco_for_ur/
  main.py                    仿真入口、实验扰动、数据导出
  feature_detection.py       【任务 A】四角点检测，1 个 TODO
  ibvs_controller.py         【任务 B】交互矩阵与控制律，2 个 TODO
  visual_servo_module.py     连接学生模块、参考管理、深度和停车保护
  rbt_module.py              相机光学坐标系下的机器人雅可比
  mujoco_render_module.py    相机 RGB 图像渲染
  opencv_render_module.py    相机窗口显示
  keyboard_controller.py    键盘移动/旋转目标
  universal_robots_ur5e/     机器人与场景模型
tests/
  test_reproduction.py       按框架/检测/控制分组的验收测试
  test_framework.py          学生模块调用与停车保护检查
```

**必做代码集中在 `feature_detection.py` 和 `ibvs_controller.py` 两个文件，共三个函数。** 保持公开函数签名和返回值约定，可以在这两个文件内增加辅助函数。正式验收时保持场景、实验扰动、框架阈值和评价代码不变；参数对比实验应另外保存并说明配置。

`visual_servo_module.py` 是已提供的连接层，不需要在里面补写算法。旧版 `cv_servo.py` 和 `test_find_cfeature.py` 已改为入口说明，不参与课程运行；`test_rtb.py` 是历史运动学脚本，不属于本课程入口，无需安装其额外依赖。

当前三个 TODO 会抛出带任务名称的 `NotImplementedError`。这是尚未完成任务的提示，不能通过删除异常、硬编码角点或始终返回零速度来完成验收。

## 三、安装与第一次运行

以下命令使用 Windows PowerShell。需要 Anaconda 或 Miniconda，以及可用的 OpenGL 图形环境。取得教师发放的课程代码后，进入同时包含 `requirements.txt` 和 `mujoco_for_ur` 的项目根目录，执行：

```powershell
conda create -n ibvs-course python=3.10 -y
conda activate ibvs-course
python -m pip install -r requirements.txt
python mujoco_for_ur/main.py --preview
```

预览模式不调用学生检测和控制模块，会打开 MuJoCo 场景窗口和 `End-Effector Camera` 相机窗口。点击相机窗口，可移动目标、按 `s` 截图、按 `Esc` 退出；机械臂不执行视觉伺服，`v` 和 `t` 在预览模式下不生效。

不打开窗口的环境检查：

```powershell
python mujoco_for_ur/main.py --headless --preview --steps 10
```

离屏模式仍然需要 OpenGL。预览产物位于 `results/preview/`；其 `summary.json` 中 `evaluation` 为 `not_evaluated`，`passed` 为 `null`，**不代表算法验收通过**。

后续启动时，激活 `ibvs-course` 环境、进入项目根目录即可。主入口不依赖 `roboticstoolbox-python` 或 `spatialmath`，依赖版本见 [requirements.txt](requirements.txt)。

## 四、任务 A：视觉检测

在 [feature_detection.py](mujoco_for_ur/feature_detection.py) 中实现：

```python
detect_feature_points(image, previous_points=None)
```

| 输入/输出 | 接口约定 |
| --- | --- |
| `image` | `uint8` RGB 数组，形状 `(H, W, 3)`；默认分辨率 320×240 |
| `previous_points` | 上次成功检测的 `(4, 2)` 浮点角点数组，首次为 `None` |
| 成功返回 | `(points, True)`，`points` 为有限浮点数组，形状 `(4, 2)` |
| 失败返回 | `(None, False)` |

每行是一个像素坐标 `(u, v)`：`u` 向右增加，`v` 向下增加。首次按左上、右上、右下、左下排列；之后应保持物理角点的编号，不能每一帧重新按图像方位编号。

建议按以下顺序完成并调试：

1. 将 RGB 图像转换到合适的颜色空间，提取红色区域；注意红色在 HSV 色相两端都有分布。
2. 筛选目标轮廓，拒绝空图、噪点、不完整、贴边或被裁切的目标。
3. 提取实际透视四边形角点，必要时做亚像素精化。不要用外接矩形替代倾斜目标的四角点。
4. 处理角点排列和相邻帧对应，尤其观察目标旋转跨越约 45° 时是否跳号。

不要修改输入数组。`previous_points` 仅用于对应关系，检测丢失时不能将旧坐标当成本帧结果。无标记正方形存在 90° 对称歧义，基本任务假定运动连续；大角度瞬间跳变和丢失重现的限制应在报告中说明。

完成 A 后，可运行 `python mujoco_for_ur/main.py`：此时伺服默认关闭，可观察绿色检测点和黄色参考点。不按 `v` 即可单独调试检测；移动目标后，检测点应跟随图像变化。尚未完成 B 时，启用伺服可能触发任务 B 的未实现提示。

## 五、任务 B：IBVS 控制

在 [ibvs_controller.py](mujoco_for_ur/ibvs_controller.py) 中实现两个函数。

### B1：图像交互矩阵

```python
calculate_interaction_matrix(points, depths, intrinsics)
```

- `points`：当前四点像素坐标，形状 `(4, 2)`。
- `depths`：逐点光学 Z 深度，形状 `(4,)`，单位 m，与角点顺序一一对应。非正或非有限深度应抛出 `ValueError`。
- `intrinsics`：`(fx, fy, cx, cy)`，单位 px。
- 返回 `L`：形状 `(8, 6)`，描述**归一化图像坐标**的变化率。

归一化坐标为 `x=(u-cx)/fx`、`y=(v-cy)/fy`。矩阵的行依次对应 `[x1,y1,x2,y2,x3,y3,x4,y4]`，列依次对应相机速度 `[vx,vy,vz,wx,wy,wz]`。请从成像模型推导各项，而不是直接将像素坐标代入归一化坐标公式。

相机采用光学坐标：X 向右、Y 向下、Z 向前。MuJoCo 的相机坐标转换已由 `rbt_module.py` 完成，学生模块不需要再次翻转坐标轴。

### B2：关节速度控制律

```python
compute_joint_velocity(points, desired_points, depths, intrinsics,
                       robot_jacobian, lambda_gain=2.0, damping=1e-4)
```

`desired_points` 与当前点有相同形状和角点编号；`robot_jacobian` 为 `(6, n)`，映射关节速度到相机光学坐标系的线速度和角速度。UR5e 的 `n=6`，返回值必须为有限浮点数组，形状 `(6,)`，单位 rad/s。

请完成：

1. 以 `current - desired` 为误差方向，构造归一化图像误差。
2. 调用 B1，并结合机器人雅可比，建立 `sdot = L J qdot`。
3. 设计使误差衰减的负反馈，用阻尼最小二乘求解关节速度，并解释控制符号、增益和阻尼的作用。

框架会在丢失角点、未启用伺服或 RMS 小于 0.2 px 时输出零速度；还会对速度进行整体缩放，使最大关节速度不超过 0.5 rad/s。学生仍需保证控制器返回值的形状与数值有效。

本实验的控制周期为 0.02 s，物理步长为 0.002 s。控制函数返回的是**关节速度**，不是关节位置，也不是已经乘过控制周期的角度增量。

## 六、分阶段检查与闭环验收

所有命令在项目根目录运行。测试按组运行，避免尚未完成的任务干扰当前阶段。

```powershell
# 0. 框架检查：不依赖学生算法，模板状态应通过
python -m unittest discover -s tests -k Framework -v

# 1. 完成任务 A 后：检测、透视四边形、跨帧对应、裁切和空图
python -m unittest discover -s tests -k DetectionTests -v

# 2. 完成任务 B 后：矩阵有限差分、无效深度、零误差和下降方向
python -m unittest discover -s tests -k ControllerTests -v

# 3. 全部完成后：完整测试
python -m unittest discover -s tests -v
```

模板状态下，检测/控制验收会因 TODO 报错；这是预期状态，不能解释为算法已经通过。控制测试不依赖视觉检测，可以在任务 A 之前独立完成。测试是接口与局部性质检查，不能代替闭环实验。

全部完成后先运行带窗口的平移实验：

```powershell
python mujoco_for_ur/main.py --demo
```

程序先采集参考图像，再施加扰动，自动启动伺服；默认运行 600 个控制周期，即 12 s 仿真时间。随后完成五种场景：

| 场景 | 扰动内容 |
| --- | --- |
| `translation` | 沿初始相机光学 X/Y 方向平移 +25/-18 mm |
| `depth` | 沿初始相机光学 Z 方向远离 30 mm |
| `rotation` | 绕目标局部 Z 轴旋转 25° |
| `tilt` | 绕目标局部 X/Y 轴旋转 +12/-10° |
| `combined` | 上述平移、深度变化，加目标局部 XYZ 旋转 +8/-6/+18° |

单场景运行并保存录像：

```powershell
python mujoco_for_ur/main.py --headless --demo --scenario combined --record
```

Windows 下执行全部测试、五组实验与结果汇总：

```powershell
powershell -ExecutionPolicy Bypass -File .\reproduce.ps1 -Python (Get-Command python).Source -Record
```

该命令显式使用当前环境的 Python。任一测试或场景失败时脚本停止，修复后可以重新运行；去掉 `-Record` 可跳过录像。

每组基本验收条件为：最终像素 RMS <1 px、全程特征丢失帧数为 0、最终目标相对相机姿态误差 <3°。三个条件同时满足时，`summary.json` 的 `passed` 为 `true`，实验返回码为 0；未通过时返回码为 1，TODO 未完成时主入口返回码为 2。验收应使用默认 600 步与原始场景，额外参数研究单独报告。

像素 RMS 定义为 `sqrt(mean((current-desired)**2))`，统计四个点的 8 个像素坐标残差。图像量化可能造成像素误差为零，此时三维位姿误差仍可能非零。目标位姿指标只参与评价，不参与学生控制反馈。

## 七、交互操作与输出

持续交互运行：

```powershell
python mujoco_for_ur/main.py
```

先点击 `End-Effector Camera` 窗口使其获得焦点。

| 按键 | 功能 |
| --- | --- |
| `v` | 开关伺服，切换时清零速度指令 |
| `t` | 将当前有效四点设为新参考；检测失败时保留原参考 |
| `s` | 保存截图 |
| 左/右、上/下方向键 | 沿世界 X/Y 轴移动目标，每次 0.01 m |
| `1` / `2` | 沿世界 Z 轴正向/负向移动目标 |
| `3` / `4` | 绕世界 Z 轴正向/负向旋转目标，每次 0.1 rad |
| `5` / `6` | 绕世界 X 轴正向/负向旋转目标 |
| `7` / `8` | 绕世界 Y 轴正向/负向旋转目标 |
| `Esc` | 退出 |

按 `t` 会改变任务目标。测试恢复原参考图像的能力时，应保留原参考。预览模式仅支持场景观察、目标移动、截图和退出。

默认输出到 `results/<scenario>/`，交互模式输出到 `results/interactive/`。

| 文件 | 内容 |
| --- | --- |
| `reference.png`、`initial.png`、`final.png` | 参考、扰动后、最终相机画面 |
| `trajectory.csv` | 时间、像素误差、检测状态、关节角、实际速度与速度指令 |
| `convergence.png` | 像素误差和六轴速度指令曲线 |
| `summary.json` | 实验配置、最终误差、丢失帧数、姿态指标与是否通过 |
| `camera.mp4` | 使用 `--record` 保存的带角点标注录像 |

完整实验脚本还生成 `results/summary.csv`、`results/comparison.png` 和 `results/overview.png`。重复运行会覆盖同目录中的同名文件；参数对比时用 `--output results/自定义实验名` 分开保存。`--steps` 可调整调试时长，`--headless` 需配合 `--demo` 或 `--preview`。

## 八、提交内容与报告要求

提交补全后的两个学生模块、实验报告及结果文件。报告建议包含：

1. 系统闭环与接口说明；用自己的语言解释相机、机器人与环境如何形成反馈。
2. 检测流程、中间图像、角点编号方式，以及倾斜和检测丢失时的处理。
3. 交互矩阵推导、控制律推导、坐标与单位约定、阻尼作用。
4. 五种场景的指标表、误差/速度曲线和至少一段闭环录像。
5. 至少一组控制增益或阻尼的对比实验，分析收敛速度、抖动和限速，而不只比较最终误差。
6. 至少一个失败案例，例如遮挡、裁切或角点跳号，并解释原因和可改进方向。

建议评价维度：视觉检测 25%、交互矩阵与控制律 35%、实验与分析 25%、代码和报告质量 15%；最终评分规则以任课教师要求为准。扩展任务可研究图像噪声、深度偏差、目标重识别或更复杂目标，基本任务应先独立完成。

## 九、常见问题

- **找不到文件或模块**：确认当前目录同时包含 `requirements.txt` 和 `mujoco_for_ur`。用 `python -c "import sys; print(sys.executable)"` 检查当前环境，再安装依赖。
- **提示任务未完成**：根据异常中的 A/B1/B2 定位 TODO；若仅检查环境，使用 `--preview`。
- **提示参考图像检测失败**：先检查 RGB/BGR 顺序、红色阈值、轮廓筛选和返回值，不要通过硬编码参考点绕过检测。
- **控制发散**：检查误差方向、归一化、交互矩阵列顺序、雅可比坐标系和角点身份。
- **倾斜场景误差异常**：检查是否使用外接矩形，以及每个深度是否对应同一个角点。
- **目标移出视野后停止**：这是框架的停车行为；恢复目标可见后应重新检测，不能发送旧的速度指令。
- **无法激活 conda**：可使用 Anaconda Prompt，或执行 `conda init powershell` 后重新打开终端。

[REPRODUCTION.md](REPRODUCTION.md) 和 [COMMAND_VALIDATION.md](COMMAND_VALIDATION.md) 保留教师参考实现改为填空版之前的历史记录；其中的通过结果不代表当前学生作业已经通过验收。

## 许可说明

UR5e 模型的许可见 [模型 LICENSE](mujoco_for_ur/universal_robots_ur5e/LICENSE)。上游 README 将项目代码标注为 MIT，但当前仓库根目录未附对应的独立 LICENSE 文件。
