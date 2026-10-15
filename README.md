# MuJoCo UR5e 图像基视觉伺服（IBVS）

本项目使用 MuJoCo 仿真 UR5e 六轴机械臂，通过末端相机检测红色目标的四个角点，实现图像基视觉伺服。支持键盘移动和旋转目标、双窗口可视化，以及自动实验和结果导出。

## 从零开始安装

以下步骤面向首次复现者，使用 Windows PowerShell。请先安装 Git 和 Anaconda 或 Miniconda，并确认终端能够执行 `git` 和 `conda`。

### 1. 新建工作目录并克隆项目

先在终端进入自己希望存放项目的位置，再执行：

```powershell
mkdir ibvs-workspace
cd ibvs-workspace
git clone https://github.com/pipauejso/IBVS_mujoco_simulation.git
```

`ibvs-workspace` 是示例工作目录名，可以自行修改。克隆后，项目位于其子目录 `IBVS_mujoco_simulation`。

仓库已包含修复后的程序、依赖清单、测试和文档，克隆后可直接继续以下安装步骤。

### 2. 新建并激活 conda 环境

保持在 `ibvs-workspace` 目录，执行：

```powershell
conda create -n ibvs-repro python=3.10 -y
conda activate ibvs-repro
```

`ibvs-repro` 是环境名，可以自行修改，后续激活时使用相同名称。

### 3. 进入项目目录并安装依赖

```powershell
cd IBVS_mujoco_simulation
python -m pip install -r requirements.txt
```

主程序直接使用 MuJoCo 模型计算运动学和相机雅可比，依赖已记录在 [requirements.txt](requirements.txt)，无需另装 `roboticstoolbox-python` 或 `spatialmath`。

### 4. 启动仿真

在项目根目录运行：

```powershell
python mujoco_for_ur/main.py
```

程序会打开 MuJoCo 场景和 `End-Effector Camera` 相机两个窗口。启动时自动保存初始参考图像，伺服初始关闭。点击相机窗口后按 `v` 启用伺服，再使用方向键移动目标观察跟踪，按 `Esc` 退出。

如需自动演示：

```powershell
python mujoco_for_ur/main.py --demo
```

自动演示会保存参考图像、施加平移扰动并启用伺服，默认执行 600 个控制周期，即 12 秒仿真时间，随后自动退出。双窗口和离屏渲染均需要可用的 OpenGL 图形环境。

## 后续启动

安装完成后，每次打开新终端，先进入之前创建的 `ibvs-workspace` 目录，然后按以下顺序执行：

```powershell
conda activate ibvs-repro
cd IBVS_mujoco_simulation
python mujoco_for_ur/main.py
```

如果已经在项目根目录，只需激活环境并运行 Python 命令。所有运行、测试命令均以项目根目录为当前目录。

## 键盘操作

请先点击 `End-Effector Camera` 窗口，使其获得键盘焦点。

| 按键 | 功能 |
| --- | --- |
| `v` | 启用或关闭视觉伺服，切换时清零速度指令 |
| `t` | 将当前检测到的四个角点设为参考；检测失败时保留原参考 |
| `s` | 保存当前相机截图 |
| 左 / 右方向键 | 沿世界 X 轴负向 / 正向平移目标，每次 0.01 m |
| 上 / 下方向键 | 沿世界 Y 轴正向 / 负向平移目标，每次 0.01 m |
| `1` / `2` | 沿世界 Z 轴正向 / 负向平移目标，每次 0.01 m |
| `3` / `4` | 绕世界 Z 轴正向 / 负向旋转目标，每次 0.1 rad |
| `5` / `6` | 绕世界 X 轴正向 / 负向旋转目标，每次 0.1 rad |
| `7` / `8` | 绕世界 Y 轴正向 / 负向旋转目标，每次 0.1 rad |
| `Esc` | 退出程序 |

绿色为检测角点，黄色为参考角点。首次检测按左上、右上、右下、左下编号，之后根据相邻帧保持角点身份；目标旋转后，编号不会重新按图像方位排列。

按 `t` 会改变期望图像。若希望相机跟随目标恢复原图像，请保留原参考。

## 自动实验与测试

激活环境并进入项目根目录后，可单独运行一组组合扰动实验并保存录像：

```powershell
python mujoco_for_ur/main.py --headless --demo --scenario combined --record
```

运行回归检查：

```powershell
python -m unittest discover -s tests -v
```

Windows 下也可以运行完整复现脚本，执行回归检查、五组实验并生成汇总：

```powershell
powershell -ExecutionPolicy Bypass -File .\reproduce.ps1 -Python (Get-Command python).Source -Record
```

`-Python` 将当前已激活环境的解释器传给脚本，支持自定义环境名。去掉 `-Record` 可跳过录像生成。

| 参数 | 作用 |
| --- | --- |
| `--demo` | 保存参考、扰动目标并自动启用伺服 |
| `--headless` | 仅使用离屏渲染，须配合 `--demo`；仍需要 OpenGL |
| `--scenario` | `translation`、`depth`、`rotation`、`tilt`、`combined`；默认 `translation` |
| `--steps` | 正整数控制周期数；演示默认 600，持续交互默认不限 |
| `--record` | 保存带角点标注的 `camera.mp4` |
| `--output` | 自定义输出目录；相对路径以启动时的工作目录为基准 |

## 验证环境与结果

已在 Windows 的 Python 3.10.19 环境中验证以下依赖组合，并使用新建的 Python 3.10.22 环境验证依赖安装和双窗口演示：

| 组件 | 版本 |
| --- | --- |
| MuJoCo | 3.4.0 |
| NumPy | 2.2.6 |
| SciPy | 1.15.3 |
| OpenCV | 4.13.0.92 |
| Matplotlib | 3.10.8 |

每组闭环实验执行 600 个控制周期，控制周期为 0.02 s，物理步长为 0.002 s。角点来自实际渲染图像的 OpenCV 检测。

| 场景 | 初始像素 RMS | 最终像素 RMS | 最终相对姿态误差 |
| --- | ---: | ---: | ---: |
| 平移 `translation` | 23.8792 px | 0.1528 px | 1.1268° |
| 深度变化 `depth` | 7.5592 px | 0.1196 px | 0.6132° |
| 绕目标法向旋转 `rotation` | 23.7316 px | 0.0000 px | 0.9400° |
| 目标倾斜 `tilt` | 3.0878 px | 0.1249 px | 1.9252° |
| 组合扰动 `combined` | 27.0614 px | 0.1689 px | 0.6027° |

五组均满足最终 RMS 小于 1 px、全程未丢失特征、目标相对相机的姿态误差小于 3°；10 项回归检查通过。

像素 RMS 定义为四点共 8 个坐标残差的均方根：`sqrt(mean((current - desired)^2))`。像素零误差受图像量化影响，并不表示三维位姿误差为零。详细修复和验证记录见 [REPRODUCTION.md](REPRODUCTION.md) 与 [COMMAND_VALIDATION.md](COMMAND_VALIDATION.md)。

## 输出文件

默认演示输出到 `results/<scenario>/`，持续交互输出到 `results/interactive/`：

| 文件 | 内容 |
| --- | --- |
| `reference.png` | 初始参考图像 |
| `initial.png` | 开始控制时的图像；演示模式下为目标扰动后的图像 |
| `final.png` | 退出时的相机图像 |
| `convergence.png` | 像素误差及六轴速度指令曲线 |
| `trajectory.csv` | 时间、误差、检测状态、关节角、实际速度和控制指令 |
| `summary.json` | 软件版本、控制配置、最终误差与验收结果 |
| `camera.mp4` | 使用 `--record` 时生成的录像 |
| `screenshot_<frame>.png` | 按 `s` 保存的截图 |

完整复现还会生成 `results/summary.csv`、`results/comparison.png` 和 `results/overview.png`。重复使用同一输出目录会覆盖同名文件，可通过 `--output` 保存不同实验。生成结果已加入 `.gitignore`。

## 已修复问题与当前限制

当前版本修复了运动学模型与仿真不一致、相机坐标变换和控制符号、角点与深度配对、透视角点检测、Windows 方向键、欧拉角奇异点、停止控制、模型路径及渲染资源释放等问题。

上游曾记录收敛后出现额外 45° 光轴偏转。修复版五组实验中未再观察到该现象，光轴旋转残差绝对值均小于 0.19°，尚未证明所有初始条件下均已消除。

当前限制：

- 深度由仿真中已知目标表面计算，实物部署需要提供深度估计或传感器数据。
- 原目标为有厚度的半透明红色方块，深度计算使用主要可见平面近似。
- 大幅倾斜、严重遮挡、裁切或移出视野可能使四边形检测失败，此时速度指令置零。
- 无标记正方形存在 90° 对称歧义；大角度瞬间跳变或丢失后重现时，无法保证物理角点身份。
- 已验证平台为 Windows，其他平台尚未完成同等验证。

## 常见问题

**找不到 `main.py` 或 `requirements.txt`**

确认已经进入克隆得到的 `IBVS_mujoco_simulation` 目录，并通过 `dir` 检查文件。项目根目录应同时包含 `requirements.txt` 和 `mujoco_for_ur` 文件夹；若使用旧版源码，请先更新到当前版本。

**提示 `No module named 'cv2'` 或其他依赖缺失**

先执行 `conda activate ibvs-repro`，再在项目根目录执行 `python -m pip install -r requirements.txt`。使用 `python -c "import sys; print(sys.executable)"` 确认解释器属于该环境。安装和运行都使用激活后的 `python`。

**终端不支持 `conda activate`**

可使用 Anaconda Prompt；如果希望使用 PowerShell，先执行 `conda init powershell`，关闭并重新打开终端。

## 许可说明

UR5e 模型附带的许可文本见 [模型 LICENSE](mujoco_for_ur/universal_robots_ur5e/LICENSE)。上游 README 将项目代码标注为 MIT，但当前仓库根目录未附对应的独立 LICENSE 文件。


