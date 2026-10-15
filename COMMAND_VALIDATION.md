# README 命令实测记录

验证日期：2026-10-15。平台：Windows PowerShell，本机 Conda 4.10.3。

## 复现到的问题

此前验证遗漏了“目标环境已经激活后，再执行 `conda run`”这一使用方式。
已在本机实际执行并得到以下结果：

```powershell
conda activate mujoco
conda run --no-capture-output -n mujoco python -c "import sys; print(sys.executable)"
# D:\Anaconda\python.exe

python -c "import sys, cv2; print(sys.executable); print(cv2.__version__)"
# D:\Anaconda\envs\mujoco\python.exe
# 4.13.0
```

第一条选择了 base 的 Python，尽管环境变量仍指向 `mujoco`。base 中没有 `cv2`，从而出现用户报告的异常。
在未激活目标环境时检查 `conda run`，无法覆盖这个问题。

修复后的 README 使用激活后的 `python` 或解释器绝对路径。
两个 `.bat` 和 `reproduce.ps1` 通过 `scripts/resolve_mujoco_python.ps1` 获取环境解释器后直接调用，不再嵌套 `conda run`。
本次未向 base 安装 OpenCV，也未升级 Conda。

## 实测命令与结果

命令以 README 中所列的工作目录和环境为前提执行。

| 项目 | 验证方式 | 结果 |
| --- | --- | --- |
| 绝对解释器路径启动 | 从外层工作区启动，目标环境已激活 | 打开两个可见窗口，退出码 0 |
| `conda activate mujoco` 后 `python -B mujoco_for_ur/main.py` | 从修复仓库根目录启动 | 打开两个可见窗口，退出码 0 |
| `python -B mujoco_for_ur/main.py --demo` | 完整执行 600 个周期 | 正常自动退出，退出码 0 |
| `run_demo.bat` | 从外层目录、已激活环境调用，完整执行 600 个周期 | 打开两个窗口，自动退出，退出码 0 |
| `run_interactive.bat` | 从外层目录、已激活环境调用 | 打开两个窗口，退出码 0 |
| 环境与依赖检查 | 实际导入 `cv2`、`mujoco` 并输出解释器位置 | 正确选中目标环境 |
| `python -m pip install -r requirements.txt` | 在 `mujoco` 和全新 `ibvs-repro` 环境分别执行 | 安装/检查成功 |
| 新建环境步骤 | 实际创建 `ibvs-repro`，激活并安装依赖 | Python 3.10.22，依赖安装成功，`pip check` 通过 |
| 新环境演示命令 | 激活 `ibvs-repro`，执行带独立输出目录的演示 | 打开两个窗口，600 个周期正常结束，退出码 0 |
| `powershell -ExecutionPolicy Bypass -File .\reproduce.ps1 -Record` | 从已激活的 `mujoco` 环境执行完整脚本 | 10 项回归检查、五组实验和汇总均通过 |
| `powershell -ExecutionPolicy Bypass -File .\reproduce.ps1` | 执行不带录像参数的完整脚本 | 检查、五组实验和汇总均通过 |
| 单独组合扰动命令 | 执行 `--headless --demo --scenario combined --record` | 退出码 0，实验通过 |
| 单独回归检查命令 | 执行 `python -B -m unittest discover -s tests -v` | 10 项全部通过 |

三个持续交互入口还通过 Windows 消息向各自所属的相机窗口发送了 `v`、`s`、`t`、`v` 和 `Esc`，
确认日志中出现启用、关闭伺服和更新参考的信息，生成截图并正常退出。
这里只操作本次启动进程所属的窗口。两个 `.bat` 使用命令行调用验证，未模拟资源管理器中的鼠标双击。

## 验证产物

本机详细产物保存在 `results/readme_validation/`，该目录随其他实验产物一起被 Git 忽略：

- `gui_commands.json`：六个 GUI 入口的原始命令、工作目录、可见窗口名称和退出码。
- `absolute_interactive.log`、`relative_interactive.log`、`relative_demo.log`：直接启动日志。
- `batch_demo.log`、`batch_interactive.log`：批处理入口日志。
- `fresh_install.log`、`fresh_environment_demo.log`：新环境安装和实际运行日志。
- `reproduce_record_fixed.log`、`reproduce_no_record_fixed.log`：完整自动复现日志。
- `unit_tests.log`、`single_combined.log`：独立检查与单组实验日志。

新环境演示结果位于 `results/fresh_environment_demo/`，五组主实验结果仍位于 `results/<scenario>/`。
验证表中的通过结果均来自实际执行，不以 `--help` 或文件存在检查代替仿真运行。
