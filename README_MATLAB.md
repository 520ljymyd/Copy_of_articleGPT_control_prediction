# VS Code → MATLAB: 通过共享引擎在 MATLAB Desktop 中运行脚本

步骤概览

1. 在 MATLAB Desktop 中共享当前会话（只需一次或每次重启 MATLAB 后执行）：

   在 MATLAB 命令窗口中执行：

   ```matlab
   matlab.engine.shareEngine('vscode')
   ```

   你可以把 `'vscode'` 换成任意名字，但使用时需一致。

2. 在你的系统中安装 MATLAB Engine for Python（一次性动作）：

   打开 PowerShell 或 CMD，运行（路径根据你的 MATLAB 安装位置调整）：

   ```powershell
   cd "D:\MATLAB\R2023b\extern\engines\python"
   python -m pip install .
   ```

3. 仓库中已有工具：

- `send_to_matlab.py`：Python 脚本，连接共享 MATLAB 会话并在该会话中运行指定的 `.m` 文件。
- `scripts/run_matlab.ps1`：PowerShell 包装器，调用上面的 Python 脚本（便于在 Windows/VS Code 中调用）。
- `.vscode/tasks.json`：包含两个任务：
  - `Run MATLAB current file`（使用 `matlab -batch` 启动一个 MATLAB 进程运行并退出）
  - `Run in shared MATLAB (via Python)`（连接到已共享的 MATLAB Desktop，会在该会话中运行当前文件）

如何使用（运行当前打开的 `.m` 文件）

方法 A — 使用共享会话（推荐，结果会在已打开的 MATLAB Desktop 中显示）

1. 在 MATLAB Desktop 中运行：

   ```matlab
   matlab.engine.shareEngine('vscode')
   ```

2. 在 VS Code 中打开你要运行的 `.m` 文件。
3. 运行 VS Code 命令面板 -> `Tasks: Run Task` -> 选择 `Run in shared MATLAB (via Python)`。

或者在终端直接运行（PowerShell）：

```powershell
.
$PWD\scripts\run_matlab.ps1 "D:\vscode_project_myjl\Copy_of_articleGPT_control_prediction\main.m"
```

方法 B — 直接启动一个 MATLAB 进程（适合无需交互的批处理）

在 VS Code 直接使用已有任务 `Run MATLAB current file`，或在终端运行：

```powershell
& "D:\MATLAB\R2023b\bin\matlab.exe" -batch "run('D:\vscode_project_myjl\Copy_of_articleGPT_control_prediction\main.m')"
```

常见问题

- 连接失败（找不到共享引擎）：确保 MATLAB Desktop 中已调用 `matlab.engine.shareEngine('<name>')`，并且名称与脚本中使用的 `--name` 或默认 `vscode` 匹配。
- 无法导入 `matlab.engine`：请确认已为正在使用的 Python 安装正确执行 `python -m pip install .`（在 MATLAB 的 `extern/engines/python` 目录下）。

我可以为你做的后续工作

- 将 `send_to_matlab.py` 注册为一个 VS Code 任务的默认快捷键（Ctrl+Shift+B）。
- 把 `scripts/run_matlab.ps1` 调整为自动保存当前文件后再运行。

如果需要我继续配置（例如把任务绑定为默认 build 或自动保存），回复我想要的行为，我来修改配置。 
