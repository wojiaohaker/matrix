
`src/robot_mc/.gitignore` 中 `!build/export/mc/bin/*` 会重新包含所有 bin 文件。需要在后面加上排除大文件的规则：


再把 `dynamicmaps/moonworld.bin` 也加到根 `.gitignore`：


完成。所有被删大文件已加入忽略：

| 文件 | 忽略位置 |
|------|----------|
| `bin/sim_launcher.bin` (63MB) | 根 `.gitignore` ✅（之前已加） |
| `src/robot_mc/.../mc_ctrl` (86MB) | `src/robot_mc/.gitignore` ✅ |
| `src/robot_mc/.../libbiomimetics.so` (158MB) | `src/robot_mc/.gitignore` ✅ |
| `dynamicmaps/*.bin` (137MB) | 根 `.gitignore` ✅ |
| `src/UeSim/Linux/` (含 CrashReportClient) | 根 `.gitignore` ✅（之前已加） |

这些文件本地仍存在可正常运行，但不会被 git 跟踪提交。


问题很明显！看 mc_ctrl 日志：

```
[MujocoCommandInterface] initialize...
```

它用的是 **MujocoCommandInterface**（type=5），不是 **UnrealCommandInterface**（type=8）！这意味着通信协议/端口可能不匹配。让我检查配置：


**找到原因了！** `motor_platform_type` 被重置为 `5`（MuJoCo），CarlaUnreal 需要 `8`（Unreal）。

这可能是之前 git filter-branch/stash 操作时配置文件被还原了。修复：


已修复。问题原因：

- `motor_platform_type: 5` → 使用 `MujocoCommandInterface`（Matrix UE 专用）
- `motor_platform_type: 8` → 使用 `UnrealCommandInterface`（CarlaUnreal 专用）

之前的 git filter-branch 操作把配置文件还原成了旧版本（type=5），导致 mc_ctrl 用了错误的通信接口，UE 发的数据根本不被处理。

现在重启 mc_ctrl 即可：

```bash
cd /home/qiyuan/Softwares/Matrix/src/robot_mc/build/export/mc/bin
export LD_LIBRARY_PATH="$(pwd):${LD_LIBRARY_PATH:-}"
export ROBOT_TYPE=XG
taskset -c 7 ./mc_ctrl r 2>&1 | tee /tmp/mc_ctrl_carlaunreal.log
```

启动后应该看到 `[UnrealCommandInterface] initialize...` 而不是 `[MujocoCommandInterface]`，然后按 U 键就能站立了。


你说得对，filter-branch 不仅删了二进制文件，还因为 `git stash` 失败导致**未提交的配置修改全部丢失**。让我全面检查还有哪些配置被还原了：


让我全面检查 git filter-branch 之后还有哪些配置被还原了。对比当前配置和 CarlaUnreal 集成所需的设置：


全面排查完毕。**git filter-branch 造成的所有影响如下：**

---

**一、配置被错误还原（真正的问题）：**

| 文件 | 变更 | 影响 | 状态 |
|------|------|------|------|
| `xg-user-parameters.yaml` | `motor_platform_type: 8→5` | mc_ctrl 用了 MujocoCommandInterface，UE 指令不被处理 | ✅ 已修复（未提交） |

这是**唯一**被错误还原的配置。`config.json` 中 `mujoco_running: true` 是正确的（UE 内嵌 MuJoCo 物理引擎）。

---

**二、大文件从 git 移除（本地仍在磁盘上，已加 .gitignore）：**

| 文件 | 大小 | 状态 |
|------|------|------|
| `bin/sim_launcher.bin` | 63MB | ✅ 已恢复 |
| `src/robot_mc/.../mc_ctrl` | 86MB | ✅ 已恢复 |
| `src/robot_mc/.../libbiomimetics.so` | 158MB | ✅ 已恢复 |
| `zsibot_mujoco_ue` + debug + sym | 2.2GB | ✅ 已恢复 |
| `dynamicmaps/moonworld.bin` | 137MB | 可选，未恢复 |

---

**三、CarlaUE5 仓库：未受影响**（干净状态，`my-ue5-dev` 分支正常）

---

**结论：filter-branch 只搞坏了一个配置** `motor_platform_type`，已修复。当前工作区唯一的未提交修改就是它：

```bash
cd /home/qiyuan/Softwares/Matrix
git diff  # 只有 xg-user-parameters.yaml: motor_platform_type: 5→8
```

现在重启 mc_ctrl 应该能看到 `[UnrealCommandInterface] initialize...`，按 U 键即可站立。