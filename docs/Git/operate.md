
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