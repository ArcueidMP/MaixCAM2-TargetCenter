# AGENTS.md · B轨 MaixCAM2 工程契约

本文件负责 B 轨工作细则；根 AGENTS.md 管跨轨协作；README.md 管当前算法与操作。
本轮版本为 `0.8.0`，BUILD_ID=`2026-10-17-canny-kalman-v0.8.0`。
旧版本的18文件包、开机自动自检、原生MaixVision图传和v20覆写补丁均不是当前实现。

## 工作边界与提交

- 只改 `maixcam2_rect_center/`。A轨根目录 main/config/vision/app/tools 不属于本轨。
- 公共文档和 `.workbuddy/memory/` 确需修改时先说明意图，只做最小更新。
- 保留用户已有改动和旧旁支；分支操作服从当前用户要求。
- 按精确路径 `git add`，禁用 `git add -A` / `git add .`，避免卷入其它会话文件。
  本轨提交前缀 `gpt:`；纯公共文档用 `repo:`/`docs:`。
- 完成一轮后立即提交：改源码→提升BUILD_ID→重生成单文件→门禁→核暂存路径→提交。
- `main.py` 曾被外部编辑器整份覆写，保持薄入口；具体实现放 `vision/`。
- 每轮验证结论追加到根 `.workbuddy/memory/YYYY-MM-DD.md`，不覆写历史日志。

## 轴映射（实机既定，不能凭直觉改）

| 代码轴 | 设备 | 引脚及复用 | ID | CCW正方向 |
|---|---|---|---|---|
| yaw | /dev/ttyS2 | B0 UART2_TX / B1 UART2_RX | 2 | 左 |
| pitch | /dev/ttyS0 | A28 UART0_TX / A29 UART0_RX | 1 | 上 |

地址跟电机走。两轴都需显式pinmap并回读；U0同时是系统日志口，启动清RX。
`DIR_SIGN=-1/-1` 和 `PULSE_PER_PX=3.67/3.67` 需现场标定，不能称为本轮实测。
串口协议固定0x6B、大端、相对位置；曲线档位非零且越小越柔。
驱动器需UART_FUN/115200/对应ID/Response=Receive/1.8°/16细分/P_Pul=PUL_OFF。

## 当前架构不可破坏的约束

1. `target_api.py` 和算法模块可在无MaixPy的PC导入；SDK导入放构造或函数内。
2. 最新帧复制后跨线程，UART在运行期间由识别线程独占；按钮只传Event。
3. 保留clock注入。先join识别线程，再停电机和关闭链路；活线程阻止再次启动。
4. 稳态只窄带精修；Canny/Otsu每次搜索最多两张掩膜；ROI失败必须同帧全图搜索。
   `SEARCH_EXHAUSTIVE=True` 不得为性能调成False。失锁帧两次搜索累计最多4张。
5. 云台用raw中心；动作完成后还要等新采图，重复/倒序/过期结果不能追加相对动作。
6. FAULT优先于补使能、自检、标定和搜索；写失败和停机失败须锁故障。
7. 本地Display在主线程创建，show在同进程显示线程；图传在独立进程，不导入maix。
   MAIXVISION只在创建本地Display时临时改0并还原，不能永久改掉会话环境。
8. 显示失败只停显示；识别线程失败必须让主循环发现并有序退出。
9. 自检为按钮触发的六段，开机不自动走；SEARCH取消不自动回零。
10. IMU是相对朝向；偏置标定需底盘静止；提前停止时请求脉冲不是已执行脉冲。

## 门禁与打包

在本目录，用具备numpy和opencv的Python 3运行（Windows可用D:/Python311/python.exe）：

```bash
python -B tools/build_single.py
python -B tools/build_single.py --check
python -B tools/build_single.py --smoke
python -B tools/check_pc.py
python -B tools/git_round.py --check
```

- 新增/改名模块同步 `tools/build_single.py` 的MODULES与依赖顺序。
- 单文件自动生成，禁止手改；vision包导出及图传子进程源码也必须嵌入。
- 应用清单为 **25文件 / FAIL=0**；保留assets/icon.png，不能排除全部PNG。
- 新视觉门禁为30/30、最大中心误差≤0.7px、负样本0误检；测试不能用多帧等待隐藏漏帧。
- 改视觉候选/验证跑real_scene；改主流程跑main_self_test/app_runtime；改显示/搜索预算跑runtime_performance；
  改协议/控制跑gimbal_*与extension_*；完整交付用check_pc汇总。
- 缺失真机正样本必须明确SKIP，不可把合成图、PC耗时或mock成功称为真机验收。
- `git_round` 已改用当前应用门禁；旧 `apply_gimbal_track_patch.py` 仅历史参考，不能向薄入口应用旧模板。
- 源码、文档、测试输入、dist进库；调试截图、pycache、e2e_report图不进库。
- 使用LF换行，应用目录保持≤5MB。改变参数前先看README和硬件标定与上板检查.md。
