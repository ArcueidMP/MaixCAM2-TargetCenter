"""把「上电自检优先 + 自动标定 + 自动解锁追踪」的接线补进 main.py（幂等，可反复执行）。

为什么要单独有这个脚本：本项目**两轨共用一个工作树**（见 AGENTS.md §1），
`main.py` 被外部进程**多次整份覆盖**（实测退回过 v6、v9），而 `vision/*` 与
`config.py` 的改动不受影响 —— 于是现象是"代码明明写了，上电却不自检/不追踪"。

    python tools/apply_gimbal_track_patch.py           # 检查并补齐（幂等）
    python tools/apply_gimbal_track_patch.py --check   # 只检查，不写盘

每条补丁的 `old` 可以是**一个字符串或若干替代形态**（tuple）：只要命中任一形态就
替换成最新形态。这样无论 main.py 被退回 v6 还是 v9，一条命令都能恢复到当前版本。
补完请务必按 B 轨契约收尾：`tools/build_single.py`（重生成单文件版）→
`tools/check_app.py --manifest`（C8 会核对两者的 BUILD_ID 一致）。
"""
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MAIN = os.path.join(ROOT, "main.py")

# 恢复接线时顺便把入口构建号提到当前值（被整份覆盖时它也会一并退回旧值）。
LATEST_BUILD = "2026-10-10-smooth-track-v15"

PATCHES = [
    (
        "import AutoCalibrate",
        "from vision.gimbal_calibrate import GimbalCalibrator\n",
        "from vision.gimbal_calibrate import GimbalCalibrator, AutoCalibrate\n",
    ),
    (
        "auto_cal 状态变量",
        "    self_test = None\n    calibrator = None\n",
        "    self_test = None\n    calibrator = None\n"
        "    auto_cal = None\n    auto_cal_done = False\n",
    ),
    (
        "自动标定初始化 / 上电自检优先",
        ("""            calibrator = GimbalCalibrator(Config, link, time.perf_counter())
            if calibrator.done:
                # 未配置标定轴：正常进入自检 + 跟踪。
                calibrator = None
                self_test = GimbalSelfTest(Config, link, time.perf_counter())
                if self_test.state == "RUNNING":
                    axes_txt = "+".join(sorted({s[0] for s in self_test.sequence}))
                    print("[gimbal] 自检计划 %d 步，参与轴=%s（链路未打开的轴已自动跳过）"
                          % (self_test.total, axes_txt))
                elif self_test.state == "SKIPPED":
                    print("[gimbal] 自检已跳过(%s) -> 上电不会看到电机动作。"
                          "要么填 YAW/PITCH_PULSES_PER_DEG、要么给 GIMBAL_GEAR_RATIO，"
                          "要么设 SELF_TEST_FALLBACK_PULSES>0 用固定脉冲验证链路"
                          % self_test.reason)
            else:
                print("[gimbal] calibration mode axis=%s（本次只标定，不闭环）" % calibrator.axis)
""",
         # v9 形态：已有 axis_sel / 自动标定分支，但自检**只在非 auto 时**创建
         # （= 上电不自检、要等画面出现靶心才开始标定的那个版本）。
         """            axis_sel = str(getattr(Config, "CALIBRATE_AXIS", "")).strip().lower()
            calibrator = GimbalCalibrator(Config, link, time.perf_counter())
            if calibrator.done:
                calibrator = None
                if axis_sel == "auto":
                    # 自动标定：依次标两轴，全部成功后自动解锁闭环 -> 直接进入跟踪。
                    auto_cal = AutoCalibrate(Config, link, time.perf_counter())
                    print("[gimbal] 自动标定模式：依次标定 %s，全部成功后自动解锁并开始跟踪"
                          % "+".join(auto_cal.axes))
                    print("[gimbal] 标定期不发闭环指令；请确保画面里能看到靶心，否则会超时失败")
                else:
                    # 未配置标定轴：正常进入自检 + 跟踪。
                    self_test = GimbalSelfTest(Config, link, time.perf_counter())
                    if self_test.state == "RUNNING":
                        axes_txt = "+".join(sorted({s[0] for s in self_test.sequence}))
                        print("[gimbal] 自检计划 %d 步，参与轴=%s（链路未打开的轴已自动跳过）"
                              % (self_test.total, axes_txt))
                    elif self_test.state == "SKIPPED":
                        print("[gimbal] 自检已跳过(%s) -> 上电不会看到电机动作。"
                              "要么填 YAW/PITCH_PULSES_PER_DEG、要么给 GIMBAL_GEAR_RATIO，"
                              "要么设 SELF_TEST_FALLBACK_PULSES>0 用固定脉冲验证链路"
                              % self_test.reason)
            else:
                print("[gimbal] calibration mode axis=%s（本次只标定，不闭环）" % calibrator.axis)
"""),
        """            axis_sel = str(getattr(Config, "CALIBRATE_AXIS", "")).strip().lower()
            calibrator = GimbalCalibrator(Config, link, time.perf_counter())
            if calibrator.done:
                calibrator = None
                # ★ 上电自检与视觉/标定**完全解耦**：只要本次不是"人工标定"模式，
                #   上电后立刻按固定脉冲走一遍（**pitch 上→下、yaw 左→右**，各回到起始位姿）。
                #   自检只用链路 + 时间，**不需要靶纸、不等黑框识别**；4 步跑完才轮到
                #   自动标定 / 追踪（见主循环：自检分支优先）。
                self_test = GimbalSelfTest(Config, link, time.perf_counter())
                if self_test.state == "RUNNING":
                    axes_txt = "+".join(sorted({s[0] for s in self_test.sequence}))
                    print("[gimbal] 上电自检 %d 步，参与轴=%s（固定脉冲，不等黑框/靶纸）"
                          % (self_test.total, axes_txt))
                elif self_test.state == "SKIPPED":
                    print("[gimbal] 自检已跳过(%s) -> 上电不会看到电机动作。"
                          "要么填 YAW/PITCH_PULSES_PER_DEG、要么给 GIMBAL_GEAR_RATIO，"
                          "要么设 SELF_TEST_PULSES>0 用固定脉冲验证链路"
                          % self_test.reason)
                if axis_sel == "auto":
                    print("[gimbal] 自检完成后自动标定 %s（标定期才需要画面里有靶心，"
                          "否则按 AUTO_CALIBRATE_TIMEOUT_S 超时失败）"
                          % "+".join(getattr(Config, "AUTO_CALIBRATE_AXES", ("yaw", "pitch"))))
            else:
                print("[gimbal] calibration mode axis=%s（本次只人工标定：不自检、不闭环）"
                      % calibrator.axis)
""",
    ),
    (
        "主循环解锁闭环 / 自检优先",
        ("""        if calibrator is not None:
            if not calibrator.done and aim is not None and aim.state != "FAULT":
                calibrator.update(after_uart, result)
        elif aim is not None and link is not None and aim.state != "FAULT":
            if self_test is not None and not self_test.done:
                self_test.update(after_uart)
            elif self_test is None or self_test.state != "FAULT":
""",
         # v9 形态：自动标定在主循环里，但**自检排在标定之后**（自检只在最后才轮到）。
         """        if calibrator is not None:
            if not calibrator.done and aim is not None and aim.state != "FAULT":
                calibrator.update(after_uart, result)
        elif auto_cal is not None and not auto_cal.done:
            # 自动标定期间只标定、不闭环（与人工标定同一口径）。
            if aim is not None and aim.state != "FAULT":
                auto_cal.update(after_uart, result)
        elif aim is not None and link is not None and aim.state != "FAULT":
            if auto_cal is not None and not auto_cal_done:
                # 自动标定刚结束：只有两轴全部标定成功才解锁闭环（否则保持锁定）。
                auto_cal_done = True
                if auto_cal.applied and aim.unlock("auto-calibrate"):
                    print("[gimbal] 自动标定完成 -> 闭环已解锁，开始跟踪")
                else:
                    print("[gimbal] 自动标定未全部成功(%s) -> 闭环保持锁定，不会发电机指令"
                          % (auto_cal.reason or auto_cal.state))
            if self_test is not None and not self_test.done:
                self_test.update(after_uart)
            elif self_test is None or self_test.state != "FAULT":
                aim_result = aim.update(result, now=after_uart)
                for axis, command in aim_result["cmds"].items():
                    if command is not None:
                        ccw, pulses, rpm = command
                        link.move(axis, ccw, pulses, rpm, Config.ACC_TRACK, now=after_uart)
""",
        """        if calibrator is not None:
            if not calibrator.done and aim is not None and aim.state != "FAULT":
                calibrator.update(after_uart, result)
        elif self_test is not None and not self_test.done:
            # ★ 上电自检优先：固定脉冲 4 步（pitch 上→下、yaw 左→右）先走完，
            #   期间不发任何标定/闭环指令；**不依赖视觉结果**（无黑框也照走）。
            #   唯一例外：链路已被判 FAULT 时不再下发任何指令。
            if aim is not None and aim.state != "FAULT":
                self_test.update(after_uart)
        elif self_test is not None and self_test.state == "FAULT":
            # 自检判定链路故障（所有参与轴都写不出去）-> 保持静默：不标定、不追踪。
            pass
        elif (auto_cal is None and aim is not None and aim.state != "FAULT"
              and str(getattr(Config, "CALIBRATE_AXIS", "")).strip().lower() == "auto"):
            # 自检结束后才真正开始自动标定：时间窗口从"这一刻"算起，
            # 不被上电自检的机械动作耗时挤占（否则自检慢时会吞掉标定预算）。
            auto_cal = AutoCalibrate(Config, link, after_uart)
            print("[gimbal] 上电自检完成 -> 开始自动标定 %s（标定期不闭环，需画面里有靶心）"
                  % "+".join(auto_cal.axes))
        elif auto_cal is not None and not auto_cal.done:
            # 自动标定期间只标定、不闭环（与人工标定同一口径）。
            if aim is not None and aim.state != "FAULT":
                auto_cal.update(after_uart, result)
        elif aim is not None and link is not None and aim.state != "FAULT":
            if auto_cal is not None and not auto_cal_done:
                # 自动标定刚结束：只有两轴全部标定成功才解锁闭环（否则保持锁定）。
                auto_cal_done = True
                if auto_cal.applied and aim.unlock("auto-calibrate"):
                    print("[gimbal] 自动标定完成 -> 闭环已解锁，开始跟踪")
                else:
                    print("[gimbal] 自动标定未全部成功(%s) -> 闭环保持锁定，不会发电机指令"
                          % (auto_cal.reason or auto_cal.state))
            aim_result = aim.update(result, now=after_uart)
            for axis, command in aim_result["cmds"].items():
                if command is not None:
                    ccw, pulses, rpm = command
                    link.move(axis, ccw, pulses, rpm, Config.ACC_TRACK, now=after_uart)
"""),
        """        if calibrator is not None:
            if not calibrator.done and aim is not None and aim.state != "FAULT":
                calibrator.update(after_uart, result)
        elif self_test is not None and not self_test.done:
            # ★ 上电自检优先：固定脉冲 4 步（pitch 上→下、yaw 左→右）先走完，
            #   期间不发任何标定/闭环指令；**不依赖视觉结果**（无黑框也照走）。
            #   唯一例外：链路已被判 FAULT 时不再下发任何指令。
            if aim is not None and aim.state != "FAULT":
                self_test.update(after_uart)
        elif self_test is not None and self_test.state == "FAULT":
            # 自检判定链路故障（所有参与轴都写不出去）-> 保持静默：不标定、不追踪。
            pass
        elif (auto_cal is None and aim is not None and aim.state != "FAULT"
              and str(getattr(Config, "CALIBRATE_AXIS", "")).strip().lower() == "auto"):
            # 自检结束后才真正开始自动标定：时间窗口从"这一刻"算起，
            # 不被上电自检的机械动作耗时挤占（否则自检慢时会吞掉标定预算）。
            auto_cal = AutoCalibrate(Config, link, after_uart)
            print("[gimbal] 上电自检完成 -> 开始自动标定 %s（标定期不闭环，需画面里有靶心）"
                  % "+".join(auto_cal.axes))
        elif auto_cal is not None and not auto_cal.done:
            # 自动标定期间只标定、不闭环（与人工标定同一口径）。
            if aim is not None and aim.state != "FAULT":
                auto_cal.update(after_uart, result)
        elif aim is not None and link is not None and aim.state != "FAULT":
            if auto_cal is not None and not auto_cal_done:
                # 自动标定刚结束：只有两轴全部标定成功才解锁闭环（否则保持锁定）。
                auto_cal_done = True
                if auto_cal.applied and aim.unlock("auto-calibrate"):
                    print("[gimbal] 自动标定完成 -> 闭环已解锁，开始跟踪")
                else:
                    print("[gimbal] 自动标定未全部成功(%s) -> 闭环保持锁定，不会发电机指令"
                          % (auto_cal.reason or auto_cal.state))
            aim_result = aim.update(result, now=after_uart)
            for axis in aim_result["stops"]:
                link.stop(axis, now=after_uart)
            if aim_result["state"] == "FAULT":
                link.stop(now=after_uart)
            for axis, command in aim_result["cmds"].items():
                if command is not None:
                    ccw, pulses, rpm = command
                    if not link.move(axis, ccw, pulses, rpm, Config.ACC_TRACK, now=after_uart):
                        link.stop(now=after_uart)
                        aim.fault("uart_write_failed:%s" % axis)
                        break
""",
    ),
    (
        "闭环用 ACC_TRACK（曲线）",
        "link.move(axis, ccw, pulses, rpm, Config.ACC_DEFAULT, now=after_uart)",
        "link.move(axis, ccw, pulses, rpm, Config.ACC_TRACK, now=after_uart)",
    ),
]


def main():
    check_only = "--check" in sys.argv[1:]
    if not os.path.exists(MAIN):
        print("[patch] 找不到 %s" % MAIN)
        return 2
    src = io.open(MAIN, encoding="utf-8", newline="").read()
    out = src
    report = []
    for name, old, new in PATCHES:
        olds = (old,) if isinstance(old, str) else tuple(old)
        if new in out:
            report.append(("skip", name, "已存在"))
            continue
        hit = next((o for o in olds if o in out), None)
        if hit is None:
            report.append(("fail", name, "锚点未找到（可能已被别处改写，需人工检查）"))
            continue
        if check_only:
            report.append(("todo", name, "缺失，需要补齐"))
            continue
        out = out.replace(hit, new, 1)
        report.append(("patch", name, "已补上"))
    # BUILD_ID：只要不是最新值就统一提升（被整份覆盖时它会一并退回旧值）
    if ('BUILD_ID = "%s"' % LATEST_BUILD) in out:
        report.append(("skip", "BUILD_ID", "已是 %s" % LATEST_BUILD))
    else:
        new_out, n = re.subn(r'^BUILD_ID = "[^"]*"',
                             'BUILD_ID = "%s"' % LATEST_BUILD,
                             out, count=1, flags=re.M)
        if n == 0:
            report.append(("fail", "BUILD_ID", "没找到 BUILD_ID 行"))
        elif check_only:
            report.append(("todo", "BUILD_ID", "不是最新 %s" % LATEST_BUILD))
        else:
            out = new_out
            report.append(("patch", "BUILD_ID", "已提升为 %s" % LATEST_BUILD))

    for kind, name, msg in report:
        print("[patch] %-6s %-16s %s" % (kind, name, msg))
    todo = [r for r in report if r[0] in ("patch", "todo")]
    if check_only:
        print("[patch] 检查完毕：%d 处需要处理" % len(todo))
        return 1 if todo else 0
    if not todo:
        print("[patch] main.py 接线完整，无需改动")
        return 0
    if any(r[0] == "fail" for r in report):
        print("[patch] 有锚点未命中 -> 已放弃写盘，请人工核对 main.py")
        return 2
    io.open(MAIN, "w", encoding="utf-8", newline="\n").write(out)
    print("[patch] 已写回 main.py；请接着跑 tools/build_single.py 与 tools/check_app.py --manifest")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
