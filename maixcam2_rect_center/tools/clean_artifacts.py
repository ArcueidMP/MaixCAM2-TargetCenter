"""清理 B 轨「可再生的调试产物」，把工作区体积压回可控范围。

默认**只扫描不删除**（打印清单与总体积）；确认无误后加 `--apply` 才真正删除。

只删「能重新生成 / 已被 .gitignore 忽略」的东西，**绝不碰**源码、测试固件、文档与单文件版：

    删除：
      - tools/e2e_report/ 下的 *.png        （抓屏/总览图，跑一次 e2e 就会重画）
      - 任意 __pycache__/ 目录与 *.pyc/*.pyo
      - screenshot*.png / frame*.png / mask*.png / debug_*.png（与 .gitignore 同口径）
    保留：
      - tools/negative_e421.png / negative_e880.png  ← **测试输入固件，必须保留**
      - tools/e2e_report/ 下的 *.html / *.json        （自包含审查报告与其结果）
      - 全部 *.py / *.md / *.yaml / assets/ / dist/

用法：
    "D:/Python311/python.exe" -B tools/clean_artifacts.py           # 扫描
    "D:/Python311/python.exe" -B tools/clean_artifacts.py --apply   # 删除
    "D:/Python311/python.exe" -B tools/clean_artifacts.py --limit-mb 5   # 超阈值提示（默认 5）
"""
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
E2E = os.path.join(ROOT, "tools", "e2e_report")
# 测试输入固件：无论规则怎么匹配都绝不动。
KEEP_NAMES = {"negative_e421.png", "negative_e880.png"}
JUNK_PREFIXES = ("screenshot", "frame", "mask", "debug_")


def _walk_size(path):
    if os.path.isfile(path):
        try:
            return os.path.getsize(path)
        except OSError:
            return 0
    total = 0
    for base, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(base, name))
            except OSError:
                pass
    return total


def collect():
    """返回 (待删文件列表, 待删目录列表)。"""
    files, dirs = [], []
    if os.path.isdir(E2E):
        for name in sorted(os.listdir(E2E)):
            if name.lower().endswith(".png") and name not in KEEP_NAMES:
                files.append(os.path.join(E2E, name))
    for base, subdirs, names in os.walk(ROOT):
        if os.path.basename(base) == "__pycache__":
            dirs.append(base)
            subdirs[:] = []
            continue
        for name in names:
            if name in KEEP_NAMES:
                continue
            full = os.path.join(base, name)
            if name.endswith((".pyc", ".pyo")):
                files.append(full)
            elif name.lower().endswith(".png") and name.lower().startswith(JUNK_PREFIXES):
                files.append(full)
    return files, dirs


def human(n):
    return "%.2f MB" % (n / 1048576.0) if n >= 1048576 else "%.0f KB" % (n / 1024.0)


def main():
    apply = "--apply" in sys.argv[1:]
    limit_mb = 5.0
    for i, arg in enumerate(sys.argv[1:]):
        if arg == "--limit-mb" and i + 2 <= len(sys.argv) - 1:
            limit_mb = float(sys.argv[i + 2])
    before = _walk_size(ROOT)
    files, dirs = collect()
    freed = sum(_walk_size(p) for p in files) + sum(_walk_size(p) for p in dirs)
    print("[clean] B 轨当前体积 = %s" % human(before))
    print("[clean] 命中可再生调试产物：%d 个文件 + %d 个 __pycache__ 目录，共 %s"
          % (len(files), len(dirs), human(freed)))
    for path in files[:20]:
        print("        - %s" % os.path.relpath(path, ROOT).replace("\\", "/"))
    if len(files) > 20:
        print("        … 其余 %d 个文件（多为 e2e 抓屏）" % (len(files) - 20))
    for path in dirs:
        print("        - %s/" % os.path.relpath(path, ROOT).replace("\\", "/"))
    if not files and not dirs:
        print("[clean] 没有需要清理的产物。")
        return 0
    if not apply:
        print("[clean] 这是**扫描**（未删除）。确认后加 --apply 执行。")
        return 0
    for path in files:
        try:
            os.remove(path)
        except OSError as exc:
            print("[clean] 删除失败 %s: %s" % (path, exc))
    for path in dirs:
        shutil.rmtree(path, ignore_errors=True)
    after = _walk_size(ROOT)
    print("[clean] 已清理：%s -> %s（释放 %s）" % (human(before), human(after), human(before - after)))
    if after > limit_mb * 1048576:
        print("[clean] ⚠ 仍超过 %.0fMB 上限，请检查是否又新增了大体积产物。" % limit_mb)
    else:
        print("[clean] OK：在 %.0fMB 上限内。" % limit_mb)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
