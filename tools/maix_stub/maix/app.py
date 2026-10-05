"""桩：app 模块，跑够 LIMIT 帧就让主循环退出。"""

_state = {"n": 0}
LIMIT = 40


def reset():
    """把帧计数清零。同一进程里要多次跑 main() 时（预检脚本）必须先调用。"""
    _state["n"] = 0


def need_exit():
    _state["n"] += 1
    return _state["n"] > LIMIT


def get_sys_config_kv(item, key):
    return ""
