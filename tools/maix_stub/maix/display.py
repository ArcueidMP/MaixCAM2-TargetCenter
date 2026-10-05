"""桩：显示模块，记录被显示的画面尺寸。"""


class Display:
    instances = 0        # 被创建了几个 Display（>0 表示走到显示分支）
    shows = 0            # show() 被调用了多少次

    def __init__(self, device=None):
        Display.instances += 1
        self.last = None

    def show(self, img):
        Display.shows += 1
        self.last = img
