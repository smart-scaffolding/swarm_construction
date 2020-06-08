class Keypress:
    def __init__(self):
        self.DEBUG = False

    def execute(self, obj, event):
        key = obj.GetKeySym()
        if key == "d":
            self.DEBUG = True if self.DEBUG is False else False
