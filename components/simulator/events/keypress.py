from collections import defaultdict


class Keypress:
    """
    Use this to control various keypresses. Add the desired keys using the register_key function
    """

    def __init__(self):
        self.DEBUG = False
        self.keys = defaultdict(lambda: self.factory(self.default_press))

        self.register_key("d", self.d_press)

    def register_key(self, key, callback):
        self.keys[key] = callback

    def execute(self, obj, event):
        key = obj.GetKeySym()
        self.keys[key]()

    def default_press(self):
        pass

    def d_press(self):
        self.DEBUG = True if self.DEBUG is False else False
        print("D Button pressed")

    def factory(self, arg):
        return arg
