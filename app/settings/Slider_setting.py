from .setting import Setting

class Slider(Setting):
    def __init__(self, name, description, minVal, maxVal, defaultValue = 0):
        self.minVal = minVal
        self.maxVal = maxVal
        super().__init__(name, description, max(self.minVal,defaultValue))

    def getValue(self):
        return self.value

    def onUpdate(self, value):
        self.value = value