from abc import ABC, abstractmethod

class Setting(ABC):
    def __init__(self, name, description, defaultValue = 0):
        self.name = name
        self.description = description
        self.value = defaultValue
        pass

    @abstractmethod
    def getValue(self):
        pass

    @abstractmethod
    def onUpdate(self, value):
        pass