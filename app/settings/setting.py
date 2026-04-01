from abc import ABC, abstractmethod

class Setting(ABC):
    def __init__(self, name, id, description, defaultValue = 0):
        self.name = name
        self.id = id
        self.description = description
        self.value = defaultValue
        self.default = defaultValue
        pass

    @abstractmethod
    def getValue(self):
        pass

    @abstractmethod
    def onUpdate(self, value):
        pass