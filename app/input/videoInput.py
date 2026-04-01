from abc import ABC, abstractmethod
import time

class VideoInput(ABC):
    def __init__(self):
        self.previous_time = time.time()
        self.fps = 0

    @abstractmethod
    def getFrame(self):
        pass
    
    @abstractmethod
    def getFrame(self):            
        current_time = time.time()
        seconds_elapsed = current_time - self.previous_time
        if seconds_elapsed > 0:
            self.fps = round(1.0 / seconds_elapsed)
        self.previous_time = current_time

    @abstractmethod
    def release(self):
        pass

    @abstractmethod
    def getSettings(self):
        pass
    
    @abstractmethod
    def onSetting(self, name, value):
        pass

    @abstractmethod
    def isOpen(self):
        pass