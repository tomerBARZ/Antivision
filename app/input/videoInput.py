from abc import ABC, abstractmethod

class VideoInput(ABC):
    @abstractmethod
    def getFrame(self):
        pass
    
    @abstractmethod
    def getFrame(self):
        print("ORIGINal")
        pass

    @abstractmethod
    def release(self):
        pass

    @abstractmethod
    def getSettings(self):
        pass
    
    @abstractmethod
    def isOpen(self):
        pass