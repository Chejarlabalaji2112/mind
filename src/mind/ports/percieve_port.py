from abc import ABC, abstractmethod

class Vision(ABC):
    """This class takes visual input."""

    @abstractmethod
    def see(self):
        pass
    
class Audition(ABC):
    """This class takes input by listening. may be from microphone or listening feedback from other pheripheral devices. The feedback can be in the form of audio or text.  """
    @abstractmethod
    def listen(self):
        pass  
    
