from abc import ABC, abstractmethod
class Presenter(ABC):
    """This class handles showing the output visually. Uses an display provided to show the ouput."""
    @abstractmethod
    def show(self, content: str) -> bool:
        pass

class Speaker(ABC): # Inherit from ABC
    """This class handles telling the output through sound. Uses an speaker provided to tell the ouput."""
    @abstractmethod
    def say(self, content: str) -> bool:      
        pass


class Manipulator(ABC):
    """This class handles manipulating the environment. Uses some actuators provided to manipulate the environment."""
    @abstractmethod
    def do(self, setting: str, value: str) -> bool:     
        pass
