from abc import ABC, abstractmethod

class BaseRobotController(ABC):

    @abstractmethod
    def wake_up():
        pass

    @abstractmethod
    def sleep():
        pass

    @abstractmethod
    def run():
        pass

    @abstractmethod
    def stop():
        pass

    @abstractmethod
    def status():
        pass

    @abstractmethod
    def shutdown():
        pass