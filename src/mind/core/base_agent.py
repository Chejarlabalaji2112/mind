from abc import ABC, abstractmethod

class BaseAgent(ABC):

    @abstractmethod
    def wakeup():
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