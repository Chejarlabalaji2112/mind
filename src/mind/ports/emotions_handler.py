from abc import ABC, abstractmethod

class EmotionsHandler(ABC):
    @abstractmethod
    def handle_emotions(self)