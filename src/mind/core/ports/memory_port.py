from abc import ABC, abstractmethod
from typing import Any

class MemoryPort(ABC):

    @abstractmethod
    def encode(self, raw_input: Any):
        """Transform raw perception or thought into a memory trace."""
        pass

    @abstractmethod
    def create(self):
        """Create a new memory entry and return its ID."""
        pass

    @abstractmethod
    def store(self):  #may be we can make it remember rathen than store.
        """Store the memory item"""
        pass

    @abstractmethod
    def retrieve(self):
        """Retrieve the memory"""
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def delete(self):
        pass

    @abstractmethod
    def consolidate(self):
        """
        compress, abstract, or merge memories. same thing that we do while sleeping.
        """
        pass

    @abstractmethod
    def store_episodic(self, content: str, context: dict = None):
        """Store an experience or event (Episodic Memory)."""
        pass

    @abstractmethod
    def store_semantic(self, subject: str, predicate: str, object_: str):
        """Store a fact (Semantic Memory)."""
        pass

    @abstractmethod
    def retrieve_relevant(self, query: str, limit: int = 5) -> list[str]:
        """Retrieve relevant memories (Episodic or Semantic) based on query."""
        pass
