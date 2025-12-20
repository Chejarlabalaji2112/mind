from abc import ABC, abstractmethod
from re import A
from typing import Optional

class NotificationPort(ABC):
    """Port for robot notifications (e.g., status updates, alerts)."""
    
    @abstractmethod
    async def notify_status(self, main_text: str, sub_text: Optional[str] = None) -> None:
        """Show a status message (e.g., 'SYSTEM ONLINE')."""
        pass

    @abstractmethod
    async def notify_content(self, url: str) -> None:
        """Show content on the robot's screen (e.g., image URL)."""
        pass

    @abstractmethod
    async def notify_clear_display(self) -> None:
        """Clear the robot's screen."""
        pass