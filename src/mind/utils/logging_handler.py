import logging
import os
from typing import Optional
from mind.utils import BASE_DIR

def setup_logger(
    name: str,
    log_file: str = "app.log",
    level: int = logging.DEBUG,
    console: bool = True,
    handler_level: Optional[int] = None,
) -> logging.Logger:
    """Configure and return a module-level logger."""
    log_dir = os.path.join(BASE_DIR, "logs")
    os.makedirs(log_dir, exist_ok=True)
    full_log_file_path = os.path.join(log_dir, log_file)

    logger = logging.getLogger(name)
    logger.setLevel(level)

    if not logger.handlers:
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        file_handler = logging.FileHandler(full_log_file_path)
        file_handler.setLevel(handler_level or level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

        if console:
            console_handler = logging.StreamHandler()
            console_handler.setLevel(handler_level or level)
            console_handler.setFormatter(formatter)
            logger.addHandler(console_handler)

    return logger
