import logging
import os
from typing import Optional


def setup_logger(
    name: str,
    log_file: str = "logs/app.log",
    level: int = logging.DEBUG,
    console: bool = True,
    handler_level: Optional[int] = None,
) -> logging.Logger:
    """Configure and return a module-level logger."""
    full_log_file_path = os.path.abspath(log_file)
    log_dir = os.path.dirname(full_log_file_path)
    if log_dir:
        print(log_dir)
        os.makedirs(log_dir, exist_ok=True)

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
