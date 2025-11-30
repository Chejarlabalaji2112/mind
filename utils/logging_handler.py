import logging
import os

def setup_logger(name:str, log_file:str="logs/app.log", level=logging.DEBUG, console:bool=True):
    # Ensure the logs directory exists
    log_dir = os.path.join(os.getcwd(), os.path.dirname(log_file))
    os.makedirs(log_dir, exist_ok=True)
    full_log_file_path = os.path.join(os.getcwd(), log_file)
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Avoid adding multiple handlers if logger already configured
    if not logger.handlers:
        handler = logging.FileHandler(full_log_file_path)
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )

        file_handler = logging.FileHandler(full_log_file_path)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

        # Console handler (optional)
        if console:
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(formatter)
            logger.addHandler(console_handler)
    return logger
