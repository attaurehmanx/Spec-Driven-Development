import logging
import sys
from datetime import datetime
from typing import Optional


def setup_logging(log_level: str = "INFO"):
    """
    Set up logging configuration for the application
    """
    # Create logger
    logger = logging.getLogger("task_management_api")
    logger.setLevel(getattr(logging, log_level.upper()))

    # Prevent adding multiple handlers if logger already configured
    if logger.handlers:
        return logger

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(getattr(logging, log_level.upper()))

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s'
    )
    console_handler.setFormatter(formatter)

    # Add handler to logger
    logger.addHandler(console_handler)

    return logger


# Create a global logger instance
logger = setup_logging()