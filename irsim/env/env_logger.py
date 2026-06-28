import sys

from loguru import logger


class EnvLogger:
    def __init__(
        self, log_file: str | None = "irsim_error.log", log_level: str = "WARNING"
    ) -> None:
        logger.remove()
        logger.add(
            sys.stdout,
            level=log_level,
            format="<green>{time:YYYY-MM-DD HH:mm}</green> | "
            "<level>{level: <8}</level> | "
            "<level>{message}</level>",
        )

        if log_file is not None:
            logger.add(log_file, level=log_level)

    def info(self, msg: str) -> None:
        logger.info(msg)

    def error(self, msg: str) -> None:
        logger.error(msg)

    def debug(self, msg: str) -> None:
        logger.debug(msg)

    def warning(self, msg: str) -> None:
        logger.warning(msg)

    def critical(self, msg: str) -> None:
        logger.critical(msg)
