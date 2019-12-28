import logging


class CustomFormatter(logging.Formatter):
    """Logging Formatter to add colors and count warning / errors"""

    grey = "\x1b[38;21m"
    # yellow = "\x1b[33;21m"
    yellow = "\x1b[33;21m"  # produces ugly underlines
    red = "\x1b[31;21m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = "%(asctime)s - %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"

    COLOR_RED = "\033[0;31m"
    COLOR_YELLOW = "\033[0;33m"
    COLOR_BOLD_YELLOW = "\033[1;33m"
    COLOR_GREEN = "\033[0;32m"
    COLOR_OCHRE = "\033[38;5;95m"
    COLOR_BLUE = "\033[0;34m"
    COLOR_WHITE = "\033[0;37m"
    COLOR_RESET = "\033[0m"

    FORMATS = {
        # logging.DEBUG: grey + format + reset,
        logging.DEBUG: format + reset,
        logging.INFO: COLOR_GREEN + format + reset,
        logging.WARNING: COLOR_BOLD_YELLOW + format + reset,
        logging.ERROR: COLOR_RED + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


# if sys.stderr.isatty():
    # logging.addLevelName(
        # logging.WARNING, "\033[1;31m%s\033[1;0m" % logging.getLevelName(logging.WARNING))
    # logging.addLevelName(
        # logging.ERROR, "\033[1;41m%s\033[1;0m" % logging.getLevelName(logging.ERROR))
