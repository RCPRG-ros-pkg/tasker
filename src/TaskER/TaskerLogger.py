import logging

from matplotlib.style import available

class CustomFormatter(logging.Formatter):

	green = "\x1b[32m"
	grey = "\x1b[38;20m"
	yellow = "\x1b[33;20m"
	red = "\x1b[31;20m"
	bold_red = "\x1b[31;1m"
	reset = "\x1b[0m"
	format = "%(asctime)s - %(name)s (%(filename)s:%(lineno)d):\n%(message)s"

	FORMATS = {
		logging.DEBUG: green + format + reset,
		logging.INFO: grey + format + reset,
		logging.WARNING: yellow + format + reset,
		logging.ERROR: red + format + reset,
		logging.CRITICAL: bold_red + format + reset
	}

	def format(self, record):
		log_fmt = self.FORMATS.get(record.levelno)
		formatter = logging.Formatter(log_fmt)
		return formatter.format(record)

class TaskerLogger():
    def __init__(self, agent_name, log_level='info'):   
        isinstance(agent_name, str)
        available_log_levels = ['info', 'debug', 'error', 'warn']
        if log_level not in available_log_levels:
            raise Exception("TaskerLogger got unsupported log_level: '{0}', available are '{1}'".format(log_level,available_log_levels))
        # create logger with 'spam_application'
        self.logger = logging.getLogger(agent_name)
        if log_level == 'info':
            self.logger.setLevel(logging.INFO)
        elif log_level == 'debug':
            self.logger.setLevel(logging.DEBUG)
        elif log_level == 'error':
            self.logger.setLevel(logging.ERROR)
        elif log_level == 'warn':
            self.logger.setLevel(logging.WARNING)


        # create console handler with a higher log level
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)

        ch.setFormatter(CustomFormatter())

        self.logger.addHandler(ch)

    def get_logger(self):
        return self.logger
