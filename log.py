import logging 
import logging.handlers

# format = '%s|%'
FORMAT = '%(asctime)s|%(levelname)s||%(name)s||%(message)s'
formatter = logging.Formatter(FORMAT)
log_txt = logging.handlers.RotatingFileHandler('log.txt', maxBytes=1024*100, backupCount=1)
log_txt.setLevel(logging.INFO)
log_err = logging.handlers.RotatingFileHandler('err.txt', maxBytes=1024*100, backupCount=1)
log_err.setLevel(logging.ERROR)
full_log_txt = logging.handlers.RotatingFileHandler('full_log.txt', maxBytes=1024*100, backupCount=1)
full_log_txt.setLevel(logging.DEBUG)
console = logging.StreamHandler()
console.setLevel(logging.INFO)
handlers = [log_txt, log_err, full_log_txt, console]
list(map(lambda i: i.setFormatter(formatter), handlers))
logger = logging.getLogger('')
logger.setLevel(logging.DEBUG)

def getLogger(name):
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    list(map(lambda i: logger.addHandler(i), handlers))
    return logger

