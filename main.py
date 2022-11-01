import time
import pylivox
import log
logger = log.getLogger(__name__)

logger.info('========== START ==========')

if __name__ == '__main__':
    lidar = pylivox.Lidar()
    time.sleep(60000)