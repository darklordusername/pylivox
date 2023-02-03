#std
import time
import socket
import ipaddress
from binascii import a2b_hex
#import proj
import log
from pylivox.control import general, lidar
from pylivox.control.utils import FrameFrom
from pylivox.lidar import Lidar

logger = log.getLogger(__name__)
logger.info('========== START ==========')

if __name__ == '__main__':
   Lidar('test', general.DeviceType.MID_40, (10,10,1,1))