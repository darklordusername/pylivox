#std
import time
import socket
import ipaddress
from binascii import a2b_hex
#import proj
import log
from pylivox.control import general, lidar

logger = log.getLogger(__name__)
logger.info('========== START ==========')

if __name__ == '__main__':
    broadcast = general.BroadcastMsg(
            general.Broadcast( 'test', 0), 
            general.DeviceType.Mid40)
    broadcast_msg = broadcast.frame
    seq = 0
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((str(ipaddress.IPv4Address("0.0.0.0")), 65000)) 
    address = ipaddress.IPv4Address('255.255.255.255')
    port = 55000
    while True:
        s.sendto(broadcast_msg, (str(address), port) )
        time.sleep(1)