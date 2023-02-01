#std
import time
import socket
import ipaddress
from binascii import a2b_hex
#import proj
import log
from pylivox.control import general, lidar
from pylivox.control.utils import FrameFrom

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
        logger.debug('broadcast...')
        s.settimeout(0.1)
        s.sendto(broadcast_msg, (str(address), port) )
        try:
            data, addr = s.recvfrom(1024)
            handshake = FrameFrom(data)
            logger.debug(handshake)
            if type(handshake) is not general.Handshake:
                raise TypeError
            # s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 0)
            akn = general.HandshakeResponse(False).frame
            s.sendto(akn, (str(handshake.ip), handshake.cmd_port))
            logger.debug('>> akn')
            while True:
                s.settimeout(5)
                data, addr = s.recvfrom(1024)
                f = FrameFrom(data)
                logger.debug(f)
        except socket.timeout:
            pass
        except Exception as e:
            logger.exception(e)
        time.sleep(1)