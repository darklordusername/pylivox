
#std
import ipaddress
import socket
#libs
#proj
from pylivox.control.frame import Frame

class Lidar:

    def __init__(self):
        self.seq = 0
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._socket.bind((str(ipaddress.IPv4Address("0.0.0.0")), 65000))

    def send(self, cmd:Frame, address:ipaddress.IPv4Address, port:int):
        self.seq += 1
        self._socket.sendto(cmd.frame, (str(address), port) )