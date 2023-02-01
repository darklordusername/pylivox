#std
import threading
import time
import socket
import ipaddress
#import proj
import log
from pylivox.control import general, lidar
from pylivox.control.frame import Frame
from pylivox.control.utils import FrameFrom

logger = log.getLogger(__name__)


class Lidar:

    def __init__(self, serial:'str|bytes'):
        self.master:general.Handshake = None
        self.heartbeat_time = time.time() - 5
        self.is_connected = False
        self.state:general.WorkState.Lidar = general.WorkState.Lidar.Standby
        self.serial = serial
        self.seq = 0
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.s.bind((str(ipaddress.IPv4Address("0.0.0.0")), 65000)) 
        self._broadcast()
        self._rx()

    def _broadcast(self):
        def t():
            while True:
                try:
                    time.sleep(1)
                    elapsed = time.time() - self.heartbeat_time
                    if elapsed < 5:
                        continue
                    self.is_connected = False
                    logger.debug('broadcast...')
                    broadcast = general.BroadcastMsg(
                            general.Broadcast( self.serial, 0), 
                            general.DeviceType.Mid40).frame
                    self.s.sendto(broadcast, ('255.255.255.255', 55000) )
                except Exception as e:
                    logger.exception(e)
        self._broadcast_thread = threading.Thread(target=t, name='broadcast', daemon=True)
        self._broadcast_thread.start()
    
    def _rx(self):
        while True:
            try:
                data, addr = self.s.recvfrom(1024*2)
                self.heartbeat_time = time.time()
                frame = FrameFrom(data)
                logger.info(f'<< {addr} {frame}')
                handler = self.HANDLERS[type(frame)]
                handler(self, frame)
            except socket.timeout:
                pass
            except KeyError as e:
                logger.warning(f'Unknown frame {data} cmd_set:{data[-6]} cmd_id:{data[-5]}')
            except Exception as e:
                logger.exception(e)
            time.sleep(1)

    def send(self, frame:Frame):
        self.s.sendto(frame.frame, (str(self.master.ip), self.master.cmd_port))

    def onHandshake(self, handshake:general.Handshake):
        if not self.is_connected:
            self.master = handshake
            akn = general.HandshakeResponse(False)
            self.heartbeat_time = time.time()
            self.is_connected = True
            self.send(akn)

    def onHeartbeat(self, heartbeat:general.Heartbeat):
        # self.heartbeat_time = time.time()   
        akn = general.HeartbeatResponse(is_error=False, 
                                        work_state=self.state, 
                                        feature_msg=0,
                                        ack_msg=0)
        self.send(akn)

    def onDisconnect(self, disconnect:general.Disconnect):
        pass

    def onQueryDeviceInformation(self, info:general.QueryDeviceInformation):
        pass

    def onReadLidarExtrinsicParameters(self, req:lidar.ReadLidarExtrinsicParameters):
        pass

    def onGetDeviceIpInformation(self, ip_info:general.GetDeviceIpInformation):
        pass

    def onGetTurnOnOffFanState(self, req:lidar.GetTurnOnOffFanState):
        pass

    def onGetImuDataPushFrequency(self, req:lidar.GetImuDataPushFrequency):
        pass


    HANDLERS = {
        general.Handshake                     : onHandshake                    ,                                        
        general.Heartbeat                     : onHeartbeat                    ,                
        general.Disconnect                    : onDisconnect                   ,                                    
        general.QueryDeviceInformation        : onQueryDeviceInformation       ,                                    
        lidar.ReadLidarExtrinsicParameters    : onReadLidarExtrinsicParameters ,                                    
        general.GetDeviceIpInformation        : onGetDeviceIpInformation       ,                                    
        lidar.GetTurnOnOffFanState            : onGetTurnOnOffFanState         ,                                    
        lidar.GetImuDataPushFrequency         : onGetImuDataPushFrequency      ,                                    
    }