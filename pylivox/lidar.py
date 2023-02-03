#std
import threading
import time
import socket
import ipaddress
import enum
#import proj
import log
from pylivox.control import general, lidar
from pylivox.control.frame import Frame
from pylivox.control.utils import FrameFrom

logger = log.getLogger(__name__)


class Lidar:
       
# 
    def __init__(self, serial:'str|bytes', model:general.DeviceType, fwv:'tuple(int,int,int,int)'):
        self.serial = serial
        general.DeviceType(model)
        self.device_type = model
        general.QueryDeviceInformationResponse(device_version=fwv)
        self.device_version = fwv
        self.master:general.Handshake = None
        #Update libs defaults device's type,version 
        general.Device_type = self.device_type
        general.Device_version = self.device_version
        #
        self.is_spherical = False
        self.extrinsic_parameters = lidar.ReadLidarExtrinsicParametersResponse(0.0, 0.0, 0.0, 0, 0, 0)
        self.heartbeat_time = time.time() - 5
        self.is_connected = False
        self.sampling = False
        self.state:general.WorkState.Lidar = general.WorkState.Lidar.Standby
        self.seq = 0
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.s.bind((str(ipaddress.IPv4Address("0.0.0.0")), 65000)) 
        self._run_broadcast()
        self._rx()

    def _run_broadcast(self):
        def t():
            msg = general.BroadcastMsg(general.Broadcast(self.serial, 0x31), self.device_type).frame
            while True:
                try:
                    time.sleep(1)
                    elapsed = time.time() - self.heartbeat_time
                    if elapsed < 5:
                        continue
                    self.is_connected = False
                    logger.debug('broadcast...')
                    self.s.sendto(msg, ('255.255.255.255', 55000) )
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
                akn = handler(self, frame)
                if akn: 
                    self.send(akn)
            except socket.timeout:
                pass
            except KeyError as e:
                logger.warning(f'Unknown frame {data.hex()} cmd_set:{data[-6]} cmd_id:{data[-5]}')
            except Exception as e:
                logger.exception(e)
            time.sleep(1)

    def send(self, frame:Frame):
        logger.debug(f'>> {frame}')
        self.s.sendto(frame.frame, (str(self.master.ip), self.master.cmd_port))

      


    def onHandshake(self, req: general.Handshake):
        if not self.is_connected:
            self.master = req
            self.heartbeat_time = time.time()
            self.is_connected = True
            return general.HandshakeResponse()
        else:
            logger.error('No handshake expected')

    def onQueryDeviceInformation(self, req: general.QueryDeviceInformation):
        return general.QueryDeviceInformationResponse()
    def onHeartbeat(self, req: general.Heartbeat):
        return general.HeartbeatResponse(work_state=self.state, 
                                        feature_msg=0,
                                        ack_msg=0)
    def onStartStopSampling(self, req: general.StartStopSampling):
        self.sampling = req.is_start
        return general.StartStopSamplingResponse()
    def onChangeCoordinateSystem(self, req: general.ChangeCoordinateSystem):
        self.is_spherical = req.is_spherical
        return general.ChangeCoordinateSystemResponse()
    def onDisconnect(self, req: general.Disconnect):
        self.heartbeat_time = time.time() - 5
    def onConfigureStaticDynamicIp(self, req: general.ConfigureStaticDynamicIp):
        pass
    def onGetDeviceIpInformation(self, req: general.GetDeviceIpInformation):
        return general.GetDeviceIpInformationResponse(True, '192.168.222.56', '255.255.255.0', '192.168.222.1')
    def onRebootDevice(self, req: general.RebootDevice):
        pass
    def onWriteConfigurationParameters(self, req: general.WriteConfigurationParameters):
        pass
    def onReadConfigurationParameters(self, req: general.ReadConfigurationParameters):
        pass



    def onSetMode(self, req: lidar.SetMode):
        pass
    def onWriteLidarExtrinsicParameters(self, req: lidar.WriteLidarExtrinsicParameters):
        self.extrinsic_parameters.roll = req.roll
        self.extrinsic_parameters.yaw = req.yaw
        self.extrinsic_parameters.pitch = req.pitch
        self.extrinsic_parameters.x = req.x
        self.extrinsic_parameters.y = req.y
        self.extrinsic_parameters.z = req.z
        return lidar.WriteLidarExtrinsicParametersResponse()
    def onReadLidarExtrinsicParameters(self, req: lidar.ReadLidarExtrinsicParameters):
        return lidar.ReadLidarExtrinsicParametersResponse(self.extrinsic_parameters.roll,
                                                          self.extrinsic_parameters.pitch,
                                                          self.extrinsic_parameters.yaw,
                                                          self.extrinsic_parameters.x,
                                                          self.extrinsic_parameters.y,
                                                          self.extrinsic_parameters.z)
    def onTurnOnOffRainFogSuppression(self, req: lidar.TurnOnOffRainFogSuppression):
        pass
    def onSetTurnOnOffFan(self, req: lidar.SetTurnOnOffFan):
        pass
    def onGetTurnOnOffFanState(self, req: lidar.GetTurnOnOffFanState):
        return lidar.GetTurnOnOffFanStateResponse(False)
    def onSetLidarReturnMode(self, req: lidar.SetLidarReturnMode):
        pass
    def onGetLidarReturnMode(self, req: lidar.GetLidarReturnMode):
        pass
    def onSetImuDataPushFrequency(self, req: lidar.SetImuDataPushFrequency):
        pass
    def onGetImuDataPushFrequency(self, req: lidar.GetImuDataPushFrequency):
        return lidar.GetImuDataPushFrequencyResponse(lidar.PushFrequency.FREQ_0HZ)
    def onUpdateUtcSynchronizationTime(self, req: lidar.UpdateUtcSynchronizationTime):
        pass
    

    HANDLERS = {
        general.Handshake                       : onHandshake, 
        general.QueryDeviceInformation          : onQueryDeviceInformation,              
        general.Heartbeat                       : onHeartbeat, 
        general.StartStopSampling               : onStartStopSampling,         
        general.ChangeCoordinateSystem          : onChangeCoordinateSystem,              
        general.Disconnect                      : onDisconnect,  
        # general.ConfigureStaticDynamicIp        : onConfigureStaticDynamicIp,                
        general.GetDeviceIpInformation          : onGetDeviceIpInformation,              
        # general.RebootDevice                    : onRebootDevice,    
        # general.WriteConfigurationParameters    : onWriteConfigurationParameters,                    
        # general.ReadConfigurationParameters     : onReadConfigurationParameters,                   

        # lidar.SetMode                           : onSetMode,       
        lidar.WriteLidarExtrinsicParameters     : onWriteLidarExtrinsicParameters,                             
        lidar.ReadLidarExtrinsicParameters      : onReadLidarExtrinsicParameters,                            
        # lidar.TurnOnOffRainFogSuppression       : onTurnOnOffRainFogSuppression,                           
        # lidar.SetTurnOnOffFan                   : onSetTurnOnOffFan,               
        lidar.GetTurnOnOffFanState              : onGetTurnOnOffFanState,                    
        # lidar.SetLidarReturnMode                : onSetLidarReturnMode,                  
        # lidar.GetLidarReturnMode                : onGetLidarReturnMode,                  
        # lidar.SetImuDataPushFrequency           : onSetImuDataPushFrequency,                       
        lidar.GetImuDataPushFrequency           : onGetImuDataPushFrequency,                       
        # lidar.UpdateUtcSynchronizationTime      : onUpdateUtcSynchronizationTime,                            
    }