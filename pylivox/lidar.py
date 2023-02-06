#std
import threading
import time
import socket
import ipaddress
import enum
#import proj
import log
from pylivox.control import general, lidar
from pylivox.control.frame import Frame, set_default_device_type, set_default_device_version
from pylivox.control.utils import FrameFrom
from pylivox.data import Frame as DataFrame

logger = log.getLogger(__name__)


class Lidar:
       
# 
    def __init__(self, serial:'str|bytes', model:general.DeviceType, fwv:'tuple(int,int,int,int)'):
        self.serial = serial
        general.DeviceType(model)
        self.device_type = model
        general.QueryDeviceInformationResponse(device_version=fwv, seq=0)
        self.device_version = fwv
        self.master:general.Handshake = None
        #Update libs defaults device's type,version 
        set_default_device_type(self.device_type)
        set_default_device_version(self.device_version)
        #
        self.is_spherical = False
        self.rain_fog_suppression = False
        self.fan = False
        self.return_mode = lidar.ReturnMode.SINGLE_RETURN_FIRST
        self.imu_data_push_freq = lidar.PushFrequency.FREQ_200HZ
        self.config_parameters = {general.ConfigurationParameter.Key.SWITCH_REPETITIVE_NON_REPETITIVE_SCANNING_PATTERN: False, 
                                    general.ConfigurationParameter.Key.SLOT_ID_CONFIGURATION: 0,
                                    general.ConfigurationParameter.Key.HIGH_SENSITIVITY_FUNCTION: False
        }
        self.heartbeat_time = time.time() - 5
        self.is_connected = False
        self.sampling = False
        self.state:general.WorkState.Lidar = general.WorkState.Lidar.Standby
        self.seq = 0
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.s.bind((str(ipaddress.IPv4Address("0.0.0.0")), 65000)) 
        self.s2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s2.bind(('0.0.0.0', 60001))
        self._run_broadcast()
        self._data_tx()
        self._rx()

    def _run_broadcast(self):
        def t():
            msg = general.BroadcastMsg(general.Broadcast(self.serial, 0x31), 0, self.device_type).frame
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
                data, addr = self.s.recvfrom(1500)
                self.heartbeat_time = time.time()
                frame = FrameFrom(data)
                if type(frame) is not general.Heartbeat:
                    logger.debug(f'<< {addr} {frame}')
                handler = self.HANDLERS[type(frame)]
                akn = handler(self, frame)
                if akn: 
                    self.send(akn)
                continue
            except socket.timeout:
                pass
            except KeyError as e:
                logger.warning(f'Unknown frame cmd type,set,id:{e}')
            except Exception as e:
                logger.exception(e)
            time.sleep(1)

    def _data_tx(self):
        def f():
            # s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # s.bind(('0.0.0.0', 37777))
            sleep_time = 0.1
            while True:
                try:
                    if not self.is_connected:
                        self.sampling = False
                    if self.sampling:
                        time_start = time.time()
                        frame = DataFrame().frame
                        for i in range(1000):
                            pass
                            self.s.sendto(DataFrame().frame, (str(self.master.ip), self.master.point_port))
                        sleep_time = time_start+1 - time.time()
                        sleep_time = sleep_time if sleep_time > 0 else 0
                except Exception as e:
                    logger.exception(e)
                    sleep_time = 0.1
                time.sleep(sleep_time)
        self._data_tx_thread = threading.Thread(target=f, name='data_tx', daemon=True)
        self._data_tx_thread.start()

    def send(self, frame:Frame):
        if type(frame) is general.HeartbeatResponse:
            self.s.sendto(frame.frame, (str(self.master.ip), self.master.cmd_port))
        else:
            self.s2.sendto(frame.frame, (str(self.master.ip), self.master.cmd_port))
            logger.debug(f'>> {frame}')



    def onHandshake(self, req: general.Handshake):
        if not self.is_connected:
            self.master = req
            self.heartbeat_time = time.time()
            self.is_connected = True
            return general.HandshakeResponse(req.seq)
        else:
            logger.error('No handshake expected')

    def onQueryDeviceInformation(self, req: general.QueryDeviceInformation):
        return general.QueryDeviceInformationResponse(req.seq)
    def onHeartbeat(self, req: general.Heartbeat):
        return general.HeartbeatResponse(work_state=self.state, 
                                        feature_msg=0,
                                        ack_msg=0, 
                                        seq=req.seq)
    def onStartStopSampling(self, req: general.StartStopSampling):
        logger.debug(f'sampling: {self.sampling}->{req.is_start}')
        self.sampling = req.is_start
        # self.state = general.WorkState.Lidar.Normal if self.sampling else general.WorkState.Lidar.Standby
        logger.debug(f'state:{self.state}')
        return general.StartStopSamplingResponse(req.seq)
    def onChangeCoordinateSystem(self, req: general.ChangeCoordinateSystem):
        self.is_spherical = req.is_spherical
        return general.ChangeCoordinateSystemResponse(req.seq)
    def onDisconnect(self, req: general.Disconnect):
        self.heartbeat_time = time.time() - 5
    def onConfigureStaticDynamicIp(self, req: general.ConfigureStaticDynamicIp):
        pass
    def onGetDeviceIpInformation(self, req: general.GetDeviceIpInformation):
        return general.GetDeviceIpInformationResponse(True, '192.168.222.56', '255.255.255.0', '192.168.222.1', req.seq)
    def onRebootDevice(self, req: general.RebootDevice):
        pass
    def onWriteConfigurationParameters(self, req: general.WriteConfigurationParameters):
        pass
    def onReadConfigurationParameters(self, req: general.ReadConfigurationParameters):
        params = [general.ConfigurationParameter(key, value) for key,value in self.config_parameters.items() if key in req.keys]
        result = general.ReadConfigurationParametersResponse(params[0].key,
                                                        general.ConfigurationParameter.ErrorCode.NO_ERROR,
                                                        params,
                                                        req.seq
        )
        return result



    def onSetMode(self, req: lidar.SetMode):
        self.state = general.WorkState.Lidar(req.power_mode.value)
        return lidar.SetModeResponse(result=lidar.SetModeResponse.Result.Switching, seq=req.seq)
    def onWriteLidarExtrinsicParameters(self, req: lidar.WriteLidarExtrinsicParameters):
        self.extrinsic_parameters.roll = req.roll
        self.extrinsic_parameters.yaw = req.yaw
        self.extrinsic_parameters.pitch = req.pitch
        self.extrinsic_parameters.x = req.x
        self.extrinsic_parameters.y = req.y
        self.extrinsic_parameters.z = req.z
        return lidar.WriteLidarExtrinsicParametersResponse(req.seq)
    def onReadLidarExtrinsicParameters(self, req: lidar.ReadLidarExtrinsicParameters):
        return lidar.ReadLidarExtrinsicParametersResponse(self.extrinsic_parameters.roll,
                                                          self.extrinsic_parameters.pitch,
                                                          self.extrinsic_parameters.yaw,
                                                          self.extrinsic_parameters.x,
                                                          self.extrinsic_parameters.y,
                                                          self.extrinsic_parameters.z,
                                                          req.seq)
    def onTurnOnOffRainFogSuppression(self, req: lidar.TurnOnOffRainFogSuppression):
        self.rain_fog_suppression = req.is_enable
        return lidar.TurnOnOffRainFogSuppressionResponse(req.seq)
    def onSetTurnOnOffFan(self, req: lidar.SetTurnOnOffFan):
        self.fan = req.is_enable
        return lidar.SetTurnOnOffFanResponse(req.seq)
    def onGetTurnOnOffFanState(self, req: lidar.GetTurnOnOffFanState):
        return lidar.GetTurnOnOffFanStateResponse(False, req.seq)
    def onSetLidarReturnMode(self, req: lidar.SetLidarReturnMode):
        self.return_mode = req.return_mode
        return lidar.SetLidarReturnModeResponse(req.seq)
    def onGetLidarReturnMode(self, req: lidar.GetLidarReturnMode):
        return lidar.GetLidarReturnModeResponse(self.return_mode, req.seq)
    def onSetImuDataPushFrequency(self, req: lidar.SetImuDataPushFrequency):
        self.imu_data_push_freq = req.frequency
        return lidar.SetImuDataPushFrequencyResponse(req.seq)
    def onGetImuDataPushFrequency(self, req: lidar.GetImuDataPushFrequency):
        return lidar.GetImuDataPushFrequencyResponse(lidar.PushFrequency.FREQ_200HZ, req.seq)
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
        general.ReadConfigurationParameters     : onReadConfigurationParameters,                   

        lidar.SetMode                           : onSetMode,       
        lidar.WriteLidarExtrinsicParameters     : onWriteLidarExtrinsicParameters,                             
        lidar.ReadLidarExtrinsicParameters      : onReadLidarExtrinsicParameters,                            
        lidar.TurnOnOffRainFogSuppression       : onTurnOnOffRainFogSuppression,                           
        lidar.SetTurnOnOffFan                   : onSetTurnOnOffFan,               
        lidar.GetTurnOnOffFanState              : onGetTurnOnOffFanState,                    
        lidar.SetLidarReturnMode                : onSetLidarReturnMode,                  
        lidar.GetLidarReturnMode                : onGetLidarReturnMode,                  
        lidar.SetImuDataPushFrequency           : onSetImuDataPushFrequency,                       
        lidar.GetImuDataPushFrequency           : onGetImuDataPushFrequency,                       
        # lidar.UpdateUtcSynchronizationTime      : onUpdateUtcSynchronizationTime,                            
    }