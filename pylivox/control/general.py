#TODO Description

#std libs
from abc import abstractmethod, abstractproperty, abstractstaticmethod
import enum
import struct 
import ipaddress

#project 
from pylivox.control.frame import Frame

DEVICE_MODE = 'LIDAR' #'HUB'

class CMD(Frame):
    CMD_SET = None
    CMD_ID = None

    def payload(self, payload_body:bytes):
        format = f'<BB{len(payload_body)}B' #CMD_SET, CMD_ID, BODY
        return struct.pack(format, self.CMD_SET.value, self.CMD_ID.value, *payload_body )

class General(CMD):
    CMD_SET = Frame.Set.GENERAL


class WorkState():
    class Lidar(enum.Enum):
        Initializing    = 0
        Normal          = 0x01
        PowerSaving     = 0x02        
        Standby         = 0x03    
        Error           = 0x04
        
    class Hub(enum.Enum):
        Initializing    = 0x00            
        Normal          = 0x01
        Error           = 0x04        

# class FiatureMsg():
#     class Lidar(enum.Enum):
#         LiDAR 
#         Feature Message:
#         <br> Bit0: Rain/Fog Suppression Switch 
#         <br>0x00: Turn Off 
#         <br>0x01: Turn On 
#         <br>Bit1 ~ Bit7: Reserved 
#         <br>**Hub is Reserved** |

class DeviceType(enum.Enum):
    Hub     = 0     
    Mid40   = 1 
    Tele15  = 2
    Horizon = 3

    



class BroadcastMsg(General):
    CMD_ID = 0
    CMD_TYPE = Frame.Type.MSG

    class Broadcast():
        __PACK_FORMAT = '<14BBx' #serial, ip_range, reserved
        PACK_SIZE = struct.calcsize(__PACK_FORMAT)

        def __init__(self, serial:int, ip_range:int):
            self._serial = serial.to_bytes(14, 'little')
            self.ip_range = ip_range

        @property
        def serial(self)->int:
            return int.from_bytes(self._serial, 'little')

        @property
        def payload(self)->bytes:
            return struct.pack(self.__PACK_FORMAT, *self._serial, self.ip_range)

        @staticmethod
        def from_payload(payload:bytes)->'BroadcastMsg.Broadcast':
            serial, ip_range = struct.unpack(BroadcastMsg.Broadcast.__PACK_FORMAT, payload)
            return BroadcastMsg.Broadcast(serial, ip_range)

    __PACK_FORMAT = f'<B{Broadcast.PACK_SIZE}sB2x' #cmd_id, {broadcast}, dev_type, reserved

    def __init__(self, broadcast:'Broadcast|bytes', dev_type:'DeviceType.Enum|int' ): 
        #TODO description
        super().__init__()
        if type(broadcast) is bytes:
            broadcast = BroadcastMsg.Broadcast.from_payload(broadcast)
        elif type(broadcast) is not self.Broadcast:
            raise TypeError(f'Bad type for broadcast. Expect "Broadcast" or "bytes". Got {type(broadcast)}')
        self._broadcast = broadcast.payload
        if type(dev_type) is int:
            dev_type = DeviceType.Enum(dev_type)
        elif type(dev_type) is not DeviceType.Enum:
            raise TypeError(f'Bad type for dev_type. Expect "DeviceType" or "int". got {type(dev_type)}')
        self._dev_type = dev_type.value

    @property
    def dev_type(self)->DeviceType:
        return DeviceType(self._dev_type)

    @property
    def broadcast(self)->'BroadcastMsg.Broadcast':
        return BroadcastMsg.Broadcast.from_payload(self._broadcast)
        
    @property
    def payload(self):
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self._broadcast, self._dev_type)

    @staticmethod
    def from_payload(payload:bytes)->'BroadcastMsg':
        cmd_id, broadcast, dev_type = struct.unpack(BroadcastMsg.__PACK_FORMAT, payload)
        BroadcastMsg._check_cmd_id(cmd_id)
        return BroadcastMsg(broadcast, dev_type)

class Handshake(General):
    #TODO description
    CMD_ID = Frame.SetGeneral.HANDSHAKE
    CMD_TYPE = Frame.Type.CMD
    _PACK_FORMAT = '<IHHH' #ip, poin_port, cmd_port, imu_port 

    def __init__(self, ip:ipaddress.IPv4Address, point_port:int, cmd_port:int, imu_port:int):
        #TODO description
        super().__init__()
        self.ip = ip
        self.point_port = point_port
        self.cmd_port = cmd_port
        self.imu_port = imu_port

    @property
    def payload(self):
        payload_body = struct.pack(self._PACK_FORMAT, int(self.ip), self.point_port, self.cmd_port, self.imu_port )
        return super().payload(payload_body)

    @property
    def ip(self)->ipaddress.IPv4Address:
        return self._ip
    
    @ip.setter
    def ip(self, val:'ipaddress.IPv4Address|str'):
        if type(val) is str:
            val = ipaddress.IPv4Address(val)
        elif type(val) is not ipaddress.IPv4Address:
            raise TypeError
        self._ip = val
    
    @property
    def cmd_port(self)->int:
        return self._cmd_port

    @cmd_port.setter
    def cmd_port(self, val:int):
        if val < 0 or val > 2**16:
            raise ValueError
        self._cmd_port = val

    @property 
    def imu_port(self)->int:
        return self._imu_port

    @imu_port.setter
    def imu_port(self, val:int):
        if val < 0 or val > 2**16:
            raise ValueError
        self._imu_port = val

    @staticmethod
    def from_payload(payload:bytes):
        ip_int, point_port, cmd_port, imu_port = struct.unpack(Handshake._PACK_FORMAT, payload)
        return Handshake(ipaddress.IPv4Address(ip_int), point_port, cmd_port, imu_port)

class HandshakeResponse(General):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.HANDSHAKE
    _PACK_FORMAT = '<?' ##response code

    def __init__(self, result:bool):
        super().__init__()
        self.response = result

    @property
    def payload(self):
        payload_body = struct.pack(self._PACK_FORMAT, self.response)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload: bytes):
        code, = struct.unpack(HandshakeResponse._PACK_FORMAT, payload)
        return HandshakeResponse(code)
    
class QueryDeviceInformation(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.QUERY_DEVICE_INFORMATION

    @property
    def payload(self):
        return super().payload(b'')
    
    @staticmethod
    def from_payload(payload:bytes):
        return QueryDeviceInformation()

class QueryDeviceInformationResponse(General):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.QUERY_DEVICE_INFORMATION
    _PACK_FORMAT = '<?I' #result, firmware version

    def __init__(self, result:bool, firmware_version:int):
        super().__init__()
        self.result = bool(result)
        self.firmware_version = firmware_version

    @property
    def payload(self):
        payload_body = struct.pack(self._PACK_FORMAT, self.result, self.firmware_version)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        result, firmware_version = struct.unpack(QueryDeviceInformationResponse._PACK_FORMAT, payload)
        return QueryDeviceInformationResponse(result, firmware_version)

class Heartbeat(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.HEARTBEAT

    @property
    def payload(self):
        return super().payload(b'')

    @staticmethod
    def from_payload(payload:bytes):
        return Heartbeat()

class HeartbeatResponse(General):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.HEARTBEAT
    _PACK_FORMAT = '<?BBI' #response, work_state, feature, ack_msg
    DEVICE_MODE = DEVICE_MODE

    def __init__(self, 
                    result:bool, 
                    work_state:'WorkState.Lidar|WorkState.Hub|int', 
                    feature_msg:int, 
                    ack_msg:int):
        super().__init__()
        self.result = result
        self.work_state = work_state
        self.feature_msg = feature_msg
        self.ack_msg = ack_msg

    @property
    def work_state(self)->int:
        return self._work_state.value

    @work_state.setter
    def work_state(self, value:'WorkState.Lidar|WorkState.Hub|int'):
        """Depend on DEVICE_MODE 

        Args:
            value (WorkState.Lidar|WorkState.Hub|int): _description_
        """
        if type(value) is int:
            if self.DEVICE_MODE == 'LIDAR':
                value = WorkState.Lidar(value)
            elif self.DEVICE_MODE == 'HUB':
                value = WorkState.Hub(value)
            else:
                raise ValueError
        elif type(value) is not WorkState.Lidar and type(value) is not WorkState.Hub:
            raise TypeError
        self._work_state = value

    @property
    def feature_msg(self)->int:
        return self._feature_msg

    @feature_msg.setter
    def feature_msg(self, value:int):
        if type(value) is not int:
            raise TypeError
        value.to_bytes(1, 'little')
        self._feature_msg = value

    @property
    def ack_msg(self)->int:
        return self._ack_msg
    
    @ack_msg.setter
    def ack_msg(self, value:int):
        if type(value) is not int:
            raise TypeError
        value.to_bytes(4, 'little')
        self._ack_msg = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.result, self.work_state, self.feature_msg, self.ack_msg )
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload: bytes):
        response, work_state, feature, ack_msg = struct.unpack(HeartbeatResponse._PACK_FORMAT, payload)
        return HeartbeatResponse(response, work_state, feature, ack_msg)

class StartStopSampling(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.START_STOP_SAMPLING
    _PACK_FORMAT = '<?' #cmd_id, is_start

    def __init__(self, is_start:bool):
        super().__init__()
        self.is_start = is_start
    
    @property
    def is_start(self)->bool:
        return self._is_start

    @is_start.setter
    def is_start(self, value:bool):
        if type(value) is not bool:
            raise TypeError
        self._is_start = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_start)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_start,  = struct.unpack(StartStopSampling._PACK_FORMAT, payload)
        return StartStopSampling(is_start)

class StartStopSamplingResponse(General):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.START_STOP_SAMPLING
    _PACK_FORMAT = '<?' #result

    def __init__(self, result:int):
        super().__init__()
        self.result = result

    @property
    def result(self)->bool:
        return self._result
    
    @result.setter
    def result(self, value:bool):
        if type(value) is not bool:
            raise TypeError
        self._result = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.result)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload: bytes):
        response, = struct.unpack(StartStopSamplingResponse._PACK_FORMAT, payload)
        return StartStopSamplingResponse(response)

class ChangeCoordinateSystem(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.CHANGE_COORDINATE_SYSTEM
    _PACK_FORMAT = '<?' #Is_Spherical_Coordinate?

    def __init__(self, is_spherical:bool):
        super().__init__()
        self.is_spherical = is_spherical

    @property
    def is_spherical(self)->bool:
        return self._is_spherical

    @is_spherical.setter
    def is_spherical(self, value:bool):
        if type(value) is not bool:
            raise TypeError
        self._is_spherical = value
    
    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_spherical)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_spherical, = struct.unpack(ChangeCoordinateSystem._PACK_FORMAT, payload)
        return ChangeCoordinateSystem(is_spherical)

class ChangeCoordinateSystemResponse(General):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.CHANGE_COORDINATE_SYSTEM
    _PACK_FORMAT = '<?' #result

    def __init__(self, result:bool):
        super().__init__()
        self.result = result
    
    @property
    def result(self)->bool:
        return self._result

    @result.setter
    def result(self, value:bool):
        if type(value) is not bool:
            raise TypeError
        self._result = value

    @property
    def payload(self):
        payload_body = struct.pack(self._PACK_FORMAT, self.result)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        result, = struct.unpack(ChangeCoordinateSystemResponse._PACK_FORMAT, payload)
        return ChangeCoordinateSystemResponse(result)

class Disconnect(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.DISCONNECT

    @property
    def payload(self):
        return super().payload(b'')

    @staticmethod
    def from_payload(payload:bytes):
        return Disconnect()

class DisconnectResponse(General):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.DISCONNECT
    _PACK_FORMAT = '<?' #result

    def __init__(self, result:bool):
        super().__init__()
        self.result = result

    @property
    def result(self)->bool:
        return self._result

    @result.setter
    def result(self, value:bool):
        if type(value) is not bool:
            raise TypeError
        self._result = value

    @property
    def payload(self):
        payload_body = struct.pack(self._PACK_FORMAT, self.result)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        response, = struct.unpack(DisconnectResponse._PACK_FORMAT, payload)
        return DisconnectResponse(response)

class PushAbnormalStatusInformation(General):
    CMD_ID = 0x07
    FRAME_TYPE = Frame.Type.MSG
    __PACK_FORMAT = '<BI' #cmd_id, status_code

    def __init__(self, status_code:int):
        super().__init__()
        self._status_code = status_code.to_bytes(4, 'little')

    @property
    def status_code(self):
        return int.from_bytes(self._status_code, 'little')

    @property
    def payload(self):
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self._status_code)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, status_code = struct.unpack(PushAbnormalStatusInformation.__PACK_FORMAT, payload)
        PushAbnormalStatusInformation._check_cmd_id(cmd_id)
        return PushAbnormalStatusInformation(status_code)

class ConfigureStaticDynamicIp(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.CONFIGURE_STATIC_DYNAMIC_IP
    _PACK_FORM = '<?III' #is_static, ip_addr, net_mask, gw

    def __init__(self, 
                    is_static:bool, 
                    ip:'ipaddress.IPv4Address|int|str', 
                    mask:'ipaddress.IPv4Address|int|str',
                    gw:'ipaddress.IPv4Address|int|str'):
        super().__init__()
        self.is_static = is_static
        self.ip = ip
        self.mask = mask
        self.gw = gw

    @property
    def is_static(self)->bool:
        return self._is_static

    @is_static.setter
    def is_static(self, value:bool):
        if type(value) is not bool:
            raise TypeError
        self._is_static = value

    @property
    def ip(self)->ipaddress.IPv4Address:
        return self._ip

    @ip.setter
    def ip(self, value:'ipaddress.IPv4Address|int|str'):
        if type(value) is int or type(value) is str:
            value = ipaddress.IPv4Address(value)
        elif type(value) is not ipaddress.IPv4Address:
            raise TypeError
        self._ip = value
    
    @property
    def mask(self)->ipaddress.IPv4Address:
        return self._mask

    @mask.setter
    def mask(self, value:'ipaddress.IPv4Address|int|str'):
        if type(value) is int or type(value) is str:
            value = ipaddress.IPv4Address(value)
        elif type(value) is not ipaddress.IPv4Address:
            raise TypeError
        self._mask = value

    @property
    def gw(self)->ipaddress.IPv4Address:
        return self._gw

    @gw.setter
    def gw(self, value:'ipaddress.IPv4Address|int|str'):
        if type(value) is int or type(value) is str:
            value = ipaddress.IPv4Address(value)
        elif type(value) is not ipaddress.IPv4Address:
            raise TypeError
        self._gw = value

    @property
    def is_static(self)->bool:
        return self._is_static

    @is_static.setter
    def is_static(self, value:bool):
        if type(value) is not bool:
            raise TypeError
        self._is_static = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORM, self.is_static, int(self.ip), int(self.mask), int(self.gw))
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_static, ip, mask, gw = struct.unpack(ConfigureStaticDynamicIp._PACK_FORM, payload)
        return ConfigureStaticDynamicIp(is_static, ip, mask, gw)

class ConfigureStaticDynamicIpResponse(General):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.CONFIGURE_STATIC_DYNAMIC_IP
    _PACK_FORMAT = '<?' # result

    def __init__(self, result:bool):
        super().__init__()
        self.result = result

    @property 
    def result(self)->bool:
        return self._result

    @result.setter
    def result(self, value:bool):
        if type(value) is not bool:
            raise TypeError
        self._result = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.result)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        result, = struct.unpack(ConfigureStaticDynamicIpResponse._PACK_FORMAT, payload)
        return ConfigureStaticDynamicIpResponse(result)

class GetDeviceIpInformation(General):
    CMD_ID = 0x09
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #cmd_id

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID)

    @staticmethod
    def from_payload(payload):
        cmd_id = struct.unpack(GetDeviceIpInformation.__PACK_FORMAT, payload)
        GetDeviceIpInformation._check_cmd_id(cmd_id)
        return GetDeviceIpInformation()

class GetDeviceIpInformationResponse(GetDeviceIpInformation):
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<BBBIII' #cmd_id, result, ip_mode, ip, mask, gw

    def __init__(self, result:bool, is_static:bool, 
                    ip:'ipaddress.IPv4Address|str|int', mask:'ipaddress.IPv4Address|str|int', gw:'ipaddress.IPv4Address|str|int' ):
        super().__init__()
        # if type(result) is not bool:
        #     raise TypeError(f'Bad type for result. Expect "bool". Got {type(result)}')
        # self.result = result
        # if type(ip_mode) is int:
        #     ip_mode = IpMode(ip_mode)
        # elif type(ip_mode) is not IpMode:
        #     raise TypeError(f'Bad type for ip_mode. Expect "IpMode" or "int". Got {type(ip_mode)}')
        # self._ip_mode = ip_mode.value
        # if type(ip) is int or type(ip) is str:
        #     ip = ipaddress.IPv4Address(ip)
        # elif type(ip) is not ipaddress.IPv4Address:
        #     raise TypeError(f'Bad type for ip. Expect "IPv4Address" or "int" or "str". Got {type(ip)}')
        # self._ip = ip
        # if type(mask) is int or type(mask) is str:
        #     mask = ipaddress.IPv4Address(mask)
        # elif type(mask) is not ipaddress.IPv4Address:
        #     raise TypeError(f'Bad type for mask. Expect "IPv4Address" or "int" or "str". Got {type(mask)}')
        # self._mask = mask 
        # if type(gw) is int or type(gw) is str:
        #     gw = ipaddress.IPv4Address(gw)
        # elif type(gw) is not ipaddress.IPv4Address:
        #     raise TypeError(f'Bad type for gw. Expect "IPv4Address" or "int" or "str". Got {type(gw)}')
        # self._gw = gw

    # @property
    # def ip_mode(self)->IpMode:
    #     return IpMode(self._ip_mode)
    
    @property
    def ip(self)->ipaddress.IPv4Address:
        return ipaddress.IPv4Address(self._ip)
    
    @property
    def mask(self)->ipaddress.IPv4Address:
        return ipaddress.IPv4Address(self._mask)

    @property
    def gw(self)->ipaddress.IPv4Address:
        return ipaddress.IPv4Address(self._gw)

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.result, self._ip_mode, self._ip, self._mask, self._gw)

    @staticmethod
    def from_payload(payload):
        cmd_id, result, ip_mode, ip, mask, gw = struct.unpack(GetDeviceIpInformationResponse.__PACK_FORMAT, payload)
        GetDeviceIpInformationResponse._check_cmd_id(cmd_id)
        return GetDeviceIpInformation(result, ip_mode, ip, mask, gw)

class RebootDevice(General):
    CMD_ID = 0x0a
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<BH' #cmd_id, timeout

    def __init__(self, timeout:int):
        super().__init__()
        self.timeout = timeout

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.timeout)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, timeout = struct.unpack(RebootDevice.__PACK_FORMAT, payload)
        RebootDevice._check_cmd_id(cmd_id)
        return RebootDevice(timeout)

class RebootDeviceResponse(RebootDevice):
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<B?' #cmd_id, result

    def __init__(self, result:bool):
        super().__init__()
        self.result = result

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.result)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, result = struct.unpack(RebootDeviceResponse.__PACK_FORMAT, payload)
        RebootDeviceResponse._check_cmd_id(cmd_id)
        return RebootDeviceResponse(result)

class WriteConfigurationParameters(General):
    CMD_ID = 0x0b
    FRAME_TYPE = Frame.Type.CMD

    def __init__(self):
        super().__init__()
        raise NotImplementedError

class WriteConfigurationParametersResponse(WriteConfigurationParameters):
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<B?HB' #cmd_id, result, error_key, error_code

    class ErrorCode(enum.Enum):
        no_error                                = enum.auto()
        the_key_is_not_supported                = enum.auto()
        execution_failed                        = enum.auto()        
        the_key_cannot_be_written               = enum.auto()                
        wrong_value                             = enum.auto()
        wrong_value_length                      = enum.auto()        
        reading_parameter_length_limit          = enum.auto()                    
        the_number_of_parameters_does_not_match = enum.auto()                            

    def __init__(self, result:bool, error_key:int, error_code:'ErrorCode|int'):
        super().__init__()
        self.result = result
        self.error_key = error_key
        if type(error_code) is int:
            error_code = self.ErrorCode(error_code)
        elif type(error_code) is not self.ErrorCode:
            raise TypeError(f'Bad type for error_code. Expect "ErrorCode" or "int". Got {type(error_code)}') 
        self._error_code = error_code.value

    @property
    def error_code(self)->ErrorCode:
        return self.ErrorCode(self._error_code)

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.result, self.error_key, self.error_code)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, result, error_key, error_code = struct.unpack(WriteConfigurationParametersResponse.__PACK_FORMAT, payload)
        WriteConfigurationParametersResponse._check_cmd_id(cmd_id)
        return WriteConfigurationParametersResponse(result, error_key, error_code)


class ReadConfigurationParameters(General):
    CMD_ID = 0x0C
    FRAME_TYPE = Frame.Type.CMD
    # __PACK_FORMAT = '<B'
    