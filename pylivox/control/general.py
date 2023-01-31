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

class IsErrorResponse:
    @property
    def is_error(self)->bool:
        return self._is_error

    @is_error.setter
    def is_error(self, value:bool):
        if type(value) is not bool:
            raise TypeError
        self._is_error = value


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

class HandshakeResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.HANDSHAKE
    _PACK_FORMAT = '<?' ##response code

    def __init__(self, is_error:bool):
        super().__init__()
        self.is_error = is_error

    @property
    def payload(self):
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error)
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

class QueryDeviceInformationResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.QUERY_DEVICE_INFORMATION
    _PACK_FORMAT = '<?I' #is_error, firmware version

    def __init__(self, is_error:bool, firmware_version:int):
        super().__init__()
        self.is_error = is_error
        self.firmware_version = firmware_version

    @property
    def payload(self):
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error, self.firmware_version)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_error, firmware_version = struct.unpack(QueryDeviceInformationResponse._PACK_FORMAT, payload)
        return QueryDeviceInformationResponse(is_error, firmware_version)

class Heartbeat(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.HEARTBEAT

    @property
    def payload(self):
        return super().payload(b'')

    @staticmethod
    def from_payload(payload:bytes):
        return Heartbeat()

class HeartbeatResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.HEARTBEAT
    _PACK_FORMAT = '<?BBI' #is_error, work_state, feature, ack_msg
    DEVICE_MODE = DEVICE_MODE

    def __init__(self, 
                    is_error:bool, 
                    work_state:'WorkState.Lidar|WorkState.Hub|int', 
                    feature_msg:int, 
                    ack_msg:int):
        super().__init__()
        self.is_error = is_error
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
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error, self.work_state, self.feature_msg, self.ack_msg )
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload: bytes):
        is_error, work_state, feature, ack_msg = struct.unpack(HeartbeatResponse._PACK_FORMAT, payload)
        return HeartbeatResponse(is_error, work_state, feature, ack_msg)

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

class StartStopSamplingResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.START_STOP_SAMPLING
    _PACK_FORMAT = '<?' #is_error

    def __init__(self, is_error:int):
        super().__init__()
        self.is_error = is_error

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error)
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

class ChangeCoordinateSystemResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.CHANGE_COORDINATE_SYSTEM
    _PACK_FORMAT = '<?' #is_error

    def __init__(self, is_error:bool):
        super().__init__()
        self.is_error = is_error

    @property
    def payload(self):
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_error, = struct.unpack(ChangeCoordinateSystemResponse._PACK_FORMAT, payload)
        return ChangeCoordinateSystemResponse(is_error)

class Disconnect(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.DISCONNECT

    @property
    def payload(self):
        return super().payload(b'')

    @staticmethod
    def from_payload(payload:bytes):
        return Disconnect()

class DisconnectResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.DISCONNECT
    _PACK_FORMAT = '<?' #is_error

    def __init__(self, is_error:bool):
        super().__init__()
        self.is_error = is_error

    @property
    def payload(self):
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_error, = struct.unpack(DisconnectResponse._PACK_FORMAT, payload)
        return DisconnectResponse(is_error)

class PushAbnormalStatusInformation(General):
    CMD_TYPE = Frame.Type.MSG
    CMD_ID = Frame.SetGeneral.PUSH_ABNORMAL_STATUS_INFORMATION
    _PACK_FORMAT = '<I' #status_code

    def __init__(self, status_code:int):
        super().__init__()
        self.status_code = status_code

    @property
    def status_code(self)->int:
        return self._status_code

    @status_code.setter
    def status_code(self, value:int):
        if type(value) is not int:
            raise TypeError
        self._status_code = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.status_code)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        status_code, = struct.unpack(PushAbnormalStatusInformation._PACK_FORMAT, payload)
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

class ConfigureStaticDynamicIpResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.CONFIGURE_STATIC_DYNAMIC_IP
    _PACK_FORMAT = '<?' # is_error

    def __init__(self, is_error:bool):
        super().__init__()
        self.is_error = is_error

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_error, = struct.unpack(ConfigureStaticDynamicIpResponse._PACK_FORMAT, payload)
        return ConfigureStaticDynamicIpResponse(is_error)

class GetDeviceIpInformation(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.GET_DEVICE_IP_INFORMATION

    @property
    def payload(self)->bytes:
        return super().payload(b'')

    @staticmethod
    def from_payload(payload):
        return GetDeviceIpInformation()

class GetDeviceIpInformationResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.GET_DEVICE_IP_INFORMATION
    _PACK_FORMAT = '<??III' #is_error, is_static, ip, mask, gw

    def __init__(self, 
                    is_error:bool, 
                    is_static:bool, 
                    ip:'ipaddress.IPv4Address|str|int', 
                    mask:'ipaddress.IPv4Address|str|int', 
                    gw:'ipaddress.IPv4Address|str|int' ):
        super().__init__()
        self.is_error = is_error
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
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error, self.is_static, int(self.ip), int(self.mask), int(self.gw))
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload):
        is_error, is_static, ip, mask, gw = struct.unpack(GetDeviceIpInformationResponse._PACK_FORMAT, payload)
        return GetDeviceIpInformationResponse(is_error, is_static, ip, mask, gw)

class RebootDevice(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.REBOOT_DEVICE
    _PACK_FORMAT = '<H' #timeout

    def __init__(self, timeout:int):
        super().__init__()
        self.timeout = timeout

    @property
    def timeout(self)->int:
        return self._timeout

    @timeout.setter
    def timeout(self, value:int):
        if type(value) is not int:
            raise TypeError
        self._timeout = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.timeout)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        timeout, = struct.unpack(RebootDevice._PACK_FORMAT, payload)
        return RebootDevice(timeout)

class RebootDeviceResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.REBOOT_DEVICE
    _PACK_FORMAT = '<?' #is_error

    def __init__(self, is_error:bool):
        super().__init__()
        self.is_error = is_error

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_error, = struct.unpack(RebootDeviceResponse._PACK_FORMAT, payload)
        return RebootDeviceResponse(is_error)

class ConfigurationParameter:

    class Key(enum.Enum):
        HIGH_SENSITIVITY_FUNCTION = 1
        SWITCH_REPETITIVE_NON_REPETITIVE_SCANNING_PATTERN = 2
        SLOT_ID_CONFIGURATION = 3
    class ErrorCode(enum.Enum):
        NO_ERROR                                = 0
        THE_KEY_IS_NOT_SUPPORTED                = 1
        EXECUTION_FAILED                        = 2        
        THE_KEY_CANNOT_BE_WRITTEN               = 3                
        WRONG_VALUE                             = 4
        WRONG_VALUE_LENGTH                      = 5        
        READING_PARAMETER_LENGTH_LIMIT          = 6                    
        THE_NUMBER_OF_PARAMETERS_DOES_NOT_MATCH = 7   

    TYPE_DIC = {
        Key.HIGH_SENSITIVITY_FUNCTION : bool,
        Key.SWITCH_REPETITIVE_NON_REPETITIVE_SCANNING_PATTERN : bool,
        Key.SLOT_ID_CONFIGURATION : int,
    }

    def __init__(self, key:'Key|int', value):
        self.key = key
        self.value = value

    def __repr__(self)->str:
        return f'ConfigParam<{self.key}:{self.value}>'

    @property
    def key(self)->Key:
        return self._key

    @key.setter
    def key(self, value:'Key|int'):
        if type(value) is int:
            value = self.Key(value)
        elif type(value) is not self.Key:
            raise TypeError
        self._key = value
        self.length = 5 # all config params in current version is 5 bytes length

    @property
    def value(self):
        return self._value
    
    @value.setter
    def value(self, value):
        T = self.TYPE_DIC[self.key]
        self._value = T(value)
    
    @property
    def payload(self)->bytes:
        return struct.pack('<HHB', self.key.value, self.length, self.value)

    @staticmethod
    def from_payload(payload:bytes)->'ConfigurationParameter':
        length, = struct.unpack('<H', payload[2:4])
        key, length, value = struct.unpack('<HHB', payload[:length])
        return ConfigurationParameter(key, value)

    @staticmethod
    def from_payload_list(payload:bytes)->'list(ConfigurationParameter)':
        result = []
        shift = 0
        while shift < len(payload):
            parameter = ConfigurationParameter.from_payload(payload[shift:])
            shift += parameter.length
            result.append(parameter)
        return result   

class WriteConfigurationParameters(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.WRITE_CONFIGURATION_PARAMETERS

    def __init__(self, param_list:'list(ConfigurationParameter)|bytes'):
        super().__init__()
        self.param_list = param_list

    @property
    def param_list(self)->'list(ConfigurationParameter)':
        return self._param_list

    @param_list.setter
    def param_list(self, value:'list(ConfigurationParameter)|bytes'):
        if type(value) is bytes:
            value = ConfigurationParameter.from_payload_list(value)
        elif type(value) is not list or [parameter for parameter in value if type(parameter) is not ConfigurationParameter]:
            raise TypeError
        self._param_list = value

    @property
    def payload(self)->bytes:
        return super().payload(b''.join([param.payload for param in self.param_list]))

    @staticmethod
    def from_payload(payload:bytes):
        return WriteConfigurationParameters(payload)

class WriteConfigurationParametersResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.WRITE_CONFIGURATION_PARAMETERS
    _PACK_FORMAT = '<?HB' #is_error, error_key, error_code

    def __init__(self,  
                    is_error:bool, 
                    error_key:'ConfigurationParameter.Key|int', 
                    error_code:'ConfigurationParameter.ErrorCode|int'):
        super().__init__()
        self.is_error = is_error
        self.error_key = error_key
        self.error_code = error_code

    @property
    def key(self)->ConfigurationParameter.Key:
        return self._key

    @key.setter
    def key(self, value:'ConfigurationParameter.Key|int'):
        if type(value) is int:
            value = ConfigurationParameter.Key(value)
        elif type(value) is not ConfigurationParameter.Key:
            raise TypeError
        self._key = value

    @property
    def error_code(self)->ConfigurationParameter.ErrorCode:
        return self._error_code
    
    @error_code.setter
    def error_code(self, value:'ConfigurationParameter.ErrorCode|int'):
        if type(value) is int:
            value = ConfigurationParameter.ErrorCode(value)
        elif type(value) is not ConfigurationParameter.ErrorCode:
            raise TypeError
        self._error_code = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error, self.error_key.value, self.error_code.value)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_error, error_key, error_code = struct.unpack(WriteConfigurationParametersResponse._PACK_FORMAT, payload)
        return WriteConfigurationParametersResponse(is_error, error_key, error_code)

class ReadConfigurationParameters(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.READ_CONFIGURATION_PARAMETERS

    def __init__(self, keys_quantity:int, keys:'list(ConfigurationParameter.Key)'):
        super().__init__()
        self.keys_quantity = keys_quantity
        self.keys = keys

    @property
    def keys_quantity(self)->int:
        return self._keys_quantity

    @keys_quantity.setter
    def keys_quantity(self, value:int):
        if type(value) is not int:
            raise TypeError
        self._keys_quantity = value

    @property
    def keys(self)->'list(ConfigurationParameter.Key)':
        return self._keys
    
    @keys.setter
    def keys(self, value:'list(ConfigurationParameter.Key)'):
        if type(value) is not list or  [key for key in value if type(key) is not ConfigurationParameter.Key]:
            raise TypeError
        self._keys = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(f'<B{self.keys_quantity}H', self.keys_quantity, *[key.value for key in self.keys])
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        keys_quantity = payload[0]
        keys_bytes = [payload[2*i+1:2*i+3] for i in range(keys_quantity)]
        keys = [ ConfigurationParameter.Key(int.from_bytes(key,'little')) for key in keys_bytes]
        return ReadConfigurationParameters(keys_quantity, keys)

class ReadConfigurationParametersResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.READ_CONFIGURATION_PARAMETERS

    def __init__(self, 
                is_error:bool, 
                error_key:'ConfigurationParameter.Key|int', 
                error_code:'ConfigurationParameter.ErrorCode|int', 
                param_list:'list(ConfigurationParameter)'):
        super().__init__()
        self.is_error = is_error
        self.error_key = error_key
        self.error_code = error_code
        self.param_list = param_list

    @property
    def error_code(self)->ConfigurationParameter.ErrorCode:
        return self._error_code
    
    @error_code.setter
    def error_code(self, value:'ConfigurationParameter.ErrorCode|int'):
        if type(value) is int:
            value = ConfigurationParameter.ErrorCode(value)
        elif type(value) is not ConfigurationParameter.ErrorCode:
            raise TypeError
        self._error_code = value

    @property
    def param_list(self)->'list(ConfigurationParameter)':
        return self._param_list

    @param_list.setter
    def param_list(self, value:'list(ConfigurationParameter)'):
        if type(value) is not list or [param for param in value if type(param) is not ConfigurationParameter]:
            raise TypeError
        self._param_list = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(f'<?HB', self.is_error, self.error_key.value, self.error_code.value) + b''.join([param.payload for param in self.param_list])
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_error, error_key, error_code = struct.unpack('<?HB', payload[:4])
        param_list = ConfigurationParameter.from_payload_list(payload[4:])
        return ReadConfigurationParametersResponse(is_error, error_key, error_code, param_list)
