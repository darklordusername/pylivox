#TODO Description

#std libs
from abc import abstractmethod, abstractproperty, abstractstaticmethod
import enum
import struct 
import ipaddress

#project 
from pylivox.control.frame import Frame

class CMD(Frame):
    CMD_SET = None
    CMD_ID = None

    def payload(self, payload_body:bytes):
        format = f'<BB{len(payload_body)}B' #CMD_SET, CMD_ID, BODY
        return struct.pack(format, self.CMD_SET.value, self.CMD_ID.value, *payload_body )

class General(CMD):
    CMD_SET = Frame.Set.GENERAL


class WorkState():
    class Lidar:
        class Enum(enum.Enum):
            Initializing    = 0
            Normal          = 0x01
            PowerSaving     = 0x02        
            Standby         = 0x03    
            Error           = 0x04
        
        def _work_state_pars(self, work_state:'Enum|int' ):
            if type(work_state) is int:
                work_state = self.Enum(work_state)
            elif type(work_state) is not self.Enum:
                raise TypeError(f'Bad type for work_state. Expect "Lidar.Enum" or "int". got {type(work_state)}')
            self._work_state = work_state.value

        @property
        def work_state(self)->Enum:
            return self.Enum(self._work_state)
        
        
    class Hub(Lidar):
        class Enum(enum.Enum):
            Initializing    = 0x00            
            Normal          = 0x01
            Error           = 0x04        

class DeviceType():
    class Enum(enum.Enum):
        Hub     = 0     
        Mid40   = 1 
        Tele15  = 2
        Horizon = 3

    def _device_type_pars(self, device_type:'Enum|int'):
        if type(device_type) is int:
            device_type = DeviceType.Enum(device_type)
        elif type(device_type) is not DeviceType.Enum:
            raise TypeError(f'Bad type for dev_type. Expect "DeviceType" or "int". Got {type(device_type)}')
        self._device_type = device_type
    
    @property
    def device_type(self)-> 'DeviceType.Enum':
        return self.Enum(self._device_type)



class BroadcastMsg(General, DeviceType):
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
        code = struct.unpack(HandshakeResponse._PACK_FORMAT, payload)
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
    CMD_ID = 0x03
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #cmd_id

    def __init__(self):
        super().__init__()

    @property
    def payload(self):
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(Heartbeat.__PACK_FORMAT, payload)
        Heartbeat._check_cmd_id(cmd_id)
        return Heartbeat()

class LidarFeature:
    def __init__(self, rain_fog_suppression:bool):
        self.rain_fog_suppresion = rain_fog_suppression

    @property
    def pack(self)->bytes:
        result = 0
        result = result | 1 if self.rain_fog_suppresion else result & ~1
        return result.to_bytes(1, 'little')

    def _lidarFeature_pars(self, nval:'LidarFeature|int'):
        if type(nval) is int:
            nval = LidarFeature(nval)
        elif type(nval) is not LidarFeature:
            raise TypeError('Bad type for LidarFeature')
        self._lidarFeature = nval

class HeartbeatResponse(Heartbeat, WorkState.Lidar, LidarFeature):
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<B?BBI' #cmd_id, response, work_state, feature, ack_msg


    def __init__(self, response:bool, work_state:'WorkState.Lidar|WorkState.Hub|int', feature:'LidarFeature|int', ack_msg:int):
        #TODO description
        super().__init__()
        self.response = response
        self._work_state_pars(work_state)
        self._feature_parse

        if type(feature) is int:
            feature = self.Feature.Lidar(feature)
        self._feature = feature.pack
        self._ack_msg  = ack_msg.to_bytes(4, 'little')    


    @property
    def ack_msg(self):
        return int.from_bytes(self._ack_msg, 'little')
    

    @property
    def payload(self):
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self._response, self._work_state, self._feature, self._ack_msg )

    @staticmethod
    def from_payload(payload: bytes):
        cmd_id, response, work_state, feature, ack_msg = struct.unpack(HeartbeatResponse.__PACK_FORMAT, payload)
        HeartbeatResponse._check_cmd_id(cmd_id)
        return HeartbeatResponse(response, work_state, feature, ack_msg)

class StartStopSampling(General):
    CMD_ID = 0x04
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B?' #cmd_id, is_start

    def __init__(self, is_start:bool):
        super().__init__()
        self.is_start = is_start

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.is_start)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, is_start = struct.unpack(StartStopSampling.__PACK_FORMAT, payload)
        StartStopSampling._check_cmd_id(cmd_id)
        return StartStopSampling(bool(is_start))

class StartStopSamplingResponse(StartStopSampling):
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<BB' #cmd_id, response

    def __init__(self, response:int):
        super().__init__()
        self._response = response.to_bytes(1, 'little')

    @property
    def response(self)->int:
        return int.from_bytes(self._response, 'little')

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self._response)

    @staticmethod
    def from_payload(payload: bytes):
        cmd_id, response = struct.unpack(StartStopSamplingResponse.__PACK_FORMAT, payload)
        StartStopSamplingResponse._check_cmd_id(cmd_id)
        return StartStopSamplingResponse(response)

class ChangeCoordinateSystem(General):
    CMD_ID = 0x05
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<BB' #cmd_id, coordinate_system

    class CoordinateSystem(enum.Enum):
        Cartesian = 0
        Spherical = 1

    def __init__(self, coordinate_system:'CoordinateSystem|int'):
        super().__init__()
        if type(coordinate_system) is int:
            coordinate_system = self.CoordinateSystem(coordinate_system) 
        self._coordinate_system = coordinate_system

    @property
    def coordinate_system(self)->CoordinateSystem:
        return self.CoordinateSystem(self._coordinate_system)
    
    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self._coordinate_system)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, coordinate_system = struct.unpack(ChangeCoordinateSystem.__PACK_FORMAT, payload)
        ChangeCoordinateSystem._check_cmd_id(cmd_id)
        return ChangeCoordinateSystem(coordinate_system)

class ChangeCoordinateSystemResponse(ChangeCoordinateSystem):
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<B?' #cmd_id, response

    def __inti__(self, response:bool):
        super().__init__()
        self.response = response

    @property
    def payload(self):
        return struct.pack(self.__PACK_FORMAT, self.response)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, response = struct.unpack(ChangeCoordinateSystemResponse.__PACK_FORMAT, payload)
        ChangeCoordinateSystemResponse._check_cmd_id(cmd_id)
        return ChangeCoordinateSystemResponse(response)

class Disconnect(General):
    CMD_ID = 0x06
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #cmd_id

    def __init__(self):
        super().__init__()

    @property
    def payload(self):
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(Disconnect.__PACK_FORMAT, payload)
        Disconnect._check_cmd_id(cmd_id)
        return Disconnect()

class DisconnectResponse(Disconnect):
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<B?' #cmd_id, response

    def __init__(self, response:bool):
        super().__init__()
        self.response = response

    @property
    def payload(self):
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.response)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, response = struct.unpack(DisconnectResponse.__PACK_FORMAT, payload)
        DisconnectResponse._check_cmd_id(cmd_id)
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

class IpMode(enum.Enum):
    Dynamic = 0
    Static = 1

class ConfigureStaticDynamicIp(General):
    CMD_ID = 0x08
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORM = '<BBII' #cmd_id, ip_mode, ip_addr, net_mask

    def __init__(self, ip_mode:'IpMode|int', ip:'ipaddress.IPv4Address|int|str', mask:'ipaddress.IPv4Address|int|str'):
        super().__init__()
        if type(ip_mode) is int:
            ip_mode = IpMode(ip_mode)
        elif type(ip_mode) is not IpMode:
            raise TypeError(f'Bad type for ip_mode. Expected "IpMode" or "int". Got {type(ip_mode)}')
        self._ip_mode = ip_mode.value
        if type(ip) is str or type(ip) is int:
            ip = ipaddress.IPv4Address(ip)
        elif type(ip) is not ipaddress.IPv4Address:
            raise TypeError(f'Bad tpe of ip. Expected "str" or "IPv4Address" or "int". Got {type(ip)}')
        self._ip = int(ip)
        if type(mask) is str or type(mask) is int:
            mask = ipaddress.IPv4Address(mask)
        elif type(mask) is not ipaddress.IPv4Address:
            raise TypeError(f'Bad tpe of mask. Expected "str" or "IPv4Address" or "int". Got {type(mask)}')
        self._ip = int(mask)

    @property
    def ip_mode(self)->IpMode:
        return IpMode(self._ip_mode)

    @property
    def ip(self)->ipaddress.IPv4Address:
        return ipaddress.IPv4Address(self._ip)

    @property
    def mask(self)->ipaddress.IPv4Address:
        return ipaddress.IPv4Address(self.mask)

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORM, self.CMD_ID, self._ip_mode, self._ip, self.mask)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, ip_mode, ip, mask = struct.unpack(ConfigureStaticDynamicIp.__PACK_FORM, payload)
        ConfigureStaticDynamicIp._check_cmd_id(cmd_id)
        return ConfigureStaticDynamicIp(ip_mode, ip, mask)

class ConfigureStaticDynamicIpResponse(ConfigureStaticDynamicIp):
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<BBIII' #cmd_id, ip_mode, ip, mask, gw

    def __init__(self, ip_mode:'IpMode|int', ip:'ipaddress.IPv4Address|int|str', mask:'ipaddress.IPv4Address|int|str', gw:'ipaddress.IPv4Address|int|str'):
        super().__init__()
        if type(ip_mode) is int:
            ip_mode = IpMode(ip_mode)
        elif type(ip_mode) is not IpMode:
            raise TypeError(f'Bad type of ip_mode. Expected "IpMode" or "str". Got {type(ip_mode)}')
        self._ip_mode = ip_mode.value
        if type(ip) is int or type(ip) is str:
            ip = ipaddress.IPv4Address(ip)
        elif type(ip) is not ipaddress.IPv4Address:
            raise TypeError(f'Bad type of ip. Expected "IPv4Address" or "int" or "str". Got {type(ip)} ')
        self._ip = int(ip)
        if type(mask) is int or type(mask) is str:
            mask = ipaddress.IPv4Address(mask)
        elif type(mask) is not ipaddress.IPv4Address:
            raise TypeError(f'Bad type of mask. Expected "IPv4Address" or "int" or "str". Got {type(mask)} ')
        self._mask = int(mask)
        if type(gw) is int or type(gw) is str:
            gw = ipaddress.IPv4Address(gw)
        elif type(gw) is not ipaddress.IPv4Address:
            raise TypeError(f'Bad type of gw. Expected "IPv4Address" or "int" or "str". Got {type(gw)} ')
        self._gw = int(gw)
    
    @property
    def ip_mode(self)->IpMode:
        return IpMode(self._ip_mode)

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
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self._ip_mode, self._ip, self._mask, self._gw)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, ip_mode, ip, mask, gw = struct.unpack(ConfigureStaticDynamicIpResponse.__PACK_FORMAT, payload)
        ConfigureStaticDynamicIpResponse._check_cmd_id(cmd_id)
        return ConfigureStaticDynamicIpResponse(ip_mode, ip, mask, gw)

class ConfigureStaticDynamicIpResponse(ConfigureStaticDynamicIp):
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<B?' #cmd_id, result

    def __init__(self, result:bool):
        super().__init__()
        self.result = result

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORM, self.CMD_ID, self.result)    

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, result = struct.unpack(ConfigureStaticDynamicIpResponse.__PACK_FORM, payload)
        ConfigureStaticDynamicIpResponse._check_cmd_id(cmd_id)
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

    def __init__(self, result:bool, ip_mode:'IpMode|int', 
                    ip:'ipaddress.IPv4Address|str|int', mask:'ipaddress.IPv4Address|str|int', gw:'ipaddress.IPv4Address|str|int' ):
        super().__init__()
        if type(result) is not bool:
            raise TypeError(f'Bad type for result. Expect "bool". Got {type(result)}')
        self.result = result
        if type(ip_mode) is int:
            ip_mode = IpMode(ip_mode)
        elif type(ip_mode) is not IpMode:
            raise TypeError(f'Bad type for ip_mode. Expect "IpMode" or "int". Got {type(ip_mode)}')
        self._ip_mode = ip_mode.value
        if type(ip) is int or type(ip) is str:
            ip = ipaddress.IPv4Address(ip)
        elif type(ip) is not ipaddress.IPv4Address:
            raise TypeError(f'Bad type for ip. Expect "IPv4Address" or "int" or "str". Got {type(ip)}')
        self._ip = ip
        if type(mask) is int or type(mask) is str:
            mask = ipaddress.IPv4Address(mask)
        elif type(mask) is not ipaddress.IPv4Address:
            raise TypeError(f'Bad type for mask. Expect "IPv4Address" or "int" or "str". Got {type(mask)}')
        self._mask = mask 
        if type(gw) is int or type(gw) is str:
            gw = ipaddress.IPv4Address(gw)
        elif type(gw) is not ipaddress.IPv4Address:
            raise TypeError(f'Bad type for gw. Expect "IPv4Address" or "int" or "str". Got {type(gw)}')
        self._gw = gw

    @property
    def ip_mode(self)->IpMode:
        return IpMode(self._ip_mode)
    
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
    