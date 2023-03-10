# TODO Description

# std libs
from abc import abstractmethod, abstractproperty, abstractstaticmethod
import enum
import struct
import ipaddress

# project
from pylivox.control.frame import (Frame, Cmd,
                                   IsErrorResponse, IsErrorResponseOnly,
                                   DeviceType, get_default_device_type, get_default_device_version, get_ignore_type_restriction,
                                   support_only, )


class General(Cmd):
    CMD_SET = Frame.Set.GENERAL


class WorkState():
    class Lidar(enum.Enum):
        Initializing = 0
        Normal = 0x01
        PowerSaving = 0x02
        Standby = 0x03
        Error = 0x04

    class Hub(enum.Enum):
        Initializing = 0x00
        Normal = 0x01
        Error = 0x04

# class FiatureMsg():
#     class Lidar(enum.Enum):
#         LiDAR
#         Feature Message:
#         <br> Bit0: Rain/Fog Suppression Switch
#         <br>0x00: Turn Off
#         <br>0x01: Turn On
#         <br>Bit1 ~ Bit7: Reserved
#         <br>**Hub is Reserved** |


class Broadcast():
    _PACK_FORMAT = '<14sBx'  # 14 byte serial, ip_range, reserved
    _PACK_LENGTH = struct.calcsize(_PACK_FORMAT)

    def __init__(self, serial: bytes, ip_range: int):
        self.serial = serial
        self.ip_range = ip_range

    @property
    def serial(self) -> bytes:
        return self._serial

    @serial.setter
    def serial(self, value: 'str|bytes'):
        if type(value) is str:
            value = value.encode()
        elif type(value) is not bytes:
            raise TypeError
        if len(value) > 14:
            raise ValueError
        self._serial = value + bytes(14-len(value))

    @property
    def ip_range(self) -> int:
        return self._ip_range

    @ip_range.setter
    def ip_range(self, value: int):
        if type(value) is not int:
            raise TypeError
        value.to_bytes(1, 'little')
        self._ip_range = value

    @property
    def payload(self) -> bytes:
        return struct.pack(self._PACK_FORMAT, self.serial, self.ip_range)

    @classmethod
    def from_payload(cls, payload: bytes) -> 'Broadcast':
        serial, ip_range = struct.unpack(cls._PACK_FORMAT, payload)
        serial = bytes(serial)
        return cls(serial, ip_range)


class BroadcastMsg(General):
    CMD_TYPE = Frame.Type.MSG
    CMD_ID = Frame.SetGeneral.BROADCAST_MESSAGE
    _PACK_FORMAT = f'<B2x'  # {broadcast}, dev_type, reserved

    def __init__(self, 
                broadcast: Broadcast, 
                seq:int,
                device_type: 'DeviceType|int' = None, 
                device_version: 'tuple(int,int,int,int)' = None):
        super().__init__(seq, device_type, device_version)
        self.broadcast = broadcast
        self.dev_type = device_type

    def __repr__(self):
        return f'{{{self.__class__} serial:{self._broadcast.serial} device{self.dev_type} }}'

    @property
    def broadcast(self) -> Broadcast:
        return self._broadcast

    @broadcast.setter
    def broadcast(self, value: Broadcast):
        if type(value) is not Broadcast:
            raise TypeError
        self._broadcast = value

    @property
    def dev_type(self) -> DeviceType:
        return self._dev_type

    @dev_type.setter
    def dev_type(self, value: 'DeviceType|int'):
        if type(value) is int:
            value = DeviceType(value)
        elif type(value) is not DeviceType:
            raise TypeError
        self._dev_type = value

    @property
    def payload(self):
        payload_body = self.broadcast.payload + \
            struct.pack(self._PACK_FORMAT, self.dev_type.value)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, seq:int, device_type=None, device_version=None) -> 'BroadcastMsg':
        broadcast_bytes = payload[:Broadcast._PACK_LENGTH]
        broadcast = Broadcast.from_payload(broadcast_bytes)
        dev_type, = struct.unpack(
            cls._PACK_FORMAT, payload[Broadcast._PACK_LENGTH:])
        assert DeviceType(dev_type) == device_type
        return cls(broadcast, seq, dev_type, device_version)


class Handshake(General):
    # TODO description
    CMD_ID = Frame.SetGeneral.HANDSHAKE
    CMD_TYPE = Frame.Type.CMD
    # _PACK_FORMAT = '<4sHH' #ip, point_port, cmd_port

    def __init__(self,
                 ip: ipaddress.IPv4Address,
                 point_port: int,
                 cmd_port: int,
                 imu_port: int,
                 seq:int,
                 device_type: DeviceType = None,
                 device_version: 'tuple(int,int,int,int)' = None
                 ):

        super().__init__(seq, device_type, device_version)
        self.ip = ip
        self.point_port = point_port
        self.cmd_port = cmd_port
        self.imu_port = imu_port

    def __repr__(self):
        return f'{{{type(self).__name__} ip:{self.ip} point_port:{self.point_port} cmd_port:{self.cmd_port} imu_port:{self.imu_port}}}'

    @property
    def payload(self):
        if ((self.device_type == DeviceType.HORIZON and self.device_version >= (6, 4, 0, 0)) or
            (self.device_type == DeviceType.TELE_15 and self.device_version >= (3, 7, 0, 0))
            ):
            payload_body = struct.pack('<4sHHH', self.ip.packed, self.point_port, self.cmd_port, self.imu_port)
        else:
            payload_body = struct.pack('<4sHH', self.ip.packed, self.point_port, self.cmd_port)
        return super().cmd_payload(payload_body)

    @property
    def ip(self) -> ipaddress.IPv4Address:
        return self._ip

    @ip.setter
    def ip(self, value: 'ipaddress.IPv4Address|str|int|bytes'):
        self._ip = ipaddress.IPv4Address(value)

    @classmethod
    def from_payload(cls, 
                    payload: bytes,
                    seq:int,
                    device_type: DeviceType = None,
                    device_version: 'tuple(int,int,int,int)' = None):
        device_type = device_type or get_default_device_type()
        device_version = device_version or get_default_device_version()
        imu_port = None
        if len(payload) == struct.calcsize('<4sHHH'):
            ip, point_port, cmd_port, imu_port = struct.unpack('<4sHHH', payload)
        elif( len(payload) == struct.calcsize('4sHH')):
            ip, point_port, cmd_port = struct.unpack('<4sHH', payload)
        else:
            raise ValueError
        return cls(ip, point_port, cmd_port, imu_port, seq, device_type, device_version)


class HandshakeResponse(General, IsErrorResponseOnly):
    CMD_ID = Frame.SetGeneral.HANDSHAKE


class QueryDeviceInformation(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.QUERY_DEVICE_INFORMATION


class QueryDeviceInformationResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.QUERY_DEVICE_INFORMATION
    _PACK_FORMAT = '<?4B'  # is_error, firmware version

    def __init__(self, seq:int, is_error: bool = False, device_type: DeviceType = None, device_version: 'tuple(int,int,int,int)' = None,):
        super().__init__(seq, device_type, device_version)
        self.is_error = is_error

    @property
    def payload(self):
        payload_body = struct.pack(
            self._PACK_FORMAT, self.is_error, *self.device_version)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, seq:int, device_type=None, device_version=None):
        is_error, *firmware_version = struct.unpack(cls._PACK_FORMAT, payload)
        assert tuple(firmware_version) == device_version
        return cls(seq, is_error, device_type, firmware_version)


class Heartbeat(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.HEARTBEAT


class HeartbeatResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.HEARTBEAT
    _PACK_FORMAT = '<?BBI'  # is_error, work_state, feature, ack_msg

    def __init__(self,
                 work_state: 'WorkState.Lidar|WorkState.Hub|int',
                 feature_msg: int,
                 ack_msg: int,
                 seq:int,
                 is_error: bool = False,
                 device_type: DeviceType = None,
                 device_version: 'tuple(int,int,int,int)' = None
                 ):
        super().__init__(seq, device_type, device_version)
        self.is_error = is_error
        # self.device_model = device_type
        self.work_state = work_state
        self.feature_msg = feature_msg
        self.ack_msg = ack_msg

    def __repr__(self):
        return f'{{{type(self).__name__} error:{self.is_error} work_state:{self.work_state} feature_msg:{self.feature_msg} ack_msg{self.ack_msg}}}'

    @property
    def work_state(self) -> int:
        return self._work_state.value

    @work_state.setter
    def work_state(self, value: 'WorkState.Lidar|WorkState.Hub|int'):
        self._work_state = WorkState.Hub(value) if self.device_type is DeviceType.HUB else WorkState.Lidar(value)

    @property
    def feature_msg(self) -> int:
        return self._feature_msg

    @feature_msg.setter
    def feature_msg(self, value: int):
        if type(value) is not int:
            raise TypeError
        value.to_bytes(1, 'little')
        self._feature_msg = value

    @property
    def ack_msg(self) -> int:
        return self._ack_msg

    @ack_msg.setter
    def ack_msg(self, value: int):
        if type(value) is not int:
            raise TypeError
        value.to_bytes(4, 'little')
        self._ack_msg = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(
            self._PACK_FORMAT, self.is_error, self.work_state, self.feature_msg, self.ack_msg)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, seq:int, device_type = None, device_version = None):
        is_error, work_state, feature, ack_msg = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(work_state, feature, ack_msg, seq, is_error, device_type, device_version)


class StartStopSampling(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.START_STOP_SAMPLING
    _PACK_FORMAT = '<?'  # is_start

    def __init__(self,
                 is_start: bool,
                 seq:int,
                 device_type: DeviceType = None,
                 device_version: 'tuple(int,int,int,int)' = None
                 ):
        super().__init__(seq, device_type, device_version)
        self.is_start = is_start

    def __repr__(self):
        return f'{{{type(self).__name__} is_start:{self.is_start}}}'

    @property
    def is_start(self) -> bool:
        return self._is_start

    @is_start.setter
    def is_start(self, value: bool):
        if type(value) is not bool:
            raise TypeError
        self._is_start = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_start)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, seq:int, device_type = None, device_version = None):
        is_start,  = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(is_start, seq, device_type, device_version)


class StartStopSamplingResponse(General, IsErrorResponseOnly):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.START_STOP_SAMPLING


class ChangeCoordinateSystem(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.CHANGE_COORDINATE_SYSTEM
    _PACK_FORMAT = '<?'  # Is_Spherical_Coordinate?

    def __init__(self, 
                is_spherical: bool, 
                seq:int ,
                device_type: DeviceType = None, 
                device_version: 'tuple(int,int,int,int)' = None):
        super().__init__(seq, device_type, device_version)
        self.is_spherical = is_spherical

    def __repr__(self):
        return f'{{{type(self).__name__} spherical:{self._is_spherical}}}'

    @property
    def is_spherical(self) -> bool:
        return self._is_spherical

    @is_spherical.setter
    def is_spherical(self, value: bool):
        if type(value) is not bool:
            raise TypeError
        self._is_spherical = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_spherical)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, seq:int, device_type = None, device_version = None):
        is_spherical, = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(is_spherical, seq, device_type, device_version)


class ChangeCoordinateSystemResponse(General, IsErrorResponseOnly):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.CHANGE_COORDINATE_SYSTEM


class Disconnect(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.DISCONNECT


class DisconnectResponse(General, IsErrorResponseOnly):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.DISCONNECT


class PushAbnormalStatusInformation(General):
    CMD_TYPE = Frame.Type.MSG
    CMD_ID = Frame.SetGeneral.PUSH_ABNORMAL_STATUS_INFORMATION
    _PACK_FORMAT = '<I'  # status_code

    def __init__(self, 
                status_code: int, 
                seq: int,
                device_type: DeviceType = None, 
                device_version: 'tuple(int,int,int,int)' = None):
        super().__init__(seq, device_type, device_version)
        self.status_code = status_code

    @property
    def status_code(self) -> int:
        return self._status_code

    @status_code.setter
    def status_code(self, value: int):
        if type(value) is not int:
            raise TypeError
        self._status_code = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.status_code)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, seq:int ,device_type = None, device_version = None):
        status_code, = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(status_code, seq, device_type, device_version)


class ConfigureStaticDynamicIp(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.CONFIGURE_STATIC_DYNAMIC_IP

    def __init__(self,
                 is_static: bool,
                 ip: 'ipaddress.IPv4Address|int|str',
                 mask: 'ipaddress.IPv4Address|int|str|None',
                 gw: 'ipaddress.IPv4Address|int|str|None',
                 seq:int,
                 device_type: DeviceType = None,
                 device_version: 'tuple(int,int,int,int)' = None
                 ):
        super().__init__(seq, device_type, device_version)
        self.is_static = is_static
        self.ip = ip
        self.mask = mask
        self.gw = gw

    def __repr__(self):
        return f'{{{type(self).__name__} static:{self.is_static} ip:{self.ip} mask:{self.mask} gw:{self.gw}}}'

    @property
    def is_static(self) -> bool:
        return self._is_static

    @is_static.setter
    def is_static(self, value: bool):
        if type(value) is not bool:
            raise TypeError
        self._is_static = value

    @property
    def ip(self) -> ipaddress.IPv4Address:
        return self._ip

    @ip.setter
    def ip(self, value: 'ipaddress.IPv4Address|int|str|bytes'):
        self._ip = ipaddress.IPv4Address(value)

    @property
    def mask(self) -> 'ipaddress.IPv4Address|None':
        return self._mask

    @mask.setter
    def mask(self, value: 'ipaddress.IPv4Address|int|str|bytes|None'):
        self._mask = ipaddress.IPv4Address(value) if value else None

    @property
    def gw(self) -> 'ipaddress.IPv4Address|None':
        return self._gw

    @gw.setter
    def gw(self, value: 'ipaddress.IPv4Address|int|str|bytes|None'):
        self._gw = ipaddress.IPv4Address(value) if value else None

    @property
    def is_static(self) -> bool:
        return self._is_static

    @is_static.setter
    def is_static(self, value: bool):
        if type(value) is not bool:
            raise TypeError
        self._is_static = value

    @property
    def payload(self) -> bytes:
        if ((self.device_type == DeviceType.HORIZON and self.device_version >= (6, 4, 0, 0)) or
            (self.device_type == DeviceType.TELE_15 and self.device_version >= (7, 3, 0, 0)) or
            (self.device_type == DeviceType.MID_70 and self.device_version >= (10, 3, 0, 0)) or
            (self.device_type == DeviceType.AVIA and self.device_version >= (11, 6, 0, 0))
            ):
            payload_body = struct.pack('<?4s4s4s', self.is_static, self.ip.packed, self.mask.packed, self.gw.packed)
        else:
            payload_body = struct.pack('<?4s', self.is_static, self.ip.packed)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, seq:int ,device_type = None, device_version = None):
        device_type = device_type or get_default_device_type() 
        device_version = device_version or get_default_device_version()
        mask = None
        gw = None
        if len(payload) == struct.calcsize('<?4s4s4s'):
            is_static, ip, mask, gw = struct.unpack('<?4s4s4s', payload)
        elif len(payload) == struct.calcsize('<?4s'):
            is_static, ip = struct.unpack('<?4s', payload)
        else:
            raise ValueError
        return cls(is_static, ip, mask, gw, seq, device_type, device_version)


class ConfigureStaticDynamicIpResponse(General, IsErrorResponseOnly):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.CONFIGURE_STATIC_DYNAMIC_IP


class GetDeviceIpInformation(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.GET_DEVICE_IP_INFORMATION


class GetDeviceIpInformationResponse(ConfigureStaticDynamicIp, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.GET_DEVICE_IP_INFORMATION

    def __init__(self,
                 is_static: bool,
                 ip: 'ipaddress.IPv4Address|str|int|bytes',
                 mask: 'ipaddress.IPv4Address|str|int|bytes|None',
                 gw: 'ipaddress.IPv4Address|str|int|bytes|None',
                 seq: int,
                 is_error: bool = False,
                 device_type: DeviceType = None,
                 device_version: 'tuple(int,int,int,int)' = None
                 ):
        super().__init__(is_static, ip, mask, gw, seq, device_type, device_version)
        self.is_error = is_error

    def __repr__(self):
        return f'{{{type(self).__name__} error:{self.is_error} static:{self.is_static} ip:{self.ip} mask{self.mask} gw:{self.gw}}}'

    @property
    def payload(self) -> bytes:
        if ((self.device_type == DeviceType.HORIZON and self.device_version >= (6, 4, 0, 0)) or
            (self.device_type == DeviceType.TELE_15 and self.device_version >= (7, 3, 0, 0)) or
            (self.device_type == DeviceType.MID_70 and self.device_version >= (10, 3, 0, 0)) or
            (self.device_type == DeviceType.AVIA and self.device_version >= (11, 6, 0, 0))
                ):
            payload_body = struct.pack('<??4s4s4s', self.is_error, self.is_static, self.ip.packed, self.mask.packed, self.gw.packed)
        else:
            payload_body = struct.pack('<??4s', self._is_error, self.is_static, self.ip.packed)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload, seq:int, device_type = None, device_version = None):
        device_type = device_type or get_default_device_type()
        device_version = device_version or get_default_device_version()
        mask = None
        gw = None
        if len(payload) == struct.calcsize('<??4s4s4s'):
            is_error, is_static, ip, mask, gw = struct.unpack('<??4s4s4s', payload)
        elif len(payload) == struct.calcsize('<??4s'):
            is_error, is_static, ip = struct.unpack('<??4s', payload)
        return cls(is_static, ip, mask, gw, seq, is_error, device_type, device_version)


@support_only([
    (DeviceType.HUB, (8, 7, 0, 0)),
    (DeviceType.MID_40, (3, 7, 0, 0)),
    (DeviceType.MID_70, (10, 3, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class RebootDevice(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.REBOOT_DEVICE
    _PACK_FORMAT = '<H'  # timeout

    def __init__(self, 
                timeout: int, 
                seq:int,
                device_type: DeviceType = None, 
                device_version: 'tuple(int,int,int,int)' = None):
        super().__init__(seq, device_type, device_version)
        self.timeout = timeout

    def __repr__(self):
        return f'{{{type(self).__name__} timeout:{self.timeout} }}'

    @property
    def timeout(self) -> int:
        return self._timeout

    @timeout.setter
    def timeout(self, value: int):
        if type(value) is not int:
            raise TypeError
        self._timeout = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.timeout)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, seq:int, device_type = None, device_version = None):
        timeout, = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(timeout, seq, device_type, device_version)


@support_only([
    (DeviceType.HUB, (8, 7, 0, 0)),
    (DeviceType.MID_40, (3, 7, 0, 0)),
    (DeviceType.MID_70, (10, 3, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class RebootDeviceResponse(General, IsErrorResponseOnly):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.REBOOT_DEVICE


class ConfigurationParameter:

    class Key(enum.Enum):
        HIGH_SENSITIVITY_FUNCTION = 1
        SWITCH_REPETITIVE_NON_REPETITIVE_SCANNING_PATTERN = 2
        SLOT_ID_CONFIGURATION = 3

    class ErrorCode(enum.Enum):
        NO_ERROR = 0
        THE_KEY_IS_NOT_SUPPORTED = 1
        EXECUTION_FAILED = 2
        THE_KEY_CANNOT_BE_WRITTEN = 3
        WRONG_VALUE = 4
        WRONG_VALUE_LENGTH = 5
        READING_PARAMETER_LENGTH_LIMIT = 6
        THE_NUMBER_OF_PARAMETERS_DOES_NOT_MATCH = 7

    TYPE_DIC = {
        Key.HIGH_SENSITIVITY_FUNCTION: bool,
        Key.SWITCH_REPETITIVE_NON_REPETITIVE_SCANNING_PATTERN: bool,
        Key.SLOT_ID_CONFIGURATION: int,
    }

    def __init__(self, key: 'Key|int', value):
        self.key = key
        self.value = value

    def __repr__(self) -> str:
        return f'ConfigParam<{self.key}:{self.value}>'

    @property
    def key(self) -> Key:
        return self._key

    @key.setter
    def key(self, value: 'Key|int'):
        if type(value) is int:
            value = self.Key(value)
        elif type(value) is not self.Key:
            raise TypeError
        self._key = value
        self.length = 5  # all config params in current version is 5 bytes length

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        T = self.TYPE_DIC[self.key]
        self._value = T(value)

    @property
    def payload(self) -> bytes:
        return struct.pack('<HHB', self.key.value, self.length, self.value)

    @classmethod
    def from_payload(cls, payload: bytes) -> 'ConfigurationParameter':
        length, = struct.unpack('<H', payload[2:4])
        key, length, value = struct.unpack('<HHB', payload[:length])
        return cls(key, value)

    @classmethod
    def from_payload_list(cls, payload: bytes) -> 'list(ConfigurationParameter)':
        result = []
        shift = 0
        while shift < len(payload):
            parameter = cls.from_payload(payload[shift:])
            shift += parameter.length
            result.append(parameter)
        return result


@support_only([
    (DeviceType.HUB, (8, 9, 0, 0)),
    (DeviceType.HORIZON, (6, 11, 0, 0)),
    (DeviceType.TELE_15, (7, 9, 0, 0)),
    (DeviceType.MID_70, (10, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class WriteConfigurationParameters(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.WRITE_CONFIGURATION_PARAMETERS

    def __init__(self, 
                param_list: 'list(ConfigurationParameter)|bytes', 
                seq:int ,
                device_type: DeviceType = None, 
                device_version: 'tuple(int,int,int,int)' = None):
        super().__init__(seq, device_type, device_version)
        self.param_list = param_list

    @property
    def param_list(self) -> 'list(ConfigurationParameter)':
        return self._param_list

    @param_list.setter
    def param_list(self, value: 'list(ConfigurationParameter)|bytes'):
        if type(value) is bytes:
            value = ConfigurationParameter.from_payload_list(value)
        elif type(value) is not list or [parameter for parameter in value if type(parameter) is not ConfigurationParameter]:
            raise TypeError
        self._param_list = value

    @property
    def payload(self) -> bytes:
        return super().cmd_payload(b''.join([param.payload for param in self.param_list]))

    @classmethod
    def from_payload(cls, payload: bytes, seq:int, device_type = None, device_version = None):
        return cls(payload, seq, device_type, device_version)


@support_only([
    (DeviceType.HUB, (8, 9, 0, 0)),
    (DeviceType.HORIZON, (6, 11, 0, 0)),
    (DeviceType.TELE_15, (7, 9, 0, 0)),
    (DeviceType.MID_70, (10, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class WriteConfigurationParametersResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.WRITE_CONFIGURATION_PARAMETERS
    _PACK_FORMAT = '<?HB'  # is_error, error_key, error_code

    def __init__(self,
                 error_key: 'ConfigurationParameter.Key|int',
                 error_code: 'ConfigurationParameter.ErrorCode|int',
                 seq:int, 
                 is_error: bool = False,
                 device_type: DeviceType = None,
                 device_version: 'tuple(int,int,int,int)' = None
                 ):
        super().__init__(seq, device_type, device_version)
        self.is_error = is_error
        self.error_key = error_key
        self.error_code = error_code

    @property
    def key(self) -> ConfigurationParameter.Key:
        return self._key

    @key.setter
    def key(self, value: 'ConfigurationParameter.Key|int'):
        if type(value) is int:
            value = ConfigurationParameter.Key(value)
        elif type(value) is not ConfigurationParameter.Key:
            raise TypeError
        self._key = value

    @property
    def error_code(self) -> ConfigurationParameter.ErrorCode:
        return self._error_code

    @error_code.setter
    def error_code(self, value: 'ConfigurationParameter.ErrorCode|int'):
        if type(value) is int:
            value = ConfigurationParameter.ErrorCode(value)
        elif type(value) is not ConfigurationParameter.ErrorCode:
            raise TypeError
        self._error_code = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(
            self._PACK_FORMAT, self.is_error, self.error_key.value, self.error_code.value)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, seq:int, device_type = None, device_version = None):
        is_error, error_key, error_code = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(error_key, error_code, seq, is_error, device_type, device_version)


@support_only([
    (DeviceType.HUB, (8, 9, 0, 0)),
    (DeviceType.HORIZON, (6, 11, 0, 0)),
    (DeviceType.TELE_15, (7, 9, 0, 0)),
    (DeviceType.MID_70, (10, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class ReadConfigurationParameters(General):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetGeneral.READ_CONFIGURATION_PARAMETERS

    def __init__(self, 
                keys_quantity: int, 
                keys: 'list(ConfigurationParameter.Key)', 
                seq:int, 
                device_type: DeviceType = None, 
                device_version: 'tuple(int,int,int,int)' = None):
        super().__init__(seq, device_type, device_version)
        self.keys_quantity = keys_quantity
        self.keys = keys

    def __repr__(self):
        return f'{{{type(self).__name__} [{self.keys}]}}'

    @property
    def keys_quantity(self) -> int:
        return self._keys_quantity

    @keys_quantity.setter
    def keys_quantity(self, value: int):
        if type(value) is not int:
            raise TypeError
        self._keys_quantity = value

    @property
    def keys(self) -> 'list(ConfigurationParameter.Key)':
        return self._keys

    @keys.setter
    def keys(self, value: 'list(ConfigurationParameter.Key)'):
        if type(value) is not list or [key for key in value if type(key) is not ConfigurationParameter.Key]:
            raise TypeError
        self._keys = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(
            f'<B{self.keys_quantity}H', self.keys_quantity, *[key.value for key in self.keys])
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, seq:int, device_type = None, device_version = None):
        keys_quantity = payload[0]
        keys_bytes = [payload[2*i+1:2*i+3] for i in range(keys_quantity)]
        keys = [ConfigurationParameter.Key(
            int.from_bytes(key, 'little')) for key in keys_bytes]
        return cls(keys_quantity, keys, seq, device_type, device_version)


@support_only([
    (DeviceType.HUB, (8, 9, 0, 0)),
    (DeviceType.HORIZON, (6, 11, 0, 0)),
    (DeviceType.TELE_15, (7, 9, 0, 0)),
    (DeviceType.MID_70, (10, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class ReadConfigurationParametersResponse(General, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetGeneral.READ_CONFIGURATION_PARAMETERS

    def __init__(self,
                 error_key: 'ConfigurationParameter.Key|int',
                 error_code: 'ConfigurationParameter.ErrorCode|int',
                 param_list: 'list(ConfigurationParameter)',
                 seq:int, 
                 is_error: bool = False,
                 device_type: DeviceType = None,
                 device_version: 'tuple(int,int,int,int)' = None
                 ):
        super().__init__(seq, device_type, device_version)
        self.is_error = is_error
        self.error_key = error_key
        self.error_code = error_code
        self.param_list = param_list

    def __repr__(self):
        return f'{{{type(self).__name__} error:{self.is_error} error_key{self.error_key} error_code{self.error_code} params:{self.param_list}}}'

    @property
    def error_key(self)->ConfigurationParameter.Key:
        return self._error_key

    @error_key.setter
    def error_key(self, value:'ConfigurationParameter.Key| int'):
        self._error_key = ConfigurationParameter.Key(value)

    @property
    def error_code(self) -> ConfigurationParameter.ErrorCode:
        return self._error_code

    @error_code.setter
    def error_code(self, value: 'ConfigurationParameter.ErrorCode|int'):
        if type(value) is int:
            value = ConfigurationParameter.ErrorCode(value)
        elif type(value) is not ConfigurationParameter.ErrorCode:
            raise TypeError
        self._error_code = value

    @property
    def param_list(self) -> 'list(ConfigurationParameter)':
        return self._param_list

    @param_list.setter
    def param_list(self, value: 'list(ConfigurationParameter)'):
        if type(value) is not list or [param for param in value if type(param) is not ConfigurationParameter]:
            raise TypeError
        self._param_list = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(f'<?HB', self.is_error, self.error_key.value,
                                   self.error_code.value) + b''.join([param.payload for param in self.param_list])
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, seq:int, device_type = None, device_version = None):
        is_error, error_key, error_code = struct.unpack('<?HB', payload[:4])
        param_list = ConfigurationParameter.from_payload_list(payload[4:])
        return cls(error_key, error_code, param_list, seq, is_error, device_type, device_version)
