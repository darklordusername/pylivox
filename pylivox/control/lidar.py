# TODO Description

# std libs
from faulthandler import is_enabled
import struct
import enum
from datetime import datetime

# project
from pylivox.control.frame import Frame, Cmd, IsErrorResponse, IsErrorResponseOnly, DeviceType, Device_type, Device_version, support_only


class PowerMode(enum.Enum):
    normal = 0x01
    powerSaving = 0x02
    standby = 0x03


class Lidar(Cmd):
    CMD_SET = Frame.Set.LIDAR


class ReturnMode(enum.Enum):
    SINGLE_RETURN_FIRST = 0x00
    SINGLE_RETURN_STRONGEST = 0x01
    DUAL_RETURN = 0x02
    TRIPLE_RETURN = 0x03


class PushFrequency(enum.Enum):
    FREQ_0HZ = 0
    FREQ_200HZ = 1


class SetMode(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.SET_MODE
    _PACK_FORMAT = '<B'  # lidar_mode

    def __init__(self, power_mode: 'PowerMode|int', device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        super().__init__(device_type, device_version)
        self.power_mode = power_mode

    @property
    def power_mode(self) -> PowerMode:
        return self._power_mode

    @power_mode.setter
    def power_mode(self, value: 'PowerMode|int'):
        if type(value) is int:
            value = PowerMode(value)
        elif type(value) is not PowerMode:
            raise TypeError
        self._power_mode = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.power_mode.value)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        lidar_mode, = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(lidar_mode, device_type, device_version)


class SetModeResponse(Lidar):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetLidar.SET_MODE
    _PACK_FORMAT = '<B'  # Result

    class Result(enum.Enum):
        Success = 0
        Fail = 1
        Switching = 2

    def __init__(self, result: 'Result|int', device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        super().__init__(device_type, device_version)
        self.result = result

    @property
    def result(self) -> Result:
        return self._result

    @result.setter
    def result(self, value: 'Result|int'):
        if type(value) is int:
            value = self.Result(value)
        elif type(value) is not self.Result:
            raise TypeError
        self._result = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.result.value)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        result, = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(result, device_type, device_version)


class WriteLidarExtrinsicParameters(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.WRITE_LIDAR_EXTRINSIC_PARAMETERS
    _PACK_FORMAT = '<fffIII'  # roll, pitch, yaw, x, y, z

    def __init__(self, roll: float, pitch: float, yaw: float, x: int, y: int, z: int, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        super().__init__(device_type, device_version)
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x = x
        self.y = y
        self.z = z

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(
            self._PACK_FORMAT, self.roll, self.pitch, self.yaw, self.x, self.y, self.z)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        roll, pitch, yaw, x, y, z = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(roll, pitch, yaw, x, y, z, device_type, device_version)


class WriteLidarExtrinsicParametersResponse(Lidar, IsErrorResponseOnly):
    CMD_ID = Frame.SetLidar.WRITE_LIDAR_EXTRINSIC_PARAMETERS


class ReadLidarExtrinsicParameters(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.READ_LIDAR_EXTRINSIC_PARAMETERS


class ReadLidarExtrinsicParametersResponse(Lidar, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetLidar.READ_LIDAR_EXTRINSIC_PARAMETERS
    _PACK_FORMAT = '<?fffIII'  # is_error, roll, pitch, yaw, x, y, z

    def __init__(self,
                 roll: float,
                 pitch: float,
                 yaw: float,
                 x: int,
                 y: int,
                 z: int,
                 is_error: bool = False,
                 device_type: DeviceType = Device_type,
                 device_version: 'tuple(int,int,int,int)' = Device_version
                 ):
        super().__init__(device_type, device_version)
        self.is_error = is_error
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x = x
        self.y = y
        self.z = z

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error,
                                   self.roll, self.pitch, self.yaw, self.x, self.y, self.z)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        is_error, roll, pitch, yaw, x, y, z = struct.unpack(
            cls._PACK_FORMAT, payload)
        return cls(roll, pitch, yaw, x, y, z, is_error, device_type, device_version)


@support_only([(DeviceType.MID_40, (0, 0, 0, 0))])
class TurnOnOffRainFogSuppression(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.TURN_ON_OFF_RAIN_FOG_SUPPRESSION
    _PACK_FORMAT = '<?'  # is_enable

    def __init__(self, is_enable: bool, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        super().__init__(device_type, device_version)
        self.is_enable = is_enable

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_enable)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        is_enable, = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(is_enable, device_type, device_version)


@support_only([(DeviceType.MID_40, (0, 0, 0, 0))])
class TurnOnOffRainFogSuppressionResponse(Lidar, IsErrorResponseOnly):
    CMD_ID = Frame.SetLidar.TURN_ON_OFF_RAIN_FOG_SUPPRESSION


@support_only([
    (DeviceType.MID_40, (3, 7, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
])
class SetTurnOnOffFan(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.SET_TURN_ON_OFF_FAN
    _PACK_FORMAT = '<?'  # is_enable

    def __init__(self, is_enable: bool, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        super().__init__(device_type, device_version)
        self.is_enable = is_enable

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_enable)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        is_enable, = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(is_enable, device_type, device_version)


@support_only([
    (DeviceType.MID_40, (3, 7, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
])
class SetTurnOnOffFanResponse(Lidar, IsErrorResponseOnly):
    CMD_ID = Frame.SetLidar.SET_TURN_ON_OFF_FAN


@support_only([
    (DeviceType.MID_40, (3, 7, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
])
class GetTurnOnOffFanState(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.GET_TURN_ON_OFF_FAN_STATE


@support_only([
    (DeviceType.MID_40, (3, 7, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
])
class GetTurnOnOffFanStateResponse(Lidar, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetLidar.GET_TURN_ON_OFF_FAN_STATE
    _PACK_FORMAT = '<??'  # is_error, state

    def __init__(self, state: bool, is_error: bool = False, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        super().__init__(device_type, device_version)
        self.is_error = is_error
        self.state = state

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(
            self._PACK_FORMAT, self.is_error, self.state)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        is_error, state = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(state, is_error, device_type, device_version)


@support_only([
    (DeviceType.MID_40, (10, 3, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class SetLidarReturnMode(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.SET_LIDAR_RETURN_MODE
    _PACK_FORMAT = '<B'  # return_mode

    def __init__(self, return_mode: 'ReturnMode|int', device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        super().__init__(device_type, device_version)
        self.return_mode = return_mode

    @property
    def return_mode(self) -> ReturnMode:
        return self._return_mode

    @return_mode.setter
    def return_mode(self, value: 'ReturnMode|int'):
        if type(value) is int:
            value = ReturnMode(value)
        elif type(value) is not ReturnMode:
            raise TypeError
        self._return_mode = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.return_mode.value)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        return_mode, = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(return_mode, device_type, device_version)


@support_only([
    (DeviceType.MID_40, (10, 3, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class SetLidarReturnModeResponse(Lidar, IsErrorResponseOnly):
    CMD_ID = Frame.SetLidar.SET_LIDAR_RETURN_MODE


@support_only([
    (DeviceType.MID_40, (10, 3, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class GetLidarReturnMode(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.GET_LIDAR_RETURN_MODE


@support_only([
    (DeviceType.MID_40, (10, 3, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class GetLidarReturnModeResponse(Lidar, IsErrorResponse):
    CMD_ID = Frame.SetLidar.GET_LIDAR_RETURN_MODE
    _PACK_FORMAT = '<?B'  # is_error, return_mode

    def __init__(self, return_mode: 'ReturnMode|int', is_error: bool = False, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        super().__init__(device_type, device_version)
        self.result = is_error
        self.return_mode = return_mode

    @property
    def return_mode(self) -> ReturnMode:
        return self._return_mode

    @return_mode.setter
    def return_mode(self, value: 'ReturnMode|int'):
        if type(value) is int:
            value = ReturnMode(value)
        elif type(value) is not ReturnMode:
            raise TypeError
        self._return_mode = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(
            self._PACK_FORMAT, self.result, self.return_mode.value)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        is_error, return_mode = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(return_mode, is_error, device_type, device_version)


@support_only([
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class SetImuDataPushFrequency(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.SET_IMU_DATA_PUSH_FREQUENCY
    _PACK_FORMAT = '<B'  # frequency

    def __init__(self, frequency: 'PushFrequency|int', device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        super().__init__(device_type, device_version)
        self.frequency = frequency

    @property
    def frequency(self) -> PushFrequency:
        return self._frequency

    @frequency.setter
    def frequency(self, value: 'PushFrequency|int'):
        if type(value) is int:
            value = PushFrequency(value)
        elif type(value) is not PushFrequency:
            raise TypeError
        self._frequency = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.frequency.value)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        frequency, = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(frequency, device_type, device_version)


@support_only([
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class SetImuDataPushFrequencyResponse(Lidar, IsErrorResponseOnly):
    CMD_ID = Frame.SetLidar.SET_IMU_DATA_PUSH_FREQUENCY


@support_only([
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class GetImuDataPushFrequency(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.GET_IMU_DATA_PUSH_FREQUENCY


@support_only([
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class GetImuDataPushFrequencyResponse(Lidar, IsErrorResponse):
    CMD_ID = Frame.SetLidar.GET_IMU_DATA_PUSH_FREQUENCY
    _PACK_FORMAT = '<?B'  # is_error, frequency

    def __init__(self, frequency: 'PushFrequency|int', is_error: bool = False, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        super().__init__(device_type, device_version)
        self.is_error = is_error
        self.frequency = frequency

    @property
    def frequency(self) -> PushFrequency:
        return self._frequency

    @frequency.setter
    def frequency(self, value: 'PushFrequency|int'):
        if type(value) is int:
            value = PushFrequency(value)
        elif type(value) is not PushFrequency:
            raise TypeError
        self._frequency = value

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(
            self._PACK_FORMAT, self.is_error, self.frequency.value)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        is_error, frequency = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(frequency, is_error, device_type, device_version)


@support_only([
    (DeviceType.MID_40, (3, 7, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class UpdateUtcSynchronizationTime(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.UPDATE_UTC_SYNCHRONIZATION_TIME
    _PACK_FORMAT = '<BBBBI'  # year, month, day, hour, microseconds

    def __init__(self, year: int, month: int, day: int, hour: int, microseconds: int, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        super().__init__(device_type, device_version)
        datetime(year, month, day, hour)
        self.year = year
        self.month = month
        self.day = day
        self.hour = hour
        self.microseconds = microseconds

    @property
    def payload(self) -> bytes:
        payload_body = struct.pack(
            self._PACK_FORMAT, self.year, self.month, self.day, self.hour, self.microseconds)
        return super().cmd_payload(payload_body)

    @classmethod
    def from_payload(cls, payload: bytes, device_type: DeviceType = Device_type, device_version: 'tuple(int,int,int,int)' = Device_version):
        year, month, day, hour, us = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(year, month, day, hour, us, device_type, device_version)


@support_only([
    (DeviceType.MID_40, (3, 7, 0, 0)),
    (DeviceType.HORIZON, (6, 4, 0, 0)),
    (DeviceType.TELE_15, (7, 3, 0, 0)),
    (DeviceType.AVIA, (11, 6, 0, 0)),
])
class UpdateUtcSynchronizationTimeResponse(Lidar, IsErrorResponseOnly):
    CMD_ID = Frame.SetLidar.UPDATE_UTC_SYNCHRONIZATION_TIME
