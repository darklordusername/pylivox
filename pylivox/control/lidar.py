#TODO Description

#std libs
from faulthandler import is_enabled
import struct 
import enum
from datetime import datetime

#project 
from  pylivox.control.frame import Frame, Cmd, IsErrorResponse, IsErrorResponseOnly

class PowerMode(enum.Enum):
    normal      = 0x01
    powerSaving = 0x02     
    standby     = 0x03 

class Lidar(Cmd):
    CMD_SET = Frame.Set.LIDAR

class ReturnMode(enum.Enum):
    SINGLE_RETURN_FIRST= 0x00
    SINGLE_RETURN_STRONGEST= 0x01
    DUAL_RETURN= 0x02
    TRIPLE_RETURN= 0x03

class PushFrequency(enum.Enum):
    FREQ_0HZ = 0
    FREQ_200HZ = 1


class SetMode(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.SET_MODE
    _PACK_FORMAT = '<B'#lidar_mode

    def __init__(self, power_mode:'PowerMode|int'):
        super().__init__()
        self.power_mode = power_mode

    @property
    def power_mode(self)->PowerMode:
        return self._power_mode

    @power_mode.setter
    def power_mode(self, value:'PowerMode|int'):
        if type(value) is int:
            value = PowerMode(value)
        elif type(value) is not PowerMode:
            raise TypeError
        self._power_mode = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.power_mode.value)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        lidar_mode, = struct.unpack(SetMode._PACK_FORMAT, payload)
        return SetMode(lidar_mode)

class SetModeResponse(Lidar):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetLidar.SET_MODE
    _PACK_FORMAT = '<B' #Result
    
    class Result(enum.Enum):
        Success = 0
        Fail = 1
        Switching = 2

    def __init__(self, result:'Result|int'):
        super().__init__()
        self.result = result

    @property
    def result(self)->Result:
        return self._result

    @result.setter
    def result(self, value:'Result|int'):
        if type(value) is int:
            value = self.Result(value)
        elif type(value) is not self.Result:
            raise TypeError
        self._result = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.result.value)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        result, = struct.unpack(SetModeResponse._PACK_FORMAT, payload)
        return SetModeResponse(result)
    
class WriteLidarExtrinsicParameters(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.WRITE_LIDAR_EXTRINSIC_PARAMETERS 
    _PACK_FORMAT = '<fffIII' #roll, pitch, yaw, x, y, z

    def __init__(self, roll:float, pitch:float, yaw:float, x:int, y:int, z:int):
        super().__init__()
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x = x
        self.y = y
        self.z = z

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.roll, self.pitch, self.yaw, self.x, self.y, self.z)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        roll, pitch, yaw, x, y, z = struct.unpack(WriteLidarExtrinsicParameters._PACK_FORMAT, payload)
        return WriteLidarExtrinsicParameters(roll, pitch, yaw, x, y, z)

class WriteLidarExtrinsicParametersResponse(Lidar, IsErrorResponseOnly):
    CMD_ID = Frame.SetLidar.WRITE_LIDAR_EXTRINSIC_PARAMETERS

class ReadLidarExtrinsicParameters(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.READ_LIDAR_EXTRINSIC_PARAMETERS 

    @property
    def payload(self)->bytes:
        return super().payload(b'')

    @staticmethod
    def from_payload(payload:bytes):
        return ReadLidarExtrinsicParameters()

class ReadLidarExtrinsicParametersResponse(Lidar, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetLidar.READ_LIDAR_EXTRINSIC_PARAMETERS
    _PACK_FORMAT = '<?fffIII' #is_error, roll, pitch, yaw, x, y, z

    def __init__(self, is_error:bool, roll:float, pitch:float, yaw:float, x:int, y:int, z:int):
        super().__init__()
        self.is_error = is_error
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x = x
        self.y = y
        self.z = z

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error, self.roll, self.pitch, self.yaw, self.x, self.y, self.z)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_error, roll, pitch, yaw, x, y, z = struct.unpack(ReadLidarExtrinsicParametersResponse._PACK_FORMAT, payload)
        return ReadLidarExtrinsicParametersResponse(is_error, roll, pitch, yaw, x, y, z)

class TurnOnOffRainFogSuppression(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.TURN_ON_OFF_RAIN_FOG_SUPPRESSION 
    _PACK_FORMAT = '<?' # is_enable

    def __init__(self, is_enable:bool):
        super().__init__()
        self.is_enable = is_enable

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_enable)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_enable, = struct.unpack(TurnOnOffRainFogSuppression._PACK_FORMAT, payload)
        return TurnOnOffRainFogSuppression(is_enable)

class TurnOnOffRainFogSuppressionResponse(Lidar, IsErrorResponseOnly):
    CMD_ID = Frame.SetLidar.TURN_ON_OFF_RAIN_FOG_SUPPRESSION

class SetTurnOnOffFan(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.SET_TURN_ON_OFF_FAN 
    _PACK_FORMAT = '<?' #is_enable

    def __init__(self, is_enable:bool):
        super().__init__()
        self.is_enable = is_enable

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_enable)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_enable, = struct.unpack(SetTurnOnOffFan._PACK_FORMAT, payload)
        return SetTurnOnOffFan(is_enable)

class SetTurnOnOffFanResponse( Lidar, IsErrorResponseOnly):
    CMD_ID = Frame.SetLidar.SET_TURN_ON_OFF_FAN    


class GetTurnOnOffFanState(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.GET_TURN_ON_OFF_FAN_STATE

    @property
    def payload(self)->bytes:
        return super().payload(b'')

    @staticmethod
    def from_payload(payload:bytes):
        return GetTurnOnOffFanState()

class GetTurnOnOffFanStateResponse( Lidar, IsErrorResponse):
    CMD_TYPE = Frame.Type.AKN
    CMD_ID = Frame.SetLidar.GET_TURN_ON_OFF_FAN_STATE
    _PACK_FORMAT = '<??' #is_error, state

    def __init__(self, is_error:bool, state:bool):
        super().__init__()
        self.is_error = is_error
        self.state = state

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error, self.state)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_error, state = struct.unpack(GetTurnOnOffFanStateResponse._PACK_FORMAT, payload)
        return GetTurnOnOffFanStateResponse(is_error, state)

class SetLidarReturnMode(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.SET_LIDAR_RETURN_MODE 
    _PACK_FORMAT = '<B' # return_mode

    def __init__(self, return_mode:'ReturnMode|int'):
        super().__init__()
        self.return_mode = return_mode

    @property
    def return_mode(self)->ReturnMode:
        return self._return_mode

    @return_mode.setter
    def return_mode(self, value:'ReturnMode|int'):
        if type(value) is int:
            value = ReturnMode(value)
        elif type(value) is not ReturnMode:
            raise TypeError
        self._return_mode = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.return_mode.value)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        return_mode, = struct.unpack(SetLidarReturnMode._PACK_FORMAT, payload)
        return SetLidarReturnMode(return_mode)

class SetLidarReturnModeResponse( Lidar, IsErrorResponseOnly):
    CMD_ID = Frame.SetLidar.SET_LIDAR_RETURN_MODE

class GetLidarReturnMode(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.GET_LIDAR_RETURN_MODE 

    @property
    def payload(self)->bytes:
        return super().payload(b'')

    @staticmethod
    def from_payload(payload:bytes):
        return GetLidarReturnMode()

class GetLidarReturnModeResponse(Lidar, IsErrorResponse):
    CMD_ID = Frame.SetLidar.GET_LIDAR_RETURN_MODE
    _PACK_FORMAT = '<?B' #is_error, return_mode

    def __init__(self, is_error:bool, return_mode:'ReturnMode|int'):
        super().__init__()
        self.result = is_error
        self.return_mode = return_mode

    @property
    def return_mode(self)->ReturnMode:
        return self._return_mode

    @return_mode.setter
    def return_mode(self, value:'ReturnMode|int'):
        if type(value) is int:
            value = ReturnMode(value)
        elif type(value) is not ReturnMode:
            raise TypeError
        self._return_mode = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.result, self.return_mode.value)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_error, return_mode = struct.unpack(GetLidarReturnModeResponse._PACK_FORMAT, payload)
        return GetLidarReturnModeResponse(is_error, return_mode)
    
class SetImuDataPushFrequency(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.SET_IMU_DATA_PUSH_FREQUENCY
    _PACK_FORMAT = '<B' # frequency

    def __init__(self, frequency:'PushFrequency|int'):
        super().__init__()
        self.frequency = frequency

    @property
    def frequency(self)->PushFrequency:
        return self._frequency

    @frequency.setter
    def frequency(self, value:'PushFrequency|int'):
        if type(value) is int:
            value = PushFrequency(value)
        elif type(value) is not PushFrequency:
            raise TypeError
        self._frequency = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.frequency.value)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        frequency, = struct.unpack(SetImuDataPushFrequency._PACK_FORMAT, payload)
        return SetImuDataPushFrequency(frequency)

class SetImuDataPushFrequencyResponse( Lidar, IsErrorResponseOnly):
    CMD_ID = Frame.SetLidar.SET_IMU_DATA_PUSH_FREQUENCY

class GetImuDataPushFrequency(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.GET_IMU_DATA_PUSH_FREQUENCY 

    @property
    def payload(self)->bytes:
        return super().payload(b'')

    @staticmethod
    def from_payload(payload:bytes):
        return GetImuDataPushFrequency()

class GetImuDataPushFrequencyResponse( Lidar, IsErrorResponse):
    CMD_ID = Frame.SetLidar.GET_IMU_DATA_PUSH_FREQUENCY
    _PACK_FORMAT = '<?B' #is_error, frequency

    def __init__(self, is_error:bool, frequency:'PushFrequency|int'):
        super().__init__()
        self.is_error = is_error
        self.frequency = frequency

    @property
    def frequency(self)->PushFrequency:
        return self._frequency

    @frequency.setter
    def frequency(self, value:'PushFrequency|int'):
        if type(value) is int:
            value = PushFrequency(value)
        elif type(value) is not PushFrequency:
            raise TypeError
        self._frequency = value

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error, self.frequency.value)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        is_error, frequency = struct.unpack(GetImuDataPushFrequencyResponse._PACK_FORMAT, payload)
        return GetImuDataPushFrequencyResponse(is_error, frequency)

class UpdateUtcSynchronizationTime(Lidar):
    CMD_TYPE = Frame.Type.CMD
    CMD_ID = Frame.SetLidar.UPDATE_UTC_SYNCHRONIZATION_TIME 
    _PACK_FORMAT = '<BBBBI' #year, month, day, hour, microseconds

    def __init__(self, year:int, month:int, day:int, hour:int, microseconds:int):
        super().__init__()
        datetime(year, month, day, hour)
        self.year = year
        self.month = month
        self.day = day
        self.hour = hour 
        self.microseconds = microseconds

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.year, self.month, self.day, self.hour, self.microseconds)
        return super().payload(payload_body)

    @staticmethod
    def from_payload(payload:bytes):
        year, month, day, hour, us = struct.unpack(UpdateUtcSynchronizationTime._PACK_FORMAT, payload)
        return UpdateUtcSynchronizationTime(year, month, day, hour, us)

class UpdateUtcSynchronizationTimeResponse( UpdateUtcSynchronizationTime):
    pass
