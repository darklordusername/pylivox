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



class ImuDataPushFrequency(enum.Enum):
    Hz_0 = 0
    Hz_200 = 1

class ReturnMode(enum.Enum):
    singleReturnFirst = 0x00
    singleReturnStrongest = 0x01
    dualReturn = 0x02
    tripleReturn = 0x03



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
    
class WriteLiDarExtrinsicParameters(Lidar):
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
        roll, pitch, yaw, x, y, z = struct.unpack(WriteLiDarExtrinsicParameters._PACK_FORMAT, payload)
        return WriteLiDarExtrinsicParameters(roll, pitch, yaw, x, y, z)

class WriteLiDarExtrinsicParametersResponse(Lidar, IsErrorResponseOnly):
    CMD_TYPE = Frame.Type.AKN
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

class ReadLidarExtrinsicParametersResponse(ReadLidarExtrinsicParameters):
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<B?IIIIII' #cmd_id, result, roll, pitch, yaw, x, y, z

    def __init__(self, result:bool, roll:int, pitch:int, yaw:int, x:int, y:int, z:int):
        self.result = result
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x = x
        self.y = y
        self.z = z

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.result, self.roll, self.pitch, self.yaw, self.x, self.y, self.z)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, result, roll, pitch, yaw, x, y, z = struct.unpack(ReadLidarExtrinsicParametersResponse.__PACK_FORMAT, payload)
        ReadLidarExtrinsicParametersResponse._check_cmd_id(cmd_id)
        return ReadLidarExtrinsicParameters(result, roll, pitch, yaw, x, y, z)

class TurnOnOffRainFogSuppression(Lidar):
    CMD_ID = 0x03 
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B?' # cmd_id, is_enable

    def __init__(self, is_enable:bool):
        super().__init__()
        self.is_enable = is_enable

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.is_enable)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, is_enable = struct.unpack(TurnOnOffRainFogSuppression.__PACK_FORMAT, payload)
        TurnOnOffRainFogSuppression._check_cmd_id(cmd_id)
        return TurnOnOffRainFogSuppression(is_enable)

class TurnOnOffRainFogSuppressionResponse( TurnOnOffRainFogSuppression):
    pass

class SetTurnOnOffFan(Lidar):
    CMD_ID = 0x04 
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B?' #cmd_id, is_enable

    def __init__(self, is_enable:bool):
        super().__init__()
        self.is_enable = is_enable

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.is_enable)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, is_enable = struct.unpack(SetTurnOnOffFan.__PACK_FORMAT, payload)
        SetTurnOnOffFan._check_cmd_id(cmd_id)
        return SetTurnOnOffFan(is_enable)

class SetTurnOnOffFanResponse( SetTurnOnOffFan):
    pass    

class GetTurnOnOffFanState(Lidar):
    CMD_ID = 0x05 
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #cmd_id

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(GetTurnOnOffFanState.__PACK_FORMAT, payload)
        GetTurnOnOffFanState._check_cmd_id(cmd_id)
        return GetTurnOnOffFanState()

class GetTurnOnOffFanStateResponse( GetTurnOnOffFanState):
    __PACK_FORMAT = '<B??' #cmd_id, result, state

    def __init__(self, result:bool, state:bool):
        super().__init__()
        self.result = result
        self.state = state

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.result, self.state)

    @staticmethod
    def payload(payload:bytes):
        cmd_id, result, state = struct.unpack(GetTurnOnOffFanStateResponse.__PACK_FORMAT, payload)
        GetTurnOnOffFanStateResponse._check_cmd_id(cmd_id)
        return GetTurnOnOffFanStateResponse(result, state)

class SetLiDarReturnMode(Lidar):
    CMD_ID = 0x06 
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = ''


    def __init__(self, mode:'ReturnMode|int'):
        super().__init__()
        if type(mode) is int:
            mode = self.ReturnMode(mode)
        elif type(mode) is not self.ReturnMode:
            raise TypeError(f'Bad Type for mode. Expected "ReturnMode" or "int". Got {type(mode)}')
        self._mode = mode.value

    @property
    def mode(self)->ReturnMode:
        return self.ReturnMode(self._mode)

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self._mode)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, mode = struct.unpack(SetLiDarReturnMode.__PACK_FORMAT, payload)
        SetLiDarReturnMode._check_cmd_id(cmd_id)
        return SetLiDarReturnMode(mode)

class SetLiDarReturnModeReponse( SetLiDarReturnMode):
    pass

class GetLiDarReturnMode(Lidar):
    CMD_ID = 0x07 
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #cmd_id

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(GetLiDarReturnMode.__PACK_FORMAT, payload)
        GetLiDarReturnMode._check_cmd_id(cmd_id)
        return GetLiDarReturnMode

class GetLiDarReturnModeResponse( GetLiDarReturnMode):
    __PACK_FORMAT = '<B?B' #cmd_id, result, mode

    def __init(self, result:bool, mode:ReturnMode):
        super().__init__()
        self.result = result
        if type(mode) is int:
            mode = ReturnMode
        elif type(mode) is not ReturnMode:
            raise TypeError(f'Bad type for mode. Expect "ReturnMode" or "int". Got {type(mode)}')
        self._mode = mode

    @property
    def mode(self)->ReturnMode:
        return ReturnMode(self._mode)

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.result, self._mode)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, result, mode = struct.unpack(GetLiDarReturnModeResponse.__PACK_FORMAT, payload)
        GetLiDarReturnModeResponse._check_cmd_id(cmd_id)
        return GetLiDarReturnModeResponse(result, mode)
    
class SetImuDataPushFrequency(Lidar):
    CMD_ID = 0x08 
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<BB' #CMD_ID, FREQ


    def __init__(self, freq:'Frequency|int'):
        super().__init__()
        # if type(freq) is int:
        #     freq = self.Frequency(freq)
        # elif type(freq) is not self.Frequency:
        #     raise TypeError(f'Bad type for freq. Expect "Frequency" or "int". Got {type(freq)}')
        # self._freq = freq.value

    # @property
    # def freq(self)->Frequency:
    #     return self.Frequency(self._freq)

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self._freq)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, freq = struct.unpack(SetImuDataPushFrequency.__PACK_FORMAT, payload)
        SetImuDataPushFrequency._check_cmd_id(cmd_id)
        return SetImuDataPushFrequency(freq)

class SetImuDataPushFrequencyResponse( SetImuDataPushFrequency):
    pass

class GetImuDataPushFrequency(Lidar):
    CMD_ID = 0x09 
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = 'B' #cmd_id

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(GetImuDataPushFrequency.__PACK_FORMAT, payload)
        GetImuDataPushFrequency._check_cmd_id(cmd_id)
        return GetImuDataPushFrequency()

class GetImuDataPushFrequencyResponse( GetImuDataPushFrequency):
    __PACK_FORMAT = '<BBB' #cmd_id, result, freq

    def __init__(self, result:bool, freq:'ImuDataPushFrequency|int'):
        super().__init__()
        self.result = result
        if type(freq) is int:
            freq = ImuDataPushFrequency(freq)
        elif type(freq) is not ImuDataPushFrequency:
            raise TypeError(f'Bad type for freq. Expect "ImuDataPushFrequency" or "int". Got {type(freq)}')
        self._freq = freq.value

    @property
    def freq(self)->ImuDataPushFrequency:
        return ImuDataPushFrequency(self._freq)

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.result, self._freq)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, result, freq = struct.unpack(GetImuDataPushFrequencyResponse.__PACK_FORMAT, payload)
        GetImuDataPushFrequencyResponse._check_cmd_id(cmd_id)
        return GetImuDataPushFrequencyResponse(result, freq)

class UpdateUtcSynchronizationTime(Lidar):
    CMD_ID = 0x0A 
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<BBBBBI' #cmd_id, year, month, day, hour, microseconds

    def __init__(self, year:int, month:int, day:int, hour:int, us:int):
        super().__init__()
        if year > 255:
            raise ValueError(f'year should be less then 255')
        datetime(year, month, day, hour)
        self.year = year
        self.month = month
        self.day = day
        self.hour = hour 
        self.us = us

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.year, self.month, self.day, self.hour, self.us)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, year, month, day, hour, us = struct.unpack(UpdateUtcSynchronizationTime.__PACK_FORMAT, payload)
        UpdateUtcSynchronizationTime._check_cmd_id(cmd_id)
        return UpdateUtcSynchronizationTime(year, month, day, hour, us)

class UpdateUtcSynchronizationTimeResponse( UpdateUtcSynchronizationTime):
    pass
