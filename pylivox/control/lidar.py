#TODO Description

#std libs
from faulthandler import is_enabled
import struct 
import enum
from datetime import datetime

#project 
from .frame import Frame, FrameResponse


class Lidar(Frame):
    #TODO description 
    CMD_SET = 1
    CMD_ID = None
    CMD_TYPE = None

    def __init__(self):
        #TODO description
        super().__init__()
        self.cmd_set = self.CMD_SET
        self.cmd_id = None

class _LidarGeneralResponse(FrameResponse, Lidar) :
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<B?' #CMD_ID, result

class ImuDataPushFrequency(enum.Enum):
    Hz_0 = 0
    Hz_200 = 1

class ReturnMode(enum.Enum):
    singleReturnFirst = 0x00
    singleReturnStrongest = 0x01
    dualReturn = 0x02
    tripleReturn = 0x03

class PowerMode(enum.Enum):
    normal      = 0x01
    powerSaving = 0x02     
    standby     = 0x03 

class SetMode(Lidar):
    CMD_ID = 0x00 
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<BB'#cmd_id, lidar_mode

    def __init__(self, lidar_mode:'PowerMode|int'):
        super().__init__()
        if type(lidar_mode) is int:
            lidar_mode = PowerMode(lidar_mode)
        elif type(lidar_mode) is not PowerMode:
            raise TypeError(f'Bad type for lidar_mode. Expect "PowerMode" or "int". Got {type(lidar_mode)}')
        self._lidar_mode = lidar_mode.value

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self._lidar_mode)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, lidar_mode = struct.unpack(SetMode.__PACK_FORMAT, payload)
        SetMode._check_cmd_id(cmd_id)
        return SetMode(lidar_mode)

class SetModeResponse(SetMode):
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<BB' #cmd_id, Result
    
    class Result(enum.Enum):
        Success = 0
        Fail = 1
        Switching = 2

    def __init__(self, result:'Result|int'):
        super().__init__()
        if type(result) is int:
            result = self.Result(result)
        elif type(result) is not self.Result:
            raise TypeError(f'Bad type for result. Expect "Result" or "int". Got {type(result)}')
        self._result = result

    @property
    def result(self)->Result:
        return self.Result(self._result)

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self._result)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, result = struct.unpack(SetModeResponse.__PACK_FORMAT, payload)
        SetModeResponse._check_cmd_id(cmd_id)
        return SetModeResponse(result)
    

class WriteLiDarExtrinsicParameters(Lidar):
    CMD_ID = 0x01 
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<BIIIIII' #cmd_id, roll, pitch, yaw, x, y, z

    def __init__(self, roll:int, pitch:int, yaw:int, x:int, y:int, z:int):
        super().__init__()
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x = x
        self.y = y
        self.z = z

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self.roll, self.pitch, self.yaw, self.x, self.y, self.z)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, roll, pitch, yaw, x, y, z = struct.unpack(WriteLiDarExtrinsicParameters.__PACK_FORMAT, payload)
        WriteLiDarExtrinsicParameters._check_cmd_id(cmd_id)
        return WriteLiDarExtrinsicParameters(roll, pitch, yaw, x, y, z)


class ReadLidarExtrinsicParameters(Lidar):
    CMD_ID = 0x02 
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #cmd_id

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(ReadLidarExtrinsicParameters.__PACK_FORMAT, payload)
        ReadLidarExtrinsicParameters._check_cmd_id(cmd_id)
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

class TurnOnOffRainFogSuppressionResponse(_LidarGeneralResponse, TurnOnOffRainFogSuppression):
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

class SetTurnOnOffFanResponse(_LidarGeneralResponse, SetTurnOnOffFan):
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

class GetTurnOnOffFanStateResponse(_LidarGeneralResponse, GetTurnOnOffFanState):
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

class SetLiDarReturnModeReponse(_LidarGeneralResponse, SetLiDarReturnMode):
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

class GetLiDarReturnModeResponse(_LidarGeneralResponse, GetLiDarReturnMode):
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
        if type(freq) is int:
            freq = self.Frequency(freq)
        elif type(freq) is not self.Frequency:
            raise TypeError(f'Bad type for freq. Expect "Frequency" or "int". Got {type(freq)}')
        self._freq = freq.value

    @property
    def freq(self)->Frequency:
        return self.Frequency(self._freq)

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID, self._freq)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id, freq = struct.unpack(SetImuDataPushFrequency.__PACK_FORMAT, payload)
        SetImuDataPushFrequency._check_cmd_id(cmd_id)
        return SetImuDataPushFrequency(freq)

class SetImuDataPushFrequencyResponse(_LidarGeneralResponse, SetImuDataPushFrequency):
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

class GetImuDataPushFrequencyResponse(_LidarGeneralResponse, GetImuDataPushFrequency):
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

class UpdateUtcSynchronizationTimeResponse(_LidarGeneralResponse, UpdateUtcSynchronizationTime):
    pass
