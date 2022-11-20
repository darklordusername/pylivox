#TODO description

import enum
import struct 
from .frame import Frame
from . import general

#project 
from .frame import Frame, FrameResponse


# def get_slices(data:bytes, item_size:int):
#     try:
#         slices = i*DeviceInfo.PACK_SIZE: DeviceInfo.PACK_SIZE] for i in range(devices_info / DeviceInfo.PACK_SIZE)]
#         ##################TODO GET SLICES. uSE IT TO MAKE LISTS 
#     except TypeError:
#         raise ValueError(f'Bad devices_info format')

class BroadCastCode:

    def _broadcast_pars(self, broadcast:'general.BroadcastMsg.Broadcast|bytes')->bytes:
        if type(broadcast) is bytes:
            broadcast = general.BroadcastMsg.Broadcast.from_payload(broadcast)
        elif type(broadcast) is not general.BroadcastMsg.Broadcast:
            raise TypeError(f'Bad type for broadcast. Expect "Broadcast" or "bytes". Got {type(broadcast)}')
        self._broadcast = broadcast.payload

    @property
    def broadcast(self)->general.BroadcastMsg.Broadcast:
        return general.BroadcastMsg.Broadcast(self._broadcast)


class Hub(Frame):
    #TODO description 
    CMD_SET = 2
    CMD_ID = None
    CMD_TYPE = None

    def __init__(self):
        #TODO description
        super().__init__()
        self.cmd_set = self.CMD_SET
        self.cmd_id = None

class _HubGeneralResponse(FrameResponse, Hub) :
    FRAME_TYPE = Frame.Type.AKN
    __PACK_FORMAT = '<B?' #CMD_ID, result


class QueryConnectedLidarDeviceCmd(Hub):
    CMD_ID = 0x00
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(QueryConnectedLidarDeviceCmd.__PACK_FORMAT, payload)
        QueryConnectedLidarDeviceCmd._check_cmd_id(cmd_id)
        return QueryConnectedLidarDeviceCmd()

class LidarPosition:
    class Enum(enum.Enum):
        left   = 1,
        middle = 2,
        right  = 3
    def _lidar_position_pars(self, position:'Enum|int'):
        if type(position) is int:
            position = self.Enum(position)
        elif type(position) is not self.Enum:
            raise TypeError(f'Bad type for position. Expect "Enum" or "int". Got {type(position)}')
        self._lidar_position = position.value

    @property
    def lidar_position(self)->Enum:
        return self.Enum(self._lidar_position)

class DeviceInfo:
    __PACK_FORMAT = f'<{general.BroadcastMsg.Broadcast.PACK_SIZE}sBIBB' #broadcast, reserved, dev_type, version, slot_id, lidar_position
    PACK_SIZE = struct.calcsize(__PACK_FORMAT)

    def __init__(self, broadcast:'general.BroadcastMsg.Broadcast|bytes', 
                        dev_type:'general.DeviceType|int', 
                        version:int, 
                        slot_id:int, 
                        lidar_position:'LidarPosition|int'):
        super().__init__()
        self._broadcast_pars(broadcast)
        self._dev_type_pars(dev_type)
        self._lidar_position_pars(lidar_position)
        self.version = version
        self.slot_id = slot_id

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self._broadcast, self._dev_type, self.version, self.slot_id, self._lidar_id )
    
    @staticmethod
    def from_payload(payload:bytes):
        broadcast, dev_type, version, slot_id, lidar_id = struct.unpack(DeviceInfo.__PACK_FORMAT, payload)
        DeviceInfo(broadcast, dev_type, version, slot_id, lidar_id)

class QueryConnectedLidarDeviceResponse(_HubGeneralResponse, QueryConnectedLidarDeviceCmd, 
        BroadCastCode, general.DeviceType, LidarPosition):

 
    def __init__(self, result:bool, devices_info:'list[DeviceInfo]|bytes'):
        super().__init__()
        self.result = result
        DeviceInfo = self.DeviceInfo
        if type(devices_info) is bytes:
            try:
                items = [ devices_info[i*DeviceInfo.PACK_SIZE: DeviceInfo.PACK_SIZE] for i in range(devices_info / DeviceInfo.PACK_SIZE)]
            except TypeError:
                raise ValueError(f'Bad devices_info format')
            devices_info = [ self.DeviceInfo.from_payload(item) for item in items]
        elif type(devices_info) is not list or any(map(lambda i: type(i) is not DeviceInfo, devices_info)):
            raise ValueError('Bad type for devices_info. Expected "List[DeviceInfo]" or "int"')
        self._devices_info = b''.join([info.payload for info in devices_info])
        self.num_devices = len(devices_info)

    @property
    def devices_info(self)->'list[DeviceInfo]':
        DeviceInfo = self.DeviceInfo
        items = [ self._devices_info[i*DeviceInfo.PACK_SIZE: DeviceInfo.PACK_SIZE] for i in range(self._devices_info / DeviceInfo.PACK_SIZE)]
        devices_info = [ self.DeviceInfo.from_payload(item) for item in items]
        return devices_info

    @property
    def payload(self)->bytes:
        return struct.pack(f'<B?B{self.num_devices * self.DeviceInfo.PACK_SIZE}B', self.CMD_ID, self.result, self.num_devices, self._devices_info)

    @staticmethod
    def from_payload(payload:bytes):
        try:
            DeviceInfo = QueryConnectedLidarDeviceResponse.DeviceInfo
            devices_info = payload[2:]
            items = [ devices_info[i*DeviceInfo.PACK_SIZE: DeviceInfo.PACK_SIZE] for i in range(devices_info / DeviceInfo.PACK_SIZE)]
        except TypeError:
            raise ValueError(f'Bad devices_info format')
        devices_info = [DeviceInfo.from_payload(item) for item in items]
        num_devices = len(items)
        cmd_id = int.from_bytes(payload[0], 'little')
        result = bool(payload[1])
        _num_devices = int.from_bytes(payload[2], 'little')
        if num_devices != _num_devices:
            raise ValueError(f'Bad count value in payload')
        return QueryConnectedLidarDeviceResponse(result, devices_info)
        

class SetLidarModeCmd(Hub, BroadCastCode, general.WorkState.Lidar):
    CMD_ID = 0x01
    FRAME_TYPE = Frame.Type.CMD

    class Config (BroadCastCode):
        __PACK_FORMAT = f'<{general.BroadcastMsg.Broadcast.PACK_SIZE}sB' #broadcast, power_mode
        PACK_SIZE = struct.calcsize(__PACK_FORMAT)

        def __inti__(self, broadcast:'general.BroadcastMsg.Broadcast|bytes', power_mode:'general.WorkState.Lidar|int'):
            super().__init__()
            self._broadcast_pars(broadcast)
            self._work_state_pars(power_mode)

        @property
        def payload(self)->bytes:
            return struct.pack(self.__PACK_FORMAT, self._broadcast, self._work_state)

        @classmethod
        def from_payload(cls, payload:bytes):
            broadcast, power_mode = struct.unpack(cls.__PACK_FORMAT, payload)
            return cls(broadcast, power_mode)

    def __init__(self, configs:'list[SetLidarModeCmd.Config]|bytes'):
        super().__init__()
        if type(configs) is bytes:
            items = []
        self.__PACK_FORMAT = f'<BB{self.Config.PACK_SIZE}' #CMD_ID

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(SetLidarModeCmd.__PACK_FORMAT, payload)
        SetLidarModeCmd._check_cmd_id(cmd_id)
        return SetLidarModeCmd()


class TurnOnOffDesignatedSlotPower(Hub):
    CMD_ID = 0x02
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class WriteLiDarExtrinsicParameters(Hub):
    CMD_ID = 0x03
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class ReadLiDarExtrinsicParameters(Hub):
    CMD_ID = 0x04
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class QueryLiDarDeviceStatus(Hub):
    CMD_ID = 0x05
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class TurnOnOffHubCalculationOfExtrinsicParameters(Hub):
    CMD_ID = 0x06
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class TurnOnOffLiDarRainFogSuppression(Hub):
    CMD_ID = 0x07
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class QueryHubSlotPowerStatus(Hub):
    CMD_ID = 0x08
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class SetLiDarTurnOnOffFan(Hub):
    CMD_ID = 0x09
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class GetLiDarTurnOnOffFanState(Hub):
    CMD_ID = 0x0A
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class SetLiDarReturnMode(Hub):
    CMD_ID = 0x0B
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class GetLiDarReturnMode(Hub):
    CMD_ID = 0x0C
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class SetLiDarImuDataPushFrequency(Hub):
    CMD_ID = 0x0D
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()


class GetLiDarImuDataPushFrequency(Hub):
    CMD_ID = 0x0E
    FRAME_TYPE = Frame.Type.CMD
    __PACK_FORMAT = '<B' #CMD_ID

    def __init__(self):
        super().__init__()

    @property
    def payload(self)->bytes:
        return struct.pack(self.__PACK_FORMAT, self.CMD_ID,)

    @staticmethod
    def from_payload(payload:bytes):
        cmd_id = struct.unpack(XXX.__PACK_FORMAT, payload)
        XXX._check_cmd_id(cmd_id)
        return XXX()
