#TODO description

#Imports
import crcmod
import enum
import struct
import abc
import log
logger = log.getLogger(__name__)

crc32 = crcmod.mkCrcFun(0x104C11DB7, rev=True, initCrc=0x564F580A, xorOut=0xFFFFFFFF)
crc16 = crcmod.mkCrcFun(0x11021, rev=True, initCrc=0x4C49)

class Frame(abc.ABC):
    #TODO description 

    class Type(enum.Enum):
        CMD = 0x00
        AKN = 0x01
        MSG = 0x02

    START = 0xAA
    VERSION = 0x01
    FRAME_TYPE:Type = Type.CMD

    def __init__(self, seq=0):
        self.seq = seq
        self.crc16 = crc16
        self.crc32 = crc32
        self.header_length = 9

    @classmethod
    def _check_cmd_id(cls, cmd_id:int, exception:bool=True)->bool:
        error = cmd_id != cls.CMD_ID
        if error and exception:
            raise ValueError(f'Bad cmd id: {cmd_id}. Expected {cls.CMD_ID}. Got {cmd_id}')
        else:
            return error

    @property
    def frame(self):
        payload_length = len(self.payload)
        return struct.pack(f'<B{payload_length}B', self.CMD_SET, self.payload)


    @property
    def header(self):
        length = len(self.frame) + self.header_length + 4 
        if length > 1400:
            raise ValueError(f"{self} is too big. Max 1400 but pack is {length}")
        return (self.START.to_bytes(1, 'little')
            + self.VERSION.to_bytes(1, 'little')
            + length.to_bytes(2, 'little')
            + self.FRAME_TYPE.value.to_bytes(1, 'little')
            + self.seq.to_bytes(2, 'little')
        )

    @property
    def header_crc(self):
        return self.crc16(self.header).to_bytes(2, 'little')

    @property
    def crc(self):
        return self.crc32(self.header + self.header_crc + self.frame).to_bytes(4, 'little')

    @abc.abstractproperty
    def frame(self):
        raise NotImplementedError

    @property
    def pack(self):
        return self.header + self.header_crc + self.frame + self.crc

    
    @abc.abstractproperty
    def payload(self):
        raise NotImplementedError

    @staticmethod
    def from_frame(cls, frame:bytes):
        raise NotImplementedError
        payload_length = len(frame)
        cmd_set, payload = struct.unpack(f'<B{payload_length}B', frame)
        if cmd_set != cls.CMD_SET:
            raise ValueError(f'Bad cmd set: {cmd_set}. Expected {cls.CMD_SET}')
        return General(payload)

    @abc.abstractstaticmethod
    def from_payload(payload:bytes):
        raise NotImplementedError

    @staticmethod
    def from_frame(frame:bytes):
        #TODO description
        header_packed = frame[0:7]
        header_crc_income = int.from_bytes(frame[7:9], 'little')
        cmd_packed = frame[9:-4]
        frame_crc_income = int.from_bytes(frame[-4:], 'little')
        # calc_header_crc = crc16(header_packed)
        frame_crc = crc32(frame[:-4])
        if frame_crc != frame_crc_income:
            raise ValueError('Frame crc error')
        header_crc = crc16(header_packed)
        if header_crc != header_crc_income:
            raise ValueError('Header crc error')
        start, version, length, cmd_type, seq = struct.unpack('<BBHBH', header_packed)
        raise NotImplementedError
        # if start != Command._START:
        #     raise ValueError(f'Wrong packed start byte {start}. Expected {Command._START}')
        # if version != Command._VERSION:
        #     raise ValueError(f'Unsupported version {version}. Expected {Command._VERSION}')
        # if cmd_type == Command.type.CMD.value:
        #     cmd_body_packed = cmd_packed[2:]
        #     cmd_header_packed = cmd_packed[:2]
        #     cmd_set, cmd_id = struct.unpack(cmd_header_packed)
        #     return COMMAND_BY_CMD_ID[cmd_id].from_frame(cmd_packed)
        # elif cmd_type == Command.type.AKN.value:
        #     pass
        # elif cmd_type == Command.type.MSG.value:
        #     pass

    def __repr__(self):
        return f'{{{self.__class__.__name__}}}'

class FrameResponse(Frame):

    def __init__(self):
        super().__init__()