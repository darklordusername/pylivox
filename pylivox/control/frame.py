#TODO description

#std
import enum
import struct
import abc
import logging
#libs
import crcmod
#projs

crc32 = crcmod.mkCrcFun(0x104C11DB7, rev=True, initCrc=0x564F580A, xorOut=0xFFFFFFFF)
crc16 = crcmod.mkCrcFun(0x11021, rev=True, initCrc=0x4C49)

logger = logging.getLogger(__name__)

class Frame(abc.ABC):
    """Low level wrapper for command,response """

    class Type(enum.Enum):
        CMD = 0
        AKN = 1
        MSG = 2

    class Set(enum.Enum):
        GENERAL = 0
        LIDAR = 1
        HUB = 2

    class SetGeneral(enum.Enum):
        BROADCAST_MESSAGE                                    = 0x00  
        HANDSHAKE                                            = 0x01
        QUERY_DEVICE_INFORMATION                             = 0x02
        HEARTBEAT                                            = 0x03
        START_STOP_SAMPLING                                  = 0x04
        CHANGE_COORDINATE_SYSTEM                             = 0x05
        DISCONNECT                                           = 0x06
        PUSH_ABNORMAL_STATUS_INFORMATION                     = 0x07
        CONFIGURE_STATIC_DYNAMIC_IP                          = 0x08
        GET_DEVICE_IP_INFORMATION                            = 0x09
        REBOOT_DEVICE                                        = 0x0A
        WRITE_CONFIGURATION_PARAMETERS                       = 0x0B
        READ_CONFIGURATION_PARAMETERS                        = 0x0C

    class SetLidar(enum.Enum):                    
        SET_MODE                                             = 0x00  
        WRITE_LIDAR_EXTRINSIC_PARAMETERS                    = 0x01                           
        READ_LIDAR_EXTRINSIC_PARAMETERS                     = 0x02                          
        TURN_ON_OFF_RAIN_FOG_SUPPRESSION                     = 0x03                          
        SET_TURN_ON_OFF_FAN                                  = 0x04             
        GET_TURN_ON_OFF_FAN_STATE                            = 0x05                   
        SET_LIDAR_RETURN_MODE                                = 0x06                
        GET_LIDAR_RETURN_MODE                                = 0x07                
        SET_IMU_DATA_PUSH_FREQUENCY                          = 0x08                     
        GET_IMU_DATA_PUSH_FREQUENCY                          = 0x09                     
        UPDATE_UTC_SYNCHRONIZATION_TIME                      = 0x0A  
        
    class SetHub(enum.Enum):
        QUERY_CONNECTED_LIDAR_DEVICE                         =  0x00                 
        SET_LIDAR_MODE                                      =  0x01 
        TURN_ON_OFF_DESIGNATED_SLOT_POWER                    =  0x02                   
        WRITE_LIDAR_EXTRINSIC_PARAMETERS                    =  0x03                   
        READ_LIDAR_EXTRINSIC_PARAMETERS                     =  0x04                  
        QUERY_LIDAR_DEVICE_STATUS                           =  0x05            
        TURN_ON_OFF_HUB_CALCULATION_OF_EXTRINSIC_PARAMETERS  =  0x06                                     
        TURN_ON_OFF_LIDAR_RAIN_FOG_SUPPRESSION              =  0x07                         
        QUERY_HUB_SLOT_POWER_STATUS                          =  0x08             
        SET_LIDAR_TURN_ON_OFF_FAN                           =  0x09            
        GET_LIDAR_TURN_ON_OFF_FAN_STATE                     =  0x0A                  
        SET_LIDAR_RETURN_MODE                               =  0x0B        
        GET_LIDAR_RETURN_MODE                               =  0x0C        
        SET_LIDAR_IMU_DATA_PUSH_FREQUENCY                   =  0x0D                    
        GET_LIDAR_IMU_DATA_PUSH_FREQUENCY                   =  0x0E       

    START = 0xAA
    VERSION = 0x01
    CMD_TYPE:Type = None
    HEADER_LENGTH = 9
    FRAME_CRC_LENGTH = 4
    FRAME_MAX_LENGTH = 1400

    def __init__(self, seq=0):
        self.seq = seq

    @classmethod
    def _check_cmd_id(cls, cmd_id:int, exception:bool=True)->bool:
        error = cmd_id != cls.CMD_ID
        if error and exception:
            raise ValueError(f'Bad cmd id: {cmd_id}. Expected {cls.CMD_ID}. Got {cmd_id}')
        else:
            return error

    @property
    def frame(self):
        return self.header + self.header_crc + self.payload + self.frame_crc 


    @property
    def header(self):
        length = len(self.payload) + self.HEADER_LENGTH + self.FRAME_CRC_LENGTH
        if length > self.FRAME_MAX_LENGTH:
            raise ValueError(f"{self} is too big. Max {self.FRAME_MAX_LENGTH} but pack is {length}")
        return (self.START.to_bytes(1, 'little')
            + self.VERSION.to_bytes(1, 'little')
            + length.to_bytes(2, 'little')
            + self.CMD_TYPE.value.to_bytes(1, 'little')
            + self.seq.to_bytes(2, 'little')
        )

    @property
    def header_crc(self):
        return crc16(self.header).to_bytes(2, 'little')

    @property
    def frame_crc(self):
        return crc32(self.header + self.header_crc + self.payload).to_bytes(4, 'little')

    @abc.abstractproperty
    def payload(self):
        raise NotImplementedError

    @abc.abstractstaticmethod
    def from_payload(payload:bytes):
        raise NotImplementedError

    @staticmethod
    def from_frame_set_type_value(cmd_set, cmd_type, cmd_id):
        pass

    @staticmethod
    def crc_header(data:bytes):
        return crc16(data)
    
    @staticmethod
    def crc(data:bytes):
        return crc32(data)


    def __repr__(self):
        return f'{{{self.__class__.__name__}}}'

class FrameResponse(Frame):

    def __init__(self):
        super().__init__()


class Cmd(Frame):
    CMD_SET = None
    CMD_ID = None

    def payload(self, payload_body:bytes):
        format = f'<BB{len(payload_body)}B' #CMD_SET, CMD_ID, BODY
        return struct.pack(format, self.CMD_SET.value, self.CMD_ID.value, *payload_body )


class IsErrorResponse(Cmd):
    CMD_TYPE = Frame.Type.AKN
    _PACK_FORMAT = '<?' # is_error

    @property
    def is_error(self)->bool:
        return self._is_error

    @is_error.setter
    def is_error(self, value:bool):
        if type(value) is not bool:
            raise TypeError
        self._is_error = value


class IsErrorResponseOnly(IsErrorResponse): 

    def __init__(self, is_error:bool=False):
        super().__init__()
        self.is_error = is_error

    @property
    def payload(self)->bytes:
        payload_body = struct.pack(self._PACK_FORMAT, self.is_error)
        return super().payload(payload_body)

    @classmethod
    def from_payload(cls, payload:bytes):
        is_error, = struct.unpack(cls._PACK_FORMAT, payload)
        return cls(is_error)

