import time
import struct
from enum import Enum
#proj
import log 
# from control.frame import DeviceType

logger = log.getLogger(__name__)

class LidarStatus:
    class Temperature(Enum):
        NORMAL    = 0
        HIGH_LOW  = 1
        EXTREMELY = 2

    class Voltage(Enum):
       NORMAL     = 0   
       HIGH       = 1 
       EXTREMELY  = 2     

    class Motor(Enum):
        NORMAL  = 0
        WARNING = 1
        ERROR   = 2 
    
    class TimeSync(Enum):
        NO_TIME_SYNC = 0
        PTP_1588 = 1
        GPS = 2
        PPS = 3
        ABNORMAL = 4

    class System(Enum):
        NORMAL  = 0  
        WARNING = 1   
        ERROR   = 2 



class DataType0:
    _PACK_FORMAT = '<IIIB' #X, Y, Z, reflectivity

class DataType1:
    _PACK_FORMAT = '<IHHB' #Depth, Zenith angle, Azimuth, reflectivity

class DataType2:
    _PACK_FORMAT = '<IIIBB' #X, Y, Z, reflectivity, tag

class DataType3:
    _PACK_FORMAT = '<IHHBB' #Depth, Zenith angle, Azimuth, reflectivity, tag

class DataType4:
    _PACK_FORMAT = '<IIIB' #X, Y, Z, reflectivity

class DataType5:
    _PACK_FORMAT = '<IIIB' #X, Y, Z, reflectivity

class DataType6:
    _PACK_FORMAT = '<IIIB' #X, Y, Z, reflectivity

class DataType7:
    _PACK_FORMAT = '<IIIB' #X, Y, Z, reflectivity



class Frame:
    PROTOCOL_VERSION = 5
    SLOT_ID = 1
    LIDAR_ID = 1

    @property
    def header(self)->bytes:
        return struct.pack('<BBBIBBQ', #protocol version, slot_id,lidar_id,reserved,status_code,time_type,data_type,time, 
                            self.PROTOCOL_VERSION,
                            self.SLOT_ID,
                            self.LIDAR_ID,
                            0, #status code
                            LidarStatus.TimeSync.NO_TIME_SYNC.value,
                            0, #datatype 
                            int(time.time() * 1000000000),
                            )

    @property
    def payload(self):
        n = struct.calcsize(DataType0._PACK_FORMAT)
        return bytes(n)*100

    @property
    def frame(self)->bytes:
        return self.header + self.payload
