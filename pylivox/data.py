import time
import os
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
    TYPE = 0
    _PACK_FORMAT = '<IIIB' #X, Y, Z, reflectivity
    N = 100

class DataType1:
    TYPE = 1
    _PACK_FORMAT = '<IHHB' #Depth, Zenith angle, Azimuth, reflectivity
    N = 100

class DataType2:
    TYPE = 2
    _PACK_FORMAT = '<IIIBB' #X, Y, Z, reflectivity, tag
    N = 96

class DataType3:
    TYPE = 3
    _PACK_FORMAT = '<IHHBB' #Depth, Zenith angle, Azimuth, reflectivity, tag
    N = 96

# class DataType4:
#     TYPE = 4
#     _PACK_FORMAT = '<IIIB' #X, Y, Z, reflectivity
#     N = 48

# class DataType5:
#     TYPE = 5
#     _PACK_FORMAT = '<IIIB' #X, Y, Z, reflectivity
#     N = 48

# class DataType6:
#     TYPE = 6
#     _PACK_FORMAT = '<IIIB' #X, Y, Z, reflectivity
#     N = 1

# class DataType7:
#     TYPE = 7
#     _PACK_FORMAT = '<IIIB' #X, Y, Z, reflectivity
#     N = 30


class Frame:
    PROTOCOL_VERSION = 5
    SLOT_ID = 1
    LIDAR_ID = 1
    DATA = DataType3()

    @property
    def header(self)->bytes:
        return struct.pack('<BBBxIBBQ', #protocol version, slot_id,lidar_id,reserved,status_code,time_type,data_type,time, 
                            self.PROTOCOL_VERSION,
                            self.SLOT_ID,
                            self.LIDAR_ID,
                            0, #status code
                            LidarStatus.TimeSync.NO_TIME_SYNC.value,
                            self.DATA.TYPE, #datatype 
                            int(time.time() * 1000000000),
                            )

    @property
    def payload(self):
        length = struct.calcsize(self.DATA._PACK_FORMAT)
        return os.urandom(length * self.DATA.N)

    @property
    def frame(self)->bytes:
        return self.header + self.payload
