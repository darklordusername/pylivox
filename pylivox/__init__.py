"""
Protocol livox
"""
import threading
import time
import enum
import socket
import struct
import crcmod
import abc
import ipaddress
import log
logger = log.getLogger(__name__)

crc32 = crcmod.mkCrcFun(0x104C11DB7, rev=True, initCrc=0x564F580A, xorOut=0xFFFFFFFF)
crc16 = crcmod.mkCrcFun(0x11021, rev=True, initCrc=0x4C49)
class Command(abc.ABC):

    _START = b'\xAA'
    _VERSION = b'\01'
    
    class type(enum.Enum):
        CMD = b'\x00'
        AKN = b'\x01'
        MSG = b'\x02'

    def __init__(self, seq=0):
        self.seq = seq
        self.crc16 = crc16
        self.crc32 = crc32
        self.header_length = 9

    @property
    def header(self):
        length = len(self.data) + self.header_length + 4 
        if length > 1400:
            raise ValueError(f"{self} is too big. Max 1400 but pack is {length}")
        return (self._START 
            + self._VERSION 
            + length.to_bytes(2, 'little')
            + Command.type.MSG.value
            + self.seq.to_bytes(2, 'little')
        )

    @property
    def header_crc(self):
        return self.crc16(self.header).to_bytes(2, 'little')

    @property
    def crc(self):
        return self.crc32(self.header + self.header_crc + self.data).to_bytes(4, 'little')

    @property
    def data(self):
        raise ValueError(f'No data available for {self}')

    @property
    def pack(self):
        return self.header + self.header_crc + self.data + self.crc

    def __repr__(self):
        return f'{{{self.__class__.__name__}}}'

    # class Lidar(Command):
    #     SET_MODE                                             = 0X00         
    #     WRITE_LIDAR_EXTRINSIC_PARAMETERS                     = 0X01         
    #     READ_LIDAR_EXTRINSIC_PARAMETERS                      = 0X02         
    #     TURN_ON_OFF_RAIN_FOG_SUPPRESSION                     = 0X03         
    #     SET_TURN_ON_OFF_FAN                                  = 0X04         
    #     GET_TURN_ON_OFF_FAN_STATE                            = 0X05         
    #     SET_LIDAR_RETURN_MODE                                = 0X06         
    #     GET_LIDAR_RETURN_MODE                                = 0X07         
    #     SET_IMU_DATA_PUSH_FREQUENCY                          = 0X08         
    #     GET_IMU_DATA_PUSH_FREQUENCY                          = 0X09         
    #     UPDATE_UTC_SYNCHRONIZATION_TIME                      = 0X0A     

    # class Hub(Command):
    #     QUERY_CONNECTED_LIDAR_DEVICE                         = 0X00       
    #     SET_LIDAR_MODE                                       = 0X01       
    #     TURN_ON_OFF_DESIGNATED_SLOT_POWER                    = 0X02       
    #     WRITE_LIDAR_EXTRINSIC_PARAMETERS                     = 0X03       
    #     READ_LIDAR_EXTRINSIC_PARAMETERS                      = 0X04       
    #     QUERY_LIDAR_DEVICE_STATUS                            = 0X05       
    #     TURN_ON_OFF_HUB_CALCULATION_OF_EXTRINSIC_PARAMETERS  = 0X06                 
    #     TURN_ON_OFF_LIDAR_RAIN_FOG_SUPPRESSION               = 0X07       
    #     QUERY_HUB_SLOT_POWER_STATUS                          = 0X08       
    #     SET_LIDAR_TURN_ON_OFF_FAN                            = 0X09       
    #     GET_LIDAR_TURN_ON_OFF_FAN_STATE                      = 0X0A       
    #     SET_LIDAR_RETURN_MODE                                = 0X0B       
    #     GET_LIDAR_RETURN_MODE                                = 0X0C       
    #     SET_LIDAR_IMU_DATA_PUSH_FREQUENCY                    = 0X0D       
    #     GET_LIDAR_IMU_DATA_PUSH_FREQUENCY                    = 0X0E       



class General(Command):
    CMD_SET = 0
    def __init__(self):
        super().__init__()
        self.cmd_set = self.CMD_SET
    @property
    def data(self):
        return struct.pack('B', self.cmd_set)

    HANDSHAKE                                            = 0X01
    QUERY_DEVICE_INFORMATION                             = 0X02
    HEARTBEAT                                            = 0X03
    START_STOP_SAMPLING                                  = 0X04
    CHANGE_COORDINATE_SYSTEM                             = 0X05
    DISCONNECT                                           = 0X06
    PUSH_ABNORMAL_STATUS_INFORMATION                     = 0X07
    CONFIGURE_STATIC_DYNAMIC_IP                          = 0X08
    GET_DEVICE_IP_INFORMATION                            = 0X09
    REBOOT_DEVICE                                        = 0X0A
    WRITE_CONFIGURATION_PARAMETERS                       = 0X0B
    READ_CONFIGURATION_PARAMETERS                        = 0x0C
    
class DeviceType(enum.Enum):
    Hub     = 0     
    Mid40   = 1 
    Tele15  = 2
    Horizon = 3

class Broadcast(General):
    CMD_ID = 0

    def __init__(self, serial:int, dev_type:DeviceType,  ): #16bytes broadcast_code
        #TODO description
        super().__init__()
        #Check
        serial.to_bytes(14, 'little')
        if not isinstance(dev_type, DeviceType):
            raise TypeError
        #Save
        self.cmd_id = self.CMD_ID
        self.dev_type = dev_type 
        self.serial = serial
        self.ip_range_code = ord('3')
        
    @property
    def serial(self)->int:
        return int.from_bytes(self._serial, 'little')

    @serial.setter
    def serial(self, nval:int):
        self._serial = nval.to_bytes(14, 'little')

    @property
    def data(self):
        return super().data + struct.pack(f'<B14BBxB2x', self.cmd_id,  *self._serial, self.ip_range_code, self.dev_type.value)


lidars = []

def _task_heartbeat():
    while True:
        try:
            for lidar in lidars:
                cmd = Broadcast(dev_type=DeviceType.Mid40, serial=0x1234)
                lidar.send(cmd, ipaddress.IPv4Address('255.255.255.255'), 55000)
                logger.info('broadcast')
            time.sleep(1)
        except Exception as e:
            pass

_t = threading.Thread(target=_task_heartbeat, name='broadcast', daemon=True)        
_t.start()

class Lidar:

    #broadcast udp 65000->55000
    #wait master send info snd AKN
    #task with heart beat
    #**NOTE**: *The Livox LiDAR/Hub will re-broadcast if heartbeat is timeout.*
    def __init__(self):
        #prepare udp socket
        self.seq = 0
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._socket.bind((str(ipaddress.IPv4Address("0.0.0.0")), 65000)) 
        #
        #registering handlers for commands 
        # self._handlers =  {
        #     Command.General.BROADCAST                                        : self._handler_broadcast                                           ,
        #     Command.General.HANDSHAKE                                        : self._handler_handshake                                           ,
        #     Command.General.QUERY_DEVICE_INFORMATION                         : self._handler_query_device_information                            ,
        #     Command.General.HEARTBEAT                                        : self._handler_heartbeat                                           ,
        #     Command.General.START_STOP_SAMPLING                              : self._handler_start_stop_sampling                                 ,
        #     Command.General.CHANGE_COORDINATE_SYSTEM                         : self._handler_change_coordinate_system                            ,
        #     Command.General.DISCONNECT                                       : self._handler_disconnect                                          ,
        #     Command.General.PUSH_ABNORMAL_STATUS_INFORMATION                 : self._handler_push_abnormal_status_information                    ,
        #     Command.General.CONFIGURE_STATIC_DYNAMIC_IP                      : self._handler_configure_static_dynamic_ip                         ,
        #     Command.General.GET_DEVICE_IP_INFORMATION                        : self._handler_get_device_ip_information                           ,
        #     Command.General.REBOOT_DEVICE                                    : self._handler_reboot_device                                       ,
        #     Command.General.WRITE_CONFIGURATION_PARAMETERS                   : self._handler_write_configuration_parameters                      ,
        #     Command.General.READ_CONFIGURATION_PARAMETERS                    : self._handler_read_configuration_parameters                       ,
        #     Command.Lidar.SET_MODE                                           : self._handler_set_mode                                            ,        
        #     Command.Lidar.WRITE_LIDAR_EXTRINSIC_PARAMETERS                   : self._handler_write_lidar_extrinsic_parameters                    ,        
        #     Command.Lidar.READ_LIDAR_EXTRINSIC_PARAMETERS                    : self._handler_read_lidar_extrinsic_parameters                     ,        
        #     Command.Lidar.TURN_ON_OFF_RAIN_FOG_SUPPRESSION                   : self._handler_turn_on_off_rain_fog_suppression                    ,        
        #     Command.Lidar.SET_TURN_ON_OFF_FAN                                : self._handler_set_turn_on_off_fan                                 ,        
        #     Command.Lidar.GET_TURN_ON_OFF_FAN_STATE                          : self._handler_get_turn_on_off_fan_state                           ,        
        #     Command.Lidar.SET_LIDAR_RETURN_MODE                              : self._handler_set_lidar_return_mode                               ,        
        #     Command.Lidar.GET_LIDAR_RETURN_MODE                              : self._handler_get_lidar_return_mode                               ,        
        #     Command.Lidar.SET_IMU_DATA_PUSH_FREQUENCY                        : self._handler_set_imu_data_push_frequency                         ,        
        #     Command.Lidar.GET_IMU_DATA_PUSH_FREQUENCY                        : self._handler_get_imu_data_push_frequency                         ,        
        #     Command.Lidar.UPDATE_UTC_SYNCHRONIZATION_TIME                    : self._handler_update_utc_synchronization_time                     ,    
        #     Command.Hub.QUERY_CONNECTED_LIDAR_DEVICE                         : self._handler_query_connected_lidar_device                        ,       
        #     Command.Hub.SET_LIDAR_MODE                                       : self._handler_set_lidar_mode                                      ,       
        #     Command.Hub.TURN_ON_OFF_DESIGNATED_SLOT_POWER                    : self._handler_turn_on_off_designated_slot_power                   ,       
        #     Command.Hub.WRITE_LIDAR_EXTRINSIC_PARAMETERS                     : self._handler_write_lidar_extrinsic_parameters                    ,       
        #     Command.Hub.READ_LIDAR_EXTRINSIC_PARAMETERS                      : self._handler_read_lidar_extrinsic_parameters                     ,       
        #     Command.Hub.QUERY_LIDAR_DEVICE_STATUS                            : self._handler_query_lidar_device_status                           ,       
        #     Command.Hub.TURN_ON_OFF_HUB_CALCULATION_OF_EXTRINSIC_PARAMETERS  : self._handler_turn_on_off_hub_calculation_of_extrinsic_parameters ,                 
        #     Command.Hub.TURN_ON_OFF_LIDAR_RAIN_FOG_SUPPRESSION               : self._handler_turn_on_off_lidar_rain_fog_suppression              ,       
        #     Command.Hub.QUERY_HUB_SLOT_POWER_STATUS                          : self._handler_query_hub_slot_power_status                         ,       
        #     Command.Hub.SET_LIDAR_TURN_ON_OFF_FAN                            : self._handler_set_lidar_turn_on_off_fan                           ,       
        #     Command.Hub.GET_LIDAR_TURN_ON_OFF_FAN_STATE                      : self._handler_get_lidar_turn_on_off_fan_state                     ,       
        #     Command.Hub.SET_LIDAR_RETURN_MODE                                : self._handler_set_lidar_return_mode                               ,       
        #     Command.Hub.GET_LIDAR_RETURN_MODE                                : self._handler_get_lidar_return_mode                               ,       
        #     Command.Hub.SET_LIDAR_IMU_DATA_PUSH_FREQUENCY                    : self._handler_set_lidar_imu_data_push_frequency                   ,       
        #     Command.Hub.GET_LIDAR_IMU_DATA_PUSH_FREQUENCY                    : self._handler_get_lidar_imu_data_push_frequency                   ,       
        # }
        lidars.append(self)

    def send(self, cmd:Command, address:ipaddress.IPv4Address, port:int):
        self.seq += 1
        cmd.seq = self.seq
        self._socket.sendto(cmd.pack, (str(address), port) )



    def _handler_broadcast                                          (self):
        pass

    def _handler_handshake                                          (self):
        pass

    def _handler_query_device_information                           (self):
        pass

    def _handler_heartbeat                                          (self):
        pass

    def _handler_start_stop_sampling                                (self):
        pass

    def _handler_change_coordinate_system                           (self):
        pass

    def _handler_disconnect                                         (self):
        pass

    def _handler_push_abnormal_status_information                   (self):
        pass

    def _handler_configure_static_dynamic_ip                        (self):
        pass

    def _handler_get_device_ip_information                          (self):
        pass

    def _handler_reboot_device                                      (self):
        pass

    def _handler_write_configuration_parameters                     (self):
        pass

    def _handler_read_configuration_parameters                      (self):
        pass

    def _handler_set_mode                                           (self):
        pass

    def _handler_write_lidar_extrinsic_parameters                   (self):
        pass

    def _handler_read_lidar_extrinsic_parameters                    (self):
        pass

    def _handler_turn_on_off_rain_fog_suppression                   (self):
        pass

    def _handler_set_turn_on_off_fan                                (self):
        pass

    def _handler_get_turn_on_off_fan_state                          (self):
        pass

    def _handler_set_lidar_return_mode                              (self):
        pass

    def _handler_get_lidar_return_mode                              (self):
        pass

    def _handler_set_imu_data_push_frequency                        (self):
        pass

    def _handler_get_imu_data_push_frequency                        (self):
        pass

    def _handler_update_utc_synchronization_time                    (self):
        pass

    def _handler_query_connected_lidar_device                       (self):
        pass

    def _handler_set_lidar_mode                                     (self):
        pass

    def _handler_turn_on_off_designated_slot_power                  (self):
        pass

    def _handler_write_lidar_extrinsic_parameters                   (self):
        pass

    def _handler_read_lidar_extrinsic_parameters                    (self):
        pass

    def _handler_query_lidar_device_status                          (self):
        pass

    def _handler_turn_on_off_hub_calculation_of_extrinsic_parameters(self):
        pass

    def _handler_turn_on_off_lidar_rain_fog_suppression             (self):
        pass

    def _handler_query_hub_slot_power_status                        (self):
        pass

    def _handler_set_lidar_turn_on_off_fan                          (self):
        pass

    def _handler_get_lidar_turn_on_off_fan_state                    (self):
        pass

    def _handler_set_lidar_return_mode                              (self):
        pass

    def _handler_get_lidar_return_mode                              (self):
        pass

    def _handler_set_lidar_imu_data_push_frequency                  (self):
        pass

    def _handler_get_lidar_imu_data_push_frequency                  (self):
        pass


    def command_handler(self, cmd:Command):
        pass

    