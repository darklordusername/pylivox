"""
Protocol livox
"""
import threading
import time
import enum
import socket
import struct
import sys 
import crcmod
import abc
import ipaddress
import inspect 
import log
logger = log.getLogger(__name__)

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
        self._listen_t = threading.Thread(target=self.listen, daemon=True, name="listen")
        self._listen_t.start()
        lidars.append(self)

    def listen(self):
        while True:
            udp_frame,(addr, port) = self._socket.recvfrom(2048)
            addr = ipaddress.IPv4Address(addr)
            cmd = Command.from_frame(udp_frame)
            pass
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

_classes = [getattr(sys.modules[__name__], member) for member 
            in dir() if inspect.isclass(getattr(sys.modules[__name__], member)) ]

COMMAND_BY_CMD_ID = { cls.CMD_ID : cls for cls in _classes if issubclass(cls, Command) and hasattr(cls, 'CMD_ID') }
