#std
from binascii import a2b_hex
import ipaddress
#libs
import pytest
#proj
from pylivox.control import general as g
from pylivox.control import lidar 
from pylivox.control.frame import Frame, DeviceType
from pylivox.control.utils import FrameFrom

def cmd_payload(payload:str):
    b = a2b_hex(payload.replace(' ', ''))
    array = bytearray(b)
    assert len(b) == int.from_bytes(b[2:4], 'little')
    crc_header = Frame.crc_header(b[:Frame.HEADER_LENGTH-2]).to_bytes(2, 'little')
    for i in range(len(crc_header)):
        array[i+Frame.HEADER_LENGTH-len(crc_header)] = crc_header[i]
    crc = Frame.crc(array[:-4]).to_bytes(4, 'little')
    for i in range(len(crc)):
        array[-4+i] = crc[i]
    return bytes(array)

@pytest.mark.parametrize(
    'T                                     , kwargs                                                                                 , frame',
    [                 
    (g.BroadcastMsg                        , {'broadcast' : g.Broadcast(serial='12345678901234', ip_range=0) ,                                                                                                                              #serial,                     ip_range, reserved, device_type, reserved           crc
                                            'device_type'  : DeviceType.MID_40}                                                     , cmd_payload('aa     01                2200    02        0000  0000      00       00                   3132333435363738393031323334 00        00        01           0000               00000000')),
    (g.Handshake                           , {'ip'        : '192.168.1.1',                 
                                            'point_port': 0x1122,                 
                                            'cmd_port'  : 0x3344,                                                                               #start, protocol version, length, cmd type, seq , head crc, cmd set, cmd id,    user ip, data port, cmd pro, imu port,             crc
                                            'imu_port'  : 0x5566, 
                                            'device_type' :DeviceType.TELE_15, 
                                            'device_version' :(8,8,8,8)}                                                          , cmd_payload('aa     01                1900    00        0000  0000      00       01         c0a80101 2211       4433     6655                  00000000')),
    (g.Handshake                           , {'ip'      : '192.168.1.1',                 
                                            'point_port': 0x1122,                 
                                            'cmd_port'  : 0x3344,                                                                                                                                                              #user ip, data port, cmd pro, imu port,             crc
                                            'imu_port'  : 0x5566,}                                                                , cmd_payload('aa     01                1700    00        0000  0000      00       01         c0a80101 2211       4433                           00000000')),
                                                                                                                                                                                                                                #is_error
    (g.HandshakeResponse                   , {'is_error': False}                                                                  , cmd_payload('aa     01                1000    01        0000  0000      00       01         00                                                 00000000')),
                                                                                                                                                                                                                                #
    (g.QueryDeviceInformation              , {}                                                                                   , cmd_payload('aa     01                0f00    00        0000  0000      00       02                                                            00000000')),
                                                                                                                                                                                                                                #is_error, firmware version
    (g.QueryDeviceInformationResponse      , {'is_error': False, 'device_version' : (0x11, 0x22, 0x33, 0x44)}                     , cmd_payload('aa     01                1400    01        0000  0000      00       02         00         11223344                                00000000')),
                                                                                                                                                                                                                                #
    (g.Heartbeat                           , {}                                                                                   , cmd_payload('aa     01                0f00    00        0000  0000      00       03                                                            00000000')),
                                                                                                                                                                                                                                #is_error, work_state, feature_msg, ack_msg
    (g.HeartbeatResponse                   , {'is_error': False, 'work_state':0, 'feature_msg': 0, 'ack_msg': 0x11223344 }        , cmd_payload('aa     01                1600    01        0000  0000      00       03         00         00          00           44332211       00000000')),
                                                                                                                                                                                                                                #sample_ctrl
    (g.StartStopSampling                   , {'is_start': True}                                                                   , cmd_payload('aa     01                1000    00        0000  0000      00       04         01                                                 00000000')),
                                                                                                                                                                                                                                #is_error
    (g.StartStopSamplingResponse           , {'is_error': False}                                                                  , cmd_payload('aa     01                1000    01        0000  0000      00       04         00                                                 00000000')),
                                                                                                                                                                                                                                #coordinate_type
    (g.ChangeCoordinateSystem              , {'is_spherical': True}                                                               , cmd_payload('aa     01                1000    00        0000  0000      00       05         01                                                 00000000')),
                                                                                                                                                                                                                                #is_error
    (g.ChangeCoordinateSystemResponse      , {'is_error': False}                                                                  , cmd_payload('aa     01                1000    01        0000  0000      00       05         00                                                 00000000')),
                                                                                                                                                                                                                                #
    (g.Disconnect                          , {}                                                                                   , cmd_payload('aa     01                0f00    00        0000  0000      00       06                                                            00000000')),
                                                                                                                                                                                                                                #is_error
    (g.DisconnectResponse                  , {'is_error': False}                                                                  , cmd_payload('aa     01                1000    01        0000  0000      00       06         00                                                 00000000')),
                                                                                                                                                                                                                                #status code
    (g.PushAbnormalStatusInformation       , {'status_code': 0x11223344}                                                          , cmd_payload('aa     01                1300    02        0000  0000      00       07         44332211                                           00000000')),
                                                                                                                                                                                                                                #is_static, ip,      net_mask, gw
    (g.ConfigureStaticDynamicIp            , {'is_static': True,  'ip':'192.168.1.1',
                                             'mask': '255.255.255.0', 'gw':'192.168.1.1',
                                             'device_type': DeviceType.AVIA,
                                             'device_version': (11,11,11,11)}                                                     , cmd_payload('aa     01                1c00    00        0000  0000      00       08         01          c0a80101 ffffff00  c0a80101            00000000')),
                                                                                                                                                                                                                                #is_static  ip                                        
    (g.ConfigureStaticDynamicIp            , {'is_static': True,  'ip':'192.168.1.1', }                                           , cmd_payload('aa     01                1400    00        0000  0000      00       08         01          c0a80101                               00000000')),
                                                                                                                                                                                                                                #is_error
    (g.ConfigureStaticDynamicIpResponse    , {'is_error': False}                                                                  , cmd_payload('aa     01                1000    01        0000  0000      00       08         00                                                 00000000')),
                                                                                                                                                                                                                                #
    (g.GetDeviceIpInformation              , {}                                                                                   , cmd_payload('aa     01                0f00    00        0000  0000      00       09                                                            00000000')),
                                                                                                                                                                                                                                #is_error, ip_mode, ip_addr, net_mask, gw
    (g.GetDeviceIpInformationResponse      ,{'is_error': False, 'is_static': True, 'ip':'192.168.1.1',
                                            'mask':'255.255.255.0', 'gw':'192.168.1.1',
                                            'device_type': DeviceType.AVIA,
                                             'device_version': (11,11,11,11)}
                                                                                                                                  , cmd_payload('aa     01                1d00    01        0000  0000      00       09         00         01       c0a80101 ffffff00  c0a80101    00000000')),
                                                                                                                                                                                                                                #is_error, ip_mode, ip_addr, 
    (g.GetDeviceIpInformationResponse      ,{'is_error': False, 'is_static': True, 'ip':'192.168.1.1',
                                            'mask':'255.255.255.0', 'gw':'192.168.1.1',}
                                                                                                                                  , cmd_payload('aa     01                1500    01        0000  0000      00       09         00         01       c0a80101                       00000000')),
                                                                                                                                                                                                                                #timeout
    (g.RebootDevice                        , {'timeout': 0x1122, 'device_type': DeviceType.AVIA, 'device_version': (11,11,11,11)} , cmd_payload('aa     01                1100    00        0000  0000      00       0a         2211                                               00000000')),
                                                                                                                                                                                                                                #is_error
    (g.RebootDeviceResponse                , {'is_error': False, 'device_type': DeviceType.AVIA, 'device_version': (11,11,11,11)} , cmd_payload('aa     01                1000    01        0000  0000      00       0a         00                                                 00000000')),
                                                                                                                                                                                                                                #param_list
    (g.WriteConfigurationParameters        , {'param_list': [g.ConfigurationParameter(g.ConfigurationParameter.Key.HIGH_SENSITIVITY_FUNCTION, True),
                                                             g.ConfigurationParameter(g.ConfigurationParameter.Key.SWITCH_REPETITIVE_NON_REPETITIVE_SCANNING_PATTERN, True),
                                                             g.ConfigurationParameter(g.ConfigurationParameter.Key.SLOT_ID_CONFIGURATION, 1)]}
                                                                                                                                  , cmd_payload('aa     01                1e00    00        0000  0000      00       0b         0100050001 0200050001 0300050001                   00000000')),
                                                                                                                                                                                                                                #is_error, error_key, error_code
    (g.WriteConfigurationParametersResponse, {'is_error': False, 
                                                'error_key' :g.ConfigurationParameter.Key.HIGH_SENSITIVITY_FUNCTION, 
                                                'error_code': g.ConfigurationParameter.ErrorCode.NO_ERROR}                        , cmd_payload('aa     01                1300    01        0000  0000      00       0b         00         0100       00                           00000000')),
                                                                                                                                                                                                                                #keys_quantity, keys                                                
    (g.ReadConfigurationParameters         , {'keys_quantity': 2, 
                                              'keys': [g.ConfigurationParameter.Key.HIGH_SENSITIVITY_FUNCTION,
                                                       g.ConfigurationParameter.Key.SWITCH_REPETITIVE_NON_REPETITIVE_SCANNING_PATTERN]} 
                                                                                                                                  , cmd_payload('aa     01                1400    00        0000  0000      00       0c         02              0100 0200                          00000000')),
                                                                                                                                                                                                                                #is_error, error_key, error_code, param_list                                                                                     
    (g.ReadConfigurationParametersResponse, {'is_error': False, 
                                            'error_key' :g.ConfigurationParameter.Key.HIGH_SENSITIVITY_FUNCTION, 
                                            'error_code': g.ConfigurationParameter.ErrorCode.NO_ERROR,
                                            'param_list': [g.ConfigurationParameter( g.ConfigurationParameter.Key.HIGH_SENSITIVITY_FUNCTION, True )]
                                            }                                                                                     , cmd_payload('aa     01                1800    01        0000  0000      00       0c          00         0100       00          0100050001       00000000')),

                                                                                                                                                                                                                                #power_mode
    (lidar.SetMode                        , {'power_mode': lidar.PowerMode.normal}                                                , cmd_payload('aa     01                1000    00        0000  0000      01       00         01                                                  00000000')),
                                                                                                                                                                                                                                #result
    (lidar.SetModeResponse                , {'result': lidar.SetModeResponse.Result.Switching}                                    , cmd_payload('aa     01                1000    01        0000  0000      01       00         02                                                  00000000')),
                                                                                                                                                                                                                                #roll,   pitch,   yaw,     x        y        z
    (lidar.WriteLidarExtrinsicParameters  , {'roll':1.0, 'pitch':1.0, 'yaw':1.0, 'x':0x11, 'y':0x22, 'z':0x33}                    , cmd_payload('aa     01                2700    00        0000  0000      01       01         0000803f 0000803f 0000803f 11000000 22000000 33000000 00000000')),
                                                                                                                                                                                                                                #is_error
    (lidar.WriteLidarExtrinsicParametersResponse, {'is_error': False}                                                             , cmd_payload('aa     01                1000    01        0000  0000      01       01         00                                                    00000000')),
                                                                                                                                                                                                                                #
    (lidar.ReadLidarExtrinsicParameters   , {}                                                                                    , cmd_payload('aa     01                0f00    00        0000  0000      01       02                                                               00000000')),
                                                                                                                                                                                                                                #is_error #roll,   pitch,   yaw,     x        y        z
    (lidar.ReadLidarExtrinsicParametersResponse , {'is_error': False, 'roll':1.0, 'pitch':1.0, 'yaw':1.0, 'x':0x11, 'y':0x22, 'z':0x33} 
                                                                                                                                  , cmd_payload('aa     01                2800    01        0000  0000      01       02         00        0000803f 0000803f 0000803f 11000000 22000000 33000000 00000000')),
                                                                                                                                                                                                                                #is_enable
    (lidar.TurnOnOffRainFogSuppression     , {'is_enable': True}                                                                  , cmd_payload('aa     01                1000    00        0000  0000      01       03         01                                                    00000000')),
                                                                                                                                                                                                                                #is_error
    (lidar.TurnOnOffRainFogSuppressionResponse  , {'is_error': False}                                                             , cmd_payload('aa     01                1000    01        0000  0000      01       03         00                                                    00000000')),
                                                                                                                                                                                                                                #is_enable
    (lidar.SetTurnOnOffFan                  , {'is_enable': True}                                                                 , cmd_payload('aa     01                1000    00        0000  0000      01       04         01                                                    00000000')),
                                                                                                                                                                                                                                #is_error
    (lidar.SetTurnOnOffFanResponse          , {'is_error': False}                                                                 , cmd_payload('aa     01                1000    01        0000  0000      01       04         00                                                     00000000')),
                                                                                                                                                                                                                                #
    (lidar.GetTurnOnOffFanState             , {}                                                                                  , cmd_payload('aa     01                0f00    00        0000  0000      01       05                                                                00000000')),
                                                                                                                                                                                                                                #is_error, state
    (lidar.GetTurnOnOffFanStateResponse     , {'is_error': False, 'state': True}                                                  , cmd_payload('aa     01                1100    01        0000  0000      01       05         00         01                                          00000000')),
                                                                                                                                                                                                                                #return_mode
    (lidar.SetLidarReturnMode               , {'return_mode': lidar.ReturnMode.DUAL_RETURN}                                       , cmd_payload('aa     01                1000    00        0000  0000      01       06         02                                                     00000000')),
                                                                                                                                                                                                                                #is_error
    (lidar.SetLidarReturnModeResponse       , {'is_error': False}                                                                 , cmd_payload('aa     01                1000    01        0000  0000      01       06         00                                                     00000000')),
                                                                                                                                                                                                                                #
    (lidar.GetLidarReturnMode               , {}                                                                                  , cmd_payload('aa     01                0f00    00        0000  0000      01       07                                                                00000000')),
                                                                                                                                                                                                                                #is_error, return_mode            
    (lidar.GetLidarReturnModeResponse       , {'is_error': False, 'return_mode': lidar.ReturnMode.DUAL_RETURN}                    , cmd_payload('aa     01                1100    01        0000  0000      01       07         00         02                                          00000000')),
                                                                                                                                                                                                                                #frequency_code        
    (lidar.SetImuDataPushFrequency          , {'frequency': lidar.PushFrequency.FREQ_0HZ}                                         , cmd_payload('aa     01                1000    00        0000  0000      01       08         00                                                     00000000')),
                                                                                                                                                                                                                                #is_error
    (lidar.SetImuDataPushFrequencyResponse  , {'is_error': False}                                                                 , cmd_payload('aa     01                1000    01        0000  0000      01       08         00                                                     00000000')),
                                            
    (lidar.GetImuDataPushFrequency          , {}                                                                                  , cmd_payload('aa     01                0f00    00        0000  0000      01       09                                                                00000000')),
                                                                                                                                                                                                                                #is_error, frequency_code
    (lidar.GetImuDataPushFrequencyResponse  , {'is_error': False, 'frequency': lidar.PushFrequency.FREQ_0HZ}                      , cmd_payload('aa     01                1100    01        0000  0000      01       09         00         00                                          00000000')),
                                                                                                                                                                                                                                #year, month, day, hour, microseconds
    (lidar.UpdateUtcSynchronizationTime     , {'year': 0x22, 'month': 2, 'day': 1, 'hour': 1, 'microseconds': 0x11223344}         , cmd_payload('aa     01                1700    00        0000  0000      01       0a         22     02     01   01    44332211                      00000000')),
                                                                                                                                                                                                                                #is_error
    (lidar.UpdateUtcSynchronizationTimeResponse , {'is_error': False}                                                             , cmd_payload('aa     01                1000    01        0000  0000      01       0a         00                                                     00000000')),
                                                                                                                                                                                                                                #
    #                                                                                                                                                                                                                             #
    # (lidar.                   , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),

    ]
)
def test_command_to_frame_and_from_frame(T, kwargs, frame:bytes):
    t = T(**kwargs)
    assert t.frame == frame
    frame_kwargs = {}
    if 'device_type' in kwargs:
        frame_kwargs['device_type'] = kwargs['device_type']
    if 'device_version' in kwargs:
        frame_kwargs['device_version'] = kwargs['device_version']
    t2 = FrameFrom(frame, **frame_kwargs)
    assert type(t2) is type(t)
