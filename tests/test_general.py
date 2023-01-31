#std
from binascii import a2b_hex
import ipaddress
#libs
import pytest
#proj
from pylivox.control import general as g
from pylivox.control import lidar 
from pylivox.control.frame import Frame
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
    # (g.BroadcastMsg                        , {'broadcast' : g.BroadcastMsg.Broadcast(serial=0x11223344, ip_range=0)                 ,
    #                                             'dev_type'  : g.DeviceType.Enum.Mid40}                                              , cmd_payload('aa 01 2200 02 0000 597e 00 00 34120000000000000000000000003300010000 b3784846')),
    (g.Handshake                           , {'ip'        : '192.168.1.1',                 
                                            'point_port': 0x1122,                 
                                            'cmd_port'  : 0x3344,                                                                               #start, protocol version, length, cmd type, seq , head crc, cmd set, cmd id,    user ip, data port, cmd pro, imu port,             crc
                                            'imu_port'  : 0x5566,}                                                                , cmd_payload('aa     01                1900    00        0000  0000      00       01         0101a8c0 2211       4433     6655                  00000000')),
                                                                                                                                                                                                                                #return code
    (g.HandshakeResponse                   , {'result'    : False}                                                                , cmd_payload('aa     01                1000    01        0000  0000      00       01         00                                                 00000000')),
                                                                                                                                                                                                                                #
    (g.QueryDeviceInformation              , {}                                                                                   , cmd_payload('aa     01                0f00    00        0000  0000      00       02                                                            00000000')),
                                                                                                                                                                                                                                #ret_code, firmware version
    (g.QueryDeviceInformationResponse      , {'result'    : True, 'firmware_version' : 0x11223344}                                , cmd_payload('aa     01                1400    01        0000  0000      00       02         01         44332211                                00000000')),
                                                                                                                                                                                                                                #
    (g.Heartbeat                           , {}                                                                                   , cmd_payload('aa     01                0f00    00        0000  0000      00       03                                                            00000000')),
                                                                                                                                                                                                                                #return code, work_state, feature_msg, ack_msg
    (g.HeartbeatResponse                   , {'result': True, 'work_state':0, 'feature_msg': 0, 'ack_msg': 0x11223344 }           , cmd_payload('aa     01                1600    01        0000  0000      00       03         01            00          00           44332211    00000000')),
                                                                                                                                                                                                                                #sample_ctrl
    (g.StartStopSampling                   , {'is_start': True}                                                                   , cmd_payload('aa     01                1000    00        0000  0000      00       04         01                                                 00000000')),
                                                                                                                                                                                                                                #ret_code
    (g.StartStopSamplingResponse           , {'result': True}                                                                     , cmd_payload('aa     01                1000    01        0000  0000      00       04         01                                                 00000000')),
                                                                                                                                                                                                                                #coordinate_type
    (g.ChangeCoordinateSystem              , {'is_spherical': True}                                                               , cmd_payload('aa     01                1000    00        0000  0000      00       05         01                                                 00000000')),
                                                                                                                                                                                                                                #ret_code
    (g.ChangeCoordinateSystemResponse      , {'result': True}                                                                     , cmd_payload('aa     01                1000    01        0000  0000      00       05         01                                                 00000000')),
                                                                                                                                                                                                                                #
    (g.Disconnect                          , {}                                                                                   , cmd_payload('aa     01                0f00    00        0000  0000      00       06                                                            00000000')),
                                                                                                                                                                                                                                #ret_code
    (g.DisconnectResponse                  , {'result': True}                                                                     , cmd_payload('aa     01                1000    01        0000  0000      00       06         01                                                 00000000')),
    #                                                                                                                                                                                                                             #status code
    # (g.PushAbnormalStatusInformation       , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01             00000000                                           00000000')),
                                                                                                                                                                                                                                #is_static, ip,      net_mask, gw
    (g.ConfigureStaticDynamicIp            , {'is_static': True,  'ip':'192.168.1.1', 'mask': '255.255.255.0', 'gw':'192.168.1.1'}, cmd_payload('aa     01                1c00    00        0000  0000      00       08         01          0101a8c0 00ffffff  0101a8c0            00000000')),
                                                                                                                                                                                                                                #ret_code
    (g.ConfigureStaticDynamicIpResponse    , {'result': True}                                                                     , cmd_payload('aa     01                1000    01        0000  0000      00       08         01                                                 00000000')),
                                                                                                                                                                                                                                #
    (g.GetDeviceIpInformation              , {}                                                                                   , cmd_payload('aa     01                0f00    00        0000  0000      00       09                                                            00000000')),
                                                                                                                                                                                                                                #ret_code, ip_mode, ip_addr, net_mask, gw
    (g.GetDeviceIpInformationResponse      ,{'result': True, 'is_static': True, 'ip':'192.168.1.1', 'mask':'255.255.255.0', 'gw':'192.168.1.1'}
                                                                                                                                  , cmd_payload('aa     01                1d00    01        0000  0000      00       09         01         01       0101a8c0 00ffffff  0101a8c0    00000000')),
                                                                                                                                                                                                                                #timeout
    (g.RebootDevice                        , {'timeout': 0x1122}                                                                  , cmd_payload('aa     01                1100    00        0000  0000      00       0a         2211                                               00000000')),
                                                                                                                                                                                                                                #ret_code
    (g.RebootDeviceResponse                , {'result': True}                                                                     , cmd_payload('aa     01                1000    01        0000  0000      00       0a         01                                                 00000000')),
                                                                                                                                                                                                                                #param_list
    (g.WriteConfigurationParameters        , {'param_list': [g.ConfigurationParameter(g.ConfigurationParameter.Key.HIGH_SENSITIVITY_FUNCTION, True),
                                                             g.ConfigurationParameter(g.ConfigurationParameter.Key.SWITCH_REPETITIVE_NON_REPETITIVE_SCANNING_PATTERN, True),
                                                             g.ConfigurationParameter(g.ConfigurationParameter.Key.SLOT_ID_CONFIGURATION, 1)]}
                                                                                                                                  , cmd_payload('aa     01                1e00    00        0000  0000      00       0b         0100050001 0200050001 0300050001                         00000000')),
                                                                                                                                                                                                                                
    # (g.ReadConfigurationParameters         , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
                                                                                                                                                                                                                                #
    # (lidar.SetMode                         , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.SetModeResponse                 , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.WriteLiDarExtrinsicParameters   , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.WriteLiDarExtrinsicParametersResponse, {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.ReadLidarExtrinsicParameters    , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.ReadLidarExtrinsicParametersResponse , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.TurnOnOffRainFogSuppression     , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.TurnOnOffRainFogSuppressionResponse  , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.SetTurnOnOffFan                  , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.SetTurnOnOffFanResponse          , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.GetTurnOnOffFanState             , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.GetTurnOnOffFanStateResponse     , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.SetLiDarReturnMode               , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # # (lidar.SetLiDarReturnModeResponse       , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.GetLiDarReturnMode               , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                         #
    # (lidar.GetLiDarReturnModeResponse       , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                         #
    # (lidar.SetImuDataPushFrequency          , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                         #
    # (lidar.SetImuDataPushFrequencyResponse  , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                         #
    # (lidar.GetImuDataPushFrequency          , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                         #
    # (lidar.GetImuDataPushFrequencyResponse  , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                         #
    # (lidar.UpdateUtcSynchronizationTime     , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
    #                                                                                                                                                                                                                             #
    # (lidar.UpdateUtcSynchronizationTimeResponse , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),
                                                                                                                                                                                                                                #
    #                                                                                                                                                                                                                             #
    # (lidar.                   , {}                                                                                   , cmd_payload('aa     01                0000    01        0000  0000      00       01                                                                00000000')),

    ]
)
def test_command_to_frame_and_from_frame(T, kwargs, frame:bytes):
    t = T(**kwargs)
    assert t.frame == frame
    t2 = FrameFrom(frame)
    assert type(t2) is type(t)
