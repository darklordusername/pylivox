#std
from binascii import a2b_hex
import ipaddress
#libs
import pytest
#proj
from pylivox.control import general as g
from pylivox.control.frame import Frame
from pylivox.control.utils import FrameFrom

def cmd_payload(payload:str):
    return a2b_hex(payload.replace(' ', ''))

@pytest.mark.parametrize(
    'T                                     , kwargs                                                                                 , frame',
    [                 
    (g.BroadcastMsg                        , {'broadcast' : g.BroadcastMsg.Broadcast(serial=0x11223344, ip_range=0)                 ,
                                                'dev_type'  : g.DeviceType.Enum.Mid40}                                              , cmd_payload('aa 01 2200 02 0000 597e 00 00 34120000000000000000000000003300010000 b3784846')),
    (g.Handshake                           , {'ip'        : '192.168.1.1',                 
                                                'point_port': 0x1122,                 
                                                'cmd_port'  : 0x3344,                 
                                                'imu_port'  : 0x5566,}                                                              , a2b_hex('0011c0a80101221144336655')),
    (g.HandshakeResponse                   , {'result'    : False}                                                                  , b''),
    (g.QueryDeviceInformation              , {}                                                                                     , b''),
    (g.QueryDeviceInformationResponse      , {'result'    : True, 'firmware_version' : 12}                                          , b''),
    (g.Heartbeat                           , {}                                                                                     , b''),
    (g.HeartbeatResponse                   , {'rain_fog_suppression' : True, }                                                      , b''),
    ]
)
def test_command_to_frame_and_from_frame(T, kwargs, frame:bytes):
    t = T(**kwargs)
    assert t.frame == frame
    t2 = FrameFrom(frame)
    assert type(t2) is type(t)
