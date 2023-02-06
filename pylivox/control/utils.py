
#std
import struct
#proj
from pylivox.control.frame import Frame, DeviceType
import pylivox.control.general as general
import pylivox.control.hub as hub
import pylivox.control.lidar as lidar

TypeDict = {
    #GENERAL CMD
    (Frame.Set.GENERAL.value, Frame.Type.MSG.value, Frame.SetGeneral.BROADCAST_MESSAGE                .value) : general.BroadcastMsg,
    (Frame.Set.GENERAL.value, Frame.Type.CMD.value, Frame.SetGeneral.HANDSHAKE                        .value) : general.Handshake,
    (Frame.Set.GENERAL.value, Frame.Type.CMD.value, Frame.SetGeneral.QUERY_DEVICE_INFORMATION         .value) : general.QueryDeviceInformation,
    (Frame.Set.GENERAL.value, Frame.Type.CMD.value, Frame.SetGeneral.HEARTBEAT                        .value) : general.Heartbeat,
    (Frame.Set.GENERAL.value, Frame.Type.CMD.value, Frame.SetGeneral.START_STOP_SAMPLING              .value) : general.StartStopSampling,
    (Frame.Set.GENERAL.value, Frame.Type.CMD.value, Frame.SetGeneral.CHANGE_COORDINATE_SYSTEM         .value) : general.ChangeCoordinateSystem,
    (Frame.Set.GENERAL.value, Frame.Type.CMD.value, Frame.SetGeneral.DISCONNECT                       .value) : general.Disconnect,
    (Frame.Set.GENERAL.value, Frame.Type.MSG.value, Frame.SetGeneral.PUSH_ABNORMAL_STATUS_INFORMATION .value) : general.PushAbnormalStatusInformation,
    (Frame.Set.GENERAL.value, Frame.Type.CMD.value, Frame.SetGeneral.CONFIGURE_STATIC_DYNAMIC_IP      .value) : general.ConfigureStaticDynamicIp,
    (Frame.Set.GENERAL.value, Frame.Type.CMD.value, Frame.SetGeneral.GET_DEVICE_IP_INFORMATION        .value) : general.GetDeviceIpInformation,
    (Frame.Set.GENERAL.value, Frame.Type.CMD.value, Frame.SetGeneral.REBOOT_DEVICE                    .value) : general.RebootDevice,
    (Frame.Set.GENERAL.value, Frame.Type.CMD.value, Frame.SetGeneral.WRITE_CONFIGURATION_PARAMETERS   .value) : general.WriteConfigurationParameters,
    (Frame.Set.GENERAL.value, Frame.Type.CMD.value, Frame.SetGeneral.READ_CONFIGURATION_PARAMETERS    .value) : general.ReadConfigurationParameters,
    #GENERAL AKN
    (Frame.Set.GENERAL.value, Frame.Type.AKN.value, Frame.SetGeneral.HANDSHAKE                        .value) : general.HandshakeResponse,
    (Frame.Set.GENERAL.value, Frame.Type.AKN.value, Frame.SetGeneral.QUERY_DEVICE_INFORMATION         .value) : general.QueryDeviceInformationResponse,
    (Frame.Set.GENERAL.value, Frame.Type.AKN.value, Frame.SetGeneral.HEARTBEAT                        .value) : general.HeartbeatResponse,
    (Frame.Set.GENERAL.value, Frame.Type.AKN.value, Frame.SetGeneral.START_STOP_SAMPLING              .value) : general.StartStopSamplingResponse,
    (Frame.Set.GENERAL.value, Frame.Type.AKN.value, Frame.SetGeneral.CHANGE_COORDINATE_SYSTEM         .value) : general.ChangeCoordinateSystemResponse,
    (Frame.Set.GENERAL.value, Frame.Type.AKN.value, Frame.SetGeneral.DISCONNECT                       .value) : general.DisconnectResponse,
    (Frame.Set.GENERAL.value, Frame.Type.AKN.value, Frame.SetGeneral.CONFIGURE_STATIC_DYNAMIC_IP      .value) : general.ConfigureStaticDynamicIpResponse,
    (Frame.Set.GENERAL.value, Frame.Type.AKN.value, Frame.SetGeneral.GET_DEVICE_IP_INFORMATION        .value) : general.GetDeviceIpInformationResponse,
    (Frame.Set.GENERAL.value, Frame.Type.AKN.value, Frame.SetGeneral.REBOOT_DEVICE                    .value) : general.RebootDeviceResponse,
    (Frame.Set.GENERAL.value, Frame.Type.AKN.value, Frame.SetGeneral.WRITE_CONFIGURATION_PARAMETERS   .value) : general.WriteConfigurationParametersResponse,
    (Frame.Set.GENERAL.value, Frame.Type.AKN.value, Frame.SetGeneral.READ_CONFIGURATION_PARAMETERS    .value) : general.ReadConfigurationParametersResponse,
    #LIDAR CMD
    (Frame.Set.LIDAR.value, Frame.Type.CMD.value, Frame.SetLidar.SET_MODE                             .value) : lidar.SetMode,
    (Frame.Set.LIDAR.value, Frame.Type.CMD.value, Frame.SetLidar.WRITE_LIDAR_EXTRINSIC_PARAMETERS    .value) : lidar.WriteLidarExtrinsicParameters,
    (Frame.Set.LIDAR.value, Frame.Type.CMD.value, Frame.SetLidar.READ_LIDAR_EXTRINSIC_PARAMETERS     .value) : lidar.ReadLidarExtrinsicParameters,
    (Frame.Set.LIDAR.value, Frame.Type.CMD.value, Frame.SetLidar.TURN_ON_OFF_RAIN_FOG_SUPPRESSION     .value) : lidar.TurnOnOffRainFogSuppression,
    (Frame.Set.LIDAR.value, Frame.Type.CMD.value, Frame.SetLidar.SET_TURN_ON_OFF_FAN                  .value) : lidar.SetTurnOnOffFan,
    (Frame.Set.LIDAR.value, Frame.Type.CMD.value, Frame.SetLidar.GET_TURN_ON_OFF_FAN_STATE            .value) : lidar.GetTurnOnOffFanState,
    (Frame.Set.LIDAR.value, Frame.Type.CMD.value, Frame.SetLidar.SET_LIDAR_RETURN_MODE                .value) : lidar.SetLidarReturnMode,
    (Frame.Set.LIDAR.value, Frame.Type.CMD.value, Frame.SetLidar.GET_LIDAR_RETURN_MODE                .value) : lidar.GetLidarReturnMode,
    (Frame.Set.LIDAR.value, Frame.Type.CMD.value, Frame.SetLidar.SET_IMU_DATA_PUSH_FREQUENCY          .value) : lidar.SetImuDataPushFrequency,
    (Frame.Set.LIDAR.value, Frame.Type.CMD.value, Frame.SetLidar.GET_IMU_DATA_PUSH_FREQUENCY          .value) : lidar.GetImuDataPushFrequency,
    (Frame.Set.LIDAR.value, Frame.Type.CMD.value, Frame.SetLidar.UPDATE_UTC_SYNCHRONIZATION_TIME      .value) : lidar.UpdateUtcSynchronizationTime,
    #LIDAR AKN
    (Frame.Set.LIDAR.value, Frame.Type.AKN.value, Frame.SetLidar.SET_MODE                             .value) : lidar.SetModeResponse,
    (Frame.Set.LIDAR.value, Frame.Type.AKN.value, Frame.SetLidar.WRITE_LIDAR_EXTRINSIC_PARAMETERS    .value) : lidar.WriteLidarExtrinsicParametersResponse,
    (Frame.Set.LIDAR.value, Frame.Type.AKN.value, Frame.SetLidar.READ_LIDAR_EXTRINSIC_PARAMETERS     .value) : lidar.ReadLidarExtrinsicParametersResponse,
    (Frame.Set.LIDAR.value, Frame.Type.AKN.value, Frame.SetLidar.TURN_ON_OFF_RAIN_FOG_SUPPRESSION     .value) : lidar.TurnOnOffRainFogSuppressionResponse,
    (Frame.Set.LIDAR.value, Frame.Type.AKN.value, Frame.SetLidar.SET_TURN_ON_OFF_FAN                  .value) : lidar.SetTurnOnOffFanResponse,
    (Frame.Set.LIDAR.value, Frame.Type.AKN.value, Frame.SetLidar.GET_TURN_ON_OFF_FAN_STATE            .value) : lidar.GetTurnOnOffFanStateResponse,
    (Frame.Set.LIDAR.value, Frame.Type.AKN.value, Frame.SetLidar.SET_LIDAR_RETURN_MODE                .value) : lidar.SetLidarReturnModeResponse,
    (Frame.Set.LIDAR.value, Frame.Type.AKN.value, Frame.SetLidar.GET_LIDAR_RETURN_MODE                .value) : lidar.GetLidarReturnModeResponse,
    (Frame.Set.LIDAR.value, Frame.Type.AKN.value, Frame.SetLidar.SET_IMU_DATA_PUSH_FREQUENCY          .value) : lidar.SetImuDataPushFrequencyResponse,
    (Frame.Set.LIDAR.value, Frame.Type.AKN.value, Frame.SetLidar.GET_IMU_DATA_PUSH_FREQUENCY          .value) : lidar.GetImuDataPushFrequencyResponse,
    (Frame.Set.LIDAR.value, Frame.Type.AKN.value, Frame.SetLidar.UPDATE_UTC_SYNCHRONIZATION_TIME      .value) : lidar.UpdateUtcSynchronizationTimeResponse,
}    

def FrameFrom(frame:bytes, 
                device_type:DeviceType=None, 
                device_version:'tuple(int,int,int,int)'=None
            )->Frame:
    #TODO description
    #parse frame parts
    header = frame[0:7]
    header_crc = int.from_bytes(frame[7:9], 'little')
    cmd_set = frame[9]
    cmd_id = frame[10]
    frame_payload = frame[11:-4]
    frame_crc = int.from_bytes(frame[-4:], 'little')
    assert len(frame) < Frame.FRAME_MAX_LENGTH
    assert frame_crc == Frame.crc(frame[:-4])
    assert header_crc == Frame.crc_header(header)
    #Check header 
    start, version, length, cmd_type, seq = struct.unpack('<BBHBH', header)
    assert start == Frame.START
    assert version == Frame.VERSION
    T = TypeDict[cmd_set, cmd_type, cmd_id]
    return T.from_payload(frame_payload, seq, device_type, device_version)