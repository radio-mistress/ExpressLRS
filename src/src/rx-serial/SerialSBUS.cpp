#include "SerialSBUS.h"
#include "CRSF.h"
#include "device.h"
#include "config.h"

#if defined(TARGET_RX)

#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

const auto UNCONNECTED_CALLBACK_INTERVAL_MS = 10;
const auto SBUS_CALLBACK_INTERVAL_MS = 9;

uint32_t SerialSBUS::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    static auto sendPackets = false;
    bool effectivelyFailsafed = failsafe || (!connectionHasModelMatch) || (!teamraceHasModelMatch);
    if ((effectivelyFailsafed && config.GetFailsafeMode() == FAILSAFE_NO_PULSES) || (!sendPackets && connectionState != connected))
    {
        return UNCONNECTED_CALLBACK_INTERVAL_MS;
    }
    sendPackets = true;

    if ((!frameAvailable && !frameMissed && !effectivelyFailsafed) || _outputPort->availableForWrite() < 25)
    {
        return DURATION_IMMEDIATELY;
    }

    // TODO: if failsafeMode == FAILSAFE_SET_POSITION then we use the set positions rather than the last values
    crsf_channels_s PackedRCdataOut;

#if defined(PLATFORM_ESP32)
    extern Stream* serial_protocol_tx;
    extern Stream* serial1_protocol_tx;

    if (((config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO) && streamOut == serial_protocol_tx)||
        ((config.GetSerial1Protocol() == PROTOCOL_SERIAL1_DJI_RS_PRO) && streamOut == serial1_protocol_tx))
#else
    if (config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO)
#endif
{
    // Channel Mapping for DJI RS Protocol
    PackedRCdataOut.ch0 = fmap(channelData[0], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Ail (Pan)
    PackedRCdataOut.ch1 = fmap(channelData[1], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Ele (Tilt)
    PackedRCdataOut.ch2 = fmap(channelData[2], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // THR (Focus Motor)
    PackedRCdataOut.ch3 = fmap(channelData[3], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Rud (roll)
    PackedRCdataOut.ch4 = fmap(channelData[5], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Record start/stop and photo
    PackedRCdataOut.ch5 = fmap(channelData[6], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Gimbal Mode Change
    PackedRCdataOut.ch6 = fmap(channelData[7], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 176,  848); // Recenter and Selfie
    PackedRCdataOut.ch7 = fmap(channelData[8], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Gimbal Speed Adjustment
    PackedRCdataOut.ch8 = fmap(channelData[9], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Zoom
    PackedRCdataOut.ch9 = (channelData[11] > CRSF_CHANNEL_VALUE_MID) ? 1696 : 352; // Locking Mode (Binary: 352 = Unlocked, 1696 = Locked)
    PackedRCdataOut.ch10 = fmap(channelData[11], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Unassigned (Reserved)
    PackedRCdataOut.ch11 = fmap(channelData[12], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Unassigned (Reserved)
    PackedRCdataOut.ch12 = fmap(channelData[13], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Unassigned (Reserved)
    PackedRCdataOut.ch13 = fmap(channelData[14], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Unassigned (Reserved)
    PackedRCdataOut.ch14 = fmap(channelData[15], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Unassigned (Reserved)
    PackedRCdataOut.ch15 = channelData[4] < CRSF_CHANNEL_VALUE_MID ? 352 : 1696; // On (Reserved)
    // Focus Motor Control (ch2)
    uint16_t focusMotorValue = PackedRCdataOut.ch2; // Use ch2 for focus motor control
    uint8_t focusMotorCommand[5] = {0}; // Focus motor command packet
    focusMotorCommand[0] = 0xAA; // Header
    focusMotorCommand[1] = 0x12; // Focus Motor Command ID
    focusMotorCommand[2] = (focusMotorValue >> 8) & 0xFF; // High byte of focus motor value
    focusMotorCommand[3] = focusMotorValue & 0xFF; // Low byte of focus motor value
    focusMotorCommand[4] = 0x00; // Footer
    // Zoom Control (ch8)
    uint16_t zoomValue = PackedRCdataOut.ch8; // Use ch8 for zoom control
    uint8_t zoomCommand[5] = {0}; // Zoom command packet
    zoomCommand[0] = 0xAA; // Header
    zoomCommand[1] = 0x13; // Zoom Command ID
    zoomCommand[2] = (zoomValue >> 8) & 0xFF; // High byte of zoom value
    zoomCommand[3] = zoomValue & 0xFF; // Low byte of zoom value
    zoomCommand[4] = 0x00; // Footer
    // Gimbal Speed Adjustment (ch7)
    uint16_t gimbalSpeedValue = PackedRCdataOut.ch7; // Use ch7 for gimbal speed adjustment
    uint8_t gimbalSpeedCommand[5] = {0}; // Gimbal speed command packet
    gimbalSpeedCommand[0] = 0xAA; // Header
    gimbalSpeedCommand[1] = 0x14; // Gimbal Speed Command ID
    gimbalSpeedCommand[2] = (gimbalSpeedValue >> 8) & 0xFF; // High byte of gimbal speed value
    gimbalSpeedCommand[3] = gimbalSpeedValue & 0xFF; // Low byte of gimbal speed value
    gimbalSpeedCommand[4] = 0x00; // Footer
    // Locking Mode (ch10)
    uint16_t lockingModeValue = PackedRCdataOut.ch10; // Use ch10 for locking mode
    uint8_t lockingModeCommand[5] = {0}; // Locking mode command packet
    lockingModeCommand[0] = 0xAA; // Header
    lockingModeCommand[1] = 0x0E; // Command Set (Gimbal Control)
    lockingModeCommand[2] = 0x0D; // Command ID (Operating Mode Settings)
    lockingModeCommand[3] = (lockingModeValue == 1696) ? 0x00 : 0x01; // Locking Mode (0x00 = Lock, 0x01 = Unlock)
    lockingModeCommand[4] = 0x00; // Footer
    
    // Send Custom DJI Packets
    _outputPort->write(focusMotorCommand, sizeof(focusMotorCommand)); // Focus Motor
    _outputPort->write(zoomCommand, sizeof(zoomCommand)); // Zoom Control
    _outputPort->write(gimbalSpeedCommand, sizeof(gimbalSpeedCommand)); // Gimbal Speed Adjustment
    _outputPort->write(lockingModeCommand, sizeof(lockingModeCommand)); // Locking Mode

}
    else
    {
        PackedRCdataOut.ch0 = channelData[0];
        PackedRCdataOut.ch1 = channelData[1];
        PackedRCdataOut.ch2 = channelData[2];
        PackedRCdataOut.ch3 = channelData[3];
        PackedRCdataOut.ch4 = channelData[4];
        PackedRCdataOut.ch5 = channelData[5];
        PackedRCdataOut.ch6 = channelData[6];
        PackedRCdataOut.ch7 = channelData[7];
        PackedRCdataOut.ch8 = channelData[8];
        PackedRCdataOut.ch9 = channelData[9];
        PackedRCdataOut.ch10 = channelData[10];
        PackedRCdataOut.ch11 = channelData[11];
        PackedRCdataOut.ch12 = channelData[12];
        PackedRCdataOut.ch13 = channelData[13];
        PackedRCdataOut.ch14 = channelData[14];
        PackedRCdataOut.ch15 = channelData[15];
    }

    uint8_t extraData = 0;
    extraData |= effectivelyFailsafed ? SBUS_FLAG_FAILSAFE_ACTIVE : 0;
    extraData |= frameMissed ? SBUS_FLAG_SIGNAL_LOSS : 0;

    _outputPort->write(0x0F);    // HEADER
    _outputPort->write((byte *)&PackedRCdataOut, sizeof(PackedRCdataOut));
    _outputPort->write((uint8_t)extraData);    // ch 17, 18, lost packet, failsafe
    _outputPort->write((uint8_t)0x00);    // FOOTER
    return SBUS_CALLBACK_INTERVAL_MS;
}

#endif
