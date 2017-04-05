#pragma once

// _A2G_ - messages from UAV to GCS
// _G2a_ - messages from GCS to UAV

/**
  * @brief Packet to control mount payload function
  * @param 0 zoom speed 0-7, reverse & 0x10
  * @param 1 0..3 shutter bits, 4..7 rec bits
  * @param 2 flir settings 0b11000000 -> on, 0b01000000 -> off
  * @param 3 reserved (hdmi switch)
  */
#define FT_G2A_DATA16_MNT_PAYLOAD_CONTROL   0x01 //0xAC

/**
  * @brief Packet to calibrate alexmos imu
  * @param 0 trigger on-demand calibration       (True/False)
  * @param 1 (Dev) setup mount IMU helper stream (0xFF accepts config from this msg, otherwise No-Op)
  * @param 2 (Dev) regular correction mode       (0..4)
  * @param 3 (Dev) correction interval           [sec]
  */
#define FT_G2A_DATA16_MNT_IMU_CALIB         0x02 //0xDC

/**
  * @brief Dead reckoning
  * @param 0 enable / disable                   (set to 0 for afs setup messages)
  * @param 1 timeout [min]
  * @param 2 roll angle demand [deg]
  * @param 3 pitch angle demand [deg]
  * @param 4 wind dir MSB                       (4,5: must add up to <0, 360> deg range)
  * @param 5 wind dir LSB [deg]                 (where from the wind is blowing)
  * @param 6 wind speed [m/s]
  * @param 7 airspeed estimate [m/s]            (non-zero speed value arms the afs)
  */
#define FT_G2A_DATA16_PROC_DEAD_RECKON      0x03 //0xBD

/**
  * @brief Mount status
  * @param 0 status
  */
#define FT_A2G_DATA16_MNT_STATUS_EXT        0x80 //0xDD
