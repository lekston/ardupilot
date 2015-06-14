// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If
// you wish to change any of the setup parameters from their default
// values, place the appropriate #define statements here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no
// longer valid! You should switch to using CONFIG_HAL_BOARD via the HAL_BOARD
// flag in your local config.mk instead.

// The following are the recommended settings for Xplane
// simulation. Remove the leading "/* and trailing "*/" to enable:

// disable HIL completely (for mission planner too!)
//#define HIL_PORT            -1
//#define HIL_MODE            ENABLED
//#define HIL_MODE            HIL_MODE_DISABLED

/*
 *  // HIL_MODE SELECTION
 *  //
 *  // Mavlink supports
 *  // 2. HIL_MODE_SENSORS: full sensor simulation
 *
 */

//Airspeed

//GPS
#define GPS_PROTOCOL        GPS_PROTOCOL_AUTO
 
//Navigation
#define MIN_GNDSPEED    3 // min GNDSPEED of 3 meters to prevent erratic behaviour in strong winds

#define WP_RADIUS_DEFAULT       50
#define LOITER_RADIUS_DEFAULT   100

//Logging
#define LOG_ATTITUDE_MED    ENABLED
#define LOG_ATTITUDE_FAST   DISABLED
#define LOG_GPS             ENABLED
#define LOG_PM              ENABLED
#define LOG_CTUN            DISABLED
#define LOG_NTUN            DISABLED
#define LOG_MODE            ENABLED
#define LOG_RAW             DISABLED
#define LOG_CMD             ENABLED
#define LOG_CUR	            ENABLED
#define LOG_CAMERA          ENABLED

//Usually mount defines are covered in AP_Mount.h
//#define MOUNT          ENABLED
//#define EXTERNAL_MOUNT
#ifdef EXTERNAL_MOUNT
    #define SERIAL1_BAUD                   38400
    #define SERIAL2_BAUD                   57600
#endif

//free some memory
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    #define GEOFENCE_ENABLED            DISABLED
    #define MAVLINK_COMM_NUM_BUFFERS    1
    #define CAMERA                      DISABLED
    //#define CLI_ENABLED               DISABLED
    #define MOUNT_RETRACT               DISABLED
    #define GPS_PROTOCOL                GPS_PROTOCOL_UBLOX
#endif
