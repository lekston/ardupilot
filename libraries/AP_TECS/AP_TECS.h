// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_TECS.h
/// @brief   Combined Total Energy Speed & Height Control. This is a instance of an
/// AP_SpdHgtControl class

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, trim rate and damping parameters and the use
 *    of easy to measure aircraft performance data
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <DataFlash/DataFlash.h>

class AP_TECS : public AP_SpdHgtControl {
public:
    AP_TECS(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms) :
        _ahrs(ahrs),
        aparm(parms)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _deceleration_counter = 0;
#ifdef TECS_SIM_BUILD
        INITIALIZE_PARAMS;
#endif //TECS_SIM_BUILD

    }

#ifdef TECS_SIM_BUILD
    struct PACKED log_TECS_Tuning {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        float hgt;
        float dhgt;
        float hgt_dem;
        float dhgt_dem;
        float spd_dem;
        float spd;
        float dspd;
        float ithr;
        float iptch;
        float thr;
        float ptch;
        float dspd_dem;
        float speed_weight;
        uint8_t flags;
    } log_tuning;
    // log data on internal state of the controller. Called at 10Hz
    void log_data(DataFlash_Class &dataflash, uint8_t msgid);
#endif //TECS_SIM_BUILD

    // Update of the estimated height and height rate internal state
    // Update of the inertial speed rate internal state
    // Should be called at 50Hz or greater

    void update_50hz(void);

    // Update the control loop calculations
    void update_pitch_throttle(int32_t hgt_dem_cm,
                               int32_t EAS_dem_cm,
                               enum FlightStage flight_stage,
                               bool is_doing_auto_land,
                               float distance_beyond_land_wp,
                               int32_t ptchMinCO_cd,
                               int16_t throttle_nudge,
                               float hgt_afe,
                               float load_factor);

    // demanded throttle in percentage
    // should return -100 to 100, usually positive unless reverse thrust is enabled via _THRminf < 0
    int32_t get_throttle_demand(void) {
        return int32_t(_throttle_dem * 100.0f);
    }

    int32_t get_throttle_raw_dem(void) const {
        return int32_t(100.0f* constrain_float(_throttle_raw_dem, _THRminf, _THRmaxf) ); 
        // range: 0 - 100, non-linearized
    }

    // demanded pitch angle in centi-degrees
    // should return between -9000 to +9000
    int32_t get_pitch_demand(void) {
        return int32_t(_pitch_dem * 5729.5781f);
    }

    // Rate of change of velocity along X body axis in m/s^2
    float get_VXdot(void) {
        return _vel_dot;
    }

    // return current target airspeed
    float get_target_airspeed(void) const {
        return _TAS_dem / _ahrs.get_EAS2TAS();
    }

    // return maximum climb rate
    float get_max_climbrate(void) const {
        return _maxClimbRate;
    }

    // return landing sink rate
    float get_land_sinkrate(void) const {
        return _land_sink;
    }

    // return landing airspeed
    float get_land_airspeed(void) const {
        return _landAirspeed;
    }

    // return height rate demand, in m/s
    float get_height_rate_demand(void) const {
        return _hgt_rate_dem;
    }

    // set path_proportion
    void set_path_proportion(float path_proportion) {
        _path_proportion = constrain_float(path_proportion, 0.0f, 1.0f);
    }

    // set pitch max limit in degrees
    void set_pitch_max_limit(int8_t pitch_limit) {
        _pitch_max_limit = pitch_limit;
    }

    // force use of synthetic airspeed for one loop
    void use_synthetic_airspeed(void) {
        _use_synthetic_airspeed = true;
    }

    void force_current_alt(float hgt_afe);

    bool get_underspeed(void) const {
        /* higher level module should check if
         * current consumption and battery levels are OK
         */
        // hal.console->printf("Low speed, check engine\n");
        return _flags.underspeed;
    }

    int32_t get_opt_bitmask(void) const {
        return int32_t(_opt_bitmask);
    }

    // this supports the TECS_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    // Last time update_50Hz was called
    uint64_t _update_50hz_last_usec;

    // Last time update_speed was called
    uint64_t _update_speed_last_usec;

    // Last time update_pitch_throttle was called
    uint64_t _update_pitch_throttle_last_usec;

    // reference to the AHRS object
    AP_AHRS &_ahrs;

    const AP_Vehicle::FixedWing &aparm;

    // TECS tuning parameters
    AP_Float _hgtCompFiltOmega;
    AP_Float _spdCompFiltOmega;
    AP_Float _maxClimbRate;
    AP_Float _minSinkRate;
    AP_Float _maxSinkRate;
    AP_Float _timeConst;
    AP_Float _landTimeConst;
    AP_Float _ptchDamp;
    AP_Float _land_pitch_damp;
    AP_Float _landDamp;
    AP_Float _thrDamp;
    AP_Float _land_throttle_damp;
    AP_Float _integGain;
    AP_Float _integGain_takeoff;
    AP_Float _integGain_land;
    AP_Float _vertAccLim;
    AP_Float _rollComp;
    AP_Float _spdWeight;
    AP_Float _spdWeightLand;
    AP_Float _landThrottle;
    AP_Float _landAirspeed;
    AP_Float _land_sink;
    AP_Float _land_sink_rate_change;
    AP_Int8  _pitch_max;
    AP_Int8  _pitch_min;
    AP_Int8  _land_pitch_max;
    AP_Float _maxSinkRate_approach;
    AP_Float _us_energy_jerk;
    AP_Int32 _opt_bitmask;
    AP_Int8  _us_fade_time_max;
    AP_Float _thr_exp;

    // temporary _pitch_max_limit. Cleared on each loop. Clear when >= 90
    int8_t _pitch_max_limit = 90;

    // current height estimate (above field elevation)
    float _height;

    // throttle demand in the range from -1.0 to 1.0, usually positive unless reverse thrust is enabled via _THRminf < 0
    float _throttle_dem;

    float _throttle_raw_dem;

    // pitch angle demand in radians
    float _pitch_dem;

    // estimated climb rate (m/s)
    float _climb_rate;

    /*
      a filter to estimate climb rate if we don't have it from the EKF
     */
    struct {
        // height filter second derivative
        float dd_height;

        // height integration
        float height;
    } _height_filter;

    // Integrator state 4 - airspeed filter first derivative
    float _integDTAS_state;

    // Integrator state 5 - true airspeed
    float _TAS_state;

    // Integrator state 6 - throttle integrator
    float _integTHR_state;

    // Integrator state 6 - pitch integrator
    float _integSEB_state;

    // throttle demand rate limiter state
    float _last_throttle_dem;

    // pitch demand rate limiter state
    float _last_pitch_dem;

    // Rate of change of speed along X axis
    float _vel_dot;

    // Equivalent airspeed
    float _EAS;

    // True airspeed limits
    float _TASmax;
    float _TASmin;

    // Current and last true airspeed demand
    float _TAS_dem;
    float _TAS_dem_last;

    // Equivalent airspeed demand
    float _EAS_dem;

    // height demands
    float _hgt_dem;
    float _hgt_dem_in_old;
    float _hgt_dem_adj;
    float _hgt_dem_adj_last;
    float _hgt_rate_dem;
    float _hgt_dem_prev;
    float _land_hgt_dem;

    // Speed demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_dem_adj;

    // Speed rate demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_rate_dem;

    // Total energy rate filter state
    float _STEdotErrLast;

    // LP filter state of Specific Kinetic Energy weighting
    float _lastSKE_weighting;

    struct flags {
        // Underspeed condition
        bool underspeed:1;

        // Bad descent condition caused by unachievable airspeed demand
        bool badDescent:1;

        // true when plane is in auto mode and executing a land mission item
        bool is_doing_auto_land:1;

        // true when we have reached target speed in takeoff
        bool reached_speed_takeoff:1;

        // Underspeed event transition
        bool us_triggered:1;

        bool using_integ_constr:1;
    };
    union {
        struct flags _flags;
        uint8_t _flags_byte;
    };

    // Underspeed transition
    uint16_t _us_fadeout;

    // time when underspeed started
    uint32_t _underspeed_start_ms;

    // auto mode flightstage
    enum FlightStage _flight_stage;

    // pitch demand before limiting
    float _pitch_dem_unc;

    // Maximum and minimum specific total energy rate limits
    float _STEdot_max;
    float _STEdot_min;

    // Maximum and minimum floating point throttle limits
    float _THRmaxf;
    float _THRminf;

    // Maximum and minimum floating point pitch limits
    float _PITCHmaxf;
    float _PITCHminf;

    // Specific energy quantities
    float _SPE_dem;
    float _SKE_dem;
    float _SPEdot_dem;
    float _SKEdot_dem;
    float _SPE_est;
    float _SKE_est;
    float _SPEdot;
    float _SKEdot;

    // Specific energy error quantities
    float _STE_error;

    // Time since last update of main TECS loop (seconds)
    float _DT;

    // counter for demanded sink rate on land final
    uint8_t _flare_counter;

    // slew height demand lag filter value when transition to land
    float hgt_dem_lag_filter_slew;

    // percent traveled along the previous and next waypoints
    float _path_proportion;

    float _distance_beyond_land_wp;

    // internal variables to be logged
    struct {
        float SKE_weighting;
        float SPE_error;
        float SKE_error;
        float SEB_delta;
    } logging;

    // use synthetic airspeed for next loop
    bool _use_synthetic_airspeed;

    // helps to detect prolonged periods of deceleration
    int32_t _deceleration_counter;

    // Update the airspeed internal state using a second order complementary filter
    void _update_speed(float load_factor);

    // Update the demanded airspeed
    void _update_speed_demand(void);

    // Update the demanded height
    void _update_height_demand(void);

    // Detect an underspeed condition
    void _detect_underspeed(void);

    // Update Specific Energy Quantities
    void _update_energies(void);

    // Update Demanded Throttle
    void _update_throttle_with_airspeed(void);

    // Update Demanded Throttle Non-Airspeed
    void _update_throttle_without_airspeed(int16_t throttle_nudge);

    // get integral gain which is flight_stage dependent
    float _get_i_gain(void);

    // Detect Bad Descent
    void _detect_bad_descent(void);

    // Update Demanded Pitch Angle
    void _update_pitch(void);

    // Initialise states and variables
    void _initialise_states(int32_t ptchMinCO_cd, float hgt_afe);

    // Calculate specific total energy rate limits
    void _update_STE_rate_lim(void);

    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;

    // current time constant
    float timeConstant(void) const;
};

#define TECS_LOG_FORMAT(msg) { msg, sizeof(AP_TECS::log_TECS_Tuning),    \
                               "TECS", "QfffffffffffffB", "TimeUS,h,dh,hdem,dhdem,spdem,sp,dsp,ith,iph,th,ph,dspdem,w,f" }

#define USE_OPT_BITMASK_DEFAULT                 0x00C0 // jerk inertial by default for all modes

#define USE_OPT_BITMASK_ANTI_WINDUP_1           (1<<0)
#define USE_OPT_BITMASK_ANTI_WINDUP_2           (1<<1)
#define USE_OPT_BITMASK_US_THROTTLE_JERK_MODE_L (1<<2) // 0-1st order inertial / 1-energy based
#define USE_OPT_BITMASK_US_THROTTLE_JERK_MODE_N (1<<3) // Normal flight

#define USE_OPT_BITMASK_US_LOAD_FACTOR_CORR_1   (1<<4)
#define USE_OPT_BITMASK_US_LOAD_FACTOR_CORR_2   (1<<5)

#define USE_OPT_BITMASK_THROTTLE_CURVE          (1<<6)  // Enabled by default; 0x40
#define USE_OPT_BITMASK_RESERVED0               (1<<7)  // Enabled by default; 0x80

#define USE_OPT_BITMASK_THR_FBWB_MANUAL         (1<<8)  // Disabled by default; 0x100
#define USE_OPT_BITMASK_THR_FBWB_MAN_LIN        (1<<9)  // Disabled by default; 0x200
#define USE_OPT_BITMASK_THR_FBWB_DEVIATION      (1<<10) // Disabled by default; 0x400

#define USE_OPT_BITMASK_THR_FBWB_TEST_REVERSE   (1<<15) // Disabled by default; 0x8000

#define THR_REV_TRUE        100
#define THR_REV_FALSE       0

// use _opt_bitmask 0x07C0 for FBWB TECS testing 

