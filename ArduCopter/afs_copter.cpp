/*
  copter specific AP_AdvancedFailsafe class
 */

#include "Copter.h"

#if ADVANCED_FAILSAFE == ENABLED

// Constructor
AP_AdvancedFailsafe_Copter::AP_AdvancedFailsafe_Copter(AP_Mission &_mission, AP_Baro &_baro, const AP_GPS &_gps, const RCMapper &_rcmap) :
    AP_AdvancedFailsafe(_mission, _baro, _gps, _rcmap)
{}


/*
  setup radio_out values for all channels to termination values
 */
void AP_AdvancedFailsafe_Copter::terminate_vehicle(void)
{
    // stop motors
    copter.motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
    copter.motors.output();

    // disarm as well
    copter.init_disarm_motors();
    
    // and set all aux channels
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_heli_rsc, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_heli_tail_rsc, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_engine_run_enable, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_ignition, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_none, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_manual, RC_Channel::RC_CHANNEL_LIMIT_TRIM);

    RC_Channel_aux::output_ch_all();
}

void AP_AdvancedFailsafe_Copter::setup_IO_failsafe(void)
{
    // setup failsafe for all aux channels
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_heli_rsc, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_heli_tail_rsc, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_engine_run_enable, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_ignition, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_none, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_manual, RC_Channel::RC_CHANNEL_LIMIT_TRIM);

#if FRAME_CONFIG != HELI_FRAME
    // setup AP_Motors outputs for failsafe
    uint16_t mask = copter.motors.get_motor_mask();
    hal.rcout->set_failsafe_pwm(mask, copter.motors.get_pwm_output_min());
#endif
}

/*
  return an AFS_MODE for current control mode
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Copter::afs_mode(void)
{
    switch (copter.control_mode) {
    case AUTO:
    case GUIDED:
    case RTL:
    case LAND:
        return AP_AdvancedFailsafe::AFS_AUTO;
    default:
        break;
    }
    return AP_AdvancedFailsafe::AFS_STABILIZED;
}

void AP_AdvancedFailsafe_Copter::vehicle_gps_loss_specific(void)
{
	const uint32_t timeout_ms = 1 * 60 * 1000;
	copter.set_mode(GUIDED_NOGPS, MODE_REASON_GCS_COMMAND);
//	const Vector3f wind = copter.ahrs.wind_estimate();
	const Location& home = copter.ahrs.get_home();
	const Location& last_fixed_loc = copter.ahrs.get_gps().location();
	int32_t bearing_cd = get_bearing_cd(last_fixed_loc, home);
	Quaternion q;
	//TODO compensate wind
	q.from_euler(0.0, ToRad(-10.0f), ToRad(bearing_cd / 100.0f));
	copter.guided_set_angle(q, 10.0, false, 0.5, timeout_ms);
}

#endif // ADVANCED_FAILSAFE
