#include "Copter.h"

#define TARGET_ALT_CM 10000.0  // 100 метров
/*
 * Init and run calls for althold, flight mode
 */

ModeUP_100::SubMode ModeUP_100::guided_mode = SubMode::TakeOff;
bool ModeUP_100::takeoff_complete;

// althold_init - initialise althold controller
bool ModeUP_100::init(bool ignore_checks)
{

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
 
    last_altitube = inertial_nav.get_position_z_up_cm();

    return true;
}

bool ModeUP_100::do_user_takeoff_start(float takeoff_alt_cm)
{
    // calculate target altitude and frame (either alt-above-ekf-origin or alt-above-terrain)
    
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
    {
        // interpret altitude as alt-above-home
        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);

        // provide target altitude as alt-above-ekf-origin
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            // this should never happen but we reject the command just in case
            return false;
        }
    }

    guided_mode = SubMode::TakeOff;

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // clear i term when we're taking off
    pos_control->init_z_controller();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff.start(alt_target_cm, alt_target_terrain);

    // record takeoff has not completed
    takeoff_complete = false;

    return true;
}

void ModeUP_100::takeoff_run()
{
    // Вызов основного контроллера взлёта
    auto_takeoff.run();
    do_user_takeoff_start(last_altitube + TARGET_ALT_CM);
    
    // Обработка завершения взлёта
    if (auto_takeoff.complete && !takeoff_complete) {
        takeoff_complete = true;
    }
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeUP_100::run()
{
    switch (guided_mode) {

        case SubMode::TakeOff:
            // run takeoff controller
            takeoff_run();
            break;
    
        case SubMode::WP:
            // run waypoint controller
            break;
    
        case SubMode::Pos:
            // run position controller
            break;
    
        case SubMode::Accel:
            break;
    
        case SubMode::VelAccel:
            break;
    
        case SubMode::PosVelAccel:
            break;
    
        case SubMode::Angle:
            break;
        }
    }
