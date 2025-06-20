#include "Copter.h"

  // 100 метров

ModeUP_100::SubMode_UP_100 ModeUP_100::althold_mode = SubMode_UP_100::TakeOff;
bool ModeUP_100::takeoff_complete;

bool ModeUP_100::init(bool ignore_checks)
{
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    pilot_roll = 0.0f;
    pilot_pitch = 0.0f;
    
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
 
    reached_target = false;
    target_alt_cm = 0;

    start_alt_cm = inertial_nav.get_position_z_up_cm();

    target_alt_cm = start_alt_cm + (copter.g.target_alt_meters.get() * 100);

    // Форсируем инициализацию торможения по всем осям
    brake.roll = 0.0f;
    brake.pitch = 0.0f;
    brake.angle_max_roll = 0.0f;
    brake.angle_max_pitch = 0.0f;
    brake.timeout_roll = 600*4;
    brake.timeout_pitch = 600*4;
    brake.time_updated_roll = false;
    brake.time_updated_pitch = false;

    return true;
}

void ModeUP_100::run()
{
    last_altitube = inertial_nav.get_position_z_up_cm();
    // Игнорируем пилотный ввод
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    
    // Фильтрация нулевого ввода
    update_pilot_lean_angle(pilot_roll, target_roll);
    update_pilot_lean_angle(pilot_pitch, target_pitch);

    // Постоянная активация торможения
    brake.roll = 0.0f;
    brake.pitch = 0.0f;
    brake.angle_max_roll = 0.0f;
    brake.angle_max_pitch = 0.0f;

    // Удержание высоты
    pos_control->set_pos_target_z_cm(last_altitube);
    pos_control->update_z_controller();

    // Применение нулевых углов
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
        brake.roll, 
        brake.pitch, 
        0.0f
    );

    float current_alt = inertial_nav.get_position_z_up_cm();
    if (!reached_target && (current_alt - target_alt_cm) > 10) {
        reached_target = true;
    }

    if (!reached_target) {
        // Продолжаем подъём с максимальной скоростью
        pos_control->land_at_climb_rate_cm(500.0, false);
    } else {
        // Удерживаем достигнутую высоту
        pos_control->land_at_climb_rate_cm(0.0, true);
        pos_control->update_z_controller();
    }
}


// poshold_update_pilot_lean_angle - update the pilot's filtered lean angle with the latest raw input received
void ModeUP_100::update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw)
{
    // if raw input is large or reversing the vehicle's lean angle immediately set the fitlered angle to the new raw angle
    if ((lean_angle_filtered > 0 && lean_angle_raw < 0) || (lean_angle_filtered < 0 && lean_angle_raw > 0) || (fabsf(lean_angle_raw) > 1800)) {
        lean_angle_filtered = lean_angle_raw;
    } else {
        // lean_angle_raw must be pulling lean_angle_filtered towards zero, smooth the decrease
        if (lean_angle_filtered > 0) {
            // reduce the filtered lean angle at 5% or the brake rate (whichever is faster).
            lean_angle_filtered -= MAX(lean_angle_filtered * 0.0125f, MAX(1.0f, g.poshold_brake_rate/(float)4));
            // do not let the filtered angle fall below the pilot's input lean angle.
            // the above line pulls the filtered angle down and the below line acts as a catch
            lean_angle_filtered = MAX(lean_angle_filtered, lean_angle_raw);
        }else{
            lean_angle_filtered += MAX(-lean_angle_filtered * 0.0125f, MAX(1.0f, g.poshold_brake_rate/(float)4));
            lean_angle_filtered = MIN(lean_angle_filtered, lean_angle_raw);
        }
    }
}

// mix_controls - mixes two controls based on the mix_ratio
//  mix_ratio of 1 = use first_control completely, 0 = use second_control completely, 0.5 = mix evenly
float ModeUP_100::mix_controls(float mix_ratio, float first_control, float second_control)
{
    mix_ratio = constrain_float(mix_ratio, 0.0f, 1.0f);
    return mix_ratio * first_control + (1.0f - mix_ratio) * second_control;
}

// update_brake_angle_from_velocity - updates the brake_angle based on the vehicle's velocity and brake_gain
//  brake_angle is slewed with the wpnav.poshold_brake_rate and constrained by the wpnav.poshold_braking_angle_max
//  velocity is assumed to be in the same direction as lean angle so for pitch you should provide the velocity backwards (i.e. -ve forward velocity)
void ModeUP_100::update_brake_angle_from_velocity(float &brake_angle, float velocity)
{
    float lean_angle;
    float brake_rate = g.poshold_brake_rate;

    brake_rate /= (float)4;
    if (brake_rate <= 1.0f) {
        brake_rate = 1.0f;
    }

    // calculate velocity-only based lean angle
    if (velocity >= 0) {
        lean_angle = -brake.gain * velocity * (1.0f + 500.0f / (velocity + 60.0f));
    } else {
        lean_angle = -brake.gain * velocity * (1.0f + 500.0f / (-velocity + 60.0f));
    }

    // do not let lean_angle be too far from brake_angle
    brake_angle = constrain_float(lean_angle, brake_angle - brake_rate, brake_angle + brake_rate);

    // constrain final brake_angle
    brake_angle = constrain_float(brake_angle, -(float)g.poshold_brake_angle_max, (float)g.poshold_brake_angle_max);
}
