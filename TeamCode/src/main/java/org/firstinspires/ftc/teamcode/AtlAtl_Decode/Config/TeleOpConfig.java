package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TeleOpConfig {

    // DRIVE CONFIG
    public static final String DRIVE_PRESET = "TANH";
    ///"QUADRATIC", "LINEAR"(default, original, raw input), "EXPONENTIAL", "LERP"(smooth linear, ADDS DRIFT), "TANH", "CUBIC_BLEND", "ADAPTIVE"
    ///for teleopbasic: LINEAR, EXPONENTIAL, QUADRATIC, TANH
    public static double DRIVE_DEADZONE = 0.07; // the first 7.5% of joystick movement is ignored, prevents accidentals
    public static double AIM_TURN_SCALE = 0.15;  // turning slowdown when A is held

    public static boolean USE_SLEW_LIMITING = true;
    public static boolean USE_PRIORITY_SUPPRESSION = true;

    //----------------------preset-based
    public static double LERP_SPEED = 0.8; //0.1 for driftiness, greater values are more like linear mode, 0.2 & 0.3 are avg - DOESNT MATTER IF PRESET ISNT ON LERP
    public static double TANH_A = 3; //a controls the steepness. 1 is linear, 5+ is very steep - DOESNT MATTER IF PRESET ISNT ON TANH
    public static double CUBIC_WEIGHT = 0.3; //0 to 1, DOESNT MATTER If PRESET ISNT ON CUBIC_BLEND


    //----------------------imu PD values
    public static final double imu_kP = 0.04; //tuned for jayans house; RETUNE AFTER BREAK
    //kp tune until fast approach without oscillation, if it oscillates too much, decrease
    //add kd in tiny amounts until reduced overshoot and decreased ocsillation, if sluggish/stalls decrease
    public static final double imu_kD = 0.01; //deriv gain, also retune


}