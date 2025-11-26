package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TeleOpConfig {

    // DRIVE CONFIG
    public static String DRIVE_PRESET = "LINEAR"; //"QUADRATIC", "LINEAR"(default, original, raw input), "EXPONENTIAL", "LERP"(smooth linear, ADDS DRIFT)

    public static double speedFactor = 0.8; //AKA SENS, range 0 to 1, percentage speed(1 means 435 rpm) IDK IF IT WORKS FOR LERP
    public static double DRIVE_DEADZONE = 0.05; // the first 5% of joystick movement is ignored, prevents accidentals
    public static double AIM_TURN_SCALE = 0.15;  // turning slowdown when A is held

    public static double LERP_SPEED = 0.38; //0.1 for driftiness, greater values are more like linear mode, 0.2 & 0.3 are avg - DOESNT MATTER IF PRESET ISNT ON LERP

    public static double imu_kP = 0.04; //tuned for jayans house; RETUNE AFTER BREAK
    public static double imu_turn_factor = 2;
    // SHOOTER CONFIG
    public static class shooter {
        //button speed presets
        public static double MID_RPM = 4000;
        public static double CLOSE_RPM = 2800;
        public static double FAR_RPM = 5600;
        public static double DEFAULT_RPM = 0; //1900
        public static double tolerance = 5000; // +/- rpm wiggle room for transfer cycle to initiate
        public static double feedtime = 1; //ms time for one ball through trnasfer
    }
}