package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TeleOpConfig {

    // DRIVE CONFIG
    public static String DRIVE_PRESET = "LERP"; //"QUADRATIC", "LINEAR"(default, original), "EXPONENTIAL", "LERP" - smooth linear, CAN ADD DRIFT
    public static double DRIVE_DEADZONE = 0.05; // the first 5% of joystick movement is ignored, prevents accidentals
    public static double AIM_TURN_SCALE = 0.15;  // turning slowdown when A is held

    public static double LERP_SPEED = 0.2; //0.1 for driftiness, greater values for responsiveness, 0.2 & 0.3 are avg lerps

}