package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double targetVel = 1100.0;

    // RUN_USING_ENCODER coefficients
    public static double kP = 100.0; //increase until flywheel reaches speed quick without oscilation
    public static double kI = 0.0;// keep small, only for getting rid of small steady state error
    public static double kD = 0.0; //add if it overshoots
    public static double kF = 12.0; //rough
}
