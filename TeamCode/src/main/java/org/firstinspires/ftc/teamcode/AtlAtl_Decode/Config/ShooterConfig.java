package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double MID_TPS = 890;
    public static double CLOSE_TPS = 540;
    public static double FAR_TPS = 1100;
    public static double DEFAULT_TPS = 0;

    // TODO: CHECK IF TOMAHAWK IS SCAMMING US
    public static double shooter_Kf = 16; // 1.0/rough max tps
    public static double shooter_Kp = 520;  // Adjust if reaction is too slow
    public static double shooter_Ki = 1.65;
    public static double shooter_Kd = 5;
}

/*
     Ticks per revolution for GoBILDA 5202 1:1 6000 == 28 TPR

            Ticks per second * 60
     RPM = ------------------------
            Ticks Per Revolution


                        RPM * Ticks Per Revolution
     Ticks per second = ---------------------------
                                    60

     Max RPM: 6000
     Max TPS: 2800
     OR
     Multiply RPM by (7/15)
 */