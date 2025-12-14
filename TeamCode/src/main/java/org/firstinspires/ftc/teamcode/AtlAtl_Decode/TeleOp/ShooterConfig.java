package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double MID_TPS = 800;
    public static double CLOSE_TPS = 260;
    public static double FAR_TPS = 1400;
    public static double DEFAULT_TPS = 180;

    public static double shooter_Kf = 0.00037; // 1.0/rough max tps
    public static double shooter_Kp = 0.00075;  // Adjust if reaction is too slow
    public static double shooter_Ki = 0.001;
    public static double shooter_Kd = 0.0;
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