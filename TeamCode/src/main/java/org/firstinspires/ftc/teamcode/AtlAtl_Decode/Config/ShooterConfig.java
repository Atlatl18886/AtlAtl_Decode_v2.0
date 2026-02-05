package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Config;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.util.Conversions;

@Config
public class ShooterConfig {
    /// when changing values in dash, change THESE
    public static double close = 1200;
    public static double mid = 1600;
    public static double far = 1850;

    /// dont change these in dash
    public static double DEFAULT = 0;

    public static double getCloseTps() { return Conversions.rpmToTps(close, 28); }
    public static double getMidTps() { return Conversions.rpmToTps(mid, 28); }
    public static double getFarTps() { return Conversions.rpmToTps(far, 28); }

    // TODO: RETUNE kI
    public static double shooter_Kf = 16.3; // 1.0/rough max tps
    public static double shooter_Kp = 515;  // Adjust if reaction is too slow
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