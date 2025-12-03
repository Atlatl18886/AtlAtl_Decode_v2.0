package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double MID_TPS = 800;
    public static double CLOSE_TPS = 350;
    public static double FAR_TPS = 1400;
    public static double DEFAULT_TPS = 0;
    public static double tolerance = 5000; // +/- rpm wiggle room for transfer cycle to initiate
    public static double feedtime = 1; //ms time for one ball through trnasfer
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
 */