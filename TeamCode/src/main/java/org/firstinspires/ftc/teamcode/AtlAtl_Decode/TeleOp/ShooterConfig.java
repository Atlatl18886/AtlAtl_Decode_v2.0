// -------------
// PID NOT IN USE RN
//------------
package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {

    // Motor: goBILDA 5202/5203 6000 RPM, 28 ticks/rev, 1:1 to flywheel.
    // Max no-load ticks/s ≈ 6000/60 * 28 ≈ 2800.

    // vel presets (ticks/sec) ~~ 80% and 65% equivalents
    public static double MID_VEL   = 2240; // mid range
    public static double CLOSE_VEL = 1820; // very close

    // PIDF for RUN_USING_ENCODER
    public static double kP = 100.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 12.0;
}
/*
TUNE UNTIL
Spin‑up: when trigger pressed, the shooter reaches the target quickly (under ~1s) without big overshoot and doesnt oscillate.

Hold under load: when balls go through, recovers quickly without slowdown after a few shots.

Steady: at steady state, getVelocity() is close to targetVel and not bouncing frame‑to‑frame.

If ocsilation -- lower kP or add a BIT of kD. If it’s sluggish -- raise kP or kF SLIGHTLY.
 */