package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;

public class HeadingController {
    private double kP, kD;
    private double prevError = 0;
    private boolean firstCycle = true;
    private final ElapsedTime timer = new ElapsedTime();

    public HeadingController(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }

    private double wrap(double angle) {
        while (angle >= 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void reset() {
        firstCycle = true;
        prevError = 0;
        timer.reset();
    }

    public double compute(double currentHeading, double targetHeading) {
        double error = wrap(targetHeading - currentHeading);

        double dt = timer.seconds();
        timer.reset();
        if (dt < 1e-3) dt = 1e-3;

        double derivative;
        if (firstCycle) {
            derivative = 0;
            firstCycle = false;
        } else {
            derivative = (error - prevError) / dt;
        }

        prevError = error;

        return (kP * error) + (kD * derivative);
    }
}
