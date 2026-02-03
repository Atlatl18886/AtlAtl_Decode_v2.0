package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain;

public class SlewRateLimiter {
    private double velocity = 0.0;
    private final double maxAccel;

    public SlewRateLimiter(double rateFactor) {
        this.maxAccel = rateFactor;
    }

    public double calculate(double target, double dt) {
        double maxChange = maxAccel * dt;
        double error = target - velocity;

        if (Math.abs(error) > maxChange) {
            velocity += Math.signum(error) * maxChange;
        } else {
            velocity = target;
        }

        return velocity;
    }

    public void reset() {
        velocity = 0.0;
    }
}