package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain;

public class SlewRateLimiter {
    private double velocity = 0.0;
    private double acceleration = 0.0;

    private final double maxOutput;
    private final double maxAccel;
    private final double maxJerk;

    public SlewRateLimiter(double rateFactor) {
        this.maxOutput = 1.25;
        this.maxAccel = rateFactor;
        this.maxJerk = maxAccel * 12.0;
    }

    public double calculate(double target, double dt) {
        if (dt < 1e-6) return velocity;

        double error = target - velocity;

        //prop damping nears the joystick target, preventing the overshoot.
        double Kp = 10.0;
        double desiredAccel = error * Kp;

        //clamp to max accel
        if (desiredAccel > maxAccel) desiredAccel = maxAccel;
        else if (desiredAccel < -maxAccel) desiredAccel = -maxAccel;

        //jerk rate limiting
        double accelError = desiredAccel - acceleration;
        double maxAccelChange = maxJerk * dt;

        if (accelError > maxAccelChange) accelError = maxAccelChange;
        else if (accelError < -maxAccelChange) accelError = -maxAccelChange;

        //update vars
        acceleration += accelError;
        velocity += acceleration * dt;

        if (target == 0 && Math.abs(velocity) < 0.01) {
            velocity = 0;
            acceleration = 0;
        }

        return velocity;
    }

    public void reset() {
        velocity = 0.0;
        acceleration = 0.0;
    }
}