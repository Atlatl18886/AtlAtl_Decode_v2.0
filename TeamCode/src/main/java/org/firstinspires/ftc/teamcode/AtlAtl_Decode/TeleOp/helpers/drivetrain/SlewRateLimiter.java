package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.drivetrain;

public class SlewRateLimiter {
    private double velocity = 0.0; // filtered output
    private double acceleration = 0.0; // internal state

    private final double maxVel;
    private final double maxAccel;

    public SlewRateLimiter(double maxVel) {
        this.maxVel = maxVel;
        this.maxAccel = maxVel * 4.0; //jerk control
    }

    public double calculate(double target, double dt) {
        if (dt < 1e-6) return velocity;

        double desiredAccel = (target - velocity) / dt;
        double accelDelta = desiredAccel - acceleration;
        double maxAccelChange = maxAccel * dt;

        if (accelDelta > maxAccelChange) accelDelta = maxAccelChange;
        else if (accelDelta < -maxAccelChange) accelDelta = -maxAccelChange;

        acceleration += accelDelta;
        velocity += acceleration * dt;

        // safety clamp
        if (velocity > maxVel) velocity = maxVel;
        else if (velocity < -maxVel) velocity = -maxVel;

        return velocity;
    }

    public void reset() {
        velocity = 0.0;
        acceleration = 0.0;
    }
}
