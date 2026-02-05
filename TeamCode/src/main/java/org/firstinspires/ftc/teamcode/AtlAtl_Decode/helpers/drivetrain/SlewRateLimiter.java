package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain;

public class SlewRateLimiter {
    private double velocity = 0.0;
    private final double maxAccelUp;
    private final double maxAccelDown;
    private final double snapEpsilon = 0.02; // snap small residuals to zero to kill drift
    private final double feedForward = 0.35; // portion of target injected each step to reduce squish

    public SlewRateLimiter(double rateFactor) {
        this(rateFactor, rateFactor);
    }

    public SlewRateLimiter(double accelUp, double accelDown) {
        this.maxAccelUp = accelUp;
        this.maxAccelDown = accelDown;
    }

    public double calculate(double target, double dt) {
        double clampedDt = Math.min(Math.max(dt, 0.015), 0.1);
        double error = target - velocity;

        //no return drift
        if (Math.abs(target) < snapEpsilon && Math.abs(velocity) < snapEpsilon) {
            velocity = 0.0;
            return 0.0;
        }

        double accel = error > 0 ? maxAccelUp : maxAccelDown;
        //extra decel when reversing
        if (Math.signum(target) != Math.signum(velocity)) {
            accel *= 1.8;
        }

        double maxChange = accel * clampedDt;
        if (Math.abs(error) > Math.abs(maxChange)) {
            velocity += Math.signum(error) * Math.abs(maxChange);
        } else {
            velocity = target;
        }

        //inject small of target
        velocity += (target - velocity) * feedForward * clampedDt;

        // db residual drift
        if (Math.abs(velocity) < snapEpsilon) velocity = 0.0;
        return velocity;
    }

    public void reset() {
        velocity = 0.0;
    }
}