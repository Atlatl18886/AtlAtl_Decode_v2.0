package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.control;

public class MotorRamp {

    private double startValue;
    private double endValue;
    private long durationMs;
    private long startTime;
    private boolean active = false;

    public MotorRamp(double startValue, double endValue, long durationMs) {
        this.startValue = startValue;
        this.endValue = endValue;
        this.durationMs = durationMs;
    }

    public void start() {
        startTime = System.currentTimeMillis();
        active = true;
    }

    public void reset(double newStart, double newEnd, long newDurationMs) {
        this.startValue = newStart;
        this.endValue = newEnd;
        this.durationMs = newDurationMs;
        start();
    }

    public boolean isActive() {
        return active;
    }

    public double update() {
        if (!active) return endValue;

        long elapsed = System.currentTimeMillis() - startTime;

        if (elapsed >= durationMs) {
            active = false;
            return endValue;
        }

        double t = (double) elapsed / durationMs;
        return startValue + (endValue - startValue) * t;
    }
}
/**
 * USAGE
 * MotorRamp ramp = new MotorRamp(0, 1, 300);
 * ramp.start();
 *
 * double power = ramp.update(); // call every loop
 */