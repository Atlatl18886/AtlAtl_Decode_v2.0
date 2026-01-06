package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.util;

public class LoopProfiler {
    private long lastTime = 0;
    private double avg = 0;
    private double min = Double.MAX_VALUE;
    private double max = 0;
    private int count = 0;

    public void start() {
        lastTime = System.nanoTime();
    }

    public double update() {
        long now = System.nanoTime();
        double ms = (now - lastTime) / 1e6;
        lastTime = now;

        count++;
        avg += (ms - avg) / count;
        min = Math.min(min, ms);
        max = Math.max(max, ms);

        return ms;
    }
    public double getCount() { return count; }
    public double getAvg() { return avg; }
    public double getMin() { return min; }
    public double getMax() { return max; }
    public double getJitter() { return max - min; }
}
/** EXAMPLE USAGE:
 * BEFORE CLASS:
 LoopProfiler profiler = new LoopProfiler();
 EventLogger events = new EventLogger(10);

 TelemetryHelper debug = TelemetryHelper.create(telemetry, "Debug");
 TelemetryHelper loop = debug.child("Loop");
 TelemetryHelper log = debug.child("Events");

 profiler.start();
 * LOOP:
 debug.clear();

 // loop profiler
 double ms = profiler.update();
 loop.addf("loop", "%.2f", ms);
 loop.addf("avg", "%.2f", profiler.getAvg());
 loop.addf("min", "%.2f", profiler.getMin());
 loop.addf("max", "%.2f", profiler.getMax());
 loop.addf("jitter", "%.2f", profiler.getJitter());

 // event logger
 events.pushTo(log);

 // example event
 if (gamepad1.a) events.add("A pressed");

 // push everything
 debug.push();
 telemetry.update();

 */