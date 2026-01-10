package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers;
public class EventLogger {
    private final String[] buffer;
    private int index = 0;
    private boolean filled = false;

    public EventLogger(int size) {
        buffer = new String[size];
    }
    public void add(String msg) {
        buffer[index] = msg;
        index = (index + 1) % buffer.length;
        if (index == 0) filled = true;
    }
    public void pushTo(TelemetryHelper group) {
        int count = filled ? buffer.length : index;
        int start = filled ? index : 0;

        for (int i = 0; i < count; i++) {
            int idx = (start + i) % buffer.length;
            group.add("[" + i + "]", buffer[idx]);
        }
    }
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