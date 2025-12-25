package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.util;

public class TimerHelper {

    private long startTime = 0;
    private long elapsedWhenStopped = 0;
    private boolean running = false;

    public TimerHelper() {}

    public void start() {
        if (!running) {
            startTime = System.currentTimeMillis();
            running = true;
        }
    }

    public void stop() {
        if (running) {
            elapsedWhenStopped = System.currentTimeMillis() - startTime;
            running = false;
        }
    }

    public void reset() {
        startTime = System.currentTimeMillis();
        elapsedWhenStopped = 0;
    }

    public long getTimer() {
        return running ? (System.currentTimeMillis() - startTime) : elapsedWhenStopped;
    }

    public boolean hasElapsed(long ms) {
        return getTimer() >= ms;
    }

    public boolean isRunning() {
        return running;
    }
}
/** USAGE
 * TimerHelper timer = new TimerHelper();
 * timer.start();
 *
 * if (timer.hasElapsed(500)) {
 *     // do something after 0.5s
 * }
 */