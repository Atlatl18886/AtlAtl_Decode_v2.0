package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.control;

public class Debouncer {

    private boolean lastState = false;
    private boolean currentState = false;

    private long lastChangeTime = 0;
    private long debounceMs;

    public Debouncer(long debounceMs) {
        this.debounceMs = debounceMs;
    }

    public void update(boolean rawInput) {
        long now = System.currentTimeMillis();

        if (rawInput != currentState && (now - lastChangeTime) > debounceMs) {
            lastState = currentState;
            currentState = rawInput;
            lastChangeTime = now;
        }
    }

    public boolean isRisingEdge() {
        return !lastState && currentState;
    }

    public boolean isFallingEdge() {
        return lastState && !currentState;
    }

    public boolean isStable() {
        return (System.currentTimeMillis() - lastChangeTime) > debounceMs;
    }

    public boolean getState() {
        return currentState;
    }
}
/**
 * Usage:
 * Debouncer intakeButton = new Debouncer(50);
 * intakeButton.update(gamepad1.a);
 *
 * if (intakeButton.isRisingEdge()) {
 *     // button just pressed
 * }
 *
 * Trigger example:
 * triggerDebounce.update(gamepad1.right_trigger > 0.5);
 */