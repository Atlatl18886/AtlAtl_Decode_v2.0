package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.control;

public class ButtonHoldAction {

    private boolean lastState = false;
    private boolean currentState = false;
    private long pressStartTime = 0;

    public ButtonHoldAction() {}

    ///upd with raw input (button or trigger > threshold)
    public void update(boolean pressed) {
        currentState = pressed;

        // detect rising edge
        if (currentState && !lastState) {
            pressStartTime = System.currentTimeMillis();
        }

        lastState = pressed;
    }

    ///true only on the frame the button is pressed
    public boolean isJustPressed() {
        return currentState && !lastState;
    }

    ///true only on the frame the button is released
    public boolean isJustReleased() {
        return !currentState && lastState;
    }

    /// True while the button is held
    public boolean isHeld() {
        return currentState;
    }

    ///true if held for at least ms milliseconds
    public boolean heldFor(long ms) {
        return currentState && (System.currentTimeMillis() - pressStartTime >= ms);
    }
}
/**
 * EXAMPLE USAGE
 *
 * BUTTON
 * ButtonHoldAction slowMode = new ButtonHoldAction();
 *
 * slowMode.update(gamepad1.left_bumper);
 *
 * if (slowMode.isJustPressed()) {
 *     // activate slow mode
 * }
 *
 * if (slowMode.isJustReleased()) {
 *     // deactivate slow mode
 * }
 *
 * if (slowMode.isHeld()) {
 *     // continuously apply slow mode
 * }
 *
 *
 * TRIGGER
 * ButtonHoldAction precisionDrive = new ButtonHoldAction();
 *
 * precisionDrive.update(gamepad1.right_trigger > 0.5);
 *
 * if (precisionDrive.heldFor(300)) {
 *     // trigger held for 300ms
 * }
 */