package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.control;

public class MultiModeToggle {
    private int mode = 0;
    private int maxModes;
    private boolean prevInput = false;

    public MultiModeToggle(int maxModes) {
        this.maxModes = maxModes;
    }

    public int update(boolean pressed) {
        if (pressed && !prevInput) {
            mode = (mode + 1) % maxModes;
        }
        prevInput = pressed;
        return mode;
    }

    public int get() {
        return mode;
    }

    public void set(int newMode) {
        mode = newMode % maxModes;
    }
}


/** EXAMPLE USAGE
 ModeToggle shooterMode = new ModeToggle(3); //3 for 3 states

 int mode = shooterMode.update(gamepad2.y);

 switch (mode) {
 case 0:
 shooter.setVelocity(0);
 break;
 case 1:
 shooter.setVelocity(1800);
 break;
 case 2:
 shooter.setVelocity(2400);
 break;
 }

 */