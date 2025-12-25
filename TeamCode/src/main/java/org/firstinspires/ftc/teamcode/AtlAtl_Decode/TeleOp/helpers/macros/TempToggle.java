package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.macros;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TempToggle {

    private final DcMotorEx[] motors;
    private final double[] overridePowers;

    private boolean active = false;
    private boolean prevPressed = false;

    private final double[] savedPowers;

    /**
     * @param motorsAndPowers alternating list: motor, power, motor, power...
     *                        Example: new TempToggleMacro(intake, -1, transfer, 1, antiroller, 0.7)
     */
    public TempToggle(Object... motorsAndPowers) {
        int count = motorsAndPowers.length / 2;

        motors = new DcMotorEx[count];
        overridePowers = new double[count];
        savedPowers = new double[count];

        for (int i = 0; i < count; i++) {
            motors[i] = (DcMotorEx) motorsAndPowers[i * 2];
            overridePowers[i] = (double) motorsAndPowers[i * 2 + 1];
        }
    }

    /** Call every loop */
    public void update(boolean pressed) {

        // Rising edge → toggle state
        if (pressed && !prevPressed) {
            active = !active;

            if (active) {
                // Save current powers
                for (int i = 0; i < motors.length; i++) {
                    savedPowers[i] = motors[i].getPower();
                }

                // Apply override powers
                for (int i = 0; i < motors.length; i++) {
                    motors[i].setPower(overridePowers[i]);
                }

            } else {
                // Restore saved powers
                for (int i = 0; i < motors.length; i++) {
                    motors[i].setPower(savedPowers[i]);
                }
            }
        }

        prevPressed = pressed;
    }

    public boolean isActive() {
        return active;
    }
}


/**EXAMPLE USAGE
 * BEFORE CLASS
 private TempPowerMacro reverseToggle;
 * INIT:
 reverseToggle = new TempToggleMacro(
 intake,     -1.0,
 transfer,    1.0,
 antiroller,  0.7
 );

 * IN LOOP:

 clearMacro.update(gamepad1.left_trigger > 0.1); //left trigger pressed once --> motor powers overriden to the reverseToggle. pressed again motor powers are restored to before
 */