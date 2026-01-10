package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.macros;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TempPower {
    private final DcMotorEx[] motors;
    private final double[] overridePowers;

    private boolean prevPressed = false;
    private boolean active = false;

    private final double[] savedPowers;

    /**
     * @param motorsAndPowers  alternating list: motor, power, motor, power...
     *                         Example: new TempPower(intake, -1, transfer, 1, antiroller, 0.7)
     */
    public TempPower(Object... motorsAndPowers) {
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

        // Rising edge → activate
        if (pressed && !prevPressed) {
            active = true;

            // Save current powers
            for (int i = 0; i < motors.length; i++) {
                savedPowers[i] = motors[i].getPower();
            }

            // Apply override powers
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(overridePowers[i]);
            }
        }

        // Falling edge --> restore
        if (!pressed && prevPressed) {
            active = false;

            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(savedPowers[i]);
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
 private TempPowerMacro clearMacro;
  * INIT:
 clearMacro = new TempPowerMacro(
 intake,     -1.0,
 transfer,    1.0,
 antiroller,  0.7
 );
 * IN LOOP:
 clearMacro.update(gamepad2.x); //while gaepad2.x is held it sets intake to -1, transfer to 1, and antis to 0.7, and will set the powers back when released
 */
