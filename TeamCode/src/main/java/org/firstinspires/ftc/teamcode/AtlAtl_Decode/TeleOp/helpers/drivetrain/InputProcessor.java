package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.drivetrain;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Config.TeleOpConfig;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.TeleOpTest.DrivePreset;

public class InputProcessor {
    private final AdaptiveGamma adaptiveGamma;

    public InputProcessor(AdaptiveGamma adaptiveGamma) {
        this.adaptiveGamma = adaptiveGamma;
    }

    //returns {strafe, vert, heading} before suppression+slew
    public double[] process(Gamepad gp, DrivePreset preset, double deadzone, double robotSpeed) {
        //db remap raw inputs
        double rStrafe  = deadbandRemap(-gp.left_stick_x, deadzone);
        double rVertical = deadbandRemap(gp.left_stick_y, deadzone);
        double rHeading = deadbandRemap(-gp.right_stick_x, deadzone);

        double strafe, vertical, heading;

        if (preset == DrivePreset.ADAPTIVE) {
            double gamma = adaptiveGamma.compute(robotSpeed);

            strafe   = applyAdaptiveShaping(rStrafe, gamma);
            vertical = applyAdaptiveShaping(rVertical, gamma);
            heading  = applyAdaptiveShaping(rHeading, gamma);

        } else {
            strafe   = processInput(rStrafe, preset);
            vertical = processInput(rVertical, preset);
            heading  = processInput(rHeading, preset);
        }

        return new double[]{strafe, vertical, heading};
    }

    //util functions --
    private double deadbandRemap(double input, double deadzone) {
        if (Math.abs(input) < deadzone) return 0.0;
        return Math.signum(input) * (Math.abs(input) - deadzone) / (1.0 - deadzone);
    }

    private double applyAdaptiveShaping(double input, double gamma) {
        if (Math.abs(input) < 1e-6) return 0.0;
        return Math.signum(input) * Math.pow(Math.abs(input), gamma);
    }

    private double processInput(double input, DrivePreset preset) {
        switch (preset) {
            case QUADRATIC:
                return input * Math.abs(input);

            case CUBIC_BLEND:
                double w = TeleOpConfig.CUBIC_WEIGHT;
                return (input * (1.0 - w)) + (Math.pow(input, 3) * w);

            case EXPONENTIAL:
                return Math.pow(input, 3);

            case TANH:
                double a = TeleOpConfig.TANH_A;
                return Math.tanh(a * input) / Math.tanh(a);

            case LERP:
            case LINEAR:
            default:
                return input;
        }
    }
}
