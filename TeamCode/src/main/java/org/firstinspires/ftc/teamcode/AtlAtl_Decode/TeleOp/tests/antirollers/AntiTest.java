package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.tests.antirollers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Antis Test", group = "tests")
public class AntiTest extends OpMode {

    private DcMotorEx antiroller;

    private boolean intakeActive = false;
    private boolean intakePrev = false;
    private double progress = 0.0;

    @Override
    public void init() {

        antiroller = hardwareMap.get(DcMotorEx.class, "antiroller");
        antiroller.setDirection(DcMotorEx.Direction.REVERSE);
        antiroller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("init done", "");
    }

    @Override
    public void loop() {
        Intake();

        telemetry.addData("AntiRoller Power", "%.2f", antiroller.getPower());
        telemetry.addData("Toggle State (LB)", intakeActive ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Ramp Progress", "%.2f", progress);
        telemetry.update();
    }

    public void Intake() {
        boolean bumperPressed = gamepad2.left_bumper;
        if (bumperPressed && !intakePrev) intakeActive = !intakeActive;

        intakePrev = bumperPressed;

        double rampIncrement = 0.05;
        if (intakeActive) {
            progress += rampIncrement;
        } else {
            progress -= rampIncrement;
        }
        progress = Math.max(0.0, Math.min(1.0, progress));

        final double min = 0.15;
        final double max = 0.7;
        final double range = max - min;

        // formula: min + (range * (Progress * Progress)) - Quadratic Ramping
        double antirollerPower = min + (range * (progress * progress));
        antirollerPower = Math.max(min, Math.min(max, antirollerPower));
        antiroller.setPower(antirollerPower);
    }
}