package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

//imu imports
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpTest extends OpMode {
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intake;
    private DcMotorEx transfer;
    private DcMotorEx shooter;

    private IMU imu;

    // memory for LERP preset
    private double prevStrafe = 0;
    private double prevVertical = 0;
    private double prevHeading = 0;

    @Override
    public void init() {
        //Initialize Drivetrain -----------------------------------------------------
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // imu init
        imu = hardwareMap.get(IMU.class, "imu");

        // important to change
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();


        // init everything else
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorEx.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

    }

    @Override
    public void loop() {
        //jsdoc bec its cool
        // @param {String} PRESET - "QUADRATIC", "LINEAR" - default, old, "EXPONENTIAL", "LERP" - smooth linear, CAN ADD DRIFT
        // @param {int} DEADZONE - 0.05 would mean the first 5% of joystick movement is ignored, prevents accidentals
        Drive("LERP", 0.05);
        Intake();
        Transfer();
        Shooter();

        // telemetry for debug with imu
        telemetry.addData("Current Heading", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Aim Mode Active (A Button)", gamepad1.a);
    }

    public void Drive(String PRESET, double deadzone) {
        double rawStrafe   = -gamepad1.left_stick_x;
        double rawVertical = gamepad1.left_stick_y;
        double rawHeading  = -gamepad1.right_stick_x;

        double strafe, vertical, heading;

        if (PRESET.equals("LERP")) {

            // lerp is special (time-based smoothing)
            // 0.1 is the "smoothing factor" (0.0 = no movement, 1.0 = instant movement)
            // lower = smoother/slower acceleration
            double lerpSpeed = 0.2;

            //apply deadzone first
            if(Math.abs(rawStrafe) < deadzone) rawStrafe = 0;
            if(Math.abs(rawVertical) < deadzone) rawVertical = 0;
            if(Math.abs(rawHeading) < deadzone) rawHeading = 0;

            strafe   = lerp(prevStrafe, rawStrafe, lerpSpeed);
            vertical = lerp(prevVertical, rawVertical, lerpSpeed);
            heading  = lerp(prevHeading, rawHeading, lerpSpeed);

            //save for next loop
            prevStrafe = strafe;
            prevVertical = vertical;
            prevHeading = heading;

        } else {
            // standard Curves (LINEAR/QUAD/EXPONENTIAL)
            strafe   = processInput(rawStrafe, PRESET, deadzone);
            vertical = processInput(rawVertical, PRESET, deadzone);
            heading  = processInput(rawHeading, PRESET, deadzone);
        }

        // aim mode
        if (gamepad1.a) {
            heading *= 0.15; //slow turning
        }

        //standard mec math
        double leftFrontPower  = vertical + strafe + heading;
        double rightFrontPower = vertical + strafe - heading;
        double leftBackPower   = vertical - strafe + heading;
        double rightBackPower  = vertical - strafe - heading;

        //noramlize
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        ));

        leftFront.setPower(leftFrontPower / max);
        rightFront.setPower(rightFrontPower / max);
        leftBack.setPower(leftBackPower / max);
        rightBack.setPower(rightBackPower / max);
    }

    //helper for curves
    private double processInput(double input, String preset, double deadzone) {
        if (Math.abs(input) < deadzone) return 0.0;
        switch (preset) {
            case "QUADRATIC": return input * Math.abs(input);
            case "EXPONENTIAL": return Math.pow(input, 3);
            default: return input;
        }
    }

    //lerp is special
    //formula: Current + (Target - Current) * Speed
    private double lerp(double current, double target, double speed) {
        return current + (target - current) * speed;
    }


    //non dt functionalities
    private boolean intakeStopped = false;
    private boolean intakePrev = false;
    public void Intake() {
        if (gamepad1.left_bumper && !intakePrev) {
            intakeStopped = !intakeStopped;
        }
        intakePrev = gamepad1.left_bumper;

        double intakePower = intakeStopped ? 0 : 1.0;
        intake.setPower(intakePower);
    }

    public void Transfer() {
        if (gamepad1.right_bumper) {
            transfer.setPower(-0.3);
        } else {
            transfer.setPower(1.0);
        }
    }

    public void Shooter() {
        if (gamepad1.right_trigger > 0.1) {
            shooter.setVelocity(1100);
        } else {
            shooter.setPower(0);
        }
    }
}