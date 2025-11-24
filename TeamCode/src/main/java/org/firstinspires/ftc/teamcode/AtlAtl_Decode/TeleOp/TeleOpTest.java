package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

// imu imports
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

    // shooter target (ticks/sec), switched by buttons
    private double shooterTargetVel = 0.0;

    @Override
    public void init() {
        //init dt
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

        // IMU orientation
        // IMPORTANT TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();


        // init everything else
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setPower(0);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setPower(0);

        //pidf setup; resets and RUN_USING_ENCODER
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // PIDF shooter setup: reset encoder + RUN_USING_ENCODER
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
        // apply PIDF coefficients from ShooterConfig
        shooter.setVelocityPIDFCoefficients(
                ShooterConfig.kP,
                ShooterConfig.kI,
                ShooterConfig.kD,
                ShooterConfig.kF
        );
        */
        shooter.setPower(0);

    }

    @Override
    public void loop() {
        //from config
        Drive(TeleOpConfig.DRIVE_PRESET, TeleOpConfig.DRIVE_DEADZONE);
        Intake();
        Transfer();
        Shooter();

        // telemetry for debug with imu + shooter
        telemetry.addData("Current Heading", "%.2f",
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Aim Mode Active (A Button)", gamepad1.a);
        telemetry.addData("Shooter Target Vel", shooterTargetVel);
        telemetry.addData("Shooter Actual Vel", shooter.getVelocity());
        telemetry.update();
    }

    public void Drive(String PRESET, double deadzone) {
        double rStrafe = -gamepad1.left_stick_x;
        double rVertical =  gamepad1.left_stick_y;
        double rHeading = -gamepad1.right_stick_x;

        double strafe, vertical, heading;

        if (PRESET.equals("LERP")) {

            // lerp is special (time-based smoothing)
            double lerpSpeed = TeleOpConfig.LERP_SPEED;

            // apply deadzone first
            if (Math.abs(rStrafe)   < deadzone) rStrafe = 0;
            if (Math.abs(rVertical) < deadzone) rVertical = 0;
            if (Math.abs(rHeading)  < deadzone) rHeading = 0;

            strafe   = lerp(prevStrafe, rStrafe,   lerpSpeed);
            vertical = lerp(prevVertical, rVertical, lerpSpeed);
            heading  = lerp(prevHeading, rHeading,  lerpSpeed);

            // save for next loop
            prevStrafe   = strafe;
            prevVertical = vertical;
            prevHeading  = heading;

        } else {
            // standard Curves (LINEAR/QUAD/EXPONENTIAL)
            strafe   = processInput(rStrafe,   PRESET, deadzone);
            vertical = processInput(rVertical, PRESET, deadzone);
            heading  = processInput(rHeading,  PRESET, deadzone);
        }

        //aim mode from config
        if (gamepad1.a) {
            heading *= TeleOpConfig.AIM_TURN_SCALE; //slow turning
        }

        //standard mec math
        double leftFrontPower  = vertical + strafe + heading;
        double rightFrontPower = vertical + strafe - heading;
        double leftBackPower   = vertical - strafe + heading;
        double rightBackPower  = vertical - strafe - heading;

        //noramlize
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(leftFrontPower),  Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower),   Math.abs(rightBackPower))
        ));

        leftFront.setPower(leftFrontPower  / max);
        rightFront.setPower(rightFrontPower / max);
        leftBack.setPower(leftBackPower    / max);
        rightBack.setPower(rightBackPower  / max);
    }

    // helper for curves
    private double processInput(double input, String preset, double deadzone) {
        if (Math.abs(input) < deadzone) return 0.0;
        switch (preset) {
            case "QUADRATIC":
                return input * Math.abs(input);
            case "EXPONENTIAL":
                return Math.pow(input, 3);
            default:
                return input;
        }
    }

    //lerp is special
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
            transfer.setPower(1);
        } else {
            transfer.setPower(0);
        }
    }

    public void Shooter() {
        // buttons for presets:
        //  Y -> mid range
        //  B -> close
        if (gamepad1.y) {
            shooterTargetVel = ShooterConfig.MID_VEL;
        } else if (gamepad1.b) {
            shooterTargetVel = ShooterConfig.CLOSE_VEL;
        } else if (gamepad1.right_trigger > 0.1) {
            shooterTargetVel = 900;
        }
        /*
        if (gamepad1.right_trigger > 0.1) {

            if (shooter.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // update for tweaking

            shooter.setVelocityPIDFCoefficients(
                    ShooterConfig.kP,
                    ShooterConfig.kI,
                    ShooterConfig.kD,
                    ShooterConfig.kF
            );

            shooter.setVelocity(shooterTargetVel);
        } else {
            shooter.setPower(0);
        }
        */
    }


}