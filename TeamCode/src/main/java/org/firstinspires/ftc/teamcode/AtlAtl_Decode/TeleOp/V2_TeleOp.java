package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Controls:
Left Joystick - forward, backward, and strafe
Right Joystick - turning
Dpad Up - Save current heading
Y - Shooter Close
X - turn to saved heading
B - Shooter Far
A - Reverse intake (outtake)
Left Trigger - stop intake
Left Bumper - toggle intake
Right Trigger - Transfer
Right bumper - aim mode for turning
*/

@TeleOp
public class V2_TeleOp extends OpMode {
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intake, transfer, shooter;
    public IMU imu;

    //mem for lerp
    private double prevStrafe = 0;
    private double prevVertical = 0;
    private double prevHeading = 0;
    //mem for turntogoal
    private double savedHeading = 0;

    private double targetVel = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double prevError = 0;
    private final ElapsedTime pdTimer = new ElapsedTime();

    @Override
    public void init() {

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

        //imu init
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();

        loopTimer.reset();
        pdTimer.reset();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void loop() {
        double loopDt = loopTimer.seconds();
        loopTimer.reset();

        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        Intake();
        Shooter();
        Transfer();

        //imu turntogoal logic
        if (gamepad1.dpad_up) {
            savedHeading = currentHeading;
        }
        if (gamepad1.x) {
            double error = savedHeading - currentHeading;
            //so that it doesnt turn over 180 degress(the long way around)
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;

            double pdDt = pdTimer.seconds();
            pdTimer.reset();
            if (pdDt < 1e-3) pdDt = 1e-3;

            double derivative = (error - prevError) / pdDt;
            prevError = error;

            double kP = TeleOpConfig.imu_kP;
            double kD = TeleOpConfig.imu_kD;
            double turnPower = (error * kP) + (derivative * kD);

            turnPower *= TeleOpConfig.imu_turn_factor;
            if (turnPower > 0.5) turnPower = 0.5;
            if (turnPower < -0.5) turnPower = -0.5;

            if (Math.abs(error) > 2.0) {
                leftFront.setPower(turnPower);
                leftBack.setPower(turnPower);
                rightFront.setPower(-turnPower);
                rightBack.setPower(-turnPower);
            } else {
                leftFront.setPower(0); rightFront.setPower(0);
                leftBack.setPower(0); rightBack.setPower(0);
            }

            telemetry.addData("IMU Status", "ALIGNING");
            telemetry.addData("Target", savedHeading);
            telemetry.addData("Err", error);

        } else {
            Drive(TeleOpConfig.DRIVE_PRESET, TeleOpConfig.DRIVE_DEADZONE, loopDt);
        }

        telemetry.addData("Heading", "%.1f", currentHeading);
        telemetry.update();
    }
    public void Drive(String PRESET, double deadzone, double dt) {
        // apply deadzone remap before any other processing
        double rStrafe = deadbandRemap(-gamepad1.left_stick_x, deadzone);
        double rVertical = deadbandRemap(gamepad1.left_stick_y, deadzone);
        double rHeading = deadbandRemap(-gamepad1.right_stick_x, deadzone);

        double strafe, vertical, heading;

        if (PRESET.equals("LERP")) {
            double lerpSpeed = TeleOpConfig.LERP_SPEED;
            strafe = lerp(prevStrafe, rStrafe, lerpSpeed, dt);
            vertical = lerp(prevVertical, rVertical, lerpSpeed, dt);
            heading = lerp(prevHeading, rHeading, lerpSpeed, dt);

            prevStrafe = strafe;
            prevVertical = vertical;
            prevHeading = heading;
        } else {
            strafe = processInput(rStrafe, PRESET);
            vertical = processInput(rVertical, PRESET);
            heading = processInput(rHeading, PRESET);
        }

        if (gamepad1.right_bumper) {
            heading *= TeleOpConfig.AIM_TURN_SCALE;
            vertical *= TeleOpConfig.AIM_TURN_SCALE;
        }

        vertical *= TeleOpConfig.speedFactor;
        heading *= TeleOpConfig.speedFactor;
        strafe *= TeleOpConfig.speedFactor;

        double leftFrontPower = vertical + strafe + heading;
        double rightFrontPower = vertical + strafe - heading;
        double leftBackPower = vertical - strafe + heading;
        double rightBackPower = vertical - strafe - heading;
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        ));
        leftFront.setPower(leftFrontPower / max);
        rightFront.setPower(rightFrontPower / max);
        leftBack.setPower(leftBackPower / max);
        rightBack.setPower(rightBackPower / max);
    }
    private double processInput(double input, String preset) {
        switch (preset) {
            case "QUADRATIC":
                return input * Math.abs(input);
            default:
                return input;
        }
    }
    private double lerp(double current, double target, double speed, double dt) {
        double alpha = speed * dt;
        if (alpha > 1.0) alpha = 1.0;
        return current + (target - current) * alpha;
    }
    private double deadbandRemap(double input, double deadzone) {
        if (Math.abs(input) < deadzone) return 0.0;
        //remap remaining range
        return Math.signum(input) * (Math.abs(input) - deadzone) / (1.0 - deadzone);
    }

    //NON DT FUNCTIONALITIES
    private boolean intakePrev = false;
    private boolean intakeOff = false;
    /*
    public enum intakeState {
            FORWARD,
            REVERSE,
            OFF;
    }*/
    public void Intake() {
        boolean bumperPressed = gamepad2.left_bumper;

        if (bumperPressed && !intakePrev) intakeOff = !intakeOff;

        double power = intakeOff ? 1.0 : 0.0;
        intake.setPower(power);
        intakePrev = bumperPressed;
    }

    private boolean transferStopped = false;
    private boolean transferPrev = false;
    public void Transfer() {
        if ((gamepad2.right_trigger>0.1) && !transferPrev) transferStopped = !transferStopped;
        transferPrev = gamepad2.right_trigger >0.1;

        double pow2 = (gamepad2.left_trigger > 0.1) ? 1.0 : (transferStopped ? 0 : -0.75);
        transfer.setPower(pow2);
    }
    public void Shooter() {
        if (gamepad2.right_bumper) {
            shooter.setPower(0.5);
        } else if (gamepad2.b) {
            shooter.setPower(0.7);
        } else if (gamepad2.y) {
            shooter.setPower(1.0);
        } else {
            shooter.setPower(0.3);
        }

        telemetry.addData("Shooter Vel", "%.0f", Math.abs(shooter.getVelocity()));
        telemetry.addData("Target Vel", "%.0f", targetVel);
    }
}