package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.TeleOpConfig;

@Autonomous(name="Blue Curry", group="V2")
public class Blue_Curry extends LinearOpMode {
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intake, transfer, shooter;
    private IMU imu;

    private static final double TICKS_PER_REV = 537.6; //gobilda 5203s 13.7:1
    private static final double WHEEL_DIAMETER_IN = 104.0 / 25.4;
    private static final double WHEEL_CIRC = WHEEL_DIAMETER_IN * Math.PI;

    //calibration factor to correct for mecanum wheel slippage.
    private static final double CALIBRATION_FACTOR = 1.1;
    private static final double TICKS_PER_INCH = (TICKS_PER_REV / WHEEL_CIRC) * CALIBRATION_FACTOR;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        resetEncoders();

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setPower(0);
        transfer.setPower(0);
        shooter.setPower(0);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        //TODO: PATHING
        transfer.setPower(-0.8);
        intake.setPower(1.0);
        drive(4, 3, 8);
        strafe(-2, 3, 4);
        transfer.setPower(0);
        intake.setPower(0);
        turnToHeading(25, 4);
        shooter.setPower(1);
        sleep(5000);
        intake.setPower(1);
        transfer.setPower(1);
        sleep(150);
        transfer.setPower(-1);
        sleep(5000);
        transfer.setPower(1);
        sleep(150);
        transfer.setPower(-1);
        sleep(300);
        intake.setPower(0);
        transfer.setPower(0);
        shooter.setPower(0);
    }

    //helper methods
    private int inchesToTicks(double inches) {
        return (int)(inches * TICKS_PER_INCH);
    }
    private double revsPerSecToVelocity(double revsPerSec) {
        return revsPerSec * TICKS_PER_REV;
    }
    public void drive(double inches, double revsPerSec, double timeoutSec) {
        resetEncoders();
        int ticks = inchesToTicks(inches);
        double velocity = revsPerSecToVelocity(revsPerSec);

        leftFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setAll(velocity);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && isBusy() && timer.seconds() < timeoutSec) {
            telemetry.addData("Path", "Driving " + inches + " inches");
            telemetry.addData("Target", ticks);
            telemetry.addData("Current", leftFront.getCurrentPosition());
            telemetry.update();
        }
        stopAll();
    }

    public void strafe(double inches, double revsPerSec, double timeoutSec) {
        resetEncoders();
        int ticks = inchesToTicks(inches);
        double velocity = revsPerSecToVelocity(revsPerSec);
        leftFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(ticks);
        rightFront.setTargetPosition(-ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setAll(velocity);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && isBusy() && timer.seconds() < timeoutSec) {
            telemetry.addData("Path", "Strafing " + inches + " inches");
            telemetry.update();
        }
        stopAll();
    }

    public void turnToHeading(double targetHeading, double timeoutSec) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime pdTimer = new ElapsedTime();
        double prevError = 0;
        double kP = TeleOpConfig.imu_kP;
        double kD = TeleOpConfig.imu_kD;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetHeading - currentHeading;
            //limit error
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;
            // Success condition
            if (Math.abs(error) < 2.0) {
                break;
            }
            double dt = pdTimer.seconds();
            pdTimer.reset();
            if (dt < 1e-3) dt = 1e-3;

            double derivative = (error - prevError) / dt;
            prevError = error;

            double turnPower = (error * kP) + (derivative * kD);

            //power clamping
            if (turnPower > 0.6) turnPower = 0.6;
            if (turnPower < -0.6) turnPower = -0.6;
            if (Math.abs(turnPower) < 0.10) turnPower = Math.copySign(0.05, turnPower);

            leftFront.setPower(turnPower);
            leftBack.setPower(turnPower);
            rightFront.setPower(-turnPower);
            rightBack.setPower(-turnPower);
            //debug
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Current", currentHeading);
            telemetry.addData("Error", error);
            telemetry.addData("Power", turnPower);
            telemetry.update();
        }
        stopAll();
    }

    // utility functions
    private void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void setAll(double velocity) {
        leftFront.setVelocity(velocity);
        leftBack.setVelocity(velocity);
        rightFront.setVelocity(velocity);
        rightBack.setVelocity(velocity);
    }
    private void stopAll() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    private boolean isBusy() {
        return leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy();
    }
}