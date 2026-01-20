package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous.templates;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//TODO: FINISH TEMP

public abstract class AutonomousBase extends LinearOpMode {

    protected DcMotorEx leftFront, rightFront, leftBack, rightBack;
    protected DcMotorEx intake, antiroller, transfer, shooter;
    protected IMU imu;

    //config
    protected static double TICKS_PER_REV = 384.5;
    protected static double WHEEL_DIAMETER_IN = 104.0 / 25.4;
    protected static double TICKS_PER_INCH = (TICKS_PER_REV / (WHEEL_DIAMETER_IN * Math.PI)) * 1.1;
    protected double turn_kP = 0.018;
    protected double turn_kD = 0.0012;
    protected double drive_correction_kP = 0.025;
    //deadbands
    protected final double TURN_DEADBAND = 3.0;//deg
    protected final double DRIVE_CORRECTION_DEADBAND = 0.5;//deg
    protected final double MIN_TURN_POWER = 0.1;//min power to move

    public void initDt() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        imu.resetYaw();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        antiroller = hardwareMap.get(DcMotorEx.class, "antiroller");
    }

    ///drive with imu correction
    public void driveFor(double inches, double power, double timeout) {
        resetEncoders();
        double targetAngle = getHeading();
        int ticks = (int)(inches * TICKS_PER_INCH);

        setTargets(ticks, ticks, ticks, ticks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && isBusy() && timer.seconds() < timeout) {
            double error = AngleUnit.normalizeDegrees(targetAngle - getHeading());
            double correction = 0;

            if (Math.abs(error) > DRIVE_CORRECTION_DEADBAND) {
                correction = error * drive_correction_kP;
            }

            leftFront.setPower(power - correction);
            leftBack.setPower(power - correction);
            rightFront.setPower(power + correction);
            rightBack.setPower(power + correction);
        }
        stopDrive();
    }

    /// strafe with imu correct
    public void strafeFor(double inches, double power, double timeout) {
        resetEncoders();
        double targetAngle = getHeading();
        int ticks = (int)(inches * TICKS_PER_INCH);

        setTargets(ticks, -ticks, -ticks, ticks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && isBusy() && timer.seconds() < timeout) {
            double error = AngleUnit.normalizeDegrees(targetAngle - getHeading());
            double correction = 0;

            if (Math.abs(error) > DRIVE_CORRECTION_DEADBAND) {
                correction = error * drive_correction_kP;
            }

            leftFront.setPower(power - correction);
            leftBack.setPower(power - correction);
            rightFront.setPower(power + correction);
            rightBack.setPower(power + correction);
        }
        stopDrive();
    }

    public void turnToHeading(double targetHeading, double timeout) {
        ElapsedTime timer = new ElapsedTime();
        double prevError = 0;
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && timer.seconds() < timeout) {
            double error = AngleUnit.normalizeDegrees(targetHeading - getHeading());

            //exit condition
            if (Math.abs(error) <= TURN_DEADBAND) {
                break;
            }

            double derivative = error - prevError;
            double turnPower = (error * turn_kP) + (derivative * turn_kD);

            if (Math.abs(turnPower) < MIN_TURN_POWER) {
                turnPower = Math.copySign(MIN_TURN_POWER, turnPower);
            }

            turnPower = Range.clip(turnPower, -0.8, 0.8);

            leftFront.setPower(turnPower);
            leftBack.setPower(turnPower);
            rightFront.setPower(-turnPower);
            rightBack.setPower(-turnPower);

            prevError = error;
        }
        stopDrive();
    }

    ///util
    protected double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    protected void resetEncoders() {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(25);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void setTargets(int lf, int rf, int lb, int rb) {
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + lf);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + rf);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + lb);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + rb);
    }
    private void setDriveMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode); rightFront.setMode(mode);
        leftBack.setMode(mode); rightBack.setMode(mode);
    }
    private void stopDrive() {
        leftFront.setPower(0); rightFront.setPower(0);
        leftBack.setPower(0); rightBack.setPower(0);
    }
    private boolean isBusy() {
        return leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy();
    }
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior b) {
        leftFront.setZeroPowerBehavior(b); rightFront.setZeroPowerBehavior(b);
        leftBack.setZeroPowerBehavior(b); rightBack.setZeroPowerBehavior(b);
    }
}