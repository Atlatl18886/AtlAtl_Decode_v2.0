package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Auton_6Ball_Diagnostics", group="FTC_Decode")
public class Auton_6Ball_Diagnostics extends LinearOpMode {

    // --- Hardware Declarations ---
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx shooterMotor;

    // --- Configuration Variables (TUNE THESE TIMES ON THE FIELD) ---
    private static final double SHOOTER_POWER = 0.85;
    private static final int SHOOTER_FEED_TIME_MS = 2250;
    private static final double INTAKE_TRANSFER_POWER = 0.6;
    private static final double DRIVE_SPEED = 0.5;

    // TUNE THESE TIME CONSTANTS FOR MOVEMENT:
    private static final int DRIVE_TO_SPIKE_MS = 2000;
    private static final int RETURN_TO_GOAL_MS = 2000;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Attempting Hardware Initialization...");
        telemetry.update();

        // If we reach here, all hardware was mapped successfully.
        telemetry.addData("Status", "Hardware Mapped Successfully.");

        // --- Motor Direction Configuration ---
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all drive motors to RUN_WITHOUT_ENCODER for time-based movement
        setChassisRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setChassisBrakeMode(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Configuration ready. Waiting for Start.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            /*
            setTransfer(-0.8);
            // --- Step 1: Shoot Preloaded 3 Balls ---
            telemetry.addData("Auto Step", "1/4: Shooting initial 3 balls.");
            telemetry.update();
            scoreArtifacts(SHOOTER_FEED_TIME_MS);

            // --- Step 2: Drive to the Spike Marks and collect 3 more balls ---
            telemetry.addData("Auto Step", "2/4: Driving to collection zone.");
            telemetry.update();
            setIntake(INTAKE_TRANSFER_POWER);
            driveForward(DRIVE_SPEED, DRIVE_TO_SPIKE_MS);

            // --- Step 3: Return to Goal Position ---
            telemetry.addData("Auto Step", "3/4: Returning to goal.");
            telemetry.update();
            driveBackward(DRIVE_SPEED, RETURN_TO_GOAL_MS);
            setIntake(0.0);

            // --- Step 4: Shoot the next 3 balls (Total 6) ---
            telemetry.addData("Auto Step", "4/4: Shooting second set of 3 balls.");
            telemetry.update();
            scoreArtifacts(SHOOTER_FEED_TIME_MS);

            telemetry.addData("Status", "6-Ball Auto Complete.");
            telemetry.update();
             */
            //setTransfer(-0.8);
            //shoot(700, 1500, 150);
            //shoot(800, 1500, 150);
            //driveBackward(-1, 200);
            driveForward(1, 250);
        }
    }

    // --- Helper Functions (Same as previous code) ---
    /*
    private void scoreArtifacts(int durationMs) {
        shooterMotor.setPower(SHOOTER_POWER);
        sleep(500);
        intakeMotor.setPower(INTAKE_TRANSFER_POWER);
        transferMotor.setPower(INTAKE_TRANSFER_POWER);
        sleep(durationMs);
        shooterMotor.setPower(0.0);
        intakeMotor.setPower(0.0);
        transferMotor.setPower(0.0);
    }
    */
    private void shoot(double power, int reload, int tolerance) {
        if (shooterMotor.getVelocity() < (power + tolerance) && shooterMotor.getVelocity() > (power -tolerance)) {
            shooterMotor.setVelocity(power);
        }
        sleep(reload);
        setTransfer(1);
        sleep(200);
    }


    private void setIntake(double power) {
        intakeMotor.setPower(power);
    }
    private void setTransfer(double power) {
        transferMotor.setPower(power);
    }
    public void driveForward(double power, int durationMs) {
        setChassisPower(Math.abs(power));
        sleep(durationMs);
        setChassisPower(0.0);
    }

    public void driveBackward(double power, int durationMs) {
        setChassisPower(-Math.abs(power));
        sleep(durationMs);
        setChassisPower(0.0);
    }

    private void setChassisPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void setChassisRunMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    private void setChassisBrakeMode(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }
}