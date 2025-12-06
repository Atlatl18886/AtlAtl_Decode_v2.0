package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Red 6Ball - Bargraph", group="FTC_Decode")
public class Red_6Ball2 extends LinearOpMode {

    // --- Hardware Declarations ---
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor intakeMotor = null;
    private DcMotor transferMotor = null;
    private DcMotor shooterMotor = null;

    // --- Configuration Variables (TUNE THESE TIMES ON THE FIELD) ---
    private static final double SHOOTER_POWER = 0.6;

    private static final int SHOOTER_FEED_TIME_MS = 2500;
    private static final double INTAKE_TRANSFER_POWER = 0.9;
    private static final double DRIVE_SPEED = 0.5;

    // TUNE THESE TIME CONSTANTS FOR MOVEMENT:
    private static final int DRIVE_TO_SPIKE_MS = 2000;
    private static final int RETURN_TO_GOAL_MS = 2000;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Attempting Hardware Initialization...");
        telemetry.update();

        // --- Hardware Initialization with Error Handling ---
        try {
            leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
            leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
        } catch (Exception e) {
            // This catches the specific error you are seeing
            telemetry.addData("ERROR:", "Hardware device not found!");
            telemetry.addData("Error Detail:", e.getMessage());
            telemetry.addData("FIX:", "Check your Driver Station Configuration names VERY carefully.");
            telemetry.addData("Expected Names:", "leftFront, rightFront, etc. MUST MATCH EXACTLY.");
            telemetry.update();
            // Stop the OpMode if configuration is wrong
            waitForStart();
            return; // Exit the runOpMode method early
        }

        // If we reach here, all hardware was mapped successfully.
        telemetry.addData("Status", "Hardware Mapped Successfully.");

        // --- Motor Direction Configuration ---
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all drive motors to RUN_WITHOUT_ENCODER for time-based movement
        setChassisRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setChassisBrakeMode(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Configuration ready. Waiting for Start.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {

            // --- Step 1: Shoot Preloaded 3 Balls ---
            telemetry.addData("Auto Step", "1/4: Shooting initial 3 balls.");
            telemetry.update();
            driveBackward(DRIVE_SPEED, 1000);
            sleep(800);
            scoreArtifacts(SHOOTER_FEED_TIME_MS);
            sleep(1000);
            turn(-0.2);
            driveBackward(DRIVE_SPEED, 2000);
            turn(-0.3);
            sleep(400);
            intakeMotor.setPower(1);
            transferMotor.setPower(1);
            driveForward(DRIVE_SPEED, 2000);
            driveBackward(DRIVE_SPEED, 2500);
            turn(0.4);
            driveForward(DRIVE_SPEED, 2000);
            scoreArtifacts(SHOOTER_FEED_TIME_MS);
            sleep(1000);
            turn(-0.2);
            driveBackward(DRIVE_SPEED, 3000);
            turn(-0.4);
            driveForward(DRIVE_SPEED, 2000);
            driveBackward(DRIVE_SPEED, 2000);
            turn(0.4);
            driveForward(DRIVE_SPEED, 2000);
            scoreArtifacts(SHOOTER_FEED_TIME_MS);
            sleep(2000);
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
        }
    }

    // --- Helper Functions (Same as previous code) ---
    private void scoreArtifacts(int durationMs) {
        shooterMotor.setPower(0.8);
        sleep(1000);
        intakeMotor.setPower(1);
        transferMotor.setPower(1);
        sleep(1000);
        shooterMotor.setPower(0.6);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        sleep(1000);
        intakeMotor.setPower(1);
        transferMotor.setPower(1);
        sleep(1000);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        shooterMotor.setPower(0.7);
        sleep(1000);
        intakeMotor.setPower(1);
        transferMotor.setPower(1);
        sleep(1000);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }

    private void setIntakeTransferPower(double power) {
        intakeMotor.setPower(power);
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

    private void turn(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);
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
