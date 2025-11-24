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
        Drive();
        Intake();
        Transfer();
        Shooter();

        // telemetry for debug
        telemetry.addData("Current Heading", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Aim Mode Active (A Button)", gamepad1.a);
    }

    public void Drive() {

        // aim mod
        double turnSpeedMultiplier = 1.0;
        if (gamepad1.a) {
            turnSpeedMultiplier = 0.25; //qaurtered turn speed for aim mode
        }

        double strafe   = -gamepad1.left_stick_x;
        double vertical = gamepad1.left_stick_y;
        double heading  =  -gamepad1.right_stick_x * turnSpeedMultiplier;

        double leftFrontPower  = vertical + strafe + heading;
        double rightFrontPower = vertical + strafe - heading;
        double leftBackPower   = vertical - strafe + heading;
        double rightBackPower  = vertical - strafe - heading;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        ));

        leftFront.setPower(leftFrontPower / max);
        rightFront.setPower(rightFrontPower / max);
        leftBack.setPower(leftBackPower / max);
        rightBack.setPower(rightBackPower / max);
    }

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