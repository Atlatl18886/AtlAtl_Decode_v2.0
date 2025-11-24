package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

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

        //Initialize Intakes -----------------------------------------------------
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setDirection(DcMotorEx.Direction.FORWARD);

        //Initialize Transfer -----------------------------------------------------
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorEx.Direction.REVERSE);

        //Initialize Shooter -----------------------------------------------------
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

    }

    @Override
    public void loop() {
        Drive();
        Intake();
        Transfer();
        Shooter();
    }

    public void Drive() {
        double strafe   = -gamepad1.left_stick_x;
        double vertical = gamepad1.left_stick_y;
        double heading  =  -gamepad1.right_stick_x;

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

    public void Intake() {
        //if a, power in 0.9
        //if y, power out 0.9
        //if none clicked, 0

        //optionally you can have it default to intake always on by putting that in the else case
        if (gamepad1.y) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(1.0);
        }
    }

    public void Transfer() {
        if (gamepad1.x) {
            transfer.setPower(-0.3);
        } else {
            transfer.setPower(1.0);
        }
    }

    public void Shooter() {
        shooter.setVelocity(1100);
    }



}
