package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name="hi")
public class blue6basic extends LinearOpMode {
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intake, transfer, shooter, antiroller;

    public static double SHOOTER_P = 50;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_D = 0;
    public static double SHOOTER_F = 12;
    public static double TARGET_VELOCITY = 2000;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "perp");
        rightBack = hardwareMap.get(DcMotorEx.class, "par");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        antiroller = hardwareMap.get(DcMotorEx.class, "antiroller");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        waitForStart();

        Drive(0.5);
        sleep(1000);
        Drive(0);

    }

    public void Drive(double p) {
        leftFront.setPower(p);
        rightFront.setPower(p);
        leftBack.setPower(p);
        rightBack.setPower(p);
    }

    public void Turn(double p) {
        leftFront.setPower(-p);
        rightFront.setPower(p);
        leftBack.setPower(-p);
        rightBack.setPower(p);
    }

    public void Intake(double p) {
        intake.setPower(p);
    }

    public void Transfer(double p) {
        transfer.setPower(p);
    }

    public void Antis(double p) {
        antiroller.setPower(p);
    }

    public void Shoot(double velocity) {
        shooter.setVelocity(velocity);
    }
}