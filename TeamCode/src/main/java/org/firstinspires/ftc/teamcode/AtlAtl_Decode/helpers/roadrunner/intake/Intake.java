package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotorEx intake;
    private final DcMotorEx antiroller;

    public static double idle = 0;
    public static double active = 1.0;
    public static double reverse = -1.0;
    public static double idleA = 0.4;
    public static double activeA = 0.7;
    public static double reverseA = -0.6;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        antiroller = hardwareMap.get(DcMotorEx.class, "antiroller");
        antiroller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Action idle() {
        return packet -> {
            intake.setPower(idle);
            antiroller.setPower(idle);
            return false;
        };
    }

    public Action setIntakePower(double p) {
        return telemetryPacket -> {
            intake.setPower(p);
            return false;
        };
    }
    public Action setAntiPower(double p) {
        return telemetryPacket -> {
            antiroller.setPower(p);
            return false;
        };
    }

    public Action in() {
        return packet -> {
            intake.setPower(active);
            antiroller.setPower(active);
            return false;
        };
    }
    public Action out() {
        return telemetryPacket -> {
            intake.setPower(reverse);
            antiroller.setPower(reverse);
            return false;
        };
    }

    public Action stop() {
        return packet -> {
            intake.setPower(0);
            antiroller.setPower(0);
            return false;
        };
    }

}