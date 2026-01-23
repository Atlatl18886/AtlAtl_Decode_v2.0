package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.transfer;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {
    private final DcMotorEx transfer;

    public static double idle = -0.4;
    public static double active = 1.0;

    public Transfer(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Action idle() {
        return packet -> {
            transfer.setPower(idle);
            return false;
        };
    }

    public Action out() {
        return packet -> {
            transfer.setPower(active);
            return false;
        };
    }

    public Action stop() {
        return packet -> {
            transfer.setPower(0);
            return false;
        };
    }

}