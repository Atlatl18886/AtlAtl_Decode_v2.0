package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.transfer;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {
    private final DcMotorEx transfer;

    public static double idle = -0.4;
    public static double active = 1.0;

    public Transfer(@NonNull HardwareMap hardwareMap) {
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
    public Action pulse(double seconds) {
        final double[] startTime = { -1 };

        return packet -> {
            if (startTime[0] < 0) {
                startTime[0] = System.currentTimeMillis() / 1000.0;
            }

            double elapsed = (System.currentTimeMillis() / 1000.0) - startTime[0];

            if (elapsed < seconds) {
                transfer.setPower(active);
                return true;
            } else {
                transfer.setPower(0);
                return false;
            }
        };
    }


}