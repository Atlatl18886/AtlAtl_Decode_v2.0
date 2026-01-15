package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.antirollers;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Antiroller {
    public DcMotorEx antiroller;

    public Antiroller(HardwareMap hardwareMap) {

        antiroller = hardwareMap.get(DcMotorEx.class, "antiroller");
        antiroller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        antiroller.setDirection(DcMotorSimple.Direction.REVERSE);
        antiroller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public class AntirollerIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            antiroller.setPower(0.7);
            return false;
        }
    }
    public Action in() {
        return new AntirollerIn();
    }

    public class AntirollerOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            antiroller.setPower(-0.7);
            return false;
        }
    }
    public Action out() {
        return new AntirollerOut();
    }

    public class AntiIdle implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            antiroller.setPower(0);
            return false;
        }
    }
    public Action idle() {
        return new AntiIdle();
    }



    public void antirollerOn() {
        antiroller.setPower(0.7);
    }
    public void antirollerOff() {
        antiroller.setPower(0.0);
    }
    public void antirollerReverse() {
        antiroller.setPower(-0.7);
    }
}
