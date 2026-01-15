package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotorEx intake;

    public Intake(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    //in intake
    public class IntakeIn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(1.0);
            return false;
        }
    }
    public Action in() {
        return new IntakeIn();
    }

    //outtake
    public class IntakeOut implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(-1.0);
            return false;
        }
    }
    public Action out() {
        return new IntakeOut();
    }

    //idle
    public class IntakeIdle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(0.0);
            return false;
        }
    }
    public Action idle() {
        return new IntakeIdle();
    }





    public void intakeOn() {
        intake.setPower(1);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void intakeReverse() {
        intake.setPower(-1);
    }

}
