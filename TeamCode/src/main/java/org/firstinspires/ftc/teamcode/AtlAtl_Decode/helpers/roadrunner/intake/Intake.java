package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public class IntakeAction implements Action {
        double power;
        public IntakeAction(double p) { this.power = p; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(power);
            return false;
        }
    }

    public Action setPower(double p) {
        return new IntakeAction(p);
    }
}