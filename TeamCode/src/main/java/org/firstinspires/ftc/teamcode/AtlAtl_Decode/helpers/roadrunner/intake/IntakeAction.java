package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeAction implements Action {
    private DcMotorEx intake;
    private final double p;

    public IntakeAction(DcMotorEx intake, double p) {
        this.intake = intake;
        this.p = p;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        intake.setPower(p);
        return true;
    }
}
