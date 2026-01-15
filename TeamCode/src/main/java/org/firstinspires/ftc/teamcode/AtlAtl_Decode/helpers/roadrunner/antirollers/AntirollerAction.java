package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.antirollers;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class AntirollerAction implements Action {
    private DcMotorEx antiroller;
    private final double p;

    public AntirollerAction(DcMotorEx intake, double p) {
        this.antiroller = intake;
        this.p = p;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        antiroller.setPower(p);
        return true;
    }
}
