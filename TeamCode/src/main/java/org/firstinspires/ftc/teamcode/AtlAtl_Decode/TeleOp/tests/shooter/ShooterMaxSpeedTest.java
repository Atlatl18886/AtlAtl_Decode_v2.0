package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.tests.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.util.Conversions;

@Config
@TeleOp(name="Shooter MAX SPEED", group="tests")
public class ShooterMaxSpeedTest extends OpMode {

    private DcMotorEx shooter;

    double tpr = Conversions.TPR_6000_RPM;

    public static double targetPower = 1.0; //max 2k
    private double currentRPM = 0;
    private double currentVel = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void start() {
        telemetry.clear();
    }
    @Override
    public void loop() {

        currentVel = shooter.getVelocity();
        currentRPM = Conversions.tpsToRpm(currentVel, tpr);
        shooter.setPower(targetPower);
        telemetry.addData("[0] currentVel ticks", "%.0f", Math.abs(currentVel));
        telemetry.addData("[1] targetRPM", "%.0f", targetPower);
        telemetry.addData("[2] currentRPM", "%.0f", Math.abs(currentRPM));
    }

}
