package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.util.Conversions;

@Config
@TeleOp(name="", group="")
@Disabled
public class template extends OpMode {

    private DcMotorEx shooter;

    double tpr = Conversions.TPR_6000_RPM;
    public static double SHOOTER_kP = 100;
    public static double SHOOTER_kI = 0;
    public static double SHOOTER_kD = 0;
    public static double SHOOTER_kF = 48;
    public static double targetRPM = 1000; //max 2k

    private double targetVel = 0;
    private double currentRPM = 0;
    private double currentVel = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        telemetry.addData("Status", "Initialized");

        targetVel = Conversions.rpmToTps(targetRPM, tpr);
    }
    @Override
    public void start() {
        telemetry.clear();
    }
    @Override
    public void loop() {
        targetVel = Conversions.rpmToTps(targetRPM, tpr);

        shooter.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        currentVel = shooter.getVelocity();
        currentRPM = Conversions.tpsToRpm(currentVel, tpr);
        shooter.setVelocity(Conversions.rpmToTps(targetRPM, tpr));
        telemetry.addData("[0] targetVel ticks", "%.0f", targetVel);
        telemetry.addData("[1] currentVel ticks", "%.0f", Math.abs(currentVel));
        telemetry.addData("[2] targetRPM", "%.0f", targetRPM);
        telemetry.addData("[3] currentRPM", "%.0f", Math.abs(currentRPM));
    }

}
