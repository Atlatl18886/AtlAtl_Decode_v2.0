package org.firstinspires.ftc.teamcode.AtlAtl_Decode.tuning;

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
@TeleOp(name="Shooter integrated tuning", group="tests")
public class ShooterOpModeTuner extends OpMode {

    private DcMotorEx shooter;

    double tpr = Conversions.TPR_6000_RPM;
    //values before cosmo -- kP:580 kI:8.8 kD:4 kF:47.85
    public static double SHOOTER_kP = 52;
    public static double SHOOTER_kI = 0.9;
    public static double SHOOTER_kD = 3;
    public static double SHOOTER_kF = 21.3;
    //before pranav(p,i,d,f) - 15,1.7,0,15
    //after 12, 0, 0.5, 19.5
    public static double targetRPM = 2800; //max 3k

    private double targetVel = 0;
    private double currentRPM = 0;
    private double currentVel = 0;
    private double error = 0;

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
        error = Math.round(targetRPM - currentRPM);

        shooter.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        currentVel = shooter.getVelocity();
        currentRPM = Conversions.tpsToRpm(currentVel, tpr);
        shooter.setVelocity(Conversions.rpmToTps(targetRPM, tpr));
        telemetry.addData("[0] targetVel ticks", "%.0f", targetVel);
        telemetry.addData("[1] currentVel ticks", "%.0f", Math.abs(currentVel));
        telemetry.addData("[2] targetRPM", "%.0f", targetRPM);
        telemetry.addData("[3] currentRPM", "%.0f", Math.abs(currentRPM));
        telemetry.addData("[4] error: ", "%.0f", Math.abs(error));
    }

}
