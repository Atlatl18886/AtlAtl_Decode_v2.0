package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.shooter;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Config.ShooterConfig;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.util.Conversions;

public class Shooter {
    private final DcMotorEx shooter;

    public static double TICKS_PER_REV = 28.0;
    public static double MAX_RPM = 6000.0;
    public static double VELOCITY_TOLERANCE = 50.0;

    public static double kP = ShooterConfig.shooter_Kp;
    public static double kI = ShooterConfig.shooter_Ki;
    public static double kD = ShooterConfig.shooter_Kd;
    public static double kF = ShooterConfig.shooter_Kf;

    public Shooter(@NonNull HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public Action spinUp(double targetRPM) {
        return packet -> {
//            shooter.setVelocityPIDFCoefficients(kP,kI,kD,kF);
            shooter.setVelocity(Conversions.rpmToTps(targetRPM, TICKS_PER_REV));
            return false;
        };
    }

    public Action stop() {
        return packet -> {
            shooter.setVelocity(0);
            return false;
        };
    }

    public Action waitForRPM(double targetRPM) {
        return packet -> {
            double currentVel = shooter.getVelocity();
            double currentRPM = Conversions.tpsToRpm(currentVel, TICKS_PER_REV);

            packet.put("TargetVel", targetRPM);
            packet.put("CurrentVel", currentRPM);

            return Math.abs(targetRPM - currentRPM) > VELOCITY_TOLERANCE;
        };
    }
}