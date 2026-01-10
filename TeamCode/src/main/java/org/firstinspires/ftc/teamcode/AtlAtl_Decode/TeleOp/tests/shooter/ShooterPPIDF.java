package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.tests.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="Shooter Tuning Manual", group="tests")
@Disabled
public class ShooterPPIDF extends OpMode {
    private DcMotorEx shooter;
    private Telemetry dashboardTelemetry;
    private ElapsedTime timer = new ElapsedTime();

    public static double shooter_Kf = 0.00093;
    public static double shooter_Kp = 0.00055;
    public static double shooter_Ki = 0.00004;
    public static double shooter_Kd = 0.000016;

    public static double velocityFilterGain = 0.85;
    public static double targetVelocity = 600;
    public static int deadband = 40;

    // Only use Ki when error is within this range to prevent "hunting" from far away
    public static double Ki_Zone = 150;

    private double filteredVelocity = 0;
    private double lastError = 0;
    private double lastPower = 0;
    private double integralSum = 0;
    private boolean isLocked = false;

    @Override
    public void init() {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() { timer.reset(); }

    @Override
    public void loop() {
        double rawVelocity = shooter.getVelocity();
        filteredVelocity = (velocityFilterGain * filteredVelocity) + ((1 - velocityFilterGain) * rawVelocity);

        double power = controller(targetVelocity, filteredVelocity);
        shooter.setPower(Range.clip(power, 0, 1));

        dashboardTelemetry.addData("0_Target", targetVelocity);
        dashboardTelemetry.addData("1_Filtered", filteredVelocity);
        dashboardTelemetry.addData("2_Raw", rawVelocity);
        dashboardTelemetry.addData("3_Locked", isLocked);
        dashboardTelemetry.addData("4_Power", power);
        dashboardTelemetry.update();
    }

    public double controller(double target, double current) {
        double error = target - current;
        double dt = timer.seconds();
        timer.reset();

        if (dt == 0 || target <= 0) {
            integralSum = 0;
            isLocked = false;
            return 0;
        }

        if (Math.abs(error) < deadband) {
            isLocked = true;
            return lastPower;
        }
        isLocked = false;

        if (Math.abs(error) < Ki_Zone) {
            integralSum += error * dt;
        } else {
            integralSum = 0; // Reset if we are far away (like during windup)
        }

        // Anti-windup
        integralSum = Range.clip(integralSum, -0.15/shooter_Ki, 0.15/shooter_Ki);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double f = shooter_Kf * target;
        double p = shooter_Kp * error;
        double i = shooter_Ki * integralSum;
        double d = shooter_Kd * derivative;

        lastPower = f + p + i + d;
        return lastPower;
    }
}