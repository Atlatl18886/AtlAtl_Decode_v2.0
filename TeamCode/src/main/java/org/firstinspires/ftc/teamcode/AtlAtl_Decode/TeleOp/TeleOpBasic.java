package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Config.ShooterConfig;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Config.TeleOpConfig;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.control.Toggle;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.TelemetryHelper;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.util.LoopProfiler;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain.SlewRateLimiter;

import java.util.List;

@TeleOp(name="V2.5 TeleOp BASIC", group="Main")
public class TeleOpBasic extends OpMode {

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intake, transfer, shooter, antiroller;

    private final Toggle intakeToggle = new Toggle();
    private TelemetryHelper driveTelem, intakeTelem, shooterTelem, debugTelem, loopTelem;
    private final LoopProfiler profiler = new LoopProfiler();
    private List<LynxModule> allHubs;

    private double targetVel = 0;
    private static final double CLOSE = ShooterConfig.CLOSE_TPS;
    private static final double MID = ShooterConfig.MID_TPS;
    private static final double FAR = ShooterConfig.FAR_TPS;
    private static final double DEFAULT = ShooterConfig.DEFAULT_TPS;

    private final double SHOOTER_kP = ShooterConfig.shooter_Kp;
    private final double SHOOTER_kI = ShooterConfig.shooter_Ki;
    private final double SHOOTER_kD = ShooterConfig.shooter_Kd;
    private final double SHOOTER_kF = ShooterConfig.shooter_Kf;

    /*
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.5);
    private final SlewRateLimiter verticalLimiter = new SlewRateLimiter(3.5);
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(3.5);
    private final ElapsedTime loopTimer = new ElapsedTime();
    */


    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "perp");
        rightBack = hardwareMap.get(DcMotorEx.class, "par");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        antiroller = hardwareMap.get(DcMotorEx.class, "antiroller");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        driveTelem = TelemetryHelper.create(telemetry, "Drive--");
        intakeTelem = TelemetryHelper.create(telemetry, "Intake--");
        shooterTelem = TelemetryHelper.create(telemetry, "Shooter--");
        debugTelem = TelemetryHelper.create(telemetry, "Debug--");
        loopTelem = debugTelem.child("--Loop");

        profiler.start();
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) { module.clearBulkCache(); }

        double loopMs = profiler.update();
        loopTelem.clear();
        loopTelem.addf("loop", "%.2f", loopMs);
        loopTelem.addf("avg", "%.2f", profiler.getAvg());

        if (profiler.getCount() % 50 == 0) {
            double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            loopTelem.addf("voltage", "%.2fV", currentVoltage);
        }
        /*
        double loopDt = loopTimer.seconds();
        loopTimer.reset();
         */

        Drive(/*loopDt*/);
        Intake();
        Transfer();
        Shooter();

        driveTelem.push();
        intakeTelem.push();
        shooterTelem.push();
        debugTelem.push();
        telemetry.update();
    }

    private void Drive(/*double dt*/) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        y = applyCurve(y);
        x = applyCurve(x);
        rx = applyCurve(rx);

        /*
        if (TeleOpConfig.USE_SLEW_LIMITING) {
            x = strafeLimiter.calculate(x, dt);
            y = verticalLimiter.calculate(y, dt);
            rx = turnLimiter.calculate(rx, dt);
        } else {
            strafeLimiter.reset();
            verticalLimiter.reset();
            turnLimiter.reset();
        }*/

        y *= TeleOpConfig.speedFactor;
        x *= TeleOpConfig.speedFactor;
        rx *= TeleOpConfig.speedFactor;

        double lf = y + x + rx;
        double lb = y - x + rx;
        double rf = y + x - rx;
        double rb = y - x - rx;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        leftFront.setPower((lf / denominator));
        leftBack.setPower((lb / denominator));
        rightFront.setPower((rf / denominator));
        rightBack.setPower((rb / denominator));

        driveTelem.add("Preset", TeleOpConfig.DRIVE_PRESET);
    }

    public void Intake() {
        boolean prevState = intakeToggle.get();
        boolean intakeActive = intakeToggle.updateTrigger(gamepad1.left_trigger, 0.1);

        if (intakeActive != prevState) {
            intakeTelem.add("", intakeActive ? "Intake toggled ON" : "Intake toggled OFF");
        }

        intake.setPower(intakeActive ? 1.0 : 0.35);
        antiroller.setPower(intakeActive ? 0.5 : 0.5);

        intakeTelem.add("state", intakeActive);
        intakeTelem.addf("power", "%.2f", intake.getPower());
    }

    private void Transfer() {
        double p = (gamepad1.right_trigger > 0.1) ? 1.0 : -0.4;
        transfer.setPower(p);
    }

    private void Shooter() {
        if (gamepad1.right_bumper) targetVel = CLOSE;
        else if (gamepad1.b) targetVel = MID;
        else if (gamepad1.y) targetVel = FAR;
        else targetVel = DEFAULT;

        try {
            shooter.setVelocity(targetVel);
        } catch (Exception ignored) {}

//        double error = Math.abs(shooter.getVelocity() - targetVel);
//        if (error < 60 && targetVel != DEFAULT) {
//            gamepad1.rumble(0.3, 0.3, 50);
//        }

        shooterTelem.addf("Target", "%.0f", targetVel);
        shooterTelem.addf("Actual", "%.0f", shooter.getVelocity());
    }
    private double applyCurve(double input) {
        if (Math.abs(input) < TeleOpConfig.DRIVE_DEADZONE) return 0;

        switch (TeleOpConfig.DRIVE_PRESET) {
            case "TANH":
                double a = TeleOpConfig.TANH_A;
                return Math.tanh(a * input) / Math.tanh(a);

            case "QUADRATIC":
                return input * Math.abs(input);

            case "EXPONENTIAL":
                return Math.pow(input, 3);

            case "LINEAR":
            default:
                return input;
        }
    }

}