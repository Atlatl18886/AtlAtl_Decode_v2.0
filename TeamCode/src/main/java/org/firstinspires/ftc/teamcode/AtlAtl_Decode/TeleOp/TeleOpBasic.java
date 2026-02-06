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
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.control.ButtonHoldAction;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.TelemetryHelper;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.util.Conversions;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.util.LoopProfiler;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain.PrioritySuppression;


import java.util.List;

@TeleOp(name="V2.5 TeleOp ROBO CENTRIC", group="Main")
public class TeleOpBasic extends OpMode {

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intake, transfer, shooter, antiroller;

    private final Toggle intakeToggle = new Toggle();
    private final ButtonHoldAction aimHold = new ButtonHoldAction();
    private TelemetryHelper driveTelem, intakeTelem, shooterTelem, debugTelem, loopTelem;
    private final LoopProfiler profiler = new LoopProfiler();
    private List<LynxModule> allHubs;

    private double targetVel = 0;
    private static final double DEFAULT = ShooterConfig.DEFAULT;

    private final double SHOOTER_kP = ShooterConfig.shooter_Kp;
    private final double SHOOTER_kI = ShooterConfig.shooter_Ki;
    private final double SHOOTER_kD = ShooterConfig.shooter_Kd;
    private final double SHOOTER_kF = ShooterConfig.shooter_Kf;

    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(6.5, 9.0);
    private final SlewRateLimiter verticalLimiter = new SlewRateLimiter(6.5, 9.0);
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(8.0, 10.0);
    private PrioritySuppression suppressionHelper = new PrioritySuppression();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double loopDt;

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
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        antiroller = hardwareMap.get(DcMotorEx.class, "antiroller");
        antiroller.setDirection(DcMotorSimple.Direction.REVERSE);
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
    }

    @Override
    public void start() {
        loopTimer.reset();
        profiler.start();
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) { module.clearBulkCache(); }

        double rawDt = loopTimer.seconds();
        loopTimer.reset();
        loopDt = Math.min(Math.max(rawDt, 0.0), 0.12);

        double loopMs = profiler.update();
        loopTelem.clear();
        loopTelem.addf("loop", "%.2f", loopMs);
        loopTelem.addf("avg", "%.2f", profiler.getAvg());

        if (profiler.getCount() % 50 == 0) {
            double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            loopTelem.addf("voltage", "%.2fV", currentVoltage);
        }

        Drive(loopDt);
        Intake();
        Transfer();
        Shooter();

        driveTelem.push();
        intakeTelem.push();
        shooterTelem.push();
        debugTelem.push();
        telemetry.update();
    }

    private void Drive(double dt) {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        y= deadbandRemap(y, TeleOpConfig.DRIVE_DEADZONE);
        x= deadbandRemap(x, TeleOpConfig.DRIVE_DEADZONE);
        rx = deadbandRemap(rx, TeleOpConfig.DRIVE_DEADZONE);

        y = applyCurve(y, TeleOpConfig.DRIVE_PRESET);
        x = applyCurve(x, TeleOpConfig.DRIVE_PRESET);
        rx = applyCurve(rx, TeleOpConfig.DRIVE_PRESET);

//        y = applyCurve(y, TeleOpConfig.FWD_PRESET);
//        x = applyCurve(x, TeleOpConfig.LAT_PRESET);
//        rx = applyCurve(rx, TeleOpConfig.TURN_PRESET);

        if (TeleOpConfig.USE_PRIORITY_SUPPRESSION) {
            double[] suppressed = suppressionHelper.apply(x, y, rx, false);
            x = suppressed[0];
            y = suppressed[1];
            rx = suppressed[2];
        }

        if (TeleOpConfig.USE_SLEW_LIMITING) {
            x = strafeLimiter.calculate(x, dt);
            y = verticalLimiter.calculate(y, dt);
            rx = turnLimiter.calculate(rx, dt);
        } else {
            strafeLimiter.reset();
            verticalLimiter.reset();
            turnLimiter.reset();
        }

        aimHold.update(gamepad1.left_bumper);
        if (aimHold.isHeld()) {
            y *= TeleOpConfig.AIM_TURN_SCALE+0.1;
            x *= TeleOpConfig.AIM_TURN_SCALE+0.1;
            rx *= TeleOpConfig.AIM_TURN_SCALE;
        }

        double lf = y + x + rx;
        double lb = y - x + rx;
        double rf = y - x - rx;
        double rb = y + x - rx;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        leftFront.setPower((lf / denominator));
        leftBack.setPower((lb / denominator));
        rightFront.setPower((rf / denominator));
        rightBack.setPower((rb / denominator));

        driveTelem.add("Preset", TeleOpConfig.DRIVE_PRESET);
        driveTelem.add("rate limiting?", TeleOpConfig.USE_SLEW_LIMITING);
        driveTelem.add("priority suppression?", TeleOpConfig.USE_PRIORITY_SUPPRESSION);
    }

    public void Intake() {
        boolean prevState = intakeToggle.get();
        boolean intakeActive = intakeToggle.updateTrigger(gamepad1.left_trigger, 0.1);

        if (intakeActive != prevState) {
            intakeTelem.add("", intakeActive ? "Intake toggled ON" : "Intake toggled OFF");
        }

        intake.setPower(intakeActive ? 1.0 : 0.35);
        antiroller.setPower(intakeActive ? 0.6 : 0.5);

        intakeTelem.add("state", intakeActive);
        intakeTelem.addf("power", "%.2f", intake.getPower());
    }
    private void Transfer() {
        double p = (gamepad1.right_trigger > 0.1) ? 1.0 : -0.4;
        transfer.setPower(p);
    }

    private void Shooter() {
        if (gamepad1.right_bumper) targetVel = ShooterConfig.getCloseTps();
        else if (gamepad1.b) targetVel = ShooterConfig.getMidTps();
        else if (gamepad1.y) targetVel = ShooterConfig.getFarTps();
        else targetVel = DEFAULT;

        try {
            shooter.setVelocity(targetVel);
        } catch (Exception ignored) {}

        double error = Math.abs(Conversions.tpsToRpm(shooter.getVelocity(), Conversions.TPR_6000_RPM) - Conversions.tpsToRpm(targetVel, Conversions.TPR_6000_RPM));
        if (error < 100 && targetVel != DEFAULT && transfer.getPower()==1.0) {
            gamepad1.rumble(0.3, 0.3, 50);
        }

        shooterTelem.addf("Target", "%.0f", targetVel);
        shooterTelem.addf("Actual", "%.0f", shooter.getVelocity());
    }
    private double applyCurve(double input, String preset) {
        if (Math.abs(input) < TeleOpConfig.DRIVE_DEADZONE) return 0;

        switch (preset) {
            case "TANH":
                double a = TeleOpConfig.TANH_A;
                return Math.tanh(a * input) / Math.tanh(a);

            case "QUADRATIC":
                return input * Math.abs(input);

            case "EXPONENTIAL":
                return Math.pow(input, 3);
            case "SMOOTH":
                double x = (input + 1) / 2;
                double y = x * x * (3 - 2 * x);
                return 2 * y - 1;
            case "CUBIC":
                double k = TeleOpConfig.CUBIC_WEIGHT;
                return ((1 - k) * input) + (k * Math.pow(input, 3));
            case "LINEAR":
            default:
                return input;
        }
    }
    private double deadbandRemap(double input, double deadzone) {
        if (Math.abs(input) < deadzone) return 0.0;
        return Math.signum(input) * (Math.abs(input) - deadzone) / (1.0 - deadzone);
    }

}