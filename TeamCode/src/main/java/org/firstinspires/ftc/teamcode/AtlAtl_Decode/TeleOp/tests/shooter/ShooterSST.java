package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.tests.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Shooter AutoTuner", group="tests")
@Disabled
public class ShooterSST extends LinearOpMode {

    private DcMotorEx shooter;
    private VoltageSensor batteryVoltageSensor;

    // --- DASHBOARD CONFIGURABLE PARAMETERS ---
    public static double TARGET_VELOCITY = 800;   // ticks/sec
    public static double TEST_DURATION   = 1.5;   // seconds per parameter test
    public static double SETTLING_TIME   = 0.5;   // seconds before scoring error
    public static double DEADBAND        = 5.0;   // ticks/sec, only for I usage
    public static double KI_LIMIT        = 0.2;   // max |I contribution| to power
    public static double FILTER_ALPHA    = 0.2;   // 0-1, velocity low-pass filter
    public static double MIN_POWER_FLOOR = 0.2;   // never command less than this power

    // Twiddle parameters for [Kp, Ki, Kd]
    // For now, we will tune Kp and Kd only; Ki stays zero.
    public static double Kp_INIT = 0.00004;
    public static double Ki_INIT = 0.0;       // start with 0; not tuned yet
    public static double Kd_INIT = 0.00093;

    public static double dKp_INIT = 0.00002;
    public static double dKi_INIT = 0.0;      // no Ki tuning in this pass
    public static double dKd_INIT = 0.00001;

    // Twiddle stopping conditions
    public static int    MIN_GLOBAL_ITERS = 4;     // minimum full Twiddle passes
    public static double DP_SUM_EPS      = 1e-5;   // dp sum threshold for convergence

    // Internal arrays: p[0]=Kp, p[1]=Ki, p[2]=Kd
    private double[] p  = new double[3];
    private double[] dp = new double[3];
    private String[] names = {"Kp", "Ki", "Kd"};

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;

    // Feedforward (Kf) measured empirically at runtime and kept fixed during tuning
    public static double kFBase = 0.00035;  // will be overwritten by measurement

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Be defensive: pick first voltage sensor if any exist
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        } else {
            batteryVoltageSensor = null;
        }

        // Make sure we are not fighting built-in PIDF
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        try {
            shooter.setVelocityPIDFCoefficients(0, 0, 0, 0);
        } catch (Exception e) {
            // Ignore if unsupported
        }

        // Initialize Twiddle params from dashboard values
        p[0] = Kp_INIT;
        p[1] = Ki_INIT;  // will stay 0 in this phase
        p[2] = Kd_INIT;

        dp[0] = dKp_INIT;
        dp[1] = dKi_INIT; // 0 => no Ki tuning
        dp[2] = dKd_INIT;

        telemetry.addLine("PIDF Shooter AutoTune (Fast KpKd)");
        telemetry.addData("TARGET_VELOCITY", TARGET_VELOCITY);
        telemetry.addLine("Press START to measure Kf and begin tuning.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 1) Measure Kf empirically at full power
        measureKf();

        telemetry.addData("Measured Kf", kFBase);
        telemetry.addLine("Spinning up with FF, then starting Twiddle...");
        telemetry.update();

        // 2) Spin up near target with pure feedforward before tuning
        preSpinup();

        // 3) Initial error benchmark
        double bestErr = runTest(p);
        int globalIters = 0;

        while (opModeIsActive()) {
            boolean improvementMade = false;
            globalIters++;

            for (int i = 0; i < p.length && opModeIsActive(); i++) {
                // Skip Ki (index 1) for this pass if dp[i] is zero
                if (dp[i] == 0.0) {
                    continue;
                }

                // Spin-up assist before each param test so we don't start from dead stop
                spinupAssist();

                // 1. Try increasing parameter
                p[i] += dp[i];
                double errUp = runTest(p);

                if (errUp < bestErr) {
                    bestErr = errUp;
                    dp[i] *= 1.1; // slightly enlarge step when we find improvement
                    improvementMade = true;
                } else {
                    // Spin-up assist again before the next test direction
                    spinupAssist();

                    // 2. Try decreasing parameter
                    p[i] -= 2 * dp[i];
                    double errDown = runTest(p);

                    if (errDown < bestErr) {
                        bestErr = errDown;
                        dp[i] *= 1.1;
                        improvementMade = true;
                    } else {
                        // 3. Revert and shrink step
                        p[i] += dp[i];
                        dp[i] *= 0.8;   // shrink a bit, but not too aggressively
                    }
                }

                if (isStopRequested()) break;

                telemetry.addLine("Twiddle step complete");
                telemetry.addData("Global Iter", globalIters);
                telemetry.addData("Param", names[i]);
                telemetry.addData("Value", "%.8f", p[i]);
                telemetry.addData("dValue", "%.8f", dp[i]);
                telemetry.addData("BestErr", bestErr);
                telemetry.addData("Measured Kf", kFBase);
                telemetry.update();
            }

            double dpSum = Math.abs(dp[0]) + Math.abs(dp[1]) + Math.abs(dp[2]);

            if (globalIters >= MIN_GLOBAL_ITERS && dpSum < DP_SUM_EPS) {
                telemetry.addLine("Twiddle convergence reached.");
                telemetry.addData("Global Iter", globalIters);
                telemetry.addData("dpSum", dpSum);
                telemetry.update();
                break;
            }

            if (!improvementMade && dpSum < DP_SUM_EPS * 5) {
                telemetry.addLine("No improvement and dpSum small-ish, stopping.");
                telemetry.addData("Global Iter", globalIters);
                telemetry.addData("dpSum", dpSum);
                telemetry.update();
                break;
            }
        }

        // Stop shooter after tuning
        shooter.setPower(0);

        while (opModeIsActive()) {
            telemetry.addLine("--- TUNING COMPLETE ---");
            telemetry.addData("Global Iterations", MIN_GLOBAL_ITERS);
            displayFinalValues(bestErr);
            telemetry.update();
        }
    }

    /**
     * Measure Kf by running at full power and reading steady-state velocity.
     */
    private void measureKf() {
        telemetry.addLine("Measuring Kf: full power spin...");
        telemetry.update();

        shooter.setPower(1.0);
        sleep(800); // let it spin up

        double vel = shooter.getVelocity();
        if (Double.isNaN(vel) || vel <= 0) {
            // fallback: keep existing or default
            vel = 2000; // arbitrary but non-zero to avoid crash
        }

        kFBase = 1.0 / vel;

        telemetry.addData("Measured maxVel", vel);
        telemetry.addData("Computed Kf", kFBase);
        telemetry.update();

        // Leave it spinning for a bit more if you want, or stop briefly
        shooter.setPower(0.0);
        sleep(200);
    }

    /**
     * Spin the shooter using only feedforward to get close to target speed.
     */
    private void preSpinup() {
        ElapsedTime spinTimer = new ElapsedTime();
        spinTimer.reset();

        while (opModeIsActive() && spinTimer.seconds() < 0.8) {
            double voltage = getBatteryVoltage();
            double fTerm = kFBase * TARGET_VELOCITY * (12.0 / voltage);
            double power = clip(fTerm);

            shooter.setPower(power);

            telemetry.addLine("Pre-spinup (FF only)");
            telemetry.addData("Target", TARGET_VELOCITY);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }

    /**
     * Quick spin-up assist between Twiddle tests to avoid starting from zero.
     */
    private void spinupAssist() {
        double assistPower = 0.6; // adjust if needed per your shooter
        shooter.setPower(assistPower);
        sleep(250);  // short burst
    }

    /**
     * Run one test with the current [Kp, Ki, Kd] parameters and return MSE.
     */
    private double runTest(double[] params) {
        timer.reset();
        loopTimer.reset();
        integralSum = 0;
        lastError = 0;

        double lastVel = 0;
        double filteredVel = 0;
        boolean firstSample = true;

        double totalErrorSq = 0;
        int samples = 0;

        while (opModeIsActive() && timer.seconds() < TEST_DURATION) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0) dt = 0.02; // ~50 Hz assumption

            // --- Read and filter velocity ---
            double rawVel = shooter.getVelocity();
            if (Double.isNaN(rawVel) || Double.isInfinite(rawVel)) {
                rawVel = 0;
            }

            if (firstSample) {
                filteredVel = rawVel;
                firstSample = false;
            } else {
                filteredVel = FILTER_ALPHA * rawVel + (1.0 - FILTER_ALPHA) * lastVel;
            }
            lastVel = filteredVel;

            double error = TARGET_VELOCITY - filteredVel;

            // --- PID terms ---
            double pTerm = params[0] * error;

            // Integral with simple deadband and anti-windup
            if (Math.abs(error) > DEADBAND && params[1] != 0.0) {
                integralSum += error * dt;
            }
            double rawI = params[1] * integralSum;
            double iTerm = clamp(rawI, -KI_LIMIT, KI_LIMIT);

            // Derivative
            double derivative = (error - lastError) / dt;
            double dTerm = params[2] * derivative;

            lastError = error;

            // --- Feedforward with voltage compensation (ONLY FF scaled) ---
            double voltage = getBatteryVoltage();
            double fTerm = kFBase * TARGET_VELOCITY * (12.0 / voltage);

            // Total power
            double totalPower = fTerm + pTerm + iTerm + dTerm;

            // Apply minimum power floor so we don't starve the shooter
            if (totalPower > 0) {
                totalPower = Math.max(totalPower, MIN_POWER_FLOOR);
            }

            totalPower = clip(totalPower);
            shooter.setPower(totalPower);

            // --- Scoring: ignore early spinup, score steady state ---
            if (timer.seconds() > SETTLING_TIME) {
                totalErrorSq += error * error;
                samples++;
            }

            // --- Telemetry for Dashboard ---
            telemetry.addData("0_Target", TARGET_VELOCITY);
            telemetry.addData("1_ActualFiltered", filteredVel);
            telemetry.addData("2_ActualRaw", rawVel);
            telemetry.addData("3_Error", error);
            telemetry.addData("4_Power", totalPower);
            telemetry.addData("5_dt", dt);

            telemetry.addData("Kp", params[0]);
            telemetry.addData("Ki", params[1]);
            telemetry.addData("Kd", params[2]);
            telemetry.addData("KfBase", kFBase);

            telemetry.update();
        }

        // Do NOT stop shooter here; keep spinning between tests for consistency
        if (samples == 0) return Double.MAX_VALUE;
        return totalErrorSq / samples; // mean squared error
    }

    private void displayFinalValues(double cost) {
        telemetry.addData("Final Cost (MSE)", cost);
        for (int i = 0; i < p.length; i++) {
            telemetry.addData("RESULT: " + names[i], "%.8f", p[i]);
        }
        telemetry.addData("Note", "Kf is kFBase = %.8f (FF only)", kFBase);
    }

    private double clip(double value) {
        return clamp(value, 0.0, 1.0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double getBatteryVoltage() {
        if (batteryVoltageSensor == null) return 12.0;
        double v = batteryVoltageSensor.getVoltage();
        if (Double.isNaN(v) || v <= 0) v = 12.0;
        return v;
    }
}
