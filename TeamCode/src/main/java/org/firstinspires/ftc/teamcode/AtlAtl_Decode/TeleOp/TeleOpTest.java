package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

@TeleOp
public class TeleOpTest extends OpMode {
    public enum DrivePreset {
        LERP, QUADRATIC, CUBIC_BLEND, EXPONENTIAL, TANH, LINEAR, ADAPTIVE
    }

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intake, transfer, shooter;
    public IMU imu;

    //bulk reads
    private List<LynxModule> allHubs;
    private double savedHeading = 0;
    private boolean isAligning = false;

    private final double CLOSE = ShooterConfig.CLOSE_TPS;
    private final double MID = ShooterConfig.MID_TPS;
    private final double FAR = ShooterConfig.FAR_TPS;
    private final double DEFAULT = ShooterConfig.DEFAULT_TPS;
    private final double SHOOTER_kP = ShooterConfig.shooter_Kp;
    private final double SHOOTER_kI = ShooterConfig.shooter_Ki;
    private final double SHOOTER_kD = ShooterConfig.shooter_Kd;
    private final double SHOOTER_kF = ShooterConfig.shooter_Kf;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double prevError = 0;
    private final ElapsedTime pdTimer = new ElapsedTime();

    //jerk(slew) rate limiters
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(7.5);
    private SlewRateLimiter verticalLimiter = new SlewRateLimiter(7.5);
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(7.5);

    //adaptive curve parameters
    private final double GAMMA_MAX = 5.5;//high finesse at low speeds
    private final double GAMMA_MIN = 0.5;// high response at high speeds
    private final int transStart = 300;// tps where transition begins
    private final int transEnd = 2250; // tps where transition completes

    //dt vel computation state
    private int prevLF = 0, prevRF = 0, prevLB = 0, prevRB = 0;
    private double robotSpeed = 0.0;
    private final double velSmooth = 0.7;//expon smoothing factor

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prevLF = leftFront.getCurrentPosition();
        prevRF = rightFront.getCurrentPosition();
        prevLB = leftBack.getCurrentPosition();
        prevRB = rightBack.getCurrentPosition();

        //imu init
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();

        loopTimer.reset();
        pdTimer.reset();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        double loopDt = loopTimer.seconds();
        loopTimer.reset();
        updVelocity(loopDt);
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        Intake();
        Shooter();
        Transfer();

        // imu turntoheading
        if (gamepad1.dpad_up) {
            savedHeading = currentHeading;
        }
        if (gamepad1.x) {
            resetDriveLimiters();

            double error = savedHeading - currentHeading;

            while (error > 180) error -= 360;
            while (error <= -180) error += 360;

            if (!isAligning) {
                pdTimer.reset();
                prevError = error;
                isAligning = true;
            }

            double pdDt = pdTimer.seconds();
            pdTimer.reset();
            if (pdDt < 1e-3) pdDt = 1e-3;

            double derivative = (error - prevError) / pdDt;
            prevError = error;

            double kP = TeleOpConfig.imu_kP;
            double kD = TeleOpConfig.imu_kD;
            double turnPower = (error * kP) + (derivative * kD);

            turnPower *= TeleOpConfig.imu_turn_factor;

            if (turnPower > 0.5) turnPower = 0.5;
            if (turnPower < -0.5) turnPower = -0.5;

            if (Math.abs(error) > 2.0) {
                leftFront.setPower(turnPower);
                leftBack.setPower(turnPower);
                rightFront.setPower(-turnPower);
                rightBack.setPower(-turnPower);
            } else {
                StopDrive();
            }

            telemetry.addData("IMU Status", "ALIGNING");
            telemetry.addData("Target", savedHeading);
            telemetry.addData("Err", error);
        } else {
            isAligning = false;
            Drive(TeleOpConfig.DRIVE_PRESET, TeleOpConfig.DRIVE_DEADZONE, loopDt);
        }

        telemetry.addData("Heading", "%.1f", currentHeading);
        telemetry.addData("Loop Rate", "%.0f Hz", 1.0/loopDt);
        telemetry.update();
    }
    private void StopDrive() {
        leftFront.setPower(0); rightFront.setPower(0);
        leftBack.setPower(0); rightBack.setPower(0);
    }
    private void resetDriveLimiters() {
        strafeLimiter.reset();
        verticalLimiter.reset();
        turnLimiter.reset();
    }
    private void updVelocity(double dt) {
        if (dt < 1e-6) return; //prevent divide by 0

        int currLF = leftFront.getCurrentPosition();
        int currRF = rightFront.getCurrentPosition();
        int currLB = leftBack.getCurrentPosition();
        int currRB = rightBack.getCurrentPosition();
        int deltaLF = currLF - prevLF;
        int deltaRF = currRF - prevRF;
        int deltaLB = currLB - prevLB;
        int deltaRB = currRB - prevRB;

        //upd prev poses
        prevLF = currLF;
        prevRF = currRF;
        prevLB = currLB;
        prevRB = currRB;

        //calc instant velocities (ticks/sec)
        double vLF = Math.abs(deltaLF / dt);
        double vRF = Math.abs(deltaRF / dt);
        double vLB = Math.abs(deltaLB / dt);
        double vRB = Math.abs(deltaRB / dt);
        double instantSpeed = Math.sqrt((vLF*vLF + vRF*vRF + vLB*vLB + vRB*vRB) / 4.0);

        //smoothing to reduce noise from encoder
        robotSpeed = (velSmooth * robotSpeed) + ((1.0 - velSmooth) * instantSpeed);
    }

    //linear interpolates between shaping gamma based on measured velociy
    private double computeAdaptiveGamma() {
        if (robotSpeed <= transStart) {
            return GAMMA_MAX;
        } else if (robotSpeed >= transEnd) {
            return GAMMA_MIN;
        } else {
            // Linear interpolation
            double t = (robotSpeed - transStart) /
                    (transEnd - transStart);
            return GAMMA_MAX + (GAMMA_MIN - GAMMA_MAX) * t;
        }
    }
    public void Drive(String presetString, double deadzone, double dt) {
        DrivePreset preset;
        try {
            preset = DrivePreset.valueOf(presetString);
        } catch (IllegalArgumentException e) {
            preset = DrivePreset.LINEAR;
        }

        double rStrafe = deadbandRemap(-gamepad1.left_stick_x, deadzone);
        double rVertical = deadbandRemap(gamepad1.left_stick_y, deadzone);
        double rHeading = deadbandRemap(-gamepad1.right_stick_x, deadzone);

        double strafe, vertical, heading;

        if (preset == DrivePreset.ADAPTIVE) {
            double gamma = computeAdaptiveGamma();

            //apply velocity-based power-law shaping
            strafe = applyAdaptiveShaping(rStrafe, gamma);
            vertical = applyAdaptiveShaping(rVertical, gamma);
            heading = applyAdaptiveShaping(rHeading, gamma);

            telemetry.addData("Robot Speed", "%.0f tps", robotSpeed);
            telemetry.addData("Gamma", "%.2f", gamma);
        } else {
            strafe = processInput(rStrafe, preset);
            vertical = processInput(rVertical, preset);
            heading = processInput(rHeading, preset);
        }

        //priority suppression
        double[] suppressed = applySuppression(strafe, vertical, heading);
        strafe = suppressed[0];
        vertical = suppressed[1];
        heading = suppressed[2];

        if (gamepad1.right_bumper) {
            heading *= TeleOpConfig.AIM_TURN_SCALE;
            vertical *= TeleOpConfig.AIM_TURN_SCALE;
        }

        vertical *= TeleOpConfig.speedFactor;
        heading *= TeleOpConfig.speedFactor;
        strafe *= TeleOpConfig.speedFactor;
        //slew rate limiting
        strafe = strafeLimiter.calculate(strafe, dt);
        vertical = verticalLimiter.calculate(vertical, dt);
        heading = turnLimiter.calculate(heading, dt);

        double leftFrontPower = vertical + strafe + heading;
        double rightFrontPower = vertical + strafe - heading;
        double leftBackPower = vertical - strafe + heading;
        double rightBackPower = vertical - strafe - heading;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        ));

        leftFront.setPower(leftFrontPower / max);
        rightFront.setPower(rightFrontPower / max);
        leftBack.setPower(leftBackPower / max);
        rightBack.setPower(rightBackPower / max);
    }

    /**
    apply adaptive power shaping
    formula -- output= sign(input) * |input|^gamma
     range of gamma is 0.5 to 5.5
    0.5 is sqrt response, agressive accel, 5.5 is aggressive dampeing, preciseness
     **/
    private double applyAdaptiveShaping(double input, double gamma) {
        if (Math.abs(input) < 1e-6) return 0.0;
        return Math.signum(input) * Math.pow(Math.abs(input), gamma);
    }

    //priority supression- ratio based
    private double[] applySuppression(double strafe, double vertical, double heading) {
        double absS = Math.abs(strafe);
        double absV = Math.abs(vertical);
        double absH = Math.abs(heading);

        double eps = 1e-6;
        double translationMag = Math.hypot(strafe, vertical);

        if (translationMag > 0.15) {
            double rotRatio = absH / (absH + translationMag + eps);
            rotRatio = Math.max(rotRatio, 0.4);
            heading *= rotRatio;
        }

        if (absV > absS) {
            double ratio = absS / (absV + eps);
            ratio = Math.max(ratio, 0.35);
            strafe *= ratio;
        } else {
            double ratio = absV / (absS + eps);
            ratio = Math.max(ratio, 0.35);
            vertical *= ratio;
        }
        if (absV < 0.3 && absS > absV) {
            vertical *= 0.5;
        }

        return new double[]{strafe, vertical, heading};
    }
    private double processInput(double input, DrivePreset preset) {
        switch (preset) {
            case QUADRATIC:
                return input * Math.abs(input);
            case CUBIC_BLEND:
                double w = TeleOpConfig.CUBIC_BLEND_WEIGHT;
                return (input * (1.0 - w)) + (Math.pow(input, 3) * w);
            case EXPONENTIAL:
                return Math.pow(input, 3);
            case TANH:
                final double a = TeleOpConfig.TANH_A;
                return Math.tanh(a * input) / Math.tanh(a);
            case LERP:
            case LINEAR:
            default:
                return input;
        }
    }
    private double deadbandRemap(double input, double deadzone) {
        if (Math.abs(input) < deadzone) return 0.0;
        return Math.signum(input) * (Math.abs(input) - deadzone) / (1.0 - deadzone);
    }
    public static class SlewRateLimiter {
        private double val = 0;
        private final double rateLimit;

        public SlewRateLimiter(double rateLimit) {
            this.rateLimit = rateLimit;
        }
        public double calculate(double target, double dt) {
            double maxChange = rateLimit * dt;
            double change = target - val;

            if (change > maxChange) change = maxChange;
            else if (change < -maxChange) change = -maxChange;

            val += change;
            return val;
        }
        public void reset() {
            val = 0.0;
        }
    }

    //NON DT FUNCTIONALITIES
    private boolean intakeActive = false;
    private boolean intakePrev = false;

    public void Intake() {
        boolean bumperPressed = gamepad2.left_bumper;
        if (bumperPressed && !intakePrev) intakeActive = !intakeActive;

        double power = intakeActive ? 1.0 : 0.0;
        intake.setPower(power);
        intakePrev = bumperPressed;
    }

    private boolean transferStopped = false;
    private boolean transferPrev = false;
    private double transferFreeVel = 0.0;
    private double learn_alfa = TeleOpConfig.learn_alfa;
    private double slipGain = TeleOpConfig.slipGain;
    private double minTransfer = TeleOpConfig.min_transfer;

    public void Transfer() {
        boolean togglePressed = gamepad2.right_trigger > 0.1;
        if (togglePressed && !transferPrev) {
            transferStopped = !transferStopped;
        }
        transferPrev = togglePressed;

        double basePower;
        if (gamepad2.left_trigger > 0.1) {
            basePower = 1.0;
        } else if (transferStopped) {
            basePower = 0.0;
        } else {
            basePower = -0.75;
        }

        if (basePower == 0.0) {
            transfer.setPower(0.0);
            return;
        }

        double vel = Math.abs(transfer.getVelocity());
        if (vel > transferFreeVel) {
            transferFreeVel += (vel - transferFreeVel) * learn_alfa;
        }

        double slip = 1.0 - (vel / Math.max(transferFreeVel, 1.0));
        slip = Math.max(0.0, Math.min(1.0, slip));

        double scaledPower = basePower * (1.0 - slip * slipGain);
        if (Math.abs(scaledPower) < minTransfer) {
            scaledPower = Math.signum(scaledPower) * minTransfer;
        }

        transfer.setPower(scaledPower);
    }

    private double targetVel = 0;
    public void Shooter() {
        if (gamepad2.right_bumper) {
            targetVel = CLOSE;
        } else if (gamepad2.b) {
            targetVel = MID;
        } else if (gamepad2.y) {
            targetVel = FAR;
        } else {
            targetVel = DEFAULT;
        }
        shooter.setVelocity(targetVel);

        telemetry.addData("Shooter Vel", "%.0f", Math.abs(shooter.getVelocity()));
        telemetry.addData("Target Vel", "%.0f", targetVel);
    }
}