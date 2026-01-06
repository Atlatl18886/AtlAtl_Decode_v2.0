package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Config.ShooterConfig;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Config.TeleOpConfig;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.control.Toggle;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.control.RobotModeFSM;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.drivetrain.AdaptiveGamma;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.drivetrain.HeadingController;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.drivetrain.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.drivetrain.InputProcessor;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.drivetrain.PrioritySuppression;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.TelemetryHelper;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.EventLogger;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.util.LoopProfiler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

@TeleOp(name="V2.5 TeleOp", group="Main")
public class TeleOpTest extends OpMode {
    public enum DrivePreset {
        LERP, QUADRATIC, CUBIC_BLEND, EXPONENTIAL, TANH, LINEAR, ADAPTIVE
    }

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intake, transfer, shooter;
    private DcMotorEx antiroller;
    public IMU imu;
    private double savedHeading = 0;

    //bulk reads
    private List<LynxModule> allHubs;

    private static final double CLOSE = ShooterConfig.CLOSE_TPS;
    private static final double MID = ShooterConfig.MID_TPS;
    private static final double FAR = ShooterConfig.FAR_TPS;
    private static final double DEFAULT = ShooterConfig.DEFAULT_TPS;
    private final double SHOOTER_kP = ShooterConfig.shooter_Kp;
    private final double SHOOTER_kI = ShooterConfig.shooter_Ki;
    private final double SHOOTER_kD = ShooterConfig.shooter_Kd;
    private final double SHOOTER_kF = ShooterConfig.shooter_Kf;

    private final ElapsedTime loopTimer = new ElapsedTime();

    //jerk(slew) rate limiters
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(7.5);
    private final SlewRateLimiter verticalLimiter = new SlewRateLimiter(7.5);
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(7.5);

    private HeadingController headingController;
    private AdaptiveGamma gammaHelper;
    private PrioritySuppression suppressionHelper = new PrioritySuppression();
    private InputProcessor inputProcessor;
    Toggle intakeToggle = new Toggle();
    private RobotModeFSM fsm;

    // telemetry helpers
    private TelemetryHelper driveT;
    private TelemetryHelper intakeT;
    private TelemetryHelper shooterT;
    private TelemetryHelper debugT;
    private TelemetryHelper loopT;
    private TelemetryHelper eventsT;

    // profiler&event logger
    private LoopProfiler profiler = new LoopProfiler();
    private EventLogger events = new EventLogger(12);

    private double voltageScale = 1.0;
    //dt vel computation state
    private double robotSpeed = 0.0;
    private final double velSmooth = 0.7;//expon smoothing factor

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            module.clearBulkCache();
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

        //imu init
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();

        loopTimer.reset();
        headingController = new HeadingController(TeleOpConfig.imu_kP, TeleOpConfig.imu_kD);
        gammaHelper = new AdaptiveGamma(5.5, 0.5, 300, 2250);
        inputProcessor = new InputProcessor(gammaHelper);
        fsm = new RobotModeFSM(events);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        antiroller = hardwareMap.get(DcMotorEx.class, "antiroller");
        antiroller.setDirection(DcMotorSimple.Direction.REVERSE);
        antiroller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        telemetry.addData("init done", "");

        driveT = TelemetryHelper.create(telemetry, "Drive");
        intakeT = TelemetryHelper.create(telemetry, "Intake");
        shooterT = TelemetryHelper.create(telemetry, "Shooter");
        debugT = TelemetryHelper.create(telemetry, "Debug");

        loopT = debugT.child("Loop");
        eventsT = debugT.child("Events");

        profiler.start();
        events.add("TeleOp init");

    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        double loopMs = profiler.update();
        loopT.clear();
        loopT.addf("loop", "%.2f", loopMs);
        loopT.addf("avg", "%.2f", profiler.getAvg());

        double loopDt = loopTimer.seconds();
        loopTimer.reset();

        if (profiler.getCount() % 40 == 0) {
            double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            voltageScale = 12.8 / currentVoltage;
        }

        fsm.update(gamepad1.x); // heading lock button

        updVelocity();
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        Intake();
        Shooter();
        Transfer();

        // imu turntoheading
        if (gamepad1.dpad_up) {
            savedHeading = currentHeading;
            events.add("Heading Saved :" + Math.round(savedHeading));
        }

        switch (fsm.getMode()) {
            case HEADING_LOCK:
                runHeadingLock(currentHeading);
                break;

            case MANUAL:
                headingController.reset(); // Reset PID when not in use
                Drive(TeleOpConfig.DRIVE_PRESET, TeleOpConfig.DRIVE_DEADZONE, loopDt);
                break;

            case DISABLED:
                stopDrive();
                break;
        }

        driveT.addf("heading", "%.1f", currentHeading);
        driveT.addf("speed", "%.0f", robotSpeed);
        events.pushTo(eventsT);
        driveT.push();
        intakeT.push();
        shooterT.push();
        debugT.push();
        telemetry.update();
    }
    private void stopDrive() {
        leftFront.setPower(0); rightFront.setPower(0);
        leftBack.setPower(0); rightBack.setPower(0);
    }
    private void runHeadingLock(double currentHeading) {
        resetLimiters();

        double turnPower = headingController.compute(currentHeading, savedHeading);
        turnPower = Math.max(-0.5, Math.min(0.5, turnPower));

        if (Math.abs(savedHeading - currentHeading) > 2.0) {
            leftFront.setPower(turnPower);
            leftBack.setPower(turnPower);
            rightFront.setPower(-turnPower);
            rightBack.setPower(-turnPower);
        } else {
            stopDrive();
        }

        telemetry.addData("Status", "ALIGNING TO " + savedHeading);
    }
    private void resetLimiters() {
        strafeLimiter.reset();
        verticalLimiter.reset();
        turnLimiter.reset();
    }
    private void updVelocity() {
        //calc instant velocities (ticks/sec)
        double vLF = Math.abs(leftFront.getVelocity());
        double vRF = Math.abs(rightFront.getVelocity());
        double vLB = Math.abs(leftBack.getVelocity());
        double vRB = Math.abs(rightBack.getVelocity());
        double instantSpeed = (vLF + vRF + vLB + vRB) / 4.0;
        //apply exponential smoothing (low-pass filter)
        robotSpeed = (velSmooth * robotSpeed) + ((1.0 - velSmooth) * instantSpeed);
    }

    public void Drive(String presetString, double deadzone, double dt) {
        DrivePreset preset;
        try {
            preset = DrivePreset.valueOf(presetString);
        } catch (IllegalArgumentException e) {
            preset = DrivePreset.LINEAR;
        }

        double[] shaped = inputProcessor.process(gamepad1, preset, deadzone, robotSpeed);
        double strafe = shaped[0];
        double vertical = shaped[1];
        double heading = shaped[2];

        //priority suppression
        double[] suppressed = suppressionHelper.apply(strafe, vertical, heading);
        strafe = suppressed[0];
        vertical = suppressed[1];
        heading = suppressed[2];

        if (gamepad1.left_bumper) {
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

        leftFront.setPower((leftFrontPower / max) * voltageScale);
        rightFront.setPower((rightFrontPower / max) * voltageScale);
        leftBack.setPower((leftBackPower / max) * voltageScale);
        rightBack.setPower((rightBackPower / max) * voltageScale);
    }


    //NON DT FUNCTIONALITIES

    public void Intake() {
        boolean prevState = intakeToggle.get();
        boolean intakeActive = intakeToggle.updateTrigger(gamepad2.left_trigger, 0.1);

        // log only on state transition
        if (intakeActive != prevState) {
            events.add(intakeActive ? "Intake toggled ON" : "Intake toggled OFF");
        }

        intake.setPower(intakeActive ? 1.0 : 0.0);

        final double min = 0.5;
        final double max = 0.9;
        antiroller.setPower(intakeActive ? max : min);

        intakeT.add("state", intakeActive);
        intakeT.addf("power", "%.2f", intake.getPower());
    }

    public void Transfer() {
        double p;

        if (gamepad2.right_trigger > 0.1) {
            p = 0.8;
        }
        else {
            p = -0.6;
        }
        transfer.setPower(p);

    }
    private double targetVel = 0;
    public void Shooter() {
        /*
        if (gamepad2.a) {
            targetVel = CLOSE;
            events.add("Shooter CLOSE");
        } else if (gamepad2.b) {
            targetVel = MID;
            events.add("Shooter MID");
        } else if (gamepad2.y) {
            targetVel = FAR;
            events.add("Shooter FAR");
        } else {
            targetVel = DEFAULT;
        }*/
        targetVel = DEFAULT;
        shooter.setVelocity(targetVel);

        shooterT.addf("vel", "%.0f", Math.abs(shooter.getVelocity()));
        shooterT.addf("target", "%.0f", targetVel);
    }

    @Override
    public void stop() {
        fsm.setDisabled();
        stopDrive();
        intake.setPower(0);
        transfer.setPower(0);
    }
}