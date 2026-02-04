package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Config.ShooterConfig;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Config.TeleOpConfig;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.control.Toggle;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.control.RobotModeFSM;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain.AdaptiveGamma;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain.HeadingController;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain.InputProcessor;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain.PrioritySuppression;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.TelemetryHelper;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.EventLogger;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.util.LoopProfiler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

@TeleOp(name="V2.5 TeleOp?", group="Main")
@Disabled
public class TeleOpTest extends OpMode {
    public enum DrivePreset {
        LERP, QUADRATIC, CUBIC_BLEND, EXPONENTIAL, TANH, LINEAR, ADAPTIVE, NORMAL
    }

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intake, transfer, shooter;
    private DcMotorEx antiroller;
    public IMU imu;
    private double savedHeading = 0;

    private List<LynxModule> allHubs;

    private static final double CLOSE = ShooterConfig.CLOSE_RPM;
    private static final double MID = ShooterConfig.MID_RPM;
    private static final double FAR = ShooterConfig.FAR_RPM;
    private static final double DEFAULT = ShooterConfig.DEFAULT_RPM;
    private final double SHOOTER_kP = ShooterConfig.shooter_Kp;
    private final double SHOOTER_kI = ShooterConfig.shooter_Ki;
    private final double SHOOTER_kD = ShooterConfig.shooter_Kd;
    private final double SHOOTER_kF = ShooterConfig.shooter_Kf;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.5);
    private final SlewRateLimiter verticalLimiter = new SlewRateLimiter(3.5);
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(3.5);

    private HeadingController headingController;
    private AdaptiveGamma gammaHelper;
    private PrioritySuppression suppressionHelper = new PrioritySuppression();
    private InputProcessor inputProcessor;
    Toggle intakeToggle = new Toggle();
    private RobotModeFSM fsm;

    private TelemetryHelper driveTelem, intakeTelem, shooterTelem, debugTelem, loopTelem, eventsTelem;
    private LoopProfiler profiler = new LoopProfiler();
    private EventLogger events = new EventLogger(12);

    private double robotSpeed = 0.0;
    private final double velSmooth = 0.55;

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

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        imu.resetYaw();

        loopTimer.reset();
        headingController = new HeadingController(TeleOpConfig.imu_kP, TeleOpConfig.imu_kD);
        gammaHelper = new AdaptiveGamma(5.5, 0.5, 300, 2250);
        //gammaMax: 5.5 - high finesse at low speeds
        //gammaMin: 0.5 - high response at high speeds
        //transStart: 300 - tps where transition begins
        //transEnd: 2250 - tps where transition completes
        inputProcessor = new InputProcessor(gammaHelper);
        fsm = new RobotModeFSM(events);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        antiroller = hardwareMap.get(DcMotorEx.class, "antiroller");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        driveTelem = TelemetryHelper.create(telemetry, "Drive");
        intakeTelem = TelemetryHelper.create(telemetry, "Intake");
        shooterTelem = TelemetryHelper.create(telemetry, "Shooter");
        debugTelem = TelemetryHelper.create(telemetry, "Debug");
        loopTelem = debugTelem.child("--Loop");
        eventsTelem = debugTelem.child("--Events");

        profiler.start();
        events.add("init done");
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) { module.clearBulkCache(); }

        //loop profiling
        double loopMs = profiler.update();
        loopTelem.clear();
        loopTelem.addf("loop", "%.2f", loopMs);
        loopTelem.addf("avg", "%.2f", profiler.getAvg());

        double loopDt = loopTimer.seconds();
        loopTimer.reset();

        fsm.update(gamepad1.x);
        updVelocity();
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        Intake();
        Shooter();
        Transfer();

        if (gamepad1.dpad_up) {
            savedHeading = currentHeading;
            events.add("Heading Saved :" + Math.round(savedHeading));
        }

        if (fsm.getMode() == RobotModeFSM.RobotMode.DISABLED) {
            stopDrive();
        } else {
            Drive(currentHeading, loopDt);
        }

        driveTelem.addf("heading", "%.1f", currentHeading);
        driveTelem.addf("speed", "%.0f", robotSpeed);
        events.pushTo(eventsTelem);

        driveTelem.push();
        intakeTelem.push();
        shooterTelem.push();
        debugTelem.push();
        telemetry.update();
    }

    private void Drive(double currentHeading, double dt) {
        DrivePreset preset;
        try { preset = DrivePreset.valueOf(TeleOpConfig.DRIVE_PRESET); }
        catch (Exception e) { preset = DrivePreset.LINEAR; }

        double[] shaped = inputProcessor.process(gamepad1, preset, TeleOpConfig.DRIVE_DEADZONE, robotSpeed);
        double strafe = shaped[0];
        double vertical = shaped[1];
        double heading;

        boolean isLocked = (fsm.getMode() == RobotModeFSM.RobotMode.HEADING_LOCK);
        if (isLocked) {
            heading = headingController.compute(currentHeading, savedHeading);
            heading = Math.max(-0.7, Math.min(0.7, heading));
        } else {
            heading = shaped[2];
            headingController.reset();
        }

        //priority supressing
        if (TeleOpConfig.USE_PRIORITY_SUPPRESSION) {
            double[] suppressed = suppressionHelper.apply(strafe, vertical, heading, isLocked);
            strafe = suppressed[0];
            vertical = suppressed[1];
            heading = suppressed[2];
        }

        if (gamepad1.left_bumper) {
            heading *= TeleOpConfig.AIM_TURN_SCALE;
            vertical *= TeleOpConfig.AIM_TURN_SCALE;
        }

        //conditional slew rate limiting
        if (TeleOpConfig.USE_SLEW_LIMITING) {
            strafe = strafeLimiter.calculate(strafe, dt);
            vertical = verticalLimiter.calculate(vertical, dt);
            heading = turnLimiter.calculate(heading, dt);
        } else {
            strafeLimiter.reset();
            verticalLimiter.reset();
            turnLimiter.reset();
        }

        double lf  = vertical + strafe + heading;
        double rf = vertical + strafe - heading;
        double lb   = vertical - strafe + heading;
        double rb  = vertical - strafe - heading;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb))));

        leftFront.setPower((lf / max));
        rightFront.setPower((rf / max));
        leftBack.setPower((lb / max));
        rightBack.setPower((rb / max));
    }

    public void Intake() {
        boolean prevState = intakeToggle.get();
        boolean intakeActive = intakeToggle.updateTrigger(gamepad2.left_trigger, 0.1);

        if (intakeActive != prevState) {
            events.add(intakeActive ? "Intake toggled ON" : "Intake toggled OFF");
        }

        intake.setPower(intakeActive ? 1.0 : 0.35);
        antiroller.setPower(intakeActive ? 0.5 : 0.5);

        intakeTelem.add("state", intakeActive);
        intakeTelem.addf("power", "%.2f", intake.getPower());
    }

    public void Transfer() {
        transfer.setPower(gamepad2.right_trigger > 0.1 ? 1.0 : -0.4);
    }

    private double targetVel = 0;
    public void Shooter() {
        if (gamepad2.a) { targetVel = CLOSE; events.add("Shooter CLOSE"); }
        else if (gamepad2.b) { targetVel = MID; events.add("Shooter MID"); }
        else if (gamepad2.y) { targetVel = FAR; events.add("Shooter FAR"); }
        else { targetVel = DEFAULT; }

        shooter.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooter.setVelocity(targetVel);

        shooterTelem.addf("vel", "%.0f", Math.abs(shooter.getVelocity()));
        shooterTelem.addf("target", "%.0f", targetVel);
    }

    private void stopDrive() {
        leftFront.setPower(0); rightFront.setPower(0);
        leftBack.setPower(0); rightBack.setPower(0);
        strafeLimiter.reset(); verticalLimiter.reset(); turnLimiter.reset();
    }

    private void updVelocity() {
        double instantSpeed = (Math.abs(leftFront.getVelocity()) + Math.abs(rightFront.getVelocity()) +
                Math.abs(leftBack.getVelocity()) + Math.abs(rightBack.getVelocity())) / 4.0;
        robotSpeed = (velSmooth * robotSpeed) + ((1.0 - velSmooth) * instantSpeed);
    }

    @Override
    public void stop() {
        fsm.setDisabled();
        stopDrive();
        intake.setPower(0);
        transfer.setPower(0);
    }
}