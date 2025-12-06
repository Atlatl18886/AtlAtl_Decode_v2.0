package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Controls:
Left Joystick - forward, backward, and strafe
Right Joystick - turning
Dpad Up - Save current heading
Y - Shooter Close
X - turn to saved heading
B - Shooter Far
A - Reverse intake (outtake)
Left Trigger - shooter mid
Left Bumper - stop intake
Right Trigger - Transfer
Right bumper - aim mode for turning
*/

@TeleOp
public class TeleOpTest extends OpMode {
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intake, transfer, shooter;
    public IMU imu;

    //mem for lerp
    private double prevStrafe = 0;
    private double prevVertical = 0;
    private double prevHeading = 0;
    //mem for turntogoal
    private double savedHeading = 0;

    private double targetVel = 0;
    private final double CLOSE = ShooterConfig.CLOSE_TPS;
    private final double MID = ShooterConfig.MID_TPS;
    private final double FAR = ShooterConfig.FAR_TPS;
    private final double DEFAULT = ShooterConfig.DEFAULT_TPS;
    private double prevShooterError = 0;
    private double shooterIntegral = 0;
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private final double shooter_Kp = ShooterConfig.shooter_Kp;
    private double shooter_Ki = ShooterConfig.shooter_Ki;
    private final double shooter_Kd = ShooterConfig.shooter_Kd;
    private final double shooter_Kf = ShooterConfig.shooter_Kf;

    private boolean cycleActive = false;
    private final ElapsedTime transferTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double prevError = 0;
    private final ElapsedTime pdTimer = new ElapsedTime();

    @Override
    public void init() {

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
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void loop() {
        double loopDt = loopTimer.seconds();
        loopTimer.reset();

        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        Intake();
        Shooter();
        Transfer();

        //imu turntogoal logic
        if (gamepad1.dpad_up) {
            savedHeading = currentHeading;
        }
        if (gamepad1.x) {
            double error = savedHeading - currentHeading;
            //so that it doesnt turn over 180 degress(the long way around)
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;

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
                leftFront.setPower(0); rightFront.setPower(0);
                leftBack.setPower(0); rightBack.setPower(0);
            }

            telemetry.addData("IMU Status", "ALIGNING");
            telemetry.addData("Target", savedHeading);
            telemetry.addData("Err", error);

        } else {
            Drive(TeleOpConfig.DRIVE_PRESET, TeleOpConfig.DRIVE_DEADZONE, loopDt);
        }

        telemetry.addData("Heading", "%.1f", currentHeading);
        telemetry.update();
    }
    public void Drive(String PRESET, double deadzone, double dt) {
        // apply deadzone remap before any other processing
        double rStrafe = deadbandRemap(-gamepad1.left_stick_x, deadzone);
        double rVertical = deadbandRemap(gamepad1.left_stick_y, deadzone);
        double rHeading = deadbandRemap(-gamepad1.right_stick_x, deadzone);

        double strafe, vertical, heading;

        if (PRESET.equals("LERP")) {
            double lerpSpeed = TeleOpConfig.LERP_SPEED;
            strafe = lerp(prevStrafe, rStrafe, lerpSpeed, dt);
            vertical = lerp(prevVertical, rVertical, lerpSpeed, dt);
            heading = lerp(prevHeading, rHeading, lerpSpeed, dt);

            prevStrafe = strafe;
            prevVertical = vertical;
            prevHeading = heading;
        } else {
            strafe = processInput(rStrafe, PRESET);
            vertical = processInput(rVertical, PRESET);
            heading = processInput(rHeading, PRESET);
        }

        if (gamepad1.right_bumper) {
            heading *= TeleOpConfig.AIM_TURN_SCALE;
            vertical *= TeleOpConfig.AIM_TURN_SCALE;
        }

        vertical *= TeleOpConfig.speedFactor;
        heading *= TeleOpConfig.speedFactor;
        strafe *= TeleOpConfig.speedFactor;

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
    private double processInput(double input, String preset) {
        switch (preset) {
            case "QUADRATIC":
                return input * Math.abs(input);
            case "CUBIC_BLEND":
                double w = org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.TeleOpConfig.CUBIC_BLEND_WEIGHT;
                return (input * (1.0 - w)) + (Math.pow(input, 3) * w);
            case "EXPONENTIAL":
                return Math.pow(input, 3);
            case "TANH":
                final double a = org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.TeleOpConfig.TANH_A;
                return Math.tanh(a * input) / Math.tanh(a);
            default:
                return input;
        }
    }
    private double lerp(double current, double target, double speed, double dt) {
        double alpha = speed * dt;
        if (alpha > 1.0) alpha = 1.0;
        return current + (target - current) * alpha;
    }
    private double deadbandRemap(double input, double deadzone) {
        if (Math.abs(input) < deadzone) return 0.0;
        //remap remaining range
        return Math.signum(input) * (Math.abs(input) - deadzone) / (1.0 - deadzone);
    }

    //NON DT FUNCTIONALITIES
    private boolean intakeStopped = false;
    private boolean intakePrev = false;

    public void Intake() {
        if (gamepad2.left_bumper && !intakePrev) intakeStopped = !intakeStopped;
        intakePrev = gamepad1.left_bumper;

        double p = gamepad2.a ? -1.0 : (intakeStopped ? 0 : 1.0);
        intake.setPower(p);
    }
    public void Transfer() {
        double currentVel = Math.abs(shooter.getVelocity());
        boolean btnPressed = gamepad2.right_trigger > 0.2;

        boolean shooterReady = Math.abs(currentVel - targetVel) < ShooterConfig.tolerance && targetVel > 0;

        if (cycleActive) {
            if (transferTimer.milliseconds() < ShooterConfig.feedtime) {
                transfer.setPower(0.67);
            } else {
                transfer.setPower(0);
                if (shooterReady) cycleActive = false;
            }
        } else if (btnPressed) {
            if (shooterReady) {
                cycleActive = true;
                transferTimer.reset();
                transfer.setPower(0.67);
            } else {
                transfer.setPower(0.67);
            }
        } else {
            transfer.setPower(-0.8);
        }
    }
/*
--OLD SHOOTER LOGIC--
 public void Shooter() {
    // Pick a velocity preset
    if (gamepad2.left_trigger > 0.1) targetVel = MID;
    else if (gamepad2.b) targetVel = FAR;
    else if (gamepad2.y) targetVel = CLOSE;
    else targetVel = DEFAULT;

    // Apply velocity or stop
    if (targetVel > 0) shooter.setVelocity(targetVel);
    else shooter.setPower(0);

    // Simple telemetry
    telemetry.addData("Shooter Vel", "%.0f", Math.abs(shooter.getVelocity()));
    telemetry.addData("Target Vel", "%.0f", targetVel);

    ADD TO INIT:
    // in init()
double kF = 1.0 / ShooterConfig.MAX_TPS;   // or whatever your feedforward is

PIDFCoefficients pidf = new PIDFCoefficients(
    0.005,   // P
    0.0,     // I
    0.0,     // D
    kF       // F
);

shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

}
 */
    public void Shooter() {
        if (gamepad2.left_trigger > 0.1) targetVel = MID;
        else if (gamepad2.b) targetVel = FAR;
        else if (gamepad2.y) targetVel = CLOSE;
        else targetVel = DEFAULT;

        //static behavior
        if (targetVel == 0) {
            shooter.setPower(0);
            shooterIntegral = 0;
            prevShooterError = 0;
            return;
        }

        double dt = shooterTimer.seconds();
        shooterTimer.reset();
        double currentVel = Math.abs(shooter.getVelocity());
        double error = targetVel - currentVel;

        // P--correction
        double p = error * shooter_Kp;

        if (Math.abs(error) < 200) {
            shooterIntegral += error * dt;
        }
        double i = shooterIntegral * shooter_Ki;
        // D--dampens oscilation
        double derivative = (error - prevShooterError) / dt;
        double d = derivative * shooter_Kd;

        // feedforward-predicts power
        double f = targetVel * shooter_Kf;
        double power = f + p + i + d;

        power = Math.max(0, Math.min(1.0, power));
        shooter.setPower(power);
        prevShooterError = error;
        telemetry.addData("Shooter Power", "%.2f", power);
        telemetry.addData("Shooter Error", "%.0f", error);
        telemetry.addData("Shooter Vel", "%.0f", Math.abs(shooter.getVelocity()));
        telemetry.addData("SHooter Target", targetVel);
    }
}