package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.tests;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.ShooterConfig;

@TeleOp
public class ShooterTest extends OpMode {
    private DcMotorEx shooter;
    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(0);
    }
    @Override
    public void loop() {
        Shooter();
    }
    public void Shooter() {
        if (gamepad1.y) {
            shooter.setVelocity(ShooterConfig.CLOSE_TPS);
        }
    }
}

