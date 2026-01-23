package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.shooter;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ShooterOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        shooter.spinUp(4500),
                        shooter.waitForRPM(4500),
                        shooter.stop()
                )
        );
    }
}
