package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake.Intake;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.shooter.Shooter;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.transfer.Transfer;

@Disabled
@TeleOp(name="One Ball Test(subsystems)")
public class OneBallTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        intake.in(),
                        new ParallelAction(
                            shooter.spinUp(4500),
                            shooter.waitForRPM(4500)
                        ),
                        transfer.out(),
                        new SleepAction(1),
                        shooter.stop()
                )
        );
    }
}
