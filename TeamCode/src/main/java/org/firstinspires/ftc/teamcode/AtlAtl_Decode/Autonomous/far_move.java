package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake.Intake;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.shooter.Shooter;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.transfer.Transfer;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Autonomous(name="farmove", group="")
public class far_move extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = Constants.BLUE_CLOSE_START;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Sequences sequences = new Sequences(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        Action move = drive.actionBuilder(Constants.BLUE_CLOSE_START)
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        move
                )
        );
    }

}