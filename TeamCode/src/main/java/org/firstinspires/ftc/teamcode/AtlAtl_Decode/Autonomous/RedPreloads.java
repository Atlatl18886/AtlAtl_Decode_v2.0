package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake.Intake;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.shooter.Shooter;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.transfer.Transfer;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous.Sequences;
@Autonomous(name="Red Preloads(3)", group="RoadRunner")
public class RedPreloads extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = Constants.RED_CLOSE_START;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Sequences sequences = new Sequences(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        Action preload = drive.actionBuilder(Constants.RED_CLOSE_START)
                .strafeToLinearHeading(Constants.RED_SHOOT,Constants.RED_ANGLE)
                .waitSeconds(0.7)
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        preload,
                        new ParallelAction(
                                sequences.scoreSet()
                        )
                )
        );
    }

}