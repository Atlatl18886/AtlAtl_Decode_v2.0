package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous.Red;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous.Constants;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake.Intake;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.shooter.Shooter;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.transfer.Transfer;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous.Sequences;
@Autonomous(name="Red 6", group="RR Red")
public class Red6Ball extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = Constants.RED_CLOSE_START;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Sequences sequences = new Sequences(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        double Xready = -12;
        double Yready = -25;

        waitForStart();
        if (isStopRequested()) return;

        Action preload = drive.actionBuilder(Constants.RED_CLOSE_START)
                .strafeToLinearHeading(Constants.RED_SHOOT,Constants.RED_ANGLE)
                .waitSeconds(0.7)
                .build();
        Action balls = drive.actionBuilder(new Pose2d(Constants.RED_SHOOT, Constants.RED_ANGLE))
                .turnTo(0)
                .lineToX(Xready)
                .turnTo(Math.toRadians(90))
                .waitSeconds(0.5)
                .lineToY(50)
                .build();
        Action back = drive.actionBuilder(new Pose2d(Xready,50, Math.toRadians(90)))
                .lineToY(Yready)
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(90))
                .build();
        Action back2 = drive.actionBuilder(new Pose2d(Xready,Yready,Math.toRadians(90)))
                .turnTo(Math.toRadians(0))
                .lineToX(-31)
                .turnTo(Constants.RED_ANGLE)
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        preload,
                        new ParallelAction(
                                sequences.scoreSet()
                        ),
                        new ParallelAction(
                                intake.setIntakePower(1.0),
                                intake.setAntiPower(1.0),
                                balls
                        ),
                        back,
                        back2,
                        new ParallelAction(
                                intake.setIntakePower(1.0),
                                intake.setAntiPower(1.0),
                                sequences.scoreSet()
                        )
                )
        );
    }

}