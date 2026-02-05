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
@Autonomous(name="Red 12", group="RR Red")
public class supersigma12 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = Constants.RED_CLOSE_START;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Sequences sequences = new Sequences(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        double RED_INTAKE_ANGLE = Constants.RED_INTAKE_ANGLE;

        waitForStart();
        if (isStopRequested()) return;

        Action preload = drive.actionBuilder(Constants.RED_CLOSE_START)
                .strafeToLinearHeading(Constants.RED_SHOOT, Constants.RED_ANGLE)
                .waitSeconds(0.7)
                .build();

        Action getRow1 = drive.actionBuilder(new Pose2d(Constants.RED_SHOOT, Constants.RED_ANGLE))
                .strafeToLinearHeading(Constants.RED_READY1, RED_INTAKE_ANGLE)
                .strafeTo(Constants.RED_ROW1)
                .waitSeconds(0.5)
                .build();

        Action scoreRow1 = drive.actionBuilder(new Pose2d(Constants.RED_ROW1, RED_INTAKE_ANGLE))
                .strafeToLinearHeading(Constants.RED_SHOOT, Constants.RED_ANGLE)
                .build();



        Action getRow2 = drive.actionBuilder(new Pose2d(Constants.RED_SHOOT, Constants.RED_ANGLE))
                .strafeToLinearHeading(Constants.RED_READY2, RED_INTAKE_ANGLE)
                .strafeTo(Constants.RED_ROW2)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(Constants.RED_READY2.x, Constants.RED_READY2.y+15))
                .build();

        Action driveToGate = drive.actionBuilder(new Pose2d(new Vector2d(Constants.RED_READY2.x, Constants.RED_READY2.y+15), Constants.RED_INTAKE_ANGLE))
                .waitSeconds(0.3)
                .strafeToLinearHeading(Constants.RED_GATE, 0)
                .waitSeconds(0.65)

                .strafeTo(Constants.RED_GATE_READY)
                .build();

        Action scoreRow2 = drive.actionBuilder(new Pose2d(Constants.RED_GATE_READY, 0))
                .strafeTo(Constants.RED_SHOOT)
                .turnTo(Constants.RED_ANGLE)
                .waitSeconds(0.5)
                .build();

        Action getRow3 = drive.actionBuilder(new Pose2d(Constants.RED_SHOOT, Constants.RED_ANGLE))
                .turnTo(0)
                .strafeTo(Constants.RED_READY3)
                .turnTo(Constants.RED_INTAKE_ANGLE)
                .strafeTo(Constants.RED_ROW3)
                .waitSeconds(0.5)
                .strafeTo(Constants.RED_READY3)
                .build();

        Action scoreRow3 = drive.actionBuilder(new Pose2d(Constants.RED_READY3, RED_INTAKE_ANGLE))
                .strafeToLinearHeading(Constants.RED_SHOOT, Constants.RED_ANGLE)
                .waitSeconds(0.5)
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
                                getRow1
                        ),
                        scoreRow1,
                        new ParallelAction(
                                sequences.scoreSet()
                        ),
                        new ParallelAction(
                                intake.setIntakePower(1.0),
                                intake.setAntiPower(1.0),
                                getRow2
                        ),
                        driveToGate,
                        scoreRow2,
                        new ParallelAction(
                                sequences.scoreSet()
                        ),
                        new ParallelAction(
                                intake.setIntakePower(1.0),
                                intake.setAntiPower(1.0),
                                getRow3
                        ),
                        scoreRow3,
                        new ParallelAction(
                                sequences.scoreSet()
                        ),
                        intake.setIntakePower(0),
                        intake.setAntiPower(0),
                        shooter.stop(),
                        transfer.stop()
                )
        );
    }

}