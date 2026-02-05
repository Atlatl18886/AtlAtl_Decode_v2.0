package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous.Blue;

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
@Autonomous(name="Blue 9", group="RR Blue")
public class ishaanmygoat9 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = Constants.BLUE_CLOSE_START;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Sequences sequences = new Sequences(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        double Xready = -12;
        double Yready = -25;

        waitForStart();
        if (isStopRequested()) return;

        Action preload = drive.actionBuilder(Constants.BLUE_CLOSE_START)
                .strafeToLinearHeading(Constants.BLUE_SHOOT, Constants.BLUE_ANGLE)
                .waitSeconds(0.7)
                .build();

        Action getRow1 = drive.actionBuilder(new Pose2d(Constants.BLUE_SHOOT, Constants.BLUE_ANGLE))
                .strafeToLinearHeading(Constants.BLUE_READY1, Constants.BLUE_INTAKE_ANGLE)
                .strafeTo(Constants.BLUE_ROW1)
                .waitSeconds(0.5)
                .build();

        Action scoreRow1 = drive.actionBuilder(new Pose2d(Constants.BLUE_ROW1, Constants.BLUE_INTAKE_ANGLE))
                .strafeToLinearHeading(Constants.BLUE_SHOOT, Constants.BLUE_ANGLE)
                .build();


        Action getRow2 = drive.actionBuilder(new Pose2d(Constants.BLUE_SHOOT, Constants.BLUE_ANGLE))
                .strafeToLinearHeading(Constants.BLUE_READY2, Constants.BLUE_INTAKE_ANGLE)
                .strafeTo(Constants.BLUE_ROW2)
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(Constants.BLUE_READY2.x, Constants.BLUE_ROW2.y+15))
                .build();

        Action driveToGate = drive.actionBuilder(new Pose2d(new Vector2d(Constants.BLUE_READY2.x, Constants.BLUE_ROW2.y+15), Constants.BLUE_INTAKE_ANGLE))

                .waitSeconds(0.3)
                .strafeToLinearHeading(Constants.BLUE_GATE, 0)
                .waitSeconds(0.65)

                .strafeTo(Constants.BLUE_GATE_READY)
                .build();


        Action scoreRow2 = drive.actionBuilder(new Pose2d(Constants.BLUE_GATE_READY, 0))
                .strafeTo(Constants.BLUE_SHOOT)
                .turnTo(Constants.BLUE_ANGLE)
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
                        intake.setIntakePower(0),
                        intake.setAntiPower(0),
                        shooter.stop(),
                        transfer.stop()
                )
        );
    }

}