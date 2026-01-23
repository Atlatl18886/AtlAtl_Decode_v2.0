package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake.Intake;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
/// TODO: RR TUNING https://www.youtube.com/watch?v=x4WbBeTiCPg
@Autonomous(name="RRBasicAuton", group="RoadRunner")
public class RRBasicAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        // 1.intake on
                        .stopAndAdd(intake.setIntakePower(1.0))

                        // 2.forward 2 ft
                        .lineToX(24)

                        // 3.intake off
                        .stopAndAdd(intake.setIntakePower(0))

                        .build()
        );
    }

}