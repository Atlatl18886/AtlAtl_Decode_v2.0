package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Autonomous(name="RRBasicAuton", group="RoadRunner")
public class RRBasicAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        // start at 0,0 facing forward
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();
        if (isStopRequested()) return;

        Action fullAutonPath = drive.actionBuilder(startPose)
                .lineToX(24) // foarward 2ft
                .lineToY(-12) //strafe right 1 ft
                .turn(Math.toRadians(90)) //90 deg cc

                .build();

        Actions.runBlocking(fullAutonPath);
    }
}