package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake.Intake;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Autonomous(name="RRBasicAuton", group="RoadRunner")
public class RRBasicAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Intake intake = new Intake();

        waitForStart();
        if (isStopRequested()) return;

        //Action path = intake.intakeReverse();

        //Action.runBlocking(path);
    }
}