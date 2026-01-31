package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Red12Ball {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        double RED_INTAKE_ANGLE = Constants.RED_INTAKE_ANGLE;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(62, 62, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.85, 17.5)
                .build();


        Action preload = myBot.getDrive().actionBuilder(Constants.RED_CLOSE_START)
                .strafeToLinearHeading(Constants.RED_SHOOT, Constants.RED_ANGLE)
                .waitSeconds(0.7)
                .build();

        Action getRow1 = myBot.getDrive().actionBuilder(new Pose2d(Constants.RED_SHOOT, Constants.RED_ANGLE))
                .strafeToLinearHeading(Constants.RED_READY1, RED_INTAKE_ANGLE)
                .strafeTo(Constants.RED_ROW1)
                .waitSeconds(0.5)
                .strafeTo(Constants.RED_READY1)
                .build();

        Action scoreRow1 = myBot.getDrive().actionBuilder(new Pose2d(Constants.RED_READY1, RED_INTAKE_ANGLE))
                .strafeToLinearHeading(Constants.RED_SHOOT, Constants.RED_ANGLE)
                .build();



        Action getRow2 = myBot.getDrive().actionBuilder(new Pose2d(Constants.RED_SHOOT, Constants.RED_ANGLE))
                .strafeToLinearHeading(Constants.RED_READY2, RED_INTAKE_ANGLE)
                .strafeTo(Constants.RED_ROW2)
                .waitSeconds(0.5)
                .strafeTo(Constants.RED_READY2)
                .build();

        Action scoreRow2 = myBot.getDrive().actionBuilder(new Pose2d(Constants.RED_READY2, RED_INTAKE_ANGLE))
                .strafeToLinearHeading(Constants.RED_SHOOT, Constants.RED_ANGLE)
                .waitSeconds(0.5)
                .build();



        Action driveToGate = myBot.getDrive().actionBuilder(new Pose2d(Constants.RED_SHOOT, Constants.RED_ANGLE))
                .strafeToLinearHeading(Constants.RED_GATE_SAFETY, 0)

                .strafeTo(Constants.RED_GATE)
                .waitSeconds(0.5)

                .strafeTo(Constants.RED_GATE_READY)
                .build();


        Action getRow3 = myBot.getDrive().actionBuilder(new Pose2d(Constants.RED_GATE_READY, 0))
                .splineToLinearHeading(new Pose2d(Constants.RED_READY3, RED_INTAKE_ANGLE), Math.toRadians(0))
                .strafeTo(Constants.RED_ROW3)
                .waitSeconds(0.5)
                .strafeTo(Constants.RED_READY3)
                .build();

        Action scoreRow3 = myBot.getDrive().actionBuilder(new Pose2d(Constants.RED_READY3, RED_INTAKE_ANGLE))
                .strafeToLinearHeading(Constants.RED_SHOOT, Constants.RED_ANGLE)
                .waitSeconds(0.5)
                .build();

        myBot.runAction(
                new SequentialAction(
                        preload,
                        getRow1,
                        scoreRow1,
                        getRow2,
                        scoreRow2,
                        driveToGate,
                        getRow3,
                        scoreRow3
                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}