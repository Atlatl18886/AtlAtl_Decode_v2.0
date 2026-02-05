package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Blue9Ball {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.85, 17.5)
                .build();

        Action preload = myBot.getDrive().actionBuilder(Constants.BLUE_CLOSE_START)
                .strafeToLinearHeading(Constants.BLUE_SHOOT, Constants.BLUE_ANGLE)
                .waitSeconds(0.7)
                .build();

        Action getRow1 = myBot.getDrive().actionBuilder(new Pose2d(Constants.BLUE_SHOOT, Constants.BLUE_ANGLE))
                .strafeToLinearHeading(Constants.BLUE_READY1, Constants.BLUE_INTAKE_ANGLE)
                .strafeTo(Constants.BLUE_ROW1)
                .waitSeconds(0.5)
                .build();

        Action scoreRow1 = myBot.getDrive().actionBuilder(new Pose2d(Constants.BLUE_ROW1, Constants.BLUE_INTAKE_ANGLE))
                .strafeToLinearHeading(Constants.BLUE_SHOOT, Constants.BLUE_ANGLE)
                .build();


        Action getRow2 = myBot.getDrive().actionBuilder(new Pose2d(Constants.BLUE_SHOOT, Constants.BLUE_ANGLE))
                .strafeToLinearHeading(Constants.BLUE_READY2, Constants.BLUE_INTAKE_ANGLE)
                .strafeTo(Constants.BLUE_ROW2)
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(Constants.BLUE_READY2.x, Constants.BLUE_ROW2.y+15))
                .build();

        Action driveToGate = myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(Constants.BLUE_READY2.x, Constants.BLUE_ROW2.y+15), Constants.BLUE_INTAKE_ANGLE))

                .waitSeconds(0.3)
                .strafeToLinearHeading(Constants.BLUE_GATE, 0)
                .waitSeconds(0.65)

                .strafeTo(Constants.BLUE_GATE_READY)

                .build();


        Action scoreRow2 = myBot.getDrive().actionBuilder(new Pose2d(Constants.BLUE_GATE_READY, 0))
                .strafeTo(Constants.BLUE_SHOOT)
                .turnTo(Constants.BLUE_ANGLE)
                .waitSeconds(0.5)
                .build();


        myBot.runAction(
                new SequentialAction(
                        preload,
                        getRow1,
                        scoreRow1,
                        getRow2,
                        driveToGate,
                        scoreRow2
                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}