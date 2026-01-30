package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class Red6Ball {
    public static void main(String[] args) {
        double Xready = -12;
        double Yready = 25;
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.85, 17.5)
                .build();
        Action preload = myBot.getDrive().actionBuilder(Constants.RED_CLOSE_START)
                .strafeToLinearHeading(Constants.RED_SHOOT,Constants.RED_ANGLE)
                .waitSeconds(0.7)
                .build();
        Action balls = myBot.getDrive().actionBuilder(new Pose2d(Constants.RED_SHOOT, Constants.RED_ANGLE))
                .turnTo(0)
                .lineToX(Xready)
                .turnTo(Math.toRadians(90))
                .waitSeconds(0.5)
                .lineToY(50)
                .build();
        Action back = myBot.getDrive().actionBuilder(new Pose2d(Xready,50, Math.toRadians(90)))
                .lineToY(Yready)
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(90))
                .build();
        Action back2 = myBot.getDrive().actionBuilder(new Pose2d(Xready,Yready,Math.toRadians(90)))
                .turnTo(Math.toRadians(0))
                .lineToX(-31)
                .turnTo(Constants.RED_ANGLE)
                .build();
        myBot.runAction(
                new SequentialAction(
                        preload,
                        balls,
                        back,
                        back2
                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}