package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public interface Constants {

    int CLOSE_VELOCITY = 1050;
    int FAR_VELOCITY = 1500;

//red artifacts
    Vector2d GPP_RED_ARTIFACT = new Vector2d(-36, -26);
    Vector2d PGP_RED_ARTIFACT = new Vector2d(-13, -23);
    Vector2d PPG_RED_ARTIFACT = new Vector2d(10, -56);
    Vector2d HP_RED_ARTIFACT = new Vector2d(-60,-58);


    //blue aritfacts
    Vector2d GPP_BLUE_ARTIFACT = new Vector2d(-36, 26);
    Vector2d PGP_BLUE_ARTIFACT = new Vector2d(-13, 23);
    Vector2d PPG_BLUE_ARTIFACT = new Vector2d(10, 56);
    Vector2d HP_BLUE_ARTIFACT = new Vector2d(-60,58);

    Vector2d BLUE_GATE = new Vector2d(4, 55);
    Vector2d RED_GATE = new Vector2d(4, -55);


    //starting poses

    Pose2d RED_CLOSE_START = new Pose2d(57, -45, Math.toRadians(-37));

    Pose2d BLUE_CLOSE_START = new Pose2d(57, 45, Math.toRadians(53));

}