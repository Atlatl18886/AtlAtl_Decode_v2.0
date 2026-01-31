package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public interface Constants {

    int CLOSE_VELOCITY = 1050;
    int FAR_VELOCITY = 1500;

//red artifacts
    Vector2d RED_READY1 = new Vector2d(-12, 25);
    Vector2d RED_ROW1 = new Vector2d(-12,50);
    double red_yready_1 = 25;
    //row/set 2
    Vector2d RED_READY2 = new Vector2d(12, 25);
    Vector2d RED_ROW2 = new Vector2d(12,50);

    //row/set 3
    Vector2d RED_READY3 = new Vector2d(35, 25);
    Vector2d RED_ROW3 = new Vector2d(35,50);



    //blue aritfacts+ready positions

    //row/set 1
    Vector2d BLUE_READY1 = new Vector2d(-12, -25);
    Vector2d BLUE_ROW1 = new Vector2d(-12,-50);
    double blue_yready_1 = -25;

    //row/set 2
    Vector2d BLUE_READY2 = new Vector2d(12, -25);
    Vector2d BLUE_ROW2 = new Vector2d(12,-50);

    //row/set 3
    Vector2d BLUE_READY3 = new Vector2d(35, -25);
    Vector2d BLUE_ROW3 = new Vector2d(35,-50);

    //gate poses
    Vector2d RED_GATE = new Vector2d(4, 58);
    Vector2d RED_GATE_READY = new Vector2d(4, 54);
    Vector2d RED_GATE_SAFETY = new Vector2d(-10, red_yready_1);

    Vector2d BLUE_GATE = new Vector2d(4, -58);
    Vector2d BLUE_GATE_READY = new Vector2d(4, -54);
    Vector2d BLUE_GATE_SAFETY = new Vector2d(-10, blue_yready_1);

    //general
    double BLUE_INTAKE_ANGLE = Math.toRadians(-90);
    double RED_INTAKE_ANGLE = Math.toRadians(90);


    //starting poses

    Pose2d RED_CLOSE_START = new Pose2d(-60, 38, Math.toRadians(0));

    Pose2d BLUE_CLOSE_START = new Pose2d(-60, -38, Math.toRadians(0));

    //shooting poses
    double BLUE_ANGLE = Math.toRadians(50);
    double RED_ANGLE = Math.toRadians(-50);
    Vector2d RED_SHOOT = new Vector2d(-31,25);
    Vector2d BLUE_SHOOT = new Vector2d(-31,-25);

}