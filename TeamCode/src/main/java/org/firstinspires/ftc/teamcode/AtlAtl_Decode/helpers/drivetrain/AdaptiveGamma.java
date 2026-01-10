package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain;

public class AdaptiveGamma {
    private final double gammaMax;
    private final double gammaMin;
    private final double transStart;
    private final double transEnd;

    public AdaptiveGamma(double gammaMax, double gammaMin, double transStart, double transEnd) {
        this.gammaMax = gammaMax;
        this.gammaMin = gammaMin;
        this.transStart = transStart;
        this.transEnd = transEnd;
    }

    public double compute(double robotSpeed) {
        if (robotSpeed <= transStart) {
            return gammaMax;
        } else if (robotSpeed >= transEnd) {
            return gammaMin;
        } else {
            double t = (robotSpeed - transStart) / (transEnd - transStart);
            return gammaMax + (gammaMin - gammaMax) * t;
        }
    }
}
