package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain;

public class PrioritySuppression {

    public double[] apply(double strafe, double vertical, double heading, boolean isLocked) {
        double absS = Math.abs(strafe);
        double absV = Math.abs(vertical);
        double absH = Math.abs(heading);

        double eps = 1e-6;
        double translationMag = Math.hypot(strafe, vertical);

        // rot supress
        if (translationMag > 0.15) {
            double rotRatio = absH / (absH + translationMag + eps);

            //in HEADING_LOCK==> heading more priority (0.7)
            // in MANUAL==> keep 0.4 priority to favor smooth driving.
            double minRotationPriority = isLocked ? 0.7 : 0.4;

            rotRatio = Math.max(rotRatio, minRotationPriority);
            heading *= rotRatio;
        }

        // vert/strafe suppress
        if (absV > absS) {
            double ratio = absS / (absV + eps);
            ratio = Math.max(ratio, 0.35);
            strafe *= ratio;
        } else {
            double ratio = absV / (absS + eps);
            ratio = Math.max(ratio, 0.35);
            vertical *= ratio;
        }

        if (absV < 0.3 && absS > absV) {
            vertical *= 0.5;
        }

        return new double[]{strafe, vertical, heading};
    }
}