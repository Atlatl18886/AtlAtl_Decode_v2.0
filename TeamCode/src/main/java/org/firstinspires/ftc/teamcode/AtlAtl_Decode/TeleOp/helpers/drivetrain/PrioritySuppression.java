package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.helpers.drivetrain;

public class PrioritySuppression {

     //applies ratio-based suppression between strafe, vertical, and heading.
    //returns 3-elem array -- {strafe, vert, heading}
    public double[] apply(double strafe, double vertical, double heading) {
        double absS = Math.abs(strafe);
        double absV = Math.abs(vertical);
        double absH = Math.abs(heading);

        double eps = 1e-6;
        double translationMag = Math.hypot(strafe, vertical);

        //rot suppression when translating
        if (translationMag > 0.15) {
            double rotRatio = absH / (absH + translationMag + eps);
            rotRatio = Math.max(rotRatio, 0.4);
            heading *= rotRatio;
        }

        // strafe/vertical suppression
        if (absV > absS) {
            double ratio = absS / (absV + eps);
            ratio = Math.max(ratio, 0.35);
            strafe *= ratio;
        } else {
            double ratio = absV / (absS + eps);
            ratio = Math.max(ratio, 0.35);
            vertical *= ratio;
        }

        //low vertical --> dampen vertical when strafe is high
        if (absV < 0.3 && absS > absV) {
            vertical *= 0.5;
        }

        return new double[]{strafe, vertical, heading};
    }
}
