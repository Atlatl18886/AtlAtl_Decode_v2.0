package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.drivetrain;

public class PrioritySuppression {

    public double[] apply(double strafe, double vertical, double heading, boolean isLocked) {
        double absS = Math.abs(strafe);
        double absV = Math.abs(vertical);

        //if one axis is more dominant than other, supress weaker axis
        if (absV > absS * 2.0) {
            //y -> supress strafe
            strafe *= 0.1;
        } else if (absS > absV * 2.0) {
            //strafe->supress y
            vertical *= 0.1;
        }
        // damp rx
        double translationMag = Math.hypot(strafe, vertical);

        if (translationMag > 0.4) {
            if (Math.abs(heading) < 0.4) {
                heading *= 0.4;
            }
        }

        //if locked, let heading thing do its stuff
        if (isLocked) {
            if (Math.abs(heading) < 0.15) {
                heading = 0.0;
            } else {
                heading *= 0.6; //soft the manual override
            }
        }

        return new double[]{strafe, vertical, heading};
    }
}