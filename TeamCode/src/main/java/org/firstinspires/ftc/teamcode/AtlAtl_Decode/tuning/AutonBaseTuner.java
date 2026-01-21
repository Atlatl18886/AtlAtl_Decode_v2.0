package org.firstinspires.ftc.teamcode.AtlAtl_Decode.tuning;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous.templates.AutonomousBase;

@Autonomous(name="debug tuner", group="Tuning")
public class AutonBaseTuner extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initDt();
        telemetry.addData("Status", "ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveFor(48, 0.4, 7.5);//if more increase, if less decrease TPI
            //New Multiplier = Current Multiplier (1.1) * (Target Distance / Actual Distance)
//            sleep(2000); // Pause to measure
//
//
//            //if overshoots, decrease turn_kP. If it oscillates, increase turn_kD.
//            turnToHeading(90, 3.0);
//            sleep(2000);
//
//            strafeFor(24, 0.4, 4.0);
//
//            driveFor(72, 0.4, 10.0);
//            //disturb while going
//            //if stays crooked or takes several feet to correct itself, increase drive_correction_kP in small increments.
//            //value to high:
//            //if rapidly shakes the nose left and right even when not touched, decrease drive_correction_kP.
//            //u want the highest value possible that does not cause shaking. This ensures the robot snaps back to its correct heading immediately after hitting a bump or an uneven tile.

            while (opModeIsActive()) {
                telemetry.addData("Final Heading", getHeading());
                telemetry.update();
            }
        }
    }
}
