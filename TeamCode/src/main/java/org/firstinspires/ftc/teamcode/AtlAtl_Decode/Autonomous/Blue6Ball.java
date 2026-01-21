package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous.templates.AutonomousBase;
@Autonomous(name="Blue 6", group="LCs")
public class Blue6Ball extends AutonomousBase {
    @Override
    public void runOpMode() {
        initDt();

        waitForStart();

        if (opModeIsActive()) {
            driveFor(24, 0.5, 3.0);      //forward 2 ft
            turnToHeading(90, 2.0);      //90 right
            shooter.setPower(0.8);
            sleep(1000);
            shooter.setPower(0);
        }
    }
}
