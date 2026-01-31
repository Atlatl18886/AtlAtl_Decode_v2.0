package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake.Intake;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.shooter.Shooter;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.transfer.Transfer;


public class Sequences {

    Intake intake;
    Transfer transfer;
    Shooter shooter;

    public Sequences(HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }

    public Action scoreSet() {
        return new SequentialAction(

                intake.in(),
                transfer.idle(),
                new SequentialAction(
                        shooter.spinUp(1550),
                        shooter.waitForRPM(1550),
                        transfer.pulse(0.4),
                        new SleepAction(2),
                        transfer.pulse(0.4),
                        new SleepAction(2),
                        transfer.pulse(0.4)
                ),
                intake.stop(),
                transfer.stop(),
                shooter.stop()
        );
    }

}
