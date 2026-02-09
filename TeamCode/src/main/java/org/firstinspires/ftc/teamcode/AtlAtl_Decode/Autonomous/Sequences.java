package org.firstinspires.ftc.teamcode.AtlAtl_Decode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.intake.Intake;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.shooter.Shooter;
import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.roadrunner.transfer.Transfer;

import java.util.HashMap;
import java.util.Map;


public class Sequences {

    Intake intake;
    Transfer transfer;
    Shooter shooter;
//    Map<String, String> map = new HashMap<String, String>();
//    map.put("dog", "type of animal");

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
                        shooter.spinUp(1535),
                        shooter.waitForRPM(1530),
                        new SleepAction(0.4),
                        transfer.pulse(0.4),
                        new SleepAction(2),
                        transfer.pulse(0.4),
                        new SleepAction(2),
                        transfer.pulse(0.6)
                ),
                intake.stop(),
                transfer.stop(),
                shooter.stop()
        );
    }
    public Action scoreSet(double rpm) {
        return new SequentialAction(
                intake.in(),
                transfer.idle(),
                new SequentialAction(
                        shooter.spinUp(rpm),
                        shooter.waitForRPM(rpm),
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
    public Action scoreSet(double rpm, double waitTime) {

        return new SequentialAction(
                intake.in(),
                transfer.idle(),
                new SequentialAction(
                        shooter.spinUp(rpm),
                        shooter.waitForRPM(rpm),
                        transfer.pulse(0.4),
                        new SleepAction(waitTime),
                        transfer.pulse(0.4),
                        new SleepAction(waitTime),
                        transfer.pulse(0.4)
                ),
                intake.stop(),
                transfer.stop(),
                shooter.stop()
        );
    }
    public Action startSub() {
        return new ParallelAction(
                intake.in(),
                transfer.idle()
        );
    }
    public Action scoreSet(String preset) {
        double rpm;
        switch (preset) {
            case "veryclose":
                rpm = 1250;
                break;
            case "mid":
                rpm = 1450;
                break;
            case "crazy":
                rpm = 2600;
                break;
            default:
                rpm = 1300;
                break;
        }
        return new SequentialAction(

                intake.in(),
                transfer.idle(),
                new SequentialAction(
                        shooter.spinUp(rpm),
                        shooter.waitForRPM(rpm),
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
