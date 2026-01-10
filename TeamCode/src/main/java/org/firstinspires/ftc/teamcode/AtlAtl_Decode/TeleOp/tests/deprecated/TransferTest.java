/*package org.firstinspires.ftc.teamcode.AtlAtl_Decode.TeleOp.tests;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.configs.TeleOpConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name="Transfer Test", group="tests")
public class TransferTest extends OpMode {
    private DcMotorEx transfer;

    @Override
    public void init() {
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        Transfer();
    }

    private boolean transferStopped = false;
    private boolean transferPrev = false;
    private double transferFreeVel = 0.0;
    private double learn_alfa = TeleOpConfig.learn_alfa;
    private double slipGain = TeleOpConfig.slipGain;
    private double minTransfer = TeleOpConfig.min_transfer;

    public void Transfer() {
        boolean togglePressed = gamepad2.right_trigger > 0.1;
        if (togglePressed && !transferPrev) {
            transferStopped = !transferStopped;
        }
        transferPrev = togglePressed;

        double basePower;
        if (gamepad2.left_trigger > 0.1) {
            basePower = 1.0;
        } else if (transferStopped) {
            basePower = 0.0;
        } else {
            basePower = -0.75;
        }

        if (basePower == 0.0) {
            transfer.setPower(0.0);
            return;
        }

        double vel = Math.abs(transfer.getVelocity());

        if (vel > transferFreeVel) {
            transferFreeVel += (vel - transferFreeVel) * learn_alfa;
        }

        double slip = 1.0 - (vel / Math.max(transferFreeVel, 1.0));
        slip = Math.max(0.0, Math.min(1.0, slip));

        double scaledPower = basePower * (1.0 - slip * slipGain);

        if (Math.abs(scaledPower) < minTransfer) {
            scaledPower = Math.signum(scaledPower) * minTransfer;
        }

        transfer.setPower(scaledPower);
    }

}

 */