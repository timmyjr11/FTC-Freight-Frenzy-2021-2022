package testCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class lm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("lmao i ded");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("LMAO I LIB olololololollo");
            telemetry.update();
        }
    }
}
