package testCodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class idk extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("LOL");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("AH");
            telemetry.update();
        }
    }
}
