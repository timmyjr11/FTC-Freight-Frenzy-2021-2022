package teachingCodes.hannahCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
@Disabled
public class PurplePotato extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("peeople");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addLine("Lmao");
            telemetry.update();
        }

    }
}
