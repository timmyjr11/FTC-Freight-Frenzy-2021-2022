package HannahCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class PurplePotato extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("peeople");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addLine("gaaaaaaaaaaaaaaaaaaaa");
            telemetry.update();
        }

    }
}
