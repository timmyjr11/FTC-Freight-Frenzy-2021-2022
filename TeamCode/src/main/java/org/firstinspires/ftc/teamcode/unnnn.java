package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class unnnn extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("LOL");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("HA LMAO");
            telemetry.update();
        }
    }
}
