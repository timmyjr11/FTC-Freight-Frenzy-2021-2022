package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class lm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("lmao i ded");
        telemetry.update();
        waitForStart();

        telemetry.addLine("LMAO I LIB");
        telemetry.update();
    }
}
