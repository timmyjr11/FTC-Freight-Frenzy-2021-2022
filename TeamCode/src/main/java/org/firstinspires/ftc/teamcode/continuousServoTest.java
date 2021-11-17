package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous
public class continuousServoTest extends LinearOpMode {

    CRServo continuous;
    CRServo continuous2;


    @Override
    public void runOpMode() throws InterruptedException {
        continuous = hardwareMap.get(CRServo.class, "continuous");
        continuous2 = hardwareMap.get(CRServo.class, "continuous2");


        waitForStart();

        while (opModeIsActive()) {
            continuous.setPower(1);
            continuous2.setPower(1);
        }
    }
}