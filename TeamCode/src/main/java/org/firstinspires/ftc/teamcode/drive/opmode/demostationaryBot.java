package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
@Autonomous
public class demostationaryBot extends LinearOpMode {
    CRServo rightServoWheel;
    CRServo leftServoWheel;

    @Override
    public void runOpMode() throws InterruptedException {

        rightServoWheel = hardwareMap.get(CRServo.class, "rightServoWheel");
        leftServoWheel = hardwareMap.get(CRServo.class, "leftServoWheel");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            rightServoWheel.setPower(0.25);
            leftServoWheel.setPower(-0.25);
        }
    }
}
