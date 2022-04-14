package testCodes.robotTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@Disabled
@TeleOp
public class servoTest extends LinearOpMode {

    CRServo leftServoWheel;
    CRServo rightServoWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        leftServoWheel = hardwareMap.get(CRServo.class, "leftServoWheel");
        rightServoWheel = hardwareMap.get(CRServo.class, "rightServoWheel");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            //Pressing left trigger more than halfway activates the left carousel wheel
            if (gamepad1.left_trigger >= 0.5) {
                leftServoWheel.setPower(1);
            } else {
                leftServoWheel.setPower(0);
            }

            //Pressing right trigger more than halfway activates the right carousel wheel
            if (gamepad1.right_trigger >= 0.5) {
                rightServoWheel.setPower(1);
            } else {
                rightServoWheel.setPower(0);
            }
        }
    }
}
