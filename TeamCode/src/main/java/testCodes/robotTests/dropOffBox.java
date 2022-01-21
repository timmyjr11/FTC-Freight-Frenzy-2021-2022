package testCodes.robotTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class dropOffBox extends LinearOpMode {
    Servo leftBox;
    Servo rightBox;

    boolean previousA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        leftBox = hardwareMap.get(Servo.class, "leftBox");
        rightBox = hardwareMap.get(Servo.class, "rightBox");

        leftBox.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        leftBox.setPosition(0);
        rightBox.setPosition(0);

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad2.a && !previousA){
                if (leftBox.getPosition() == 0 && rightBox.getPosition() == 0) {
                    leftBox.setPosition(1);
                    rightBox.setPosition(1);
                } else if (rightBox.getPosition() == 1 && rightBox.getPosition() ==1) {
                    leftBox.setPosition(0);
                    rightBox.setPosition(0);
                }
            }

            previousA = gamepad2.a;
        }
    }
}
